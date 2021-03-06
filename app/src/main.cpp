#include <stm32.h>
#include <irq.h>
#include "uavcan.h"
#include "can.h"
#include "configuration.h"
#include "fastmath.h"
#include "shared.h"


#include "uavcan/protocol/param/ExecuteOpcode.hpp"
#include "uavcan/protocol/param/GetSet.hpp"
#include "uavcan/protocol/file/BeginFirmwareUpdate.hpp"
#include "uavcan/protocol/GetNodeInfo.hpp"
#include "uavcan/protocol/NodeStatus.hpp"
#include "uavcan/protocol/RestartNode.hpp"
#include "uavcan/equipment/air_data/TrueAirspeed.hpp"
#include "uavcan/equipment/air_data/IndicatedAirspeed.hpp"
#include "uavcan/equipment/air_data/StaticPressure.hpp"
#include "uavcan/equipment/air_data/StaticTemperature.hpp"


enum uavcan_dtid_filter_id_t {
    UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE = 0u,
    UAVCAN_PROTOCOL_PARAM_GETSET,
    UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE,
    UAVCAN_PROTOCOL_GETNODEINFO,
    UAVCAN_PROTOCOL_RESTARTNODE,
    UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE,
    UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE
};


static volatile uint32_t g_uptime;
static struct bootloader_app_shared_t g_bootloader_app_shared;


/* Written to the firmware image in post-processing */
extern volatile struct bootloader_app_descriptor flash_app_descriptor;


#define SENSOR_STARTUP_MS 200u


static void spi_init(void);
static uint32_t spi_read(void);


static void node_init(uint8_t node_id);
static void node_run(uint8_t node_id, Configuration& configuration);


extern "C" void main(void) {
    up_cxxinitialize();
    board_initialize();
    up_timer_initialize();
    irqenable();

    Configuration configuration;

    /*
    Read bootloader auto-baud and node ID values, then set up the node. We've
    just come from the bootloader, so CAN must already be configured
    correctly.
    */
    if (bootloader_read(&g_bootloader_app_shared)) {
        spi_init();
        can_init(g_bootloader_app_shared.bus_speed);
        node_init(g_bootloader_app_shared.node_id);
        node_run(g_bootloader_app_shared.node_id, configuration);
    } else {
        /* Uh-oh */
        up_systemreset();
    }
}


extern "C" void sched_process_timer(void) {
    g_uptime++;
}


static void node_init(uint8_t node_id) {
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE, true,
        uavcan::protocol::param::ExecuteOpcode::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_PARAM_GETSET, true,
        uavcan::protocol::param::GetSet::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE, true,
        uavcan::protocol::file::BeginFirmwareUpdate::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_GETNODEINFO, true,
        uavcan::protocol::GetNodeInfo::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        1u, UAVCAN_PROTOCOL_RESTARTNODE, true,
        uavcan::protocol::RestartNode::DefaultDataTypeID,
        node_id);

    can_set_dtid_filter(
        0u, UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE, false,
        uavcan::equipment::air_data::StaticPressure::DefaultDataTypeID,
        node_id);
    can_set_dtid_filter(
        0u, UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE, false,
        uavcan::equipment::air_data::StaticTemperature::DefaultDataTypeID,
        node_id);
}


static void spi_init(void) {
    /*
    SPI device configuration. Refer to:
    "SPI Communication with Honeywell Digital Output Pressure Sensors"
    (Honeywell TN008202)

    Clock is low when idle (CPOL=0); data is latched on the positive-going
    clock edge (CPHA=0). Frequency must be 50-800 kHz.

    At least 2.5 us is required between SS going low and the first clock
    transition. At least 2 us is required from SS going high to SS going low
    again.

    Here, set f = PCLK / 64 (i.e. 562.5 kHz) and MSTR mode.
    */
    putreg16(SPI_CR1_FPCLCKd64 | SPI_CR1_MSTR, STM32_SPI3_CR1);

    /*
    Set FRXTH to trigger RXNE when 16 bits have been received, DS to
    1111 for 16-bit, and SSOE to enable the NSS pin.
    */
    putreg16(SPI_CR2_FRXTH | SPI_CR1_DS_16BIT | SPI_CR2_SSOE, STM32_SPI3_CR2);
}


static uint32_t spi_read(void) {
    volatile uint32_t result;

    /* Enable SPI and drive NSS low */
    putreg16(getreg16(STM32_SPI3_CR1) | SPI_CR1_SPE, STM32_SPI3_CR1);
    stm32_gpiowrite(GPIO_NSS, 0u);

    /* Wait a bit */
    for (volatile uint32_t x = 128u; x != 0; x--);

    /* Write 16 bits to the data register */
    putreg16(0, STM32_SPI3_DR);

    /* Wait for TXE and RXNE to go high, and BSY to go low */
    const uint16_t done_status = SPI_SR_TXE | SPI_SR_RXNE;
    const uint16_t done_mask = SPI_SR_TXE | SPI_SR_RXNE | SPI_SR_BSY;
    while ((getreg16(STM32_SPI3_SR) ^ done_status) & done_mask);

    /* Collect the data */
    result = getreg16(STM32_SPI3_DR);

    /* Disable SPI and drive NSS high */
    putreg16(getreg16(STM32_SPI3_CR1) & ~SPI_CR1_SPE, STM32_SPI3_CR1);
    stm32_gpiowrite(GPIO_NSS, 1u);

    return result;
}


static float pressure_from_count(uint16_t count) {
    /* Params for HSCMRRN100MDSA3 */
    const double COUNT_MAX = 16383.0;
    const double PRESSURE_MIN = -100.0, PRESSURE_MAX = 100.0;
    const double PA_PER_MBAR = 100.0;

    float temp;

    temp = float(count) * float(1.0 / COUNT_MAX) - 0.1f;
    temp = temp * float((PRESSURE_MAX - PRESSURE_MIN) / 0.8);
    temp = (temp + float(PRESSURE_MIN)) * float(PA_PER_MBAR);

    return temp;
}


static void __attribute__((noreturn)) node_run(
    uint8_t node_id,
    Configuration& configuration
) {
    size_t length;
    uint32_t message_id, current_time, status_time, tas_time, ias_time,
             status_interval, tas_interval, ias_interval, sensor_data;
    uint8_t filter_id, tas_transfer_id, status_transfer_id,
            ias_transfer_id, message[8], broadcast_filter_id,
            service_filter_id;
    bool param_valid, wants_bootloader_restart;
    struct param_t param;
    float value, ias_temp, tas_temp, ias_lpf_coeff, tas_lpf_coeff, ias_out,
          tas_out, rc, offset_pressure_pa, t_s;

    UAVCANTransferManager broadcast_manager(node_id);
    UAVCANTransferManager service_manager(node_id);

    uavcan::equipment::air_data::StaticTemperature static_temp;
    uavcan::equipment::air_data::StaticPressure static_pressure;
    uavcan::protocol::param::ExecuteOpcode::Request xo_req;
    uavcan::protocol::param::GetSet::Request gs_req;
    uavcan::protocol::RestartNode::Request rn_req;

    float static_pressure_pa, static_pressure_var_pa2;
    float static_temp_k, static_temp_var_k2;
    float differential_pressure_pa;

    tas_transfer_id = status_transfer_id = ias_transfer_id = 0u;
    tas_time = status_time = ias_time = 0u;

    broadcast_filter_id = service_filter_id = 0xFFu;

    wants_bootloader_restart = false;

    status_interval = 900u;

    static_pressure_pa = STANDARD_PRESSURE_PA;
    static_temp_k = STANDARD_TEMP_K;
    differential_pressure_pa = 0.0f;
    offset_pressure_pa = 0.0f;
    tas_out = ias_out = 0.0f;

    t_s = 0.5e-3f;

    while (true) {
        current_time = g_uptime;

        /* Allow intervals to be changed without restarting */
        tas_interval = uint32_t(configuration.get_param_value_by_index(
            PARAM_UAVCAN_TRUEAIRSPEED_INTERVAL) * 1e-3f);
        if (tas_interval == 0.0f) {
            tas_lpf_coeff = 1.0f;
        } else {
            rc = float(tas_interval) / float(2000.0 * M_PI);
            tas_lpf_coeff = t_s / (t_s + 4.0f * rc);
        }

        ias_interval = uint32_t(configuration.get_param_value_by_index(
            PARAM_UAVCAN_INDICATEDAIRSPEED_INTERVAL) * 1e-3f);
        if (ias_interval == 0.0f) {
            ias_lpf_coeff = 1.0f;
        } else {
            rc = float(ias_interval) / float(2000.0 * M_PI);
            tas_lpf_coeff = t_s / (t_s + 4.0f * rc);
        }

        /*
        Read pressure (and temperature?); skip if the status bits are set,
        which means we're reading stale data or there's been a sensor fault.
        */
        if (current_time > SENSOR_STARTUP_MS) {
            sensor_data = spi_read();
            if (!(sensor_data & 0xC000u)) {
                differential_pressure_pa =
                    pressure_from_count(sensor_data & 0x3FFFu);

                if (std::isnan(differential_pressure_pa)) {
                    differential_pressure_pa = 0.0f;
                } else if (current_time < SENSOR_STARTUP_MS * 10) {
                    offset_pressure_pa +=
                        (differential_pressure_pa - offset_pressure_pa) * 0.01f;
                } else {
                    differential_pressure_pa -= offset_pressure_pa;

                    /*
                    Convert dynamic pressure to IAS and TAS based on current
                    static pressure/temp.
                    */
                    ias_temp =
                        airspeed_from_pressure_temp(differential_pressure_pa,
                                                    STANDARD_PRESSURE_PA,
                                                    STANDARD_TEMP_K);
                    tas_temp =
                        airspeed_from_pressure_temp(differential_pressure_pa,
                                                    static_pressure_pa,
                                                    static_temp_k);

                    /* Low-pass both values */
                    ias_out += (ias_temp - ias_out) * ias_lpf_coeff;
                    tas_out += (tas_temp - tas_out) * tas_lpf_coeff;
                }
            }
        }


        /*
        Check for UAVCAN static pressure/temp (FIFO 0) -- these are broadcasts
        */
        while (can_rx(0u, &filter_id, &message_id, &length, message)) {
            uavcan::TransferCRC crc;

            /* Filter IDs are per-FIFO, so convert this back to the bank index */
            filter_id = (uint8_t)(filter_id + UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE);
            switch (filter_id) {
                case UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE:
                    crc = uavcan::equipment::air_data::StaticPressure::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE:
                    crc = uavcan::equipment::air_data::StaticTemperature::getDataTypeSignature().toTransferCRC();
                    break;
                default:
                    break;
            }
            broadcast_manager.receive_frame(current_time, message_id, crc,
                                            length, message);

            if (broadcast_manager.is_rx_done()) {
                broadcast_filter_id = filter_id;
                break;
            }
        }

        if (broadcast_manager.is_rx_done()) {
            if (broadcast_filter_id == UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE &&
                    broadcast_manager.decode(static_pressure)) {
                static_pressure_pa = static_pressure.static_pressure;
                static_pressure_var_pa2 =
                    static_pressure.static_pressure_variance;
            } else if (broadcast_filter_id == UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE &&
                        broadcast_manager.decode(static_temp)) {
                static_temp_k = static_temp.static_temperature;
                static_temp_var_k2 =
                    static_temp.static_temperature_variance;
            }

            broadcast_manager.receive_acknowledge();
        }

        /*
        Check for UAVCAN service requests (FIFO 1) -- only process if the
        first byte of the data is the local node ID
        */
        while (can_rx(1u, &filter_id, &message_id, &length, message)) {
            uavcan::TransferCRC crc;
            switch (filter_id) {
                case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE:
                    crc = uavcan::protocol::param::ExecuteOpcode::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_PARAM_GETSET:
                    crc = uavcan::protocol::param::GetSet::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE:
                    crc = uavcan::protocol::file::BeginFirmwareUpdate::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_GETNODEINFO:
                    crc = uavcan::protocol::GetNodeInfo::getDataTypeSignature().toTransferCRC();
                    break;
                case UAVCAN_PROTOCOL_RESTARTNODE:
                    crc = uavcan::protocol::RestartNode::getDataTypeSignature().toTransferCRC();
                    break;
                default:
                    break;
            }
            service_manager.receive_frame(current_time, message_id, crc,
                                          length, message);

            if (service_manager.is_rx_done()) {
                service_filter_id = filter_id;
                break;
            }
        }

        /*
        Don't process service requests until the last service response is
        completely sent, to avoid overwriting the TX buffer.
        */
        if (service_manager.is_rx_done() && service_manager.is_tx_done()) {
            if (service_filter_id == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE &&
                    service_manager.decode(xo_req)) {
                /*
                Return OK if the opcode is understood and the controller is
                stopped, otherwise reject.
                */
                uavcan::protocol::param::ExecuteOpcode::Response xo_resp;
                xo_resp.ok = false;
                if (xo_req.opcode == xo_req.OPCODE_SAVE) {
                    configuration.write_params();
                    xo_resp.ok = true;
                } else if (xo_req.opcode == xo_req.OPCODE_ERASE) {
                    /*
                    Set all parameters to default values, then erase the flash
                    */
                    configuration.reset_params();
                    configuration.write_params();
                    xo_resp.ok = true;
                }
                service_manager.encode_response<uavcan::protocol::param::ExecuteOpcode>(xo_resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_PARAM_GETSET &&
                    service_manager.decode(gs_req)) {
                uavcan::protocol::param::GetSet::Response resp;

                if (!gs_req.name.empty()) {
                    param_valid = configuration.get_param_by_name(
                        param, gs_req.name.c_str());
                } else {
                    param_valid = configuration.get_param_by_index(
                        param, (uint8_t)gs_req.index);
                }

                if (param_valid) {
                    if (param.public_type == PARAM_TYPE_FLOAT && !gs_req.name.empty() &&
                            gs_req.value.is(uavcan::protocol::param::Value::Tag::real_value)) {
                        value = gs_req.value.to<uavcan::protocol::param::Value::Tag::real_value>();
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    } else if (param.public_type == PARAM_TYPE_INT && !gs_req.name.empty() &&
                            gs_req.value.is(uavcan::protocol::param::Value::Tag::integer_value)) {
                        value = (float)((int32_t)gs_req.value.to<uavcan::protocol::param::Value::Tag::integer_value>());
                        configuration.set_param_value_by_index(param.index,
                                                               value);
                    }

                    value = configuration.get_param_value_by_index(
                        param.index);

                    resp.name = (const char*)param.name;
                    if (param.public_type == PARAM_TYPE_FLOAT) {
                        resp.value.to<uavcan::protocol::param::Value::Tag::real_value>() = value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::real_value>() = param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = param.max_value;
                    } else if (param.public_type == PARAM_TYPE_INT) {
                        resp.value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int32_t)value;
                        resp.default_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = (int32_t)param.default_value;
                        resp.min_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int32_t)param.min_value;
                        resp.max_value.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = (int32_t)param.max_value;
                    }
                }

                service_manager.encode_response<uavcan::protocol::param::GetSet>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE) {
                uavcan::protocol::file::BeginFirmwareUpdate::Response resp;

                /*
                Don't actually need to decode since we don't care about the
                request data
                */
                resp.error = resp.ERROR_OK;
                wants_bootloader_restart = true;
                service_manager.encode_response<uavcan::protocol::file::BeginFirmwareUpdate>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_GETNODEINFO) {
                uavcan::protocol::GetNodeInfo::Response resp;

                /* Empty request so don't need to decode */
                resp.status.uptime_sec = current_time / 1000u;
                resp.status.health = resp.status.HEALTH_OK;
                resp.status.mode = resp.status.MODE_OPERATIONAL;
                resp.status.sub_mode = 0u;
                resp.status.vendor_specific_status_code = 0u;
                resp.software_version.major =
                    flash_app_descriptor.major_version;
                resp.software_version.minor =
                    flash_app_descriptor.minor_version;
                resp.software_version.optional_field_flags =
                    resp.software_version.OPTIONAL_FIELD_FLAG_VCS_COMMIT |
                    resp.software_version.OPTIONAL_FIELD_FLAG_IMAGE_CRC;
                resp.software_version.vcs_commit =
                    flash_app_descriptor.vcs_commit;
                resp.software_version.image_crc =
                    flash_app_descriptor.image_crc;
                resp.hardware_version.major = HW_VERSION_MAJOR;
                resp.hardware_version.minor = HW_VERSION_MINOR;
                /* Set the unique ID */
                memset(resp.hardware_version.unique_id.begin(), 0u,
                       resp.hardware_version.unique_id.size());
                memcpy(resp.hardware_version.unique_id.begin(),
                       (uint8_t*)0x1ffff7ac, 12u);
                /* Set the hardware name */
                resp.name = HW_UAVCAN_NAME;

                service_manager.encode_response<uavcan::protocol::GetNodeInfo>(resp);
            } else if (service_filter_id == UAVCAN_PROTOCOL_RESTARTNODE &&
                    service_manager.decode(rn_req)) {
                uavcan::protocol::RestartNode::Response resp;

                /*
                Restart if the magic number is correct, otherwise reject.
                */
                if (rn_req.magic_number == rn_req.MAGIC_NUMBER) {
                    resp.ok = true;
                    wants_bootloader_restart = true;
                } else {
                    resp.ok = false;
                }
                service_manager.encode_response<uavcan::protocol::RestartNode>(resp);
            }

            service_manager.receive_acknowledge();
        }

        /* Transmit service responses if available */
        if (can_is_ready(1u) &&
                service_manager.transmit_frame(message_id, length, message)) {
            can_tx(1u, message_id, length, message);
        }

        if (broadcast_manager.is_tx_done() && service_manager.is_tx_done() &&
                !service_manager.is_rx_in_progress(current_time) &&
                current_time > SENSOR_STARTUP_MS) {
            if (tas_interval && current_time - tas_time >= tas_interval) {
                uavcan::equipment::air_data::TrueAirspeed msg;

                msg.true_airspeed = std::abs(tas_out);
                msg.true_airspeed_variance = 1.0f;

                broadcast_manager.encode_message(tas_transfer_id++, msg);
                tas_time = current_time;
            } else if (ias_interval &&
                        current_time - ias_time >= ias_interval) {
                uavcan::equipment::air_data::IndicatedAirspeed msg;

                msg.indicated_airspeed = std::abs(ias_out);
                msg.indicated_airspeed_variance = 1.0f;

                broadcast_manager.encode_message(ias_transfer_id++, msg);
                ias_time = current_time;
            } else if (current_time - status_time >= status_interval) {
                uavcan::protocol::NodeStatus msg;

                msg.uptime_sec = current_time / 1000u;
                msg.health = msg.HEALTH_OK;
                msg.mode = msg.MODE_OPERATIONAL;
                msg.sub_mode = 0u;
                msg.vendor_specific_status_code = 0u;
                broadcast_manager.encode_message(status_transfer_id++, msg);
                status_time = current_time;
            }
        }

        /* Transmit broadcast CAN frames if available */
        if (can_is_ready(0u) &&
                broadcast_manager.transmit_frame(message_id, length, message)) {
            can_tx(0u, message_id, length, message);
        }

        /*
        Only restart into the bootloader if the acknowledgement message has
        been sent.
        */
        if (broadcast_manager.is_tx_done() && service_manager.is_tx_done() &&
                can_is_ready(0u) && can_is_ready(1u) &&
                wants_bootloader_restart) {
            up_systemreset();
        }
    }
}
