/*
Copyright (C) 2014-2015 Thiemar Pty Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <cstdlib>
#include <cstring>
#include <up_progmem.h>
#include <uavcan/data_type.hpp>

#include "configuration.h"


struct flash_param_values_t {
    uint64_t crc;
    uint32_t version;
    float values[64];
};


volatile flash_param_values_t *flash_param_values =
    (volatile flash_param_values_t*)FLASH_PARAM_ADDRESS;


__attribute__((section(".app_descriptor"),used))
volatile struct bootloader_app_descriptor
flash_app_descriptor = {
    .signature = 0x3030637365445041L,
    .image_crc = 0L,
    .image_size = 0L,
    .vcs_commit = 0L,
    .major_version = 0u,
    .minor_version = 1u,
    .reserved = {0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu}
};


static struct param_t param_config_[NUM_PARAMS] = {
    /*
    Index, type, name,
        default value, min value, max value
    */

    /*
    Interval in microseconds at which UAVCAN TrueAirspeed messages
    should be sent. Zero disables publication.
    */
    {PARAM_UAVCAN_TRUEAIRSPEED_INTERVAL, PARAM_TYPE_INT,
        "uavcan.pubp-uavcan.equipment.air_data.TrueAirspeed",
        50e3f, 0.0f, 1e6f},

    /*
    Interval in microseconds at which UAVCAN IndicatedAirspeed messages
    should be sent. Zero disables publication.
    */
    {PARAM_UAVCAN_INDICATEDAIRSPEED_INTERVAL, PARAM_TYPE_INT,
        "uavcan.pubp-uavcan.equipment.air_data.IndicatedAirspeed",
        50e3f, 0.0f, 1e6f},
};


Configuration::Configuration(void) {
    size_t i;
    uavcan::DataTypeSignatureCRC crc;
    crc.add((uint8_t*)(FLASH_PARAM_ADDRESS + sizeof(uint64_t)),
            sizeof(struct flash_param_values_t) - sizeof(uint64_t));

    if (crc.get() == flash_param_values->crc &&
            flash_param_values->version == FLASH_PARAM_VERSION) {
        for (i = 0; i < NUM_PARAMS; i++) {
            if (param_config_[i].min_value <= flash_param_values->values[i] &&
                flash_param_values->values[i] <= param_config_[i].max_value) {
                params_[i] = flash_param_values->values[i];
            } else {
                params_[i] = param_config_[i].default_value;
            }
        }
    } else {
        for (i = 0; i < NUM_PARAMS; i++) {
            params_[i] = param_config_[i].default_value;
        }
    }
}


static size_t _get_param_name_len(const char* name) {
    size_t i;

    for (i = 0; i < PARAM_NAME_MAX_LEN; i++) {
        if (name[i] == 0) {
            return i + 1u;
        }
    }

    return 0;
}


static size_t _find_param_index_by_name(
    const char* name,
    struct param_t params[],
    size_t num_params
) {
    size_t i, name_len;

    name_len = _get_param_name_len(name);
    if (name_len <= 1) {
        return num_params;
    }

    for (i = 0; i < num_params; i++) {
        if (memcmp(params[i].name, name, name_len) == 0) {
            return i;
        }
    }

    return num_params;
}


bool Configuration::get_param_by_name(
    struct param_t& out_param,
    const char* name
) {
    size_t idx;

    idx = _find_param_index_by_name(name, param_config_, NUM_PARAMS);
    return get_param_by_index(out_param, (uint8_t)idx);
}


bool Configuration::get_param_by_index(
    struct param_t& out_param,
    uint8_t index
) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    memcpy(&out_param, &param_config_[index], sizeof(struct param_t));
    return true;
}


bool Configuration::set_param_value_by_name(const char* name, float value) {
    size_t idx;

    idx = _find_param_index_by_name(name, param_config_, NUM_PARAMS);
    return set_param_value_by_index((uint8_t)idx, value);
}


bool Configuration::set_param_value_by_index(uint8_t index, float value) {
    if (index >= NUM_PARAMS) {
        return false;
    }

    if (param_config_[index].min_value <= value &&
            value <= param_config_[index].max_value) {
        params_[index] = value;
        return true;
    } else {
        return false;
    }
}


void Configuration::write_params(void) {
    static_assert(!(sizeof(params_) & 0x3u),
                  "Size of params_ must be a multiple of 4");

    size_t page;
    struct flash_param_values_t new_flash_param_values;
    uavcan::DataTypeSignatureCRC crc;

    memset(&new_flash_param_values, 0xFFu,
           sizeof(struct flash_param_values_t));
    memcpy(new_flash_param_values.values, params_, sizeof(params_));
    new_flash_param_values.version = FLASH_PARAM_VERSION;

    crc.add((uint8_t*)(&new_flash_param_values.version),
            sizeof(struct flash_param_values_t) - sizeof(uint64_t));
    new_flash_param_values.crc = crc.get();

    page = up_progmem_getpage((size_t)flash_param_values);
    up_progmem_erasepage(page);
    up_progmem_write((size_t)flash_param_values, &new_flash_param_values,
                     sizeof(struct flash_param_values_t));
}
