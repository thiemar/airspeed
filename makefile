##############################################################################
# GNU GCC ARM Embeded Toolchain base directories and binaries
##############################################################################
GCC_BASE = /usr/local/opt/gcc-arm-none-eabi/
GCC_BIN  = $(GCC_BASE)bin/
GCC_LIB  = $(GCC_BASE)arm-none-eabi/lib/
GCC_INC  = $(GCC_BASE)arm-none-eabi/include/
AS       = $(GCC_BIN)arm-none-eabi-as
CC       = $(GCC_BIN)arm-none-eabi-gcc
CPP      = $(GCC_BIN)arm-none-eabi-g++
LD       = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY  = $(GCC_BIN)arm-none-eabi-objcopy
SIZE     = $(GCC_BIN)arm-none-eabi-size


##############################################################################
# Custom options for cortex-m and cortex-r processors
##############################################################################
CORTEX_M0PLUS_CC_FLAGS  = -mthumb -mcpu=cortex-m0plus
CORTEX_M0PLUS_LIB_PATH  = $(GCC_LIB)armv6-m
CORTEX_M0_CC_FLAGS      = -mthumb -mcpu=cortex-m0
CORTEX_M0_LIB_PATH      = $(GCC_LIB)armv6-m
CORTEX_M1_CC_FLAGS      = -mthumb -mcpu=cortex-m1
CORTEX_M1_LIB_PATH      = $(GCC_LIB)armv6-m
CORTEX_M3_CC_FLAGS      = -mthumb -mcpu=cortex-m3
CORTEX_M3_LIB_PATH      = $(GCC_LIB)armv7-m
CORTEX_M4_NOFP_CC_FLAGS = -mthumb -mcpu=cortex-m4
CORTEX_M4_NOFP_LIB_PATH = $(GCC_LIB)armv7e-m
CORTEX_M4_SWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16
CORTEX_M4_SWFP_LIB_PATH = $(GCC_LIB)armv7e-m/softfp
CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CORTEX_M4_HWFP_LIB_PATH = $(GCC_LIB)armv7e-m/fpu
CORTEX_R4_NOFP_CC_FLAGS = -mthumb -march=armv7-r
CORTEX_R4_NOFP_LIB_PATH = $(GCC_LIB)armv7-r/thumb
CORTEX_R4_SWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=softfp -mfpu=vfpv3-d16
CORTEX_R4_SWFP_LIB_PATH = $(GCC_LIB)armv7-r/thumb/softfp
CORTEX_R4_HWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=hard -mfpu=vfpv3-d16
CORTEX_R4_HWFP_LIB_PATH = $(GCC_LIB)armv7-r/thumb/fpu
CORTEX_R5_NOFP_CC_FLAGS = -mthumb -march=armv7-r
CORTEX_R5_NOFP_LIB_PATH = $(GCC_LIB)armv7-r/thumb
CORTEX_R5_SWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=softfp -mfpu=vfpv3-d16
CORTEX_R5_SWFP_LIB_PATH = $(GCC_LIB)armv7-r/thumb/softfp
CORTEX_R5_HWFP_CC_FLAGS = -mthumb -march=armv7-r -mfloat-abi=softfp -mfloat-abi=hard -mfpu=vfpv3-d16
CORTEX_R5_HWFP_LIB_PATH = $(GCC_LIB)armv7-r/thumb/fpu


##############################################################################
# Main makefile project configuration
#    PROJECT      = <name of the target to be built>
#    MCU_CC_FLAGS = <one of the CC_FLAGS above>
#    MCU_LIB_PATH = <one of the LIB_PATH above>
#    OPTIMIZE_FOR = < SIZE or nothing >
#    DEBUG_LEVEL  = < -g compiler option or nothing >
#    OPTIM_LEVEL  = < -O compiler option or nothing >
##############################################################################
PROJECT           = app
BUILD             = build/
PROJECT_SRC       = $(PROJECT)/src/
PROJECT_BUILD     = $(BUILD)$(PROJECT)/
MCU_CC_FLAGS      = $(CORTEX_M4_HWFP_CC_FLAGS)
MCU_LIB_PATH      = $(CORTEX_M4_HWFP_LIB_PATH)
DEBUG_LEVEL       =
OPTIM_FLAGS       = -O3
LINKER_SCRIPT     = $(PROJECT)/$(PROJECT).ld
PROJECT_OBJECTS   = $(addprefix $(PROJECT_BUILD), \
					  main.o)
PROJECT_INC_PATHS = -I$(PROJECT)/include/
PROJECT_LIB_PATHS = -L.
PROJECT_LIBRARIES =
PROJECT_SYMBOLS   = -include config.h -DTOOLCHAIN_GCC_ARM -DNO_RELOC='0'


##############################################################################
# Bootloader build configuration
#    BL              = <name of the target to be built>
#    BL_OPTIMIZE_FOR = < SIZE or nothing >
#    BL_DEBUG_LEVEL  = < -g compiler option or nothing >
#    BL_OPTIM_LEVEL  = < -O compiler option or nothing >
##############################################################################
BL                = bootloader
BL_SRC            = $(BL)/src/
BL_BUILD          = $(BUILD)$(BL)/
BL_OPTIMIZE_FOR   = SIZE
BL_DEBUG_LEVEL    =
BL_OPTIM_FLAGS    = -Os
BL_LINKER_SCRIPT  = $(BL)/bootloader.ld
BL_OBJECTS        = $(addprefix $(BL_BUILD), \
					  main.o can.o crc.o flash.o random.o shared.o timer.o \
					  uavcan.o)
BL_INC_PATHS      = -I$(BL)/include/
BL_LIB_PATHS      = -L.
BL_LIBRARIES      =
BL_SYMBOLS        = -include config.h -DTOOLCHAIN_GCC_ARM -DNO_RELOC='0'


##############################################################################
# Main makefile system configuration
##############################################################################
ARCH_SRC = arch/src/
ARCH_BUILD = $(BUILD)arch/
SYS_OBJECTS = $(addprefix $(ARCH_BUILD), \
				stm32_vectors.o up_fpu.o stm32_start.o up_systemreset.o \
				stm32_flash.o up_modifyreg32.o stm32_rcc.o stm32_irq.o \
				stm32_gpio.o stm32_timerisr.o irq_attach.o irq_dispatch.o \
				irq_initialize.o irq_unexpectedisr.o up_doirq.o up_exit.o \
				libc.o)
SYS_INC_PATHS = -Iarch/include -Iarch/src -Iarch/src/chip -I$(GCC_INC)
SYS_LIB_PATHS = -L$(MCU_LIB_PATH)
SYS_LIBRARIES =
SYS_LD_FLAGS  = --specs=nano.specs


##############################################################################
# libuavcan make configuration
##############################################################################
UAVCAN = libuavcan
UAVCAN_SRC = $(UAVCAN)/src/
UAVCAN_BUILD = $(BUILD)libuavcan/
UAVCAN_OBJECTS = $(addprefix $(UAVCAN_BUILD), \
				   marshal/uc_bit_array_copy.o marshal/uc_bit_stream.o \
				   marshal/uc_float_spec.o marshal/uc_scalar_codec.o \
				   node/uc_timer.o transport/uc_crc.o \
				   transport/uc_transfer_buffer.o uc_data_type.o \
				   uc_error.o)
UAVCAN_INC_PATHS = -I$(UAVCAN)/include -I$(UAVCAN)/include/dsdlc_generated -I$(GCC_INC)
UAVCAN_LIB_PATHS = -L$(MCU_LIB_PATH)
UAVCAN_LIBRARIES =
UAVCAN_LD_FLAGS  = --specs=nosys.specs


###############################################################################
# Command line building
###############################################################################
ifdef DEBUG_LEVEL
CC_DEBUG_FLAGS = -g$(DEBUG_LEVEL)
CC_SYMBOLS = -DDEBUG $(PROJECT_SYMBOLS)
else
CC_DEBUG_FLAGS =
CC_SYMBOLS = -DNODEBUG $(PROJECT_SYMBOLS)
endif

ifdef BL_DEBUG_LEVEL
BL_CC_DEBUG_FLAGS = -g$(BL_DEBUG_LEVEL)
BL_CC_SYMBOLS = -DDEBUG $(BL_SYMBOLS)
else
BL_CC_DEBUG_FLAGS =
BL_CC_SYMBOLS = -DNODEBUG $(BL_SYMBOLS)
endif

ARCH_INCLUDE_PATHS = $(SYS_INC_PATHS)
ARCH_CC_FLAGS = $(MCU_CC_FLAGS) -c -Os $(CC_DEBUG_FLAGS) -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections

INCLUDE_PATHS = $(PROJECT_INC_PATHS) $(SYS_INC_PATHS) $(UAVCAN_INC_PATHS)
LIBRARY_PATHS = $(PROJECT_LIB_PATHS) $(SYS_LIB_PATHS)
CC_FLAGS = $(MCU_CC_FLAGS) -c $(OPTIM_FLAGS) $(CC_DEBUG_FLAGS) -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections
LD_FLAGS = $(MCU_CC_FLAGS) -Wl,--gc-sections $(SYS_LD_FLAGS) -Wl,-Map=$(PROJECT).map -nostartfiles -nostdlib -nodefaultlibs
LD_SYS_LIBS = $(SYS_LIBRARIES)

BL_INCLUDE_PATHS = $(BL_INC_PATHS) $(SYS_INC_PATHS)
BL_LIBRARY_PATHS = $(BL_LIB_PATHS) $(SYS_LIB_PATHS)
BL_CC_FLAGS = $(MCU_CC_FLAGS) -c $(BL_OPTIM_FLAGS) $(BL_CC_DEBUG_FLAGS) -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections
BL_LD_FLAGS = $(MCU_CC_FLAGS) -Wl,--gc-sections $(SYS_LD_FLAGS) -Wl,-Map=$(BL).map -nostartfiles -nostdlib -nodefaultlibs

UAVCAN_CC_FLAGS = $(MCU_CC_FLAGS) -c -O3 -fno-common -fmessage-length=0 -Wall -fno-exceptions -ffunction-sections -fdata-sections -fno-rtti -DUAVCAN_TINY=1 -DUAVCAN_DEBUG=0 -DUAVCAN_GENERAL_PURPOSE_PLATFORM=0 -DUAVCAN_NOEXCEPT=1 -DUAVCAN_TOSTRING=0 -DUAVCAN_USE_EXTERNAL_SNPRINTF=1 -DUAVCAN_NO_ASSERTIONS=1

###############################################################################
# Makefile execution
###############################################################################

all: $(BL).bin $(PROJECT).bin

clean:
	rm -f $(BL).bin $(BL).elf $(BL_OBJECTS) \
		  $(PROJECT).bin $(PROJECT).elf $(PROJECT_OBJECTS) \
		  $(SYS_OBJECTS) $(UAVCAN_OBJECTS)

$(ARCH_BUILD)%.o: $(ARCH_SRC)%.S
	@mkdir -p $(@D)
	$(CC) $(ARCH_CC_FLAGS) $(CC_SYMBOLS) -D__ASSEMBLY__ $(ARCH_INCLUDE_PATHS) -o $@ $<

$(ARCH_BUILD)%.o: $(ARCH_SRC)%.c
	@mkdir -p $(@D)
	$(CC)  $(ARCH_CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(ARCH_INCLUDE_PATHS) -o $@ $<

$(BL_BUILD)%.o: $(BL_SRC)%.c
	@mkdir -p $(@D)
	$(CC)  $(BL_CC_FLAGS) $(BL_CC_SYMBOLS) -std=gnu99   $(BL_INCLUDE_PATHS) -o $@ $<

$(UAVCAN_BUILD)%.o: $(UAVCAN_SRC)%.cpp
	@mkdir -p $(@D)
	$(CPP) $(UAVCAN_CC_FLAGS) $(CC_SYMBOLS) -std=c++03 $(SYS_INC_PATHS) $(UAVCAN_INC_PATHS) -o $@ $<

$(PROJECT_BUILD)%.o: $(PROJECT_SRC)%.S
	@mkdir -p $(@D)
	$(CC) $(CC_FLAGS) $(CC_SYMBOLS) -D__ASSEMBLY__ $(INCLUDE_PATHS) -o $@ $<

$(PROJECT_BUILD)%.o: $(PROJECT_SRC)%.c
	@mkdir -p $(@D)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -o $@ $<

$(PROJECT_BUILD)%.o: $(PROJECT_SRC)%.cpp
	@mkdir -p $(@D)
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=c++11 $(INCLUDE_PATHS) $(UAVCAN_INCLUDE_PATHS) -o $@ $<

$(BL).elf: $(BL_OBJECTS) $(SYS_OBJECTS)
	$(LD) $(BL_LD_FLAGS) -T$(BL_LINKER_SCRIPT) $(BL_LIBRARY_PATHS) -o $@ $^ $(BL_LIBRARIES) $(SYS_LIBRARIES)

$(BL).bin: $(BL).elf
	$(SIZE) $<
	$(OBJCOPY) -O binary $< $@

$(PROJECT).elf: $(PROJECT_OBJECTS) $(SYS_OBJECTS) $(UAVCAN_OBJECTS)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(PROJECT_LIBRARIES) $(SYS_LIBRARIES)

$(PROJECT).bin: $(PROJECT).elf
	$(SIZE) $<
	$(OBJCOPY) -O binary $< $@
