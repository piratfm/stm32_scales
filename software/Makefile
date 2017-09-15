######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = stm32_weightmeter

######################################
# building variables
######################################
# debug build?
DEBUG = 1
DEBUG_TO_UART = 1
# optimization
#OPT = -O0
OPT = -O2

#######################################
# pathes
#######################################
# Build path
BUILD_DIR = build
OPENOCD_PATH=/opt/gnuarmeclipse/openocd/0.10.0-201601101000-dev

######################################
# source
######################################
C_SOURCES = \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c \
  Libraries/STM32F10x_StdPeriph_Driver/src/misc.c \
  Libraries/CMSIS/Core/CM3/system_stm32f10x.c \
  Libraries/CMSIS/Core/CM3/core_cm3.c \
  src/glasslcd_RTC.c \
  src/stm32f10x_it_RTC.c \
  src/SystemConfig_RTC.c \
  src/hx711.c \
  src/main_RTC.c



C_SOURCES += Libraries/CMSIS/Core/CM3/startup/gcc/startup_stm32f10x_md.c

#  Libraries/CMSIS/Core/CM3/startup/gcc/startup_stm32f10x_md.c \
#  Libraries/CMSIS/Core/CM3/startup/gcc/startup_stm32f10x_hd.c \

#ASM_SOURCES = \
#  startup/startup_stm32f103xb.s

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS =
C_DEFS = -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -DSTM32F103xB -Wno-implicit-int -Wno-strict-aliasing
#C_DEFS = -u _printf_float
ifeq ($(DEBUG_TO_UART), 1)
C_DEFS += -DDEBUG_TO_UART=1
endif
# includes for gcc
AS_INCLUDES =
C_INCLUDES = -Isrc
C_INCLUDES += -ILibraries/STM32F10x_StdPeriph_Driver/inc
C_INCLUDES += -ILibraries/CMSIS/Core/CM3
# compile gcc flags
ASFLAGS = -mthumb -mcpu=cortex-m3 $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m3 $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CFLAGS += -std=gnu99 -MD -MP -MF .dep/$(@F).d


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103C8Tx_FLASH.ld
# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = -mthumb -mcpu=cortex-m3 -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

ifeq ($(DEBUG_TO_UART), 1)
LIBS += -lnosys
LDFLAGS += -specs=nosys.specs
else
LIBS += -lrdimon
LDFLAGS += -specs=rdimon.specs
endif

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
  

erase:
	$(OPENOCD_PATH)/bin/openocd -s $(OPENOCD_PATH) \
		-c "source [find interface/stlink-v2.cfg]" -c "set WORKAREASIZE 0x1000" -c "transport select hla_swd" \
		-c "source [find target/stm32f1x.cfg]" -c "reset_config srst_only" \
		-c "init" -c "stm32f1x mass_erase 0" -c "reset" -c "exit"

flash:
	$(OPENOCD_PATH)/bin/openocd -s $(OPENOCD_PATH) \
		-c "source [find interface/stlink-v2.cfg]" -c "set WORKAREASIZE 0x1000" \
		-c "source [find target/stm32f1x.cfg]" -c "reset_config srst_only" \
		-c "program build/$(TARGET).bin 0x08000000 reset verify exit"

#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

.PHONY: clean all

# *** EOF ***
