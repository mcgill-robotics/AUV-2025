# Project name.
PROJECT = hydrophones

# Find all source files.
SRCS = $(shell find src drivers -name '*.c')
OBJS = $(SRCS:.c=.o)

# Compiler flags.
CFLAGS  = -mcpu=cortex-m4 -march=armv7e-m -mthumb -g
CFLAGS += -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -std=c99 -O2
CFLAGS += -ffunction-sections -fdata-sections -T "drivers/linker.ld" -Xlinker
CFLAGS += --gc-sections -Wl,-Map,$(PROJECT).map --specs=rdimon.specs
CFLAGS += -lc -lrdimon -I"include" -I"drivers" -I"drivers/CMSIS/core"
CFLAGS += -I"drivers/CMSIS/device" -I"drivers/HAL/include"
CFLAGS += -D STM32F303xE -D NUCLEO_F303RE -D USE_HAL_DRIVER -D STM32F303RETx
CFLAGS += -D ARM_MATH_CM4 -D __FPU_PRESENT=1

# Add debug flag.
ifeq (${DEBUG}, 1)
    CFLAGS += -D DEBUG
endif

# Add twelve bit mode flag.
ifeq (${TWELVE_BIT_MODE}, 1)
    CFLAGS += -D TWELVE_BIT_MODE
endif

# Add serial flag, default is USART2.
SERIAL?=2
CFLAGS += -D SERIAL=USART${SERIAL}

FILE_BUILD = -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c
STARTUP = drivers/startup_stm32f303xe.s

%.o: %.c
	@echo 'Building file: $<'
	arm-none-eabi-gcc $(CFLAGS) $(FILE_BUILD) -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

all: $(PROJECT).bin

$(PROJECT).bin: $(OBJS)
	@echo 'Building $(PROJECT) project...'
	arm-none-eabi-gcc $(CFLAGS) -o $(PROJECT).elf $(OBJS) $(STARTUP)
	@echo ' '
	@echo 'Generating binary...'
	arm-none-eabi-objcopy -O binary $(PROJECT).elf $(PROJECT).bin
	arm-none-eabi-size $(PROJECT).elf

clean:
	@echo 'Cleaning up user space...'
	find src -name '*~' | xargs rm -f
	find src -name '*.o' | xargs rm -f
	find src -name '*.d' | xargs rm -f
	find . -name '*.elf' | xargs rm -f
	find . -name '*.bin' | xargs rm -f
	find . -name '*.map' | xargs rm -f

wipe: clean
	@echo 'Cleaning up libraries...'
	find . -name '*~' | xargs rm -f
	find . -name '*.o' | xargs rm -f
	find . -name '*.d' | xargs rm -f

install: $(PROJECT).bin
	@echo 'Flashing device...'
	st-flash --reset write $(PROJECT).bin 0x8000000
