# Set to 1 to use first USB port as a Mecrisp-Stellaris Forth terminal
# and leave USART1 for free use within Forth (serial-emit etc. words)
# TODO: do not clone mecrisp when not set
FORTH ?= 1

# Select board - blue pill with stm32f103x8
BOARD ?= bluepill

# maplemini is stm32f103xb with PB9 initialization to enable USB
# BOARD ?= maplemini

# MCU ?= stm32f103xb

# Enable e.g. for bluepill board, no usb reset pin here
# MCU ?= stm32f103x8

# TODO: f4 support
# MCU ?= stm32f405x6

# When set, use this timer to flush forth usb-emit characters
# utilize usb packet buffer up to it's size.
# When not set, usb packet is send as soon as possible
USE_TIM4 ?= 1

ENABLE_DEBUG ?=
ENABLE_TRACE ?=


OPT_FLAGS ?= -Os

LD_SCRIPTS_DIR = ../libopencm3/lib/stm32
FORTH_SRC_DIR = ../mecrisp-stellaris/mecrisp-stellaris-source

ifneq ($(V), 1)
MAKEFLAGS += --no-print-dir
Q := @
endif


ifeq ($(BOARD), bluepill)
MCU = stm32f103x8
NAME_PF = bp
endif

ifeq ($(BOARD), maplemini)
MCU = stm32f103xb
CFLAGS += -DUSB_MAPLEMINI
NAME_PF = mm
endif


CFLAGS += -Wall -Wextra -Werror -Wno-char-subscripts \
	$(OPT_FLAGS) -std=gnu99 -g3 -MD -I.
LDFLAGS += $(OPT_FLAGS)

ifeq ($(ENABLE_DEBUG), 1)
CFLAGS += -DENABLE_DEBUG
endif

ifeq ($(ENABLE_TRACE), 1)
CFLAGS += -DENABLE_TRACE
endif


SRC =			\
	main.c		\
	platform.c	\
	cdcacm.c	\
	usbuart.c


ifeq ($(FORTH), 1)
NAME = mecrisp_pillserial_$(NAME_PF)
CFLAGS += -DENABLE_FORTH
SRC += $(NAME).s
else
NAME = pillserial_$(NAME_PF)
endif

ifeq ($(USE_TIM4), 1)
CFLAGS += -DUSB_IN_TIM4
endif


CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size


CFLAGS += -mthumb -DDISCOVERY_STLINK -I../libopencm3/include -I .
LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -Wl,-gc-sections -L../libopencm3/lib


ifneq (,$(findstring stm32f1,$(MCU)))
FORTH_DIR     = $(FORTH_SRC_DIR)/stm32f103-ra
CFLAGS       += -mcpu=cortex-m3 -DSTM32F1
LDFLAGS_BOOT += -lopencm3_stm32f1 -mcpu=cortex-m3 \
		-Wl,-T,$(LD_SCRIPTS_DIR)/f1/$(MCU).ld

else ifneq (,$(findstring stm32f4,$(MCU)))
FORTH_DIR     = $(FORTH_SRC_DIR)/stm32f407-ra
CFLAGS       += -mcpu=cortex-m4 -DSTM32F4
LDFLAGS_BOOT += -lopencm3_stm32f4 -mcpu=cortex-m4 \
		-Wl,-T,$(LD_SCRIPTS_DIR)/f4/$(MCU).ld

endif

LDFLAGS = $(LDFLAGS_BOOT) -I $(FORTH_DIR)



ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

all:	$(NAME).bin $(NAME).lss $(NAME).def
	$(Q)$(SIZE) $(NAME).elf 

flash:	${NAME}.bin
	@echo "  FLASH   ${NAME}.bin"
	st-flash erase
	st-flash --reset write ${NAME}.bin 0x8000000

host_clean:
	-$(Q)$(RM) ${NAME}.bin

OBJ = $(SRC:.c=.o)

${NAME}.elf: version.h $(OBJ)
	@echo "  LD      $@"
	$(Q)$(CC) -o $@ $(OBJ) $(LDFLAGS) \
	-Xlinker -Map=${NAME}.map

%.def:  
	@echo "  DEF     $@"
	@echo $(Q)$(CC) -dM -E $(LDFLAGS) > $@


%.o:	%.c
	@echo "  CC      $<"
	$(Q)$(CC) $(CFLAGS) -c $< -o $@

%.bin:	%.elf
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O binary $^ $@
	cp $@ ../bin

%.hex:	%
	@echo "  OBJCOPY $@"
	$(Q)$(OBJCOPY) -O ihex $^ $@

%.lss:	%.elf
	@echo "  OBJDUMP $@"
	$(Q)$(OBJDUMP) -Std $^ > $@

.PHONY:	clean host_clean FORCE

clean:	host_clean
	$(Q)echo "  CLEAN"
	-$(Q)$(RM) -f *.o *.d *~ *.elf *.bin *.lss *.def *.map $(HOSTFILES)
	-$(Q)$(RM) -f mapfile version.h

version.h: FORCE
	$(Q)echo "  GIT     version.h"
	$(Q)echo "#define FIRMWARE_VERSION \"`git describe --always --dirty`\"" > $@

-include *.d
