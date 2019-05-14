
ARMGNU = arm-none-eabi

CPU 		?= cortex-m0
OPT 		?= -Os
M0ROM_ADDR 	?= 0x10000
TARGET_ARCH ?= -mcpu=$(CPU) -mthumb
TARGET		?= firmware

CFLAGS  = -Wall -g $(OPT) $(TARGET_ARCH) -ffreestanding  -D_MSP430_M0_
LDFLAGS = -nodefaultlibs -nostartfiles

.PHONY: all clean clobber install

all : $(TARGET).hex
	$(ARMGNU)-size $(TARGET).elf

vectors.o : vectors.s
	$(ARMGNU)-as vectors.s -o vectors.o

main.o : main.c 
	$(ARMGNU)-gcc $(CFLAGS) $(TARGET_ARCH) -c $< -o $@

$(TARGET).elf : vectors.o main.o msp430m0.ld
	$(ARMGNU)-gcc $(LDFLAGS) -T msp430m0.ld vectors.o main.o -o $(TARGET).elf

$(TARGET).hex : $(TARGET).elf
	$(ARMGNU)-objdump -CS $< > $(TARGET).list
	$(ARMGNU)-objcopy --change-addresses=$(M0ROM_ADDR) -O ihex $< $@

clean:
	rm -f *.bin
	rm -f *.bc
	rm -f *.o
	rm -f *.elf
	rm -f *.list

clobber: clean
	rm -f *.hex

install:
	export M0_FIRMWARE=blinky/$(TARGET).hex; \
	make -e -C .. m0install

# vim: set ts=4 sw=4 noexpandtab :
