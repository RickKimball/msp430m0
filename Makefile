TARGET 		?= msp430m0
OPT 		?= -O2
TARGET_ARCH	?= -mmcu=msp430fr6989 -mlarge
M0_FIRMWARE ?= blinky/firmware.hex

CFLAGS	= -Wall -g $(OPT) -Wno-misleading-indentation
LDFLAGS = -L. -T./msp430fr6989.ld -Wl,--gc-sections -Wl,-Map=$(TARGET).map

.PHONY: all clean mspdebug install m0install

all:	$(TARGET).elf
	@msp430-elf-nm -S -n $< 2>/dev/null | grep ' ram'
	@msp430-elf-nm -S -n $< 2>/dev/null | grep ' rom'
	msp430-elf-size $<

$(TARGET).elf: $(TARGET).c msp430fr6989.ld
	msp430-elf-gcc $(CFLAGS) $(LDFLAGS) $(TARGET_ARCH) -Wl,--gc-sections $< -o $@
	msp430-elf-objdump -DCS $@ > $(TARGET).list

clean :
	rm -f *.o *.elf *.map *.list

mspdebug:
	xterm -e mspdebug tilib 'opt gdb_loop' 'gdb' &
	
install: mspdebug
	msp430-elf-gdb -ex 'target remote :2000' \
		 -ex 'load' \
		 -ex 'restore $(M0_FIRMWARE)' \
		 -ex 'mon reset run' \
		 -ex 'quit' \
		 $(TARGET).elf

m0install: mspdebug
	msp430-elf-gdb -ex 'target remote :2000' \
		 -ex 'restore $(M0_FIRMWARE)' \
		 -ex 'mon reset run' \
		 -ex 'quit' \
		 $(TARGET).elf

debug: mspdebug
	msp430-elf-gdb -ex 'target remote :2000' \
		 -ex 'mon reset' \
		 $(TARGET).elf

# vim: set ts=4 sw=4 noexpandtab :

