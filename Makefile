all:
	@echo "usage: make build     -- compile:"
	@echo "       make isp       -- test ISP configuration"
	@echo "       make fuse      -- set target fuses"
	@echo "       make flash     -- load .HEX into target"
	@echo "       make shh       -- connect to ISP host"
	@echo "       make monitor   -- monitor log messages"


# Linux board connected to target AVR ISP (Raspberry Pi)...
HOST      = pi@raspberrypi.local
#HOST      = pi@192.168.0.173
HOST_KEY  = ~/git/jlpi/docker-raspbian/id_pi.pem
HOST_SSH  = ssh -i $(HOST_KEY) $(HOST)

AVRDUDE   = sudo avrdude -p $(DEVICE) -C+avrdude.conf -v
#AVRDUDE   = $(HOST_SSH) sudo avrdude -p $(DEVICE) -C+avrdude.conf-c gpio1 -v


# AVR Target device...
#DEVICE    = atmega32
DEVICE    = atmega328p
#DEVICE    = atmega32
#DEVICE    = atmega2560
CLOCK     = 16000000
FUSES     = -U hfuse:w:0xdf:m -U lfuse:w:0xff:m # 328p 16Mhz Crystal
#FUSES     = -U hfuse:w:0xc9:m -U lfuse:w:0xff:m # 16Mhz Crystal
#FUSES    = -U hfuse:w:0xd9:m -U lfuse:w:0xe1:m # 1MHz RC


# C Compiler...
include gcc.conf
CFLAGS    = -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) $(GCCFLAGS)


# Target source code...
SOURCE    = src/main.c

# Compilation...
build: main.hex

.PHONY:main.elf
main.elf:
	rm -f main.elf main.S main.hex main.dump
	avr-gcc $(CFLAGS) -o main.elf $(SOURCE)
	avr-gcc $(CFLAGS) -S --verbose-asm -o main.S $(SOURCE)
	avr-objdump -x -S -d -r -t -h main.elf > main.dump

main.hex: main.elf
	avr-objcopy -j .text -j .data -O ihex $< $@

unit_test: src/main.c
	gcc -D UNIT_TEST -Wall $< -o $@
	./$@

# AVRDUDE ISP...

#flash: main.hex
flash:
	$(AVRDUDE) -U flash:w:main.hex:i -c gpio1

fuse:
	$(AVRDUDE) -v $(FUSES) -c gpio1


# Debug Monitor...
.PHONY:monitor
monitor:
	rm -f monitor
	$(HOST_SSH) "gcc -o /mnt/monitor /mnt/monitor.c -l pigpio; \
                 sudo killall -q monitor || true; \
                 sudo /mnt/monitor"


# Misc...
clean:
	rm -f main.hex main.elf main.dump main.S

ssh:
	$(HOST_SSH)

isp1:
	$(AVRDUDE) -c gpio1

isp2:
	$(AVRDUDE) -c gpio2
