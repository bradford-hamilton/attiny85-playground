# Compile for attiny85 - optimize for size
compile:
	avr-gcc -Wall -Os -mmcu=attiny85 src/main.c

# Get the assembly
assembly:
	avr-gcc -S -Os -mmcu=attiny85 src/main.c -o assembly.asm

# Dump hex from out file
dump: compile
	hexdump a.out

# Create Intel hex from out file
hex: compile
	avr-objcopy -O ihex -j .text -j .data a.out a.hex

# Flash ATtiny85 with hex through usbtiny
flash: hex
	avrdude -C /opt/homebrew/etc/avrdude.conf -v -p t85 -c usbtiny -U flash:w:a.hex:i
