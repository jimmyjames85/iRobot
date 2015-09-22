MMCU=atmega128
PARTNO=m128
PROGRAMMER=usbasp
F_CPU=1000000UL
##PORT=/dev/ttyACM0


.PHONY: compile link objCopy upload all clean usart

all: objCopy

usart: usart/usart.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c usart/usart.c -o usart.o

compile: blink.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c blink.c -o blink.o

link: compile usart
	avr-gcc -mmcu=$(MMCU) usart.o blink.o -o blink.a

objCopy: link
	avr-objcopy -O ihex -R .eeprom blink.a blink.hex

upload: objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -U flash:w:blink.hex
##	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -P $(PORT) -U flash:w:blink.hex


clean:
	rm blink.o blink.a blink.hex usart.o usart.a usart.hex
