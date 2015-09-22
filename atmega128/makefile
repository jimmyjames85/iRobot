MMCU=atmega128
PARTNO=m128
PROGRAMMER=usbasp
#F_CPU=16000000UL
F_CPU=8000000UL
#F_CPU=1000000UL
#F_CPU=1843200UL
##PORT=/dev/ttyACM0


.PHONY: clock movement bluetooth open_interface atmega128 main link objCopy upload all clean usart freshUpload

all: objCopy

usart: usart/usart.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c usart/usart.c -o usart.o

clock: clock.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c clock.c -o clock.o

bluetooth: blue_tooth_HC05.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c blue_tooth_HC05.c -o bt.o

movement: movement.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c movement.c -o movement.o

hw: hw.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c hw.c -o hw.o

main: main.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c main.c -o main.o

open_interface: open_interface.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c open_interface.c -o oi.o

atmega128: atmega128.c
	avr-gcc -Os -DF_CPU=$(F_CPU) -mmcu=$(MMCU) -c atmega128.c -o atmega128.o

link: main atmega128 usart bluetooth open_interface hw movement clock
	avr-gcc -mmcu=$(MMCU) main.o hw.o oi.o bt.o usart.o movement.o atmega128.o clock.o -o main.a

objCopy: link
	avr-objcopy -O ihex -R .eeprom main.a main.hex

upload: objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -U flash:w:main.hex
##	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -P $(PORT) -U flash:w:blink.hex

freshUpload: clean upload

clean:
	rm *.o *.a *.hex 
