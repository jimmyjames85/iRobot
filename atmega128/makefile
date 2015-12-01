############## UNO ##################
MMCU=atmega328p
PARTNO=m328p
PROGRAMMER=arduino
F_CPU=16000000UL
PORT=/dev/ttyACM0
PORT_OPTIONS=-P $(PORT)

############## Atmega 128 ##################
MMCU=atmega128
PARTNO=m128
PROGRAMMER=usbasp
F_CPU=8000000UL
PORT_OPTIONS=

############################################################################

CC=avr-gcc
OBJCOPY=avr-objcopy
COMPILE_OPTIONS=-Os -DF_CPU=$(F_CPU) -D___$(MMCU)=1 -mmcu=$(MMCU)  

############################################################################
.PHONY: link objCopy upload clean freshUpload all make

OBJECTS= main.o agent.o llist.o adc.o open_interface.o usart.o servo.o movement.o util.o list.o timer.o ping.o ir_sensor.o cmd_interpreter.o

all: link

make:
	make clean; clear; make all

%.o: %.c 
	$(CC) $(COMPILE_OPTIONS) -c -o $@ $<

link: $(OBJECTS)
	$(CC) -mmcu=$(MMCU) $(OBJECTS) -o main.a

objCopy: link
	$(OBJCOPY) -O ihex -R .eeprom main.a main.hex

upload: objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -B 0.3 -U flash:w:main.hex
#-b 57600
##	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -P $(PORT) -U flash:w:blink.hex

freshUpload: clean upload

clean:
	rm -f *.o *.a *.hex 
