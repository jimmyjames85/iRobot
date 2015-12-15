############## UNO ##################
MMCU=atmega328p
PARTNO=m328p
PROGRAMMER=arduino
F_CPU=16000000UL
PORT=/dev/ttyACM0
PORT_OPTIONS=-P $(PORT)

############################################################################

CC=avr-gcc
OBJCOPY=avr-objcopy
COMPILE_OPTIONS=-Os -DF_CPU=$(F_CPU) -D___$(MMCU)=1 -mmcu=$(MMCU)  

############################################################################
.PHONY: link objCopy upload clean freshUpload make 

#OBJECTS= adc.o open_interface.o usart.o servo.o movement.o util.o list.o timer.o ping.o ir_sensor.o 
OBJECTS= usart.o util.o list.o timer.o main.o asm.o

make:
	make clean; clear; make all

all: link

%.o: %.c 
	$(CC) $(COMPILE_OPTIONS) -c -o $@ $<

%.o: %.s 
	$(CC) $(COMPILE_OPTIONS) -c -o $@ $<

link: $(OBJECTS)
	$(CC) -mmcu=$(MMCU) $(OBJECTS) -o main.a

objCopy: link
	$(OBJCOPY) -O ihex -R .eeprom main.a main.hex

upload: objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:main.hex
#	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -U flash:w:main.hex


freshUpload: clean upload

clean:
	rm -f *.o *.a *.hex 
