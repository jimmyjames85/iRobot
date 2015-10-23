############## UNO ##################
MMCU=atmega328p
PARTNO=m328p
PROGRAMMER=arduino
F_CPU=16000000UL
PORT=/dev/ttyACM0
PORT_OPTIONS=-P $(PORT)
COMPILE_OPTIONS=-Os -DF_CPU=$(F_CPU) -D___$(MMCU)=1 -mmcu=$(MMCU)

############## Atmega 128 ##################
#MMCU=atmega128
#PARTNO=m128
#PROGRAMMER=usbasp
#F_CPU=8000000UL
#PORT_OPTIONS=

#F_CPU=1000000UL
#F_CPU=1843200UL



.PHONY: adc adc_test ping_test_link ping_test_objCopy ping_test_upload clock movement bluetooth open_interface atmega128 main timer link util objCopy upload all clean usart freshUpload ping

all: objCopy

adc: adc.c
	avr-gcc $(COMPILE_OPTIONS) -c adc.c -o adc.o

################### pwm test

pwm_test: pwm_test.c
	avr-gcc $(COMPILE_OPTIONS) -c pwm_test.c -o pwm_test.o

pwm_test_link: pwm_test usart util timer
	avr-gcc -mmcu=$(MMCU)  usart.o pwm_test.o timer.o util.o -o pwm_test.a

pwm_test_objCopy: pwm_test_link
	avr-objcopy -O ihex -R .eeprom pwm_test.a pwm_test.hex

pwm_test_upload: pwm_test_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:pwm_test.hex

################### adc test

adc_test: adc_test.c
	avr-gcc $(COMPILE_OPTIONS) -c adc_test.c -o adc_test.o

adc_test_link: adc adc_test usart util timer
	avr-gcc -mmcu=$(MMCU)  usart.o adc.o adc_test.o timer.o util.o -o adc_test.a

adc_test_objCopy: adc_test_link
	avr-objcopy -O ihex -R .eeprom adc_test.a adc_test.hex

adc_test_upload: adc_test_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:adc_test.hex

################### adc test
	
usart: usart/usart.c
	avr-gcc $(COMPILE_OPTIONS) -c usart/usart.c -o usart.o

clock: clock.c
	avr-gcc $(COMPILE_OPTIONS) -c clock.c -o clock.o

bluetooth: blue_tooth_HC05.c
	avr-gcc $(COMPILE_OPTIONS) -c blue_tooth_HC05.c -o bt.o

movement: movement.c
	avr-gcc $(COMPILE_OPTIONS) -c movement.c -o movement.o

hw: hw.c
	avr-gcc $(COMPILE_OPTIONS) -c hw.c -o hw.o

main: main.c
	avr-gcc $(COMPILE_OPTIONS) -c main.c -o main.o

open_interface: open_interface.c
	avr-gcc $(COMPILE_OPTIONS) -c open_interface.c -o oi.o

atmega128: atmega128.c
	avr-gcc $(COMPILE_OPTIONS) -c atmega128.c -o atmega128.o

link: main atmega128 usart bluetooth open_interface hw movement clock util ping timer
	avr-gcc -mmcu=$(MMCU) main.o hw.o oi.o bt.o usart.o movement.o atmega128.o ping.o timer.o util.o clock.o -o main.a

util: util.c
	avr-gcc $(COMPILE_OPTIONS) -c util.c -o util.o

timer: timer.c
	avr-gcc $(COMPILE_OPTIONS) -c timer.c -o timer.o

ping: ping.c
	avr-gcc $(COMPILE_OPTIONS) -c ping.c -o ping.o

ping_test: ping_test.c
	avr-gcc $(COMPILE_OPTIONS) -c ping_test.c -o ping_test.o

ping_test_link: ping_test usart timer util
	avr-gcc -mmcu=$(MMCU)  usart.o ping_test.o util.o timer.o -o ping_test.a

ping_test_objCopy: ping_test_link
	avr-objcopy -O ihex -R .eeprom ping_test.a ping_test.hex

ping_test_upload: ping_test_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:ping_test.hex

	
objCopy: link
	avr-objcopy -O ihex -R .eeprom main.a main.hex

upload: objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -U flash:w:main.hex
##	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -P $(PORT) -U flash:w:blink.hex

freshUpload: clean upload

clean:
	rm *.o *.a *.hex 
