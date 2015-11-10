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
.PHONY: adc adc_test ping_test_link ping_test_objCopy ping_test_upload clock movement bluetooth open_interface atmega128 main timer link util objCopy upload all clean usart freshUpload ping

all: iRobot_link

adc: adc.c
	$(CC) $(COMPILE_OPTIONS) -c adc.c -o adc.o

################### iRobot real

iRobot: iRobot.c
	$(CC) $(COMPILE_OPTIONS) -c iRobot.c -o iRobot.o

iRobot_link: iRobot ir_sensor usart ping servo util timer adc list open_interface bluetooth
	$(CC) -mmcu=$(MMCU) open_interface.o usart.o ping.o servo.o iRobot.o adc.o list.o bluetooth.o ir_sensor.o timer.o util.o -o iRobot.a

iRobot_objCopy: iRobot_link
	$(OBJCOPY) -O ihex -R .eeprom iRobot.a iRobot.hex

iRobot_upload: iRobot_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:iRobot.hex


################### pwm test

pwm_test: pwm_test.c
	$(CC) $(COMPILE_OPTIONS) -c pwm_test.c -o pwm_test.o

pwm_test_link: pwm_test usart util timer servo
	$(CC) -mmcu=$(MMCU)  usart.o servo.o pwm_test.o timer.o util.o -o pwm_test.a

pwm_test_objCopy: pwm_test_link
	$(OBJCOPY) -O ihex -R .eeprom pwm_test.a pwm_test.hex

pwm_test_upload: pwm_test_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:pwm_test.hex

################### adc test

adc_test: adc_test.c
	$(CC) $(COMPILE_OPTIONS) -c adc_test.c -o adc_test.o

adc_test_link: adc adc_test usart util timer list
	$(CC) -mmcu=$(MMCU)  usart.o adc.o adc_test.o list.o timer.o util.o -o adc_test.a

adc_test_objCopy: adc_test_link
	$(OBJCOPY) -O ihex -R .eeprom adc_test.a adc_test.hex

adc_test_upload: adc_test_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:adc_test.hex

################### adc test

usart: usart.c
	$(CC) $(COMPILE_OPTIONS) -c usart.c -o usart.o

clock: clock.c
	$(CC) $(COMPILE_OPTIONS) -c clock.c -o clock.o

bluetooth: blue_tooth_HC05.c
	$(CC) $(COMPILE_OPTIONS) -c blue_tooth_HC05.c -o bluetooth.o

movement: movement.c
	$(CC) $(COMPILE_OPTIONS) -c movement.c -o movement.o

servo: servo.c
	$(CC) $(COMPILE_OPTIONS) -c servo.c -o servo.o

hw: hw.c
	$(CC) $(COMPILE_OPTIONS) -c hw.c -o hw.o

main: main.c
	$(CC) $(COMPILE_OPTIONS) -c main.c -o main.o

open_interface: open_interface.c
	$(CC) $(COMPILE_OPTIONS) -c open_interface.c -o open_interface.o

atmega128: atmega128.c
	$(CC) $(COMPILE_OPTIONS) -c atmega128.c -o atmega128.o

link: main atmega128 usart bluetooth open_interface hw movement clock util ping timer
	$(CC) -mmcu=$(MMCU) main.o hw.o oi.o bluetooth.o usart.o movement.o atmega128.o ping.o timer.o util.o clock.o -o main.a

util: util.c
	$(CC) $(COMPILE_OPTIONS) -c util.c -o util.o

list: list.c
	$(CC) $(COMPILE_OPTIONS) -c list.c -o list.o

timer: timer.c
	$(CC) $(COMPILE_OPTIONS) -c timer.c -o timer.o

ping: ping.c
	$(CC) $(COMPILE_OPTIONS) -c ping.c -o ping.o

ir_sensor: ir_sensor.c
	$(CC) $(COMPILE_OPTIONS) -c ir_sensor.c -o ir_sensor.o

ping_test: ping_test.c
	$(CC) $(COMPILE_OPTIONS) -c ping_test.c -o ping_test.o

ping_test_link: ping_test usart timer util
	$(CC) -mmcu=$(MMCU)  usart.o ping_test.o util.o timer.o -o ping_test.a

ping_test_objCopy: ping_test_link
	$(OBJCOPY) -O ihex -R .eeprom ping_test.a ping_test.hex

ping_test_upload: ping_test_objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) $(PORT_OPTIONS) -U flash:w:ping_test.hex

objCopy: link
	$(OBJCOPY) -O ihex -R .eeprom main.a main.hex

upload: objCopy
	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -U flash:w:main.hex
##	avrdude -v -p $(PARTNO) -c $(PROGRAMMER) -P $(PORT) -U flash:w:blink.hex

freshUpload: clean upload

clean:
	rm *.o *.a *.hex 
