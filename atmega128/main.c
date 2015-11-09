#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "atmega128.h"
#include "usart/usart.h"
#include "open_interface.h"
#include "blue_tooth_HC05.h"
#include "movement.h"
#include "util.h"
#include "ping.h"
#define ENCODER_MULTIPLIER (4/9)


float distance_traveled(oi_encoder_t * encoder)
{
	//TODO include revolutions
	float distance = 0.444564997642 * encoder->encoder_count;
	return distance;
}
void commandCenterLoop_main()
{
	oi_t sensor;
	//oi_encoder_t right_encoder = { 0 };
	//oi_encoder_t left_encoder =	{ 0 };

	oi_tare_encoders(&(sensor.left_encoder), &(sensor.right_encoder));

	set_onboard_led(1, 1);
	set_onboard_led(0, 0);
	_delay_ms(333);
	set_onboard_led(1, 0);
	set_onboard_led(0, 1);

	bprintf("Hello world! ! !\r\n");

	int16_t velocity = 0;
	int16_t radius = 0;
	int16_t leftWheelVelocity = 0;
	int16_t rightWheelVelocity = 0;

	while (1)
	{

		char c;
		while (oi_isAvailable())
		{
			bsendChar(oi_getChar());
		}

		if (bt_isAvailable())
		{
			c = bgetChar();
			bprintf("\r\n(0x%02X) You pressed: %c\r\n", c, c);

			if (c == 'B')
			{
				bprintf("Changing OI Baud Rate to 19200\r\n");
				oi_switch_baud_rate();
				//init_USART1(OI_ALTERNATE_BAUD_RATE, F_CPU);
			}
			else if (c == 'I')
			{
				bprintf("Initiating OI..\r\n");
				oi_init();
			}
			else if (c == '~')
			{
				bprintf("Reseting..\r\n");
				oi_reset();
				//init_USART1(OI_DEFAULT_BAUD_RATE, F_CPU);
			}
			else if (c == 'l')
			{
				bprintf("Led Test\r\n");
				oi_set_leds(1, 1, 1, 1, 0xFF, 0xFF);
			}
			else if (c == 'L')
			{
				bprintf("Stop Led Test\r\n");
				oi_set_leds(0, 0, 0, 0, 0x00, 0x00);
			}
			else if (c == 'F')
			{
				bprintf("Full Mode\r\n");
				oi_full_mode();
			}
			else if (c == 'e')
			{
				leftWheelVelocity += 50;
				oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
			}
			else if (c == 'd')
			{
				leftWheelVelocity -= 50;
				oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
			}
			else if (c == 'r')
			{
				rightWheelVelocity += 50;
				oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
			}
			else if (c == 'f')
			{
				rightWheelVelocity -= 50;
				oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
			}
			else if (c == 't')
			{
				turn_CW(&sensor, -90);
			}
			else if (c == ' ')
			{
				radius = 0;
				velocity = 0;
				rightWheelVelocity = 0;
				leftWheelVelocity = 0;
				oi_set_wheels(0, 0);
			}
			else if (c == 'H')
			{
				bprintf("Shutting down...\r\n\r\n");
				oi_seek_dock();
			}
			else if (c == 'S')
			{
				bprintf("Loading Sensor data...\r\n\r\n");
				oi_load_sensor_data(&sensor);
				bprintf("distance: 0x%04X\r\n", sensor.distance);
				bprintf("angle: ox%04X\r\n", sensor.angle);
				bprintf("left requested velocity: 0x%04X\r\n", sensor.left_requested_velocity);
				bprintf("right requested velocity: 0x%04X\r\n", sensor.right_requested_velocity);
				bprintf("Voltage: %d mV\r\n", sensor.voltage);
				bprintf("\r\n\r\n");

				bprintf("Left Encoder Count Raw: %d \r\n", sensor.left_encoder.encoder_count_raw);
				bprintf("Right Encoder Count Raw: %d \r\n", sensor.right_encoder.encoder_count_raw);

				bprintf("\r\n\r\n");
				bprintf("Left Encoder Count: %d\t  Left Encoder Rev: %d\r\n", sensor.left_encoder.encoder_count, sensor.left_encoder.encoder_revolution_count);
				bprintf("Right Encoder Count: %d\t Right Encoder Rev: %d\r\n", sensor.right_encoder.encoder_count, sensor.right_encoder.encoder_revolution_count);
				bprintf("\r\n\r\n");

				char strBuffer[30];
				ftoa(strBuffer, distance_traveled(&(sensor.left_encoder)));
				bprintf("Left Distance Traveled: %s\r\n", strBuffer);

				ftoa(strBuffer, distance_traveled(&(sensor.right_encoder)));
				bprintf("Right Distance Traveled: %s\r\n", strBuffer);
				bprintf("\r\n\r\n");
			}
			else if (c == 'T')
			{
				bprintf("Taring...\r\n");
				oi_tare_encoders(&(sensor.left_encoder), &(sensor.right_encoder));
			}
			else if (c == '?')
			{
				uint16_t x = -1;
				int16_t y = -1;

				bprintf("x = 0x%04X\ty = 0x%04X \r\n", x, y);
				bprintf("x = %d\ty = %d\r\n", x, y);
				/*
				 uint16_t buffer_size = 2000;
				 oi_switch_baud_rate();
				 oi_reset();
				 oi_switch_baud_rate();

				 char iRobot_stats[buffer_size];
				 while(!oi_isAvailable());//wait till we are ready to receive data
				 int i=0;
				 while(oi_isAvailable() && i<buffer_size)
				 iRobot_stats[i++]=oi_getChar();

				 iRobot_stats[i]='\0';
				 bprintf("STATS: %s\r\n",iRobot_stats);*/

			}
			else if (c >= '1' && c <= '9')
			{
				int dist_requested_cm = (c - '0') * 10;
				bprintf("Traveling Foward %d cm...\r\n", dist_requested_cm);
				move_forward_cm(&sensor, dist_requested_cm);
			}
			/*			else if (c == 'C')	//Class hw lab ..etc
			 {
			 lab4_part1_clock_main();
			 }
			 else if (c == 'c')	//Class hw lab ..etc
			 {
			 lab4_part2_clock_main();
			 }*/
		}
	}
}

void setup(void)
{
	setup_onboard_leds();

	//TODO define these pins in main for usar/usart.h
	//RXD0/(PDI) PE0
	//(TXD0/PDO) PE1
	//(RXD1/INT2)PD2
	//(TXD1/INT3)PD3
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);	//TODO integrate pins (usart0 usart1 usart2 etc) into blue_tooth.h and open_interface.h
	init_USART1(OI_ALTERNATE_BAUD_RATE, F_CPU);

	_delay_ms(333);
	oi_switch_baud_rate();
	oi_init();
	oi_full_mode();

	int i;
	int val;
	for (i = 0; i < 10; i++)
	{
		val = i % 2;
		oi_set_leds(val, val, val, val, 0xFF * val, 0xFF);
		set_onboard_led(0, val);
		set_onboard_led(1, val);
		_delay_ms(50);
	}
	oi_init();
}

char max_0s(unsigned long x)
{
	int i;
	char cur_count = 0;
	char max_count = 0;
	char length = sizeof(unsigned long) * 8; //32 bits
	for (i = 0; i < length; i++)
	{
		if (x & 0x01)
		{
			cur_count = 0;
		}
		else
		{
			cur_count++;
			if (cur_count > max_count)
				max_count = cur_count;
		}

		x = x >> 1;
	}
	return max_count;
}

int main(void)
{

	uint8_t i = 0;

	setup();

	initPing();
	doPingLoop();

	unsigned long x = 0xFF00ABCD;
	char count = max_0s(x);
	bprintf("#of consecutive_0s in 0x%lX is %d\r\n", x , count);

	x = 0xF0FC801F;
	count = max_0s(x);
	bprintf("#of consecutive_0s in 0x%lX is %d\r\n", x , count);

	x = 0xF00FFE08;
	count = max_0s(x);
	bprintf("#of consecutive_0s in 0x%0lX is %d\r\n", x , count);

	commandCenterLoop_main();
	while (1)
	{
		set_onboard_led(1, 1);
		set_onboard_led(0, 0);
		_delay_ms(1000);
		set_onboard_led(1, 0);
		set_onboard_led(0, 1);
		_delay_ms(1000);
	}

	return 0; /* CODE SHOULD NEVER REACH HERE */
}
