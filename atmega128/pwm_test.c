/*
 * pwm_test.c
 *
 *  Created on: Oct 22, 2015
 *      Author: jim
 */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart.h"
#include <stdlib.h>
#include "timer.h"
#include "util.h"
#include "blue_tooth_HC05.h"
#include "servo.h"
volatile char compa;
volatile char compb;
ISR(TIMER1_COMPA_vect)
{
	compa = 1;
}

ISR(TIMER1_COMPB_vect)
{
	compb = 1;
}

/*
 * When changing the TOP value the program must ensure that the new TOP value is higher or
 * equal to the value of all of the Compare Registers. If the TOP value is lower than any of the
 * Compare Registers, a compare match will never occur between the TCNT1 and the OCR1x.
 * Note that when using fixed TOP values the unused bits are masked to zero when any of the
 * OCR1x Registers are written
 */

void timer1_mode_test()
{

	unsigned int i;
	int j;
	for (i = 0; i < 20; i++)
	{
		tmr1_set_mode(i);

		unsigned wgm[4];
		wgm[0] = TCCR1A & _BV(WGM10);
		wgm[1] = TCCR1A & _BV(WGM11);
		wgm[2] = TCCR1B & _BV(WGM12);
		wgm[3] = TCCR1B & _BV(WGM13);

		printf0("mode = %d   ", i);
		for (j = 3; j >= 0; j--)
		{
			if (wgm[j])
				printf0("1");
			else
				printf0("0");
		}

		printf0("   TCCR1A=0x%02X     TCCR1B=0x%02X\r\n", TCCR1A, TCCR1B);
		_delay_ms(1000);
	}
}
void tmr3_set_compareA_test()
{
#if ___atmega128
	unsigned int testVals[2] =
	{ 0xFF9a, 0x0A4B };
	int i;
	for (i = 0; i < 2; i++)
	{
		unsigned int curVal = testVals[i];
		printf0("setting ocra to: 0x%04X\r\n", curVal);
		tmr3_set_output_compare_A_value(curVal);
		unsigned int readTop = OCR3AL;
		readTop |= (OCR3AH << 8);
		printf0("reading ocra at: 0x%04X\r\n", readTop);
	}
#endif
}
void timer3_mode_test()
{
#if ___atmega128

	unsigned int i;
	int j;
	for (i = 0; i < 20; i++)
	{
		tmr3_set_mode(i);

		unsigned wgm[4];
		wgm[0] = TCCR3A & _BV(WGM30);
		wgm[1] = TCCR3A & _BV(WGM31);
		wgm[2] = TCCR3B & _BV(WGM32);
		wgm[3] = TCCR3B & _BV(WGM33);

		printf0("mode = %d   ", i);
		for (j = 3; j >= 0; j--)
		{
			if (wgm[j])
				printf0("1");
			else
				printf0("0");
		}

		printf0("   TCCR3A=0x%02X     TCCR3B=0x%02X\r\n", TCCR3A, TCCR3B);
		_delay_ms(1000);
	}
#endif
}

void servo_test()
{
	servo_data_t rservo;//fake malloc
	servo_data_t * servo = &rservo;
	servo_init(servo);
	servo_calibrate(servo);

	while(1)
	{
		if(isAvailable0())
		{
			char c = getChar0();
			switch(c)
			{
				case '0':
					servo_set_position(servo,0);
					break;
				case '4':
					servo_set_position(servo,45);
					break;
				case '9':
					servo_set_position(servo, 90);
					break;
				case '3':
					servo_set_position(servo,135);
					break;
				case '8':
					servo_set_position(servo, 180);
					break;
				case '+':
					servo_increment_degrees(servo,5);
					break;
				case '-':
					servo_decrement_degrees(servo,5);
					break;
				default:
					break;
			}

			printf0("You entered: %c\r\n",c);
			servo_print_servo(servo);


		}


	}
}
int main(void)
{
	//uno input capture pin is PB0 (uno D8)
	//128 input capture pin is PD4

#if ___atmega328p
	init_USART0(9600, F_CPU);
#elif ___atmega128
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
#endif

	servo_test();
	unsigned int ticks_per_ms = 0; //ticks per ms

#if ___atmega328p
	tmr1_set_prescaler(ONE_64TH); //clock 4 us per tick @ 16MHz base

	ticks_per_ms = 250;//ticks per ms
	DDRB |= _BV(PB2);//set up output for servo
#elif ___atmega128
	tmr1_set_prescaler(ONE_64TH); //clock 8 us per tick @ 8MHz base
	tmr3_set_prescaler(ONE_64TH);//clock 4 us per tick @ 16MHz base
	ticks_per_ms = 125;//ticks per ms
	DDRB |= _BV(PB6);// OC1B set up output for servo

	DDRE |= _BV(PE4);// OC3B set up output for servo
	DDRE |= _BV(PE5);//OC3C

#endif

	//sei();
	//TODO try removing this

	double pulseWidth_ms = 0.3;
	char str[100];

	while (1)
	{
		if (pulseWidth_ms > 2.3)
			pulseWidth_ms = 0.1;

		ftoa(str, pulseWidth_ms);
		printf0("pulse width: %s ms\r\n", str);

		tmr1_set_mode(TIMER_MODE_NORMAL_MAX_TOP); //stop timer while we make updates
		tmr1_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too
		tmr1_set_output_compare_C_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too

		tmr3_set_mode(TIMER_MODE_NORMAL_MAX_TOP); //stop timer while we make updates
		tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too
		tmr3_set_output_compare_C_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too

		unsigned int pulse_width = pulseWidth_ms * ticks_per_ms; //250 * 4 us per tick = 1000 us... we want 1.5 ms
		unsigned int top = pulse_width + 20 * ticks_per_ms; // 20ms low time plus the pulse width
		printf0("pulse_width: %u", pulse_width);

		tmr1_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);
		tmr1_set_output_compare_A_value(top);
		tmr1_set_output_compare_B_value(top - pulse_width); //This is OC1B which is PB2...
		tmr1_set_output_compare_C_value(top - pulse_width); //This is OC1B which is PB2...
		tmr1_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
		tmr1_set_output_compare_C_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);

		tmr3_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);
		tmr3_set_output_compare_A_value(top);
		tmr3_set_output_compare_B_value(top - pulse_width); //This is OC1B which is PB2...
		tmr3_set_output_compare_C_value(top - pulse_width); //This is OC1B which is PB2...
		tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
		tmr3_set_output_compare_C_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);

		pulseWidth_ms += 0.1;
		_delay_ms(1000);
	}
	return 0; /* CODE SHOULD NEVER REACH HERE */
}
