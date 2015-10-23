/*
 * pwm_test.c
 *
 *  Created on: Oct 22, 2015
 *      Author: jim
 */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart/usart.h"
#include <stdlib.h>
#include "timer.h"
#include "util.h"
#include "blue_tooth_HC05.h"

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

void timer_mode_test()
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

int main(void)
{
	//uno input capture pin is PB0 (uno D8)
	//128 input capture pin is PD4

#if ___atmega328p
	init_USART0(9600, F_CPU);
#elif ___atmega128
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
#endif

	tmr1_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);

	tmr1_enable_output_compare_A_match_isr(0); //enable interupts
	tmr1_enable_output_compare_B_match_isr(0); //

	tmr1_set_output_compare_A_value(0xFFFF); //set TOP to 65534
	tmr1_set_prescaler(ONE_64TH); //clock 4 us per tick
	unsigned int pulse_width = 1.5 * 2000; //2000 ticks = 1000 us... we want 1.5 ms

	//top - pulse_width
	tmr1_set_output_compare_B_value(pulse_width); //This is OC1B which is PB2...
	DDRB |= _BV(PB2); //set up output for servo

	unsigned char low = OCR1AL;
	unsigned char high = OCR1AH;

	printf0("OCRA=x%02X%02X  ", high, low);
	low = OCR1BL;
	high = OCR1BH;
	printf0("OCRB=x%02X%02X  \r\n", high, low);

	printf0("TIMSK=x%02X\r\n", TIMSK1);
	sei();
	compa = 0;
	compb = 0;
	unsigned long sec = 0;
	while (1)
	{
		if (compa)
		{
			compa = 0;
	//		printf0("A!\r\n");
		}
		if (compb)
		{
			compb = 0;
		//	printf0("B!\r\n");
		}

		if (tmr1_read_count() % 8000 == 0)
		{
			printf0("%02u\r\n", ++sec);
		}

	}
	return 0; /* CODE SHOULD NEVER REACH HERE */
}
