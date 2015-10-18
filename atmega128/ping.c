/*
 * ping.c
 *
 *  Created on: Oct 13, 2015
 *      Author: jim
 */

//#include "ping.h"

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart/usart.h"
#include <stdlib.h>
#include "timer.h"
#include "blue_tooth_HC05.h"

volatile unsigned end_time;
volatile unsigned start_time;
volatile short have_reading;

void initPing(void)
{

	set_mode_timer1(0);//normal mode
	set_prescaler_timer1(ONE_64TH);
	set_enable_input_capture_isr_timer1(1);
	set_enable_overflow_isr_timer1(1);
}

volatile unsigned int waiting_for_ping_response;
volatile short timer1_overflow_detected;

ISR(TIMER1_OVF_vect)
{
	if (waiting_for_ping_response)
		timer1_overflow_detected = 1;
}

void ping_send_pulse()
{
	set_enable_input_capture_isr_timer1(0);

#if ___atmega328p
	DDRB |= _BV(0); // setup for output so we can trigger
	PORTB |= _BV(0); // send high
	_delay_us(5); // tout = 2 us , 5us typical
	PORTB &= ~_BV(0); // send low
	DDRB &= ~_BV(0); // setup for input

//	PORTB &= ~_BV(0);//pull down
#elif ___atmega128
	DDRD |= _BV(4); // setup for output so we can trigger */
	PORTD |= _BV(4);//send high */
	_delay_us(5);// tout = 2us
	PORTD &= ~_BV(4);//send low */
	DDRD &= ~_BV(4);// setup for input*/
#endif

	clear_capture_flag_timer1();
	set_enable_input_capture_isr_timer1(1);

}
volatile short timer1_input_capture_detected;
volatile unsigned int capVal;

ISR( TIMER1_CAPT_vect)
{
	timer1_input_capture_detected = 1;
	waiting_for_ping_response=0; //WE GOT IT!!!!
	capVal= read_input_capture_timer1();
}

#define BLINK_DELAY_MS 43

long ticks_to_mm(unsigned long ticks, timer_prescaler_t clock_prescaler)
{
	long ttps;	//timerTicksPerSec

	switch (clock_prescaler)
	{

		//TODO reverse the case order and remove the breaks
		case ONE_8TH:
			ttps = F_CPU / 8;
		break;
		case ONE_64TH:
			ttps = F_CPU / 64;
		break;
		case ONE_256TH:
			ttps = F_CPU / 256;
		break;
		case ONE_1024TH:
			ttps = F_CPU / 1024;
		break;
		case NO_PRESCALING:
		default:
			ttps = F_CPU;
		break;
	}
	printf0("ticks=%u\r\n",ticks);
	return (ticks * (340000L / 2)) / ttps;
}

int main(void)
{

#if ___atmega328p
	init_USART0(9600, F_CPU);
#elif ___atmega128
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
#endif
	//initPing();


	timer_prescaler_t prescaler = ONE_64TH;
	timer1_overflow_detected = 0;
	set_mode_timer1(0);//normal mode
	set_prescaler_timer1(prescaler);
	set_enable_input_capture_isr_timer1(1);
	set_enable_overflow_isr_timer1(1);

	//doPingLoop();
	DDRB |= _BV(DDB5); /* set pin 5 of PORTB for output*/

	unsigned int oldTimerVal = 0;
	unsigned long start_pulse_time = 0;
	waiting_for_ping_response = 0;
	unsigned int continous_pulse = 0;
	unsigned int overflows = 0;
	sei();
	while (1)
	{
		if (timer1_overflow_detected)
		{
			sendString0("\r\noverflow!");
			//stop_timer1();
			//clear_timer1_val();
			timer1_overflow_detected = 0;
			overflows++;
		}
		if (timer1_input_capture_detected)
		{
			waiting_for_ping_response = 0;

			long delta;
			if(overflows)
				capVal = capVal << 8;

			delta = capVal-start_pulse_time;
/*			delta  = overflows;
			delta = delta*128;
			delta |= capVal;
			delta -= start_pulse_time;*/
			//		(((long)( overflows << 8)) | capVal) - start_pulse_time;

			long mm = ticks_to_mm(delta, prescaler);
			printf0("(delta= %u) dist = %u mm , %u cm\r\n", delta, mm, (mm/10));

			if (overflows)
			{
				printf0(" <--overflow capVal= %u ovf=%u srt= %u, delta = %u\r\n", capVal,overflows, start_pulse_time, delta);
			}

			timer1_input_capture_detected = 0;
			if (continous_pulse)
			{
				overflows = 0;
				ping_send_pulse();
				start_pulse_time = read_timer1();
				waiting_for_ping_response = 1;
			}
		}
		unsigned int timerVal = read_timer1();

		if (timerVal != oldTimerVal)
		{
			//printf0("TCCR1A= x%02X TCCR1B= x%02X TIMSK1= x%02X\r\n", TCCR1A, TCCR1B, TIMSK);
			//printf0("time: %d\r\n",timerVal);
			oldTimerVal = timerVal;
		}

		if (isAvailable0())
		{
			char c = getChar0();
			if (c == ' ')
			{
				printf0("starting timer\r\n");
				start_timer1(prescaler); //start timer
			}
			else if (c == 's')
			{
				stop_timer1();
				printf0("user stopped timer \r\n ");
			}
			else if (c == 'p')
			{
				printf0("user sent pulse at \r\n");
				overflows = 0;
				ping_send_pulse();
				start_pulse_time = read_timer1();
				waiting_for_ping_response = 1;
			}
			else if (c == 'c')
			{
				continous_pulse = 1;
			}
			else if (c == 'C')
			{
				continous_pulse = 0;
			}
			else if(c=='h')
			{
				continous_pulse = 0;
				stop_timer1();
				clear_timer1();
			}
		}

		PORTB |= _BV(PORTB5); /* set pin 5 high to turn led on */
		_delay_ms(BLINK_DELAY_MS);

		PORTB &= ~_BV(PORTB5); /* set pin 5 low to turn led off */
		_delay_ms(BLINK_DELAY_MS);
	}

	return 0; /* CODE SHOULD NEVER REACH HERE */
}
