/*
 * ping_test.c
 *
 *  Created on: Oct 21, 2015
 *      Author: jim
 */


//TODO #include "ping.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart/usart.h"
#include <stdlib.h>
#include "timer.h"
#include "util.h"
#include "blue_tooth_HC05.h"

volatile short timer1_input_capture_detected;
volatile unsigned long capVal;
volatile unsigned last_time;
volatile unsigned current_time;
volatile unsigned long start_pulse_time;
volatile int haveReading;
volatile unsigned int waiting_for_ping_response;
volatile short timer1_overflow_detected;

void initPing(timer_prescaler_t prescaler)
{
	tmr1_set_mode(0); //normal mode
	tmr1_set_prescaler(prescaler);
	tmr1_enable_input_capture_isr(1);
	tmr1_enable_overflow_isr(1);
}


ISR(TIMER1_OVF_vect)
{
	if (waiting_for_ping_response)
		timer1_overflow_detected = 1;
}

ISR( TIMER1_CAPT_vect)
{
	capVal = (long) tmr1_read_input_capture_count();
	waiting_for_ping_response = 0; //WE GOT IT!!!!
	timer1_input_capture_detected = 1;
}

void ping_send_pulse(volatile unsigned long * startTime)
{
	tmr1_enable_input_capture_isr(0);

#if ___atmega328p
	DDRB |= _BV(0); // setup for output so we can trigger */
	PORTB |= _BV(0);//send high */
	_delay_us(5);// tout = 2us, 5us typical
	PORTB &= ~_BV(0);//send low */
	DDRB &= ~_BV(0);// setup for input*/
#elif ___atmega128
	DDRD |= _BV(4); // setup for output so we can trigger */
	PORTD |= _BV(4);//send high */
	_delay_us(5);// tout = 2us
	PORTD &= ~_BV(4);//send low */
	DDRD &= ~_BV(4);// setup for input*/
#endif
	*startTime = tmr1_read_count();
	tmr1_clear_capture_flag();
	tmr1_enable_input_capture_isr(1);
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

	timer_prescaler_t prescaler = ONE_64TH;
	initPing(prescaler);
	timer1_overflow_detected = 0;
	waiting_for_ping_response = 0;

	unsigned int oldTimerVal = 0;
	unsigned int continous_pulse = 1;
	unsigned long overflows = 0;

	//sei();

	////////////////////////////////////////////////////////////
	//TODO implement single pulse that blocks

	ping_send_pulse(&start_pulse_time);
	while(tmr1_read_capture_flag()==0)
		printf0(".");
	unsigned long end_time_cap = tmr1_read_count();
	unsigned int delta = end_time_cap- start_pulse_time;
	char rd[300];
	ftoa(rd,ticks_to_secs(delta, prescaler, F_CPU));


	printf0("\r\nCAPTURE!\r\n");
	printf0("delta = %u, time = %s\r\n", delta, rd);
	tmr1_clear_capture_flag();
	if(tmr1_read_capture_flag())
		printf0("clear did not work\r\n");

	capVal = (long) tmr1_read_input_capture_count();
	waiting_for_ping_response = 0; //WE GOT IT!!!!
	timer1_input_capture_detected = 1;
	////////////////////////////////////////////////////////////

	sei();


	while (1)
	{
		if (timer1_overflow_detected)
		{
			timer1_overflow_detected = 0;
			overflows++;
		}
		if (timer1_input_capture_detected)
		{
			timer1_input_capture_detected = 0;
			waiting_for_ping_response = 0;

			long delta = 0;
			delta = ((overflows << 16) | capVal);
			delta -= start_pulse_time;

			double seconds = ticks_to_secs(delta, prescaler, F_CPU) - 0.00075; // ping sensor delay is 750 us
			int cm = ((double) seconds * ((double) 34300 / 2));
			double dinches = cm;
			dinches /=2.54;
			char d[300];
			ftoa(d,dinches);
			printf0("%s in , %d cm\r\n",d, cm);
			//unsigned inches = (int)(dinches*10);
			//printf0("%d cm (%d) \t %s\r\n", cm, overflows, d);

			if (continous_pulse)
			{
				_delay_ms(200);
				overflows = 0;
				ping_send_pulse(&start_pulse_time);
				start_pulse_time = tmr1_read_count();
				waiting_for_ping_response = 1;
			}
		}
	}

	return 0; /* CODE SHOULD NEVER REACH HERE */
}
