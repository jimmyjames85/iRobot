/*
 * ping.c
 *
 *  Created on: Oct 13, 2015
 *      Author: jim
 */

#include <avr/io.h>
#include <util/delay.h>

#include "ping.h"
#include "timer.h"
//#include "usart/usart.h" //for ping loop bellow ....

void ping_send_pulse(volatile unsigned long * startTime)
{
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

}


//TODO I don't think this is necessary as long as the timer is not stopped
void ping_init(timer_prescaler_t prescaler)
{
	tmr1_set_mode(TIMER_MODE_NORMAL_MAX_TOP); //normal mode
	tmr1_set_prescaler(prescaler);//TODO insure prescaler is not timer off
	tmr1_enable_input_capture_isr(0);
	tmr1_enable_overflow_isr(0);
}

//uses tmr1
/**
 * uses timer1
 * assumes timer_mode == 0, but I think as long as the timer is running it should be okay
 * uses busy wait for both tmr1_capture_flag  and tmr1_overflow_flag
 * ISRs should be disabled
 *
 * assumes ping is connected to:
 *
 * ___atmega328p PB0
 * ___atemga128  PD4
 *
 * returns clock tick count for round trip ping
 *
 */
unsigned long ping()
{
	//TODO temporarily disable ISRs ???
	unsigned long start_pulse_time;
	unsigned long overflows = 0;

	ping_send_pulse(&start_pulse_time);
	tmr1_clear_overflow_flag(); //We should clear the overflow flag in case it hasn't been done before
	tmr1_clear_capture_flag();//This triggers the capture flag so we must clear it
	while (tmr1_read_capture_flag() == 0) //then wait for it to be triggered by the ping sensor
		if (tmr1_read_overflow_flag())
		{
			tmr1_clear_overflow_flag(); //Keep track of overflows
			overflows++;
		};
	tmr1_clear_capture_flag();



	unsigned long end_capture_count = tmr1_read_input_capture_count();
	unsigned long end_time_cap = (overflows << 16) | end_capture_count;
	unsigned long delta = end_time_cap - start_pulse_time;
	return delta;
}

unsigned ping_cm(timer_prescaler_t prescaler)
{
	//TODO write tmr1_get_current_prescaler()

	unsigned long delta = ping();
	double seconds = ticks_to_secs(delta, prescaler, F_CPU) - 0.00075; // ping sensor delay is 750 us
	return (seconds * 34300 / 2);
	//double dinches = cm / 2.54;
}

/*
 void doPingLoop(void)
 {
 tmr1_set_mode(0); //normal mode
 timer_prescaler_t prescaler = TIMER_ONE_1024TH;
 tmr1_set_prescaler(prescaler);
 tmr1_enable_input_capture_isr(0);
 tmr1_enable_overflow_isr(0);
 while (1)
 {
 unsigned long delta = ping();
 double seconds = ticks_to_secs(delta, prescaler, F_CPU) - 0.00075; // ping sensor delay is 750 us
 int cm = (seconds * 34300 / 2);
 double dinches = cm / 2.54;
 char d[300];
 ftoa(d, dinches);
 printf0("%s in , %d cm\r\n", d, cm);
 }
 }
 */
