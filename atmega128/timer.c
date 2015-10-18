/*
 * timer.c
 *
 *  Created on: Oct 15, 2015
 *      Author: jim
 */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>

// TODO add timer_t to hold timer settings such as current prescaler
// add functions to calculate ticks_to_us, ticks_to_ms, ticks_to_seconds

void clear_capture_flag_timer1(void)
{

#if ___atmega328p
	TIFR1 |= _BV(ICF1);
#elif ___atmega128
	TIFR |= _BV(ICF1);
#endif
}

void set_mode_timer1(unsigned char mode)
{
	/*
	 * see
	 * uno table 15-4 pg 136
	 * atmega 128 table 61 pg 134
	 */

	if (mode > 15)
		return;

	unsigned char mask = mode & 0x03;

#if ___atemga328p
	//TODO FIX THIS
	TCCR1A = (TCCR1A & 0xFC) | (mode & 0x03);// sets WGM10 and WGM11
	TCCR1B = (TCCR1B & 0xFC) | ((mode & 0x0F) >> 2);// sets WGM12 and WGM13 <---wrong
#elif ___atemga128
	TCCR1A =(TCCR1A & ~(_BV(WGM10)|_BV(WGM11))) | (_BV(WGM10)|_BV(WGM11))
	TCCR1B =(TCCR1B & ~(_BV(WGM12)|_BV(WGM13))) | (_BV(WGM12)|_BV(WGM13))
#endif
}

void set_prescaler_timer1(timer_prescaler_t prescaler)
{
	start_timer1(prescaler);
}

void set_input_capture_edge_timer1(char enableRisingEdge)
{
	if (enableRisingEdge)
		TCCR1B |= _BV(ICES1);
	else
		TCCR1B &= ~_BV(ICES1);
}

void set_enable_overflow_isr_timer1(char enabled)
{
#if ___atmega328p
	if (enabled)
	TIMSK1 |= _BV(TOIE1);
	else
	TIMSK1 &= ~_BV(TOIE1);
#elif ___atmega128
	if (enabled)
	TIMSK |= _BV(TOIE1);
	else
	TIMSK &= ~_BV(TOIE1);
#endif
}
void set_enable_input_capture_isr_timer1(char enabled)
{

#if ___atmega328p
	if (enabled)
		TIMSK1 |= _BV(ICIE1);
	else
		TIMSK1 &= ~_BV(ICIE1);
#elif ___atmega128
	if (enabled)
		TIMSK |= _BV(TICIE1);
	else
		TIMSK &= ~_BV(TICIE1);
#endif
}

void clear_timer1(void)
{
	//only clear's time if timer is stoped
	//must write temp (high) byte first, then low byte, in order to write both bytes synchronously
	if (!(TCCR1B & 0x07))
	{
		TCNT1H = 0x0;
		TCNT1L = 0x0;
	}
}

void stop_timer1(void)
{
	TCCR1B &= (0xF8);	// set no clock for timer
}

void start_timer1(timer_prescaler_t prescaler)
{
	TCCR1B = (TCCR1B & 0xFC) | prescaler; //sets CS1 [2:0]
}
unsigned read_timer1(void)
{
	//must read low byte first in order to read temp (high) byte
	unsigned int read = TCNT1L;
	read |= (TCNT1H << 8);
	return read;
}
unsigned read_input_capture_timer1(void)
{
	//must read low byte first in order to read temp (high) byte
	unsigned int read = ICR1L;
	read |= (ICR1H << 8);
	return read;
}

