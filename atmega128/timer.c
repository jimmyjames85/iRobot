/*
 * timer.c
 *
 *  Created on: Oct 15, 2015
 *      Author: jim
 */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>

double ticks_to_secs(unsigned long ticks, timer_prescaler_t clock_prescaler, unsigned long fcpu)
{
	long ttps;	//timerTicksPerSec
	ttps = fcpu;
	switch (clock_prescaler)
	{
		case ONE_1024TH:
			ttps = ttps >> 2;
			/* no break */
		case ONE_256TH:
			ttps = ttps >> 2;
			/* no break */
		case ONE_64TH:
			ttps = ttps >> 3;
			/* no break */
		case ONE_8TH:
			ttps = ttps >> 3;
			/* no break */
		case NO_PRESCALING:
			/* no break */
		default:
		break;
	}
	return (double) ticks / ttps;
}

unsigned char tmr1_read_capture_flag()
{
#if ___atmega328p
	return TIFR1 & _BV(ICF1);
#elif ___atmega128
	return TIFR & _BV(ICF1);
#endif
	return 0;
}
void tmr1_clear_capture_flag(void)
{

#if ___atmega328p
	TIFR1 |= _BV(ICF1);
#elif ___atmega128
	TIFR |= _BV(ICF1);
#endif
}

void tmr1_set_mode(unsigned char mode)
{
	/*
	 * see
	 * uno table 15-4 pg 136
	 * atmega 128 table 61 pg 134
	 */

	if (mode > 15)
		return;

	//These mask or clear out the WGM1[3:0] bits
	char TCCR1A_mask = TCCR1A & ~( _BV(WGM11) | _BV(WGM10));
	char TCCR1B_mask = TCCR1B & ~( _BV(WGM13) | _BV(WGM12));

	if (mode & 0x01) //bit0
		TCCR1A_mask |= _BV(WGM10);

	if (mode & 0x02) //bit1
		TCCR1A_mask |= _BV(WGM11);

	if (mode & 0x04) //bit2
		TCCR1B_mask |= _BV(WGM12);

	if (mode & 0x08) //bit3
		TCCR1B_mask |= _BV(WGM13);

	TCCR1A = TCCR1A_mask;
	TCCR1B = TCCR1B_mask;
}

void tmr1_set_prescaler(timer_prescaler_t prescaler)
{
	TCCR1B = (TCCR1B & 0xF8) | prescaler; //sets CS1 [2:0]
}

void tmr1_set_input_capture_edge(char true_is_rising_edge)
{
	if (true_is_rising_edge)
		TCCR1B |= _BV(ICES1);
	else
		TCCR1B &= ~_BV(ICES1);
}

void tmr1_enable_overflow_isr(char enable_bool)
{
#if ___atmega328p
	if (enable_bool)
	TIMSK1 |= _BV(TOIE1);
	else
	TIMSK1 &= ~_BV(TOIE1);
#elif ___atmega128
	if (enable_bool)
	TIMSK |= _BV(TOIE1);
	else
	TIMSK &= ~_BV(TOIE1);
#endif
}
void tmr1_enable_input_capture_isr(char enable_bool)
{

#if ___atmega328p
	if (enable_bool)
	TIMSK1 |= _BV(ICIE1);
	else
	TIMSK1 &= ~_BV(ICIE1);
#elif ___atmega128
	if (enable_bool)
	TIMSK |= _BV(TICIE1);
	else
	TIMSK &= ~_BV(TICIE1);
#endif
}

void tmr1_clear_count(void)
{
	//only clear's time if timer is stopped
	//must write temp (high) byte first, then low byte, in order to write both bytes synchronously
	if (!(TCCR1B & 0x07))
	{
		TCNT1H = 0x0;
		TCNT1L = 0x0;
	}
}

void tmr1_stop(void)
{
	tmr1_set_prescaler(STOPPED); //set no clock for timer
}

void tmr1_start(timer_prescaler_t prescaler)
{
	tmr1_set_prescaler(prescaler);
}
unsigned tmr1_read_count(void)
{
	//must read low byte first in order to read temp (high) byte
	unsigned int read = TCNT1L;
	read |= (TCNT1H << 8);
	return read;
}
unsigned tmr1_read_input_capture_count(void)
{
	//must read low byte first in order to read temp (high) byte
	unsigned int read = ICR1L;
	read |= (ICR1H << 8);
	return read;
}

void tmr1_enable_output_compare_A_match_isr(char enable_bool)
{
#if ___atmega328p
	if(enable_bool)
	TIMSK1 |= _BV(OCIE1A);
	else
	TIMSK1 &= ~_BV(OCIE1A);
#elif ___atmega128
	if(enable_bool)
	TIMSK |= _BV(OCIE1A);
	else
	TIMSK &= ~_BV(OCIE1A);
#endif
}

void tmr1_enable_output_compare_B_match_isr(char enable_bool)
{
#if ___atmega328p
	if(enable_bool)
	TIMSK1 |= _BV(OCIE1B);
	else
	TIMSK1 &= ~_BV(OCIE1B);
#elif ___atmega128
	if(enable_bool)
	TIMSK |= _BV(OCIE1B);
	else
	TIMSK &= ~_BV(OCIE1B);
#endif
}

void tmr1_set_output_compare_A_value(unsigned int compare_val)
{
	OCR1AH = compare_val >> 8;
	OCR1AL = compare_val & 0x00FF;
}
void tmr1_set_output_compare_B_value(unsigned int compare_val)
{
	OCR1BH = compare_val >> 8;
	OCR1BL = compare_val & 0x00FF;
}

