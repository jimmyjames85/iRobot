/*
 * timer.c
 *
 *  Created on: Oct 15, 2015
 *      Author: jim
 */

#include "timer.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include "usart.h"
double ticks_to_secs(unsigned long ticks, timer_prescaler_t clock_prescaler, unsigned long fcpu)
{
	long ttps;	//timerTicksPerSec
	ttps = fcpu;
	switch (clock_prescaler)
	{
		case TIMER_ONE_1024TH:
			ttps = ttps >> 2;
			/* no break */
		case TIMER_ONE_256TH:
			ttps = ttps >> 2;
			/* no break */
		case TIMER_ONE_64TH:
			ttps = ttps >> 3;
			/* no break */
		case TIMER_ONE_8TH:
			ttps = ttps >> 3;
			/* no break */
		case TIMER_NO_PRESCALING:
			/* no break */
		default:
		break;
	}
	return (double) ticks / ttps;
}

unsigned char tmr1_read_capture_flag()
{
#if ___atmega328p || ___atmega2560
	return TIFR1 & _BV(ICF1);
#elif ___atmega128
	return TIFR & _BV(ICF1);
#endif
	return 0;
}

unsigned char tmr0_read_overflow_flag()
{
#if ___atmega328p
	return TIFR0 & _BV(TOV0);
#endif
return 0;
}

unsigned char tmr1_read_overflow_flag()
{
#if ___atmega328p || ___atmega2560
	return TIFR1 & _BV(TOV1);
#elif ___atmega128
	return TIFR & _BV(TOV1);
#endif
	return 0;
}

unsigned char tmr3_read_capture_flag()
{
#if ___atmega128
	return ETIFR & _BV(ICF3);
#elif ___atmega2560
	return TIFR1 & _BV(ICF3);
#endif
	return 0;
}

void tmr1_clear_capture_flag(void)
{

#if ___atmega328p || ___atmega2560 
	TIFR1 |= _BV(ICF1);
#elif ___atmega128
	TIFR |= _BV(ICF1);
#endif
}
void tmr0_clear_overflow_flag()
{
#if ___atmega328p
	TIFR0 |= _BV(TOV0);
#endif
}

void tmr1_clear_overflow_flag()
{
#if ___atmega328p || ___atmega2560
	TIFR1 |= _BV(TOV1);
#elif ___atmega128
	TIFR |= _BV(TOV1);
#endif
}

void tmr3_clear_capture_flag(void)
{
#if ___atmega128
	ETIFR |= _BV(ICF3);
#elif ___atmega2560
	TIFR3 |= _BV(ICF3);
#endif
}

void tmr0_set_mode(timer_mode_t mode)
{
#if ___atmega328p
	//uno see pg 108
	switch (mode)
	{
		case TIMER_MODE_NORMAL_MAX_TOP:
			mode = 0;
		break;
		case TIMER_MODE_PWM_PHASE_CORRECT_8_BIT_TOP:
			mode = 1;
		break;
		case TIMER_MODE_CTC_OCRnA_TOP:
			mode = 2;
		break;
		case TIMER_MODE_PWM_FAST_8_BIT_TOP:
			mode = 3;
		break;
		case TIMER_MODE_RESERVED:
			mode = 4;
		break;
		case TIMER_MODE_PWM_PHASE_CORRECT_OCRnA_TOP:
			mode = 5;
		break;
		case TIMER_MODE_RESERVED2:
			mode = 6;
		break;
		case TIMER_MODE_PWM_FAST_OCRnA_TOP:
			mode = 7;
		break;
		default:
			return;
		break;
	}

	//These mask or clear out the WGM0[2:0] bits
	char TCCR0A_mask = TCCR0A & ~( _BV(WGM01) | _BV(WGM00));
	char TCCR0B_mask = TCCR0B & ~(_BV(WGM02));

	if (mode & 0x01) //bit0
		TCCR0A_mask |= _BV(WGM00);

	if (mode & 0x02) //bit1
		TCCR0A_mask |= _BV(WGM01);

	if (mode & 0x04) //bit2
		TCCR0B_mask |= _BV(WGM02);

	TCCR0A = TCCR0A_mask;
	TCCR0B = TCCR0B_mask;
#endif
}

void tmr1_set_mode(timer_mode_t mode)
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

void tmr3_set_mode(timer_mode_t mode)
{
#if ___atmega128 || ___atmega2560
	/*
	 * see
	 * uno table 15-4 pg 136
	 * atmega 128 table 61 pg 134
	 */

	if (mode > 15)
	return;

	//These mask or clear out the WGM1[3:0] bits
	char TCCR3A_mask = TCCR3A & ~( _BV(WGM31) | _BV(WGM30));
	char TCCR3B_mask = TCCR3B & ~( _BV(WGM33) | _BV(WGM32));

	if (mode & 0x01)//bit0
	TCCR3A_mask |= _BV(WGM30);

	if (mode & 0x02)//bit1
	TCCR3A_mask |= _BV(WGM31);

	if (mode & 0x04)//bit2
	TCCR3B_mask |= _BV(WGM32);

	if (mode & 0x08)//bit3
	TCCR3B_mask |= _BV(WGM33);

	TCCR3A = TCCR3A_mask;
	TCCR3B = TCCR3B_mask;

#endif
}

void tmr0_set_prescaler(timer_prescaler_t prescaler)
{
#if ___atmega328p
	TCCR0B = (TCCR0B & 0xF8) | prescaler; //sets CS0 [2:0]
#endif
}

void tmr1_set_prescaler(timer_prescaler_t prescaler)
{
	TCCR1B = (TCCR1B & 0xF8) | prescaler; //sets CS1 [2:0]
}

void tmr3_set_prescaler(timer_prescaler_t prescaler)
{
#if ___atmega128 || ___atmega2560
	TCCR3B = (TCCR3B & 0xF8) | prescaler; //sets CS1 [2:0]
#endif
}


void tmr1_set_input_capture_edge(char true_is_rising_edge)
{
	if (true_is_rising_edge)
		TCCR1B |= _BV(ICES1);
	else
		TCCR1B &= ~_BV(ICES1);
}

void tmr3_set_input_capture_edge(char true_is_rising_edge)
{
#if ___atmega128 || ___atmega2560
	if (true_is_rising_edge)
	TCCR3B |= _BV(ICES3);
	else
	TCCR3B &= ~_BV(ICES3);
#endif
}

void tmr0_enable_overflow_isr(char enable_bool)
{
#if ___atmega328p

	if (enable_bool)
		TIMSK0 |= _BV(TOIE0);
	else
		TIMSK0 &= ~_BV(TOIE0);
#endif
}

void tmr1_enable_overflow_isr(char enable_bool)
{
#if ___atmega328p || ___atmega2560
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

void tmr3_enable_overflow_isr(char enable_bool)
{
#if ___atmega128
	if (enable_bool)
	ETIMSK |= _BV(TOIE3);
	else
	ETIMSK &= ~_BV(TOIE3);
#elif ___atmega2560
	if (enable_bool)
	TIMSK3 |= _BV(TOIE3);
	else
	TIMSK3 &= ~_BV(TOIE3);
#endif
}


void tmr1_enable_input_capture_isr(char enable_bool)
{
#if ___atmega328p  || ___atmega2560
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

void tmr3_enable_input_capture_isr(char enable_bool)
{

#if ___atmega128
	if (enable_bool)
	ETIMSK |= _BV(TICIE3);
	else
	ETIMSK &= ~_BV(TICIE3);
#elif ___atmega2560
	if (enable_bool)
	TIMSK3 |= _BV(TICIE3);
	else
	TIMSK3 &= ~_BV(TICIE3);
#endif
}

void tmr0_clear_count(void)
{
#if ___atmega328p
	//only clear's time if timer is stopped
	if (!(TCCR0B & 0x07))
		TCNT0 = 0x0;
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

void tmr3_clear_count(void)
{
#if ___atmega128  || ___atmega2560
	//only clear's time if timer is stopped
	//must write temp (high) byte first, then low byte, in order to write both bytes synchronously
	if (!(TCCR3B & 0x07))
	{
		TCNT3H = 0x0;
		TCNT3L = 0x0;
	}
#endif
}

void tmr0_stop(void)
{
#if ___atmega328p
	tmr0_set_prescaler(TIMER_STOPPED); //set no clock for timer
#endif
}

void tmr1_stop(void)
{
	tmr1_set_prescaler(TIMER_STOPPED); //set no clock for timer
}

void tmr3_stop(void)
{
#if ___atmega128  || ___atmega2560
	tmr3_set_prescaler(TIMER_STOPPED); //set no clock for timer
#endif
}


void tmr0_start(timer_prescaler_t prescaler)
{
#if ___atmega328p
	tmr0_set_prescaler(prescaler);
#endif
}

void tmr1_start(timer_prescaler_t prescaler)
{
	tmr1_set_prescaler(prescaler);
}

void tmr3_start(timer_prescaler_t prescaler)
{
#if ___atmega128  || ___atmega2560
	tmr3_set_prescaler(prescaler);
#endif
}

unsigned char tmr0_read_count(void)
{
#if ___atmega328p
	return TCNT0;
#endif
	return 0;
}

unsigned tmr1_read_count(void)
{
	//must read low byte first in order to read temp (high) byte
	unsigned int read = TCNT1L;
	read |= (TCNT1H << 8);
	return read;
}

unsigned tmr3_read_count(void)
{
#if ___atmega128  || ___atmega2560
	//must read low byte first in order to read temp (high) byte
	unsigned int read = TCNT3L;
	read |= (TCNT3H << 8);
	return read;
#endif
}

unsigned tmr1_read_input_capture_count(void)
{
	//must read low byte first in order to read temp (high) byte
	unsigned int read = ICR1L;
	read |= (ICR1H << 8);
	return read;
}

unsigned tmr3_read_input_capture_count(void)
{
#if ___atmega128  || ___atmega2560
	//must read low byte first in order to read temp (high) byte
	unsigned int read = ICR3L;
	read |= (ICR3H << 8);
	return read;
#endif
}

void tmr0_enable_output_compare_A_match_isr(char enable_bool)
{
#if ___atmega328p
	if (enable_bool)
		TIMSK0 |= _BV(OCIE0A);
	else
		TIMSK0 &= ~_BV(OCIE0A);
#endif
}

void tmr1_enable_output_compare_A_match_isr(char enable_bool)
{
#if ___atmega328p  || ___atmega2560
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

void tmr3_enable_output_compare_A_match_isr(char enable_bool)
{
#if ___atmega128
	if(enable_bool)
	ETIMSK |= _BV(OCIE3A);
	else
	ETIMSK &= ~_BV(OCIE3A);
#elif ___atmega2560
	if(enable_bool)
	TIMSK3 |= _BV(OCIE3A);
	else
	TIMSK3 &= ~_BV(OCIE3A);
#endif
}

void tmr0_enable_output_compare_B_match_isr(char enable_bool)
{
#if ___atmega328p
	if (enable_bool)
		TIMSK0 |= _BV(OCIE0B);
	else
		TIMSK0 &= ~_BV(OCIE0B);
#endif
}

void tmr1_enable_output_compare_B_match_isr(char enable_bool)
{
#if ___atmega328p || ___atmega2560
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

void tmr3_enable_output_compare_B_match_isr(char enable_bool)
{
#if ___atmega128 
	if(enable_bool)
	ETIMSK |= _BV(OCIE3B);
	else
	ETIMSK &= ~_BV(OCIE3B);
#elif ___atmega2560
	if(enable_bool)
	TIMSK3 |= _BV(OCIE3B);
	else
	TIMSK3 &= ~_BV(OCIE3B);
#endif
}

void tmr1_enable_output_compare_C_match_isr(char enable_bool)
{
#if ___atmega328p
	return; //no support for uno
#elif ___atmega128
	if(enable_bool)
	ETIMSK |= _BV(OCIE1C);
	else
	ETIMSK &= ~_BV(OCIE1C);
#elif ___atmega2560
	if(enable_bool)
	TIMSK1 |= _BV(OCIE1C);
	else
	TIMSK1 &= ~_BV(OCIE1C);
#endif
}

void tmr3_enable_output_compare_C_match_isr(char enable_bool)
{
#if ___atmega128
	if(enable_bool)
	ETIMSK |= _BV(OCIE3C);
	else
	ETIMSK &= ~_BV(OCIE3C);
#elif ___atmega2560
	if(enable_bool)
	TIMSK3 |= _BV(OCIE3C);
	else
	TIMSK3 &= ~_BV(OCIE3C);
#endif
}

void tmr0_set_output_compare_A_value(unsigned char compare_val)
{
#if ___atmega328p
	OCR0A = compare_val;
#endif
}

void tmr1_set_output_compare_A_value(unsigned int compare_val)
{
	OCR1AH = compare_val >> 8;
	OCR1AL = compare_val & 0x00FF;
}

void tmr3_set_output_compare_A_value(unsigned int compare_val)
{
#if ___atmega128 || ___atmega2560
	OCR3AH = compare_val >> 8;
	OCR3AL = compare_val & 0x00FF;
#endif
}

void tmr0_set_output_compare_B_value(unsigned char compare_val)
{
#if ___atmega328p
	OCR0B = compare_val;
#endif
}

void tmr1_set_output_compare_B_value(unsigned int compare_val)
{
	OCR1BH = compare_val >> 8;
	OCR1BL = compare_val & 0x00FF;
}

void tmr3_set_output_compare_B_value(unsigned int compare_val)
{
#if ___atmega128 || ___atmega2560
	OCR3BH = compare_val >> 8;
	OCR3BL = compare_val & 0x00FF;
#endif
}

void tmr1_set_output_compare_C_value(unsigned int compare_val)
{
#if ___atmega128 || ___atmega2560
	OCR1CH = compare_val >> 8;
	OCR1CL = compare_val & 0x00FF;
#endif
}

void tmr3_set_output_compare_C_value(unsigned int compare_val)
{
#if ___atmega128 || ___atmega2560
	OCR3CH = compare_val >> 8;
	OCR3CL = compare_val & 0x00FF;
#endif
}

void tmr0_set_output_compare_A_mode(timer_compare_output_mode_t mode)
{
#if ___atmega328p
	char TCCR0A_mask = TCCR0A & ~( _BV(COM0A1) | _BV(COM0A0));

	if (mode & 0x01) //bit0
		TCCR0A_mask |= _BV(COM0A0);

	if (mode & 0x02) //bit1
		TCCR0A_mask |= _BV(COM0A1);

	TCCR0A = TCCR0A_mask;
#endif
}

void tmr1_set_output_compare_A_mode(timer_compare_output_mode_t mode)
{
	char TCCR1A_mask = TCCR1A & ~( _BV(COM1A1) | _BV(COM1A0));

	if (mode & 0x01) //bit0
		TCCR1A_mask |= _BV(COM1A0);

	if (mode & 0x02) //bit1
		TCCR1A_mask |= _BV(COM1A1);

	TCCR1A = TCCR1A_mask;
}

void tmr3_set_output_compare_A_mode(timer_compare_output_mode_t mode)
{
#if ___atmega128 || ___atmega2560
	char TCCR3A_mask = TCCR3A & ~( _BV(COM3A1) | _BV(COM3A0));

	if (mode & 0x01) //bit0
	TCCR3A_mask |= _BV(COM3A0);

	if (mode & 0x02)//bit1
	TCCR3A_mask |= _BV(COM3A1);

	TCCR3A = TCCR3A_mask;
#endif
}

void tmr0_set_output_compare_B_mode(timer_compare_output_mode_t mode)
{
#if ___atmega328p
	char TCCR0A_mask = TCCR0A & ~( _BV(COM0B1) | _BV(COM0B0));

	if (mode & 0x01) //bit0
		TCCR0A_mask |= _BV(COM0B0);

	if (mode & 0x02) //bit1
		TCCR0A_mask |= _BV(COM0B1);

	TCCR0A = TCCR0A_mask;
#endif
}

void tmr1_set_output_compare_B_mode(timer_compare_output_mode_t mode)
{
	char TCCR1A_mask = TCCR1A & ~( _BV(COM1B1) | _BV(COM1B0));

	if (mode & 0x01) //bit0
		TCCR1A_mask |= _BV(COM1B0);

	if (mode & 0x02) //bit1
		TCCR1A_mask |= _BV(COM1B1);

	TCCR1A = TCCR1A_mask;
}

void tmr3_set_output_compare_B_mode(timer_compare_output_mode_t mode)
{
#if ___atmega128 || ___atmega2560

	char TCCR3A_mask = TCCR3A & ~( _BV(COM3B1) | _BV(COM3B0));

	if (mode & 0x01) //bit0
	TCCR3A_mask |= _BV(COM3B0);

	if (mode & 0x02)//bit1
	TCCR3A_mask |= _BV(COM3B1);

	TCCR3A = TCCR3A_mask;
#endif
}

void tmr1_set_output_compare_C_mode(timer_compare_output_mode_t mode)
{
#if ___atmega128 || ___atmega2560
	char TCCR1A_mask = TCCR1A & ~( _BV(COM1C1) | _BV(COM1C0));

	if (mode & 0x01) //bit0
	TCCR1A_mask |= _BV(COM1C0);

	if (mode & 0x02)//bit1
	TCCR1A_mask |= _BV(COM1C1);

	TCCR1A = TCCR1A_mask;
#endif
}

void tmr3_set_output_compare_C_mode(timer_compare_output_mode_t mode)
{
#if ___atmega128 || ___atmega2560
	char TCCR3A_mask = TCCR3A & ~( _BV(COM3C1) | _BV(COM3C0));

	if (mode & 0x01) //bit0
	TCCR3A_mask |= _BV(COM3C0);

	if (mode & 0x02)//bit1
	TCCR3A_mask |= _BV(COM3C1);

	TCCR3A = TCCR3A_mask;
#endif
}

