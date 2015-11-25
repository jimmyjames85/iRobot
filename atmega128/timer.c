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
    long ttps;    //timerTicksPerSec
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


unsigned char tmr0_read_overflow_flag()
{
    return TIFR & _BV(TOV0);
}

void tmr0_clear_overflow_flag()
{
    TIFR |= _BV(TOV0);
}

void tmr0_set_mode(timer_mode_t mode)
{
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
        default:
            return;
            break;
    }

    //These mask or clear out the WGM0[2:0] bits
    char TCCR0A_mask = TCCR0 & ~(_BV(WGM01) | _BV(WGM00));

    if (mode & 0x01) //bit0
        TCCR0A_mask |= _BV(WGM00);

    if (mode & 0x02) //bit1
        TCCR0A_mask |= _BV(WGM01);

    TCCR0 = TCCR0A_mask;
}

void tmr0_set_prescaler(timer_prescaler_t prescaler)
{
    TCCR0 = (TCCR0 & 0xF8) | prescaler; //sets CS0 [2:0]
}

void tmr0_enable_overflow_isr(char enable_bool)
{
    if (enable_bool)
        TIMSK |= _BV(TOIE0);
    else
        TIMSK &= ~_BV(TOIE0);
}

void tmr0_clear_count(void)
{
    if (!(TCCR0 & 0x07))
        TCNT0 = 0x0;
}

void tmr0_stop(void)
{
    tmr0_set_prescaler(TIMER_STOPPED); //set no clock for timer
}

void tmr0_start(timer_prescaler_t prescaler)
{
    tmr0_set_prescaler(prescaler);
}

unsigned char tmr0_read_count(void)
{
    return TCNT0;
}

void tmr0_enable_output_compare_A_match_isr(char enable_bool)
{
    if (enable_bool)
        TIMSK |= _BV(OCIE0);
    else
        TIMSK &= ~_BV(OCIE0);
}

void tmr0_set_output_compare_A_value(unsigned char compare_val)
{
    OCR0 = compare_val;
}

void tmr0_set_output_compare_A_mode(timer_compare_output_mode_t mode)
{
    char TCCR0A_mask = TCCR0 & ~(_BV(COM01) | _BV(COM00));

    if (mode & 0x01) //bit0
        TCCR0A_mask |= _BV(COM00);

    if (mode & 0x02) //bit1
        TCCR0A_mask |= _BV(COM01);

    TCCR0 = TCCR0A_mask;
}


unsigned char tmr1_read_capture_flag()
{
    return TIFR & _BV(ICF1);
}


unsigned char tmr1_read_overflow_flag()
{
    return TIFR & _BV(TOV1);
}

unsigned char tmr3_read_capture_flag()
{
    return ETIFR & _BV(ICF3);
}

void tmr1_clear_capture_flag(void)
{
    TIFR |= _BV(ICF1);
}

void tmr1_clear_overflow_flag()
{
    TIFR |= _BV(TOV1);
}

void tmr3_clear_capture_flag(void)
{
    ETIFR |= _BV(ICF3);
}

void tmr1_set_mode(timer_mode_t mode)
{
    /*
     * see atmega 128 table 61 pg 134
     */

    if (mode > 15)
        return;

    //These mask or clear out the WGM1[3:0] bits
    char TCCR1A_mask = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
    char TCCR1B_mask = TCCR1B & ~(_BV(WGM13) | _BV(WGM12));

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
    /*
     * see
     * uno table 15-4 pg 136
     * atmega 128 table 61 pg 134
     */

    if (mode > 15)
        return;

    //These mask or clear out the WGM1[3:0] bits
    char TCCR3A_mask = TCCR3A & ~(_BV(WGM31) | _BV(WGM30));
    char TCCR3B_mask = TCCR3B & ~(_BV(WGM33) | _BV(WGM32));

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
}

void tmr1_set_prescaler(timer_prescaler_t prescaler)
{
    TCCR1B = (TCCR1B & 0xF8) | prescaler; //sets CS1 [2:0]
}

void tmr3_set_prescaler(timer_prescaler_t prescaler)
{
    TCCR3B = (TCCR3B & 0xF8) | prescaler; //sets CS1 [2:0]
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
    if (true_is_rising_edge)
        TCCR3B |= _BV(ICES3);
    else
        TCCR3B &= ~_BV(ICES3);
}


void tmr1_enable_overflow_isr(char enable_bool)
{
    if (enable_bool)
        TIMSK |= _BV(TOIE1);
    else
        TIMSK &= ~_BV(TOIE1);
}

void tmr3_enable_overflow_isr(char enable_bool)
{
    if (enable_bool)
        ETIMSK |= _BV(TOIE3);
    else
        ETIMSK &= ~_BV(TOIE3);
}


void tmr1_enable_input_capture_isr(char enable_bool)
{
    if (enable_bool)
        TIMSK |= _BV(TICIE1);
    else
        TIMSK &= ~_BV(TICIE1);
}

void tmr3_enable_input_capture_isr(char enable_bool)
{
    if (enable_bool)
        ETIMSK |= _BV(TICIE3);
    else
        ETIMSK &= ~_BV(TICIE3);
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
    //only clear's time if timer is stopped
    //must write temp (high) byte first, then low byte, in order to write both bytes synchronously
    if (!(TCCR3B & 0x07))
    {
        TCNT3H = 0x0;
        TCNT3L = 0x0;
    }
}


void tmr1_stop(void)
{
    tmr1_set_prescaler(TIMER_STOPPED); //set no clock for timer
}

void tmr3_stop(void)
{
    tmr3_set_prescaler(TIMER_STOPPED); //set no clock for timer
}


void tmr1_start(timer_prescaler_t prescaler)
{
    tmr1_set_prescaler(prescaler);
}

void tmr3_start(timer_prescaler_t prescaler)
{
    tmr3_set_prescaler(prescaler);
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
    //must read low byte first in order to read temp (high) byte
    unsigned int read = TCNT3L;
    read |= (TCNT3H << 8);
    return read;
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
    //must read low byte first in order to read temp (high) byte
    unsigned int read = ICR3L;
    read |= (ICR3H << 8);
    return read;
}


void tmr1_enable_output_compare_A_match_isr(char enable_bool)
{
    if (enable_bool)
        TIMSK |= _BV(OCIE1A);
    else
        TIMSK &= ~_BV(OCIE1A);
}

void tmr3_enable_output_compare_A_match_isr(char enable_bool)
{
    if (enable_bool)
        ETIMSK |= _BV(OCIE3A);
    else
        ETIMSK &= ~_BV(OCIE3A);
}


void tmr1_enable_output_compare_B_match_isr(char enable_bool)
{

    if (enable_bool)
        TIMSK |= _BV(OCIE1B);
    else
        TIMSK &= ~_BV(OCIE1B);

}

void tmr3_enable_output_compare_B_match_isr(char enable_bool)
{
    if (enable_bool)
        ETIMSK |= _BV(OCIE3B);
    else
        ETIMSK &= ~_BV(OCIE3B);

}

void tmr1_enable_output_compare_C_match_isr(char enable_bool)
{
    if (enable_bool)
        ETIMSK |= _BV(OCIE1C);
    else
        ETIMSK &= ~_BV(OCIE1C);
}

void tmr3_enable_output_compare_C_match_isr(char enable_bool)
{
    if (enable_bool)
        ETIMSK |= _BV(OCIE3C);
    else
        ETIMSK &= ~_BV(OCIE3C);
}

void tmr1_set_output_compare_A_value(unsigned int compare_val)
{
    OCR1AH = compare_val >> 8;
    OCR1AL = compare_val & 0x00FF;
}

void tmr3_set_output_compare_A_value(unsigned int compare_val)
{
    OCR3AH = compare_val >> 8;
    OCR3AL = compare_val & 0x00FF;
}


void tmr1_set_output_compare_B_value(unsigned int compare_val)
{
    OCR1BH = compare_val >> 8;
    OCR1BL = compare_val & 0x00FF;
}

void tmr3_set_output_compare_B_value(unsigned int compare_val)
{
    OCR3BH = compare_val >> 8;
    OCR3BL = compare_val & 0x00FF;
}

void tmr1_set_output_compare_C_value(unsigned int compare_val)
{
    OCR1CH = compare_val >> 8;
    OCR1CL = compare_val & 0x00FF;
}

void tmr3_set_output_compare_C_value(unsigned int compare_val)
{
    OCR3CH = compare_val >> 8;
    OCR3CL = compare_val & 0x00FF;
}


void tmr1_set_output_compare_A_mode(timer_compare_output_mode_t mode)
{
    char TCCR1A_mask = TCCR1A & ~(_BV(COM1A1) | _BV(COM1A0));

    if (mode & 0x01) //bit0
        TCCR1A_mask |= _BV(COM1A0);

    if (mode & 0x02) //bit1
        TCCR1A_mask |= _BV(COM1A1);

    TCCR1A = TCCR1A_mask;
}

void tmr3_set_output_compare_A_mode(timer_compare_output_mode_t mode)
{
    char TCCR3A_mask = TCCR3A & ~(_BV(COM3A1) | _BV(COM3A0));

    if (mode & 0x01) //bit0
        TCCR3A_mask |= _BV(COM3A0);

    if (mode & 0x02)//bit1
        TCCR3A_mask |= _BV(COM3A1);

    TCCR3A = TCCR3A_mask;
}


void tmr1_set_output_compare_B_mode(timer_compare_output_mode_t mode)
{
    char TCCR1A_mask = TCCR1A & ~(_BV(COM1B1) | _BV(COM1B0));

    if (mode & 0x01) //bit0
        TCCR1A_mask |= _BV(COM1B0);

    if (mode & 0x02) //bit1
        TCCR1A_mask |= _BV(COM1B1);

    TCCR1A = TCCR1A_mask;
}

void tmr3_set_output_compare_B_mode(timer_compare_output_mode_t mode)
{
    char TCCR3A_mask = TCCR3A & ~(_BV(COM3B1) | _BV(COM3B0));

    if (mode & 0x01) //bit0
        TCCR3A_mask |= _BV(COM3B0);

    if (mode & 0x02)//bit1
        TCCR3A_mask |= _BV(COM3B1);

    TCCR3A = TCCR3A_mask;
}

void tmr1_set_output_compare_C_mode(timer_compare_output_mode_t mode)
{
    char TCCR1A_mask = TCCR1A & ~(_BV(COM1C1) | _BV(COM1C0));

    if (mode & 0x01) //bit0
        TCCR1A_mask |= _BV(COM1C0);

    if (mode & 0x02)//bit1
        TCCR1A_mask |= _BV(COM1C1);

    TCCR1A = TCCR1A_mask;
}

void tmr3_set_output_compare_C_mode(timer_compare_output_mode_t mode)
{
    char TCCR3A_mask = TCCR3A & ~(_BV(COM3C1) | _BV(COM3C0));

    if (mode & 0x01) //bit0
        TCCR3A_mask |= _BV(COM3C0);

    if (mode & 0x02)//bit1
        TCCR3A_mask |= _BV(COM3C1);

    TCCR3A = TCCR3A_mask;
}

