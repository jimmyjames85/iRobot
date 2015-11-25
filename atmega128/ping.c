/*
 * ping.c
 *
 *  Created on: Oct 13, 2015
 *      Author: jim
 */

#include <avr/io.h>
#include <util/delay.h>
#include "ping.h"

void ping_enable_isrs(unsigned char enable_bool)
{
    tmr1_enable_input_capture_isr(enable_bool);
    tmr1_enable_overflow_isr(enable_bool);
}

void ping_send_pulse(volatile unsigned long *startTime)
{
    DDRD |= _BV(4); // setup for output so we can trigger */
    PORTD |= _BV(4);//send high */
    _delay_us(5);// tout = 2us
    PORTD &= ~_BV(4);//send low */
    DDRD &= ~_BV(4);// setup for input*/

    *startTime = tmr1_read_count();
}

//TODO I don't think this is necessary as long as the timer is not stopped
void ping_init(timer_prescaler_t prescaler)
{
    tmr1_set_mode(TIMER_MODE_NORMAL_MAX_TOP); //normal mode
    tmr1_set_prescaler(prescaler); //TODO insure prescaler is not timer off
    //tmr1_enable_input_capture_isr(0);
    //tmr1_enable_overflow_isr(0);
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
 * ___atemga128  PD4
 *
 * returns clock tick count for round trip ping
 *
 */
unsigned long ping_busy_wait()
{
    //TODO temporarily disable ISRs ???
    unsigned long start_pulse_time;
    unsigned long overflows = 0;

    ping_send_pulse(&start_pulse_time);
    tmr1_clear_overflow_flag(); //We should clear the overflow flag in case it hasn't been done before
    tmr1_clear_capture_flag(); //This triggers the capture flag so we must clear it
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

unsigned ping_count_to_cm(timer_prescaler_t prescaler, unsigned long ping_count)
{
    return ping_count_to_mm(prescaler, ping_count) / 10;
}

unsigned ping_cm_busy_wait(timer_prescaler_t prescaler)
{
    unsigned long delta = ping_busy_wait();
    return ping_count_to_cm(prescaler, delta);
}

unsigned ping_count_to_mm(timer_prescaler_t prescaler, unsigned long ping_count)
{
    double seconds = ticks_to_secs(ping_count, prescaler, F_CPU) - 0.00075; // ping sensor delay is 750 us
    return (seconds * 343000 / 2);
}

unsigned ping_mm_busy_wait(timer_prescaler_t prescaler)
{
    //TODO write tmr1_get_current_prescaler()

    unsigned long delta = ping_busy_wait();
    return ping_count_to_mm(prescaler, delta);
}
