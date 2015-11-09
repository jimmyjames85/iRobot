/*
 * ping.h
 *
 *  Created on: Oct 13, 2015
 *      Author: jim
 *
 *      Ping is setup to user timer1 !!!!
 */

#ifndef ___PING_H_
#define ___PING_H_
#include "timer.h"

void ping_init(timer_prescaler_t prescaler);
void ping_enable_isrs(unsigned char enable_bool);
void ping_send_pulse(volatile unsigned long * startTime);
unsigned long ping_busy_wait();
unsigned ping_count_to_cm(timer_prescaler_t prescaler, unsigned long ping_count);
unsigned ping_count_to_mm(timer_prescaler_t prescaler, unsigned long ping_count);
unsigned ping_cm_busy_wait(timer_prescaler_t prescaler);
unsigned ping_mm_busy_wait(timer_prescaler_t prescaler);
#endif /* ___PING_H_*/
