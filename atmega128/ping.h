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
typedef struct PING_DATA
{
    timer_prescaler_t prescaler;
} ping_data_t;

ping_data_t * newPing();
inline void freePing(ping_data_t * ping);
void ping_init(ping_data_t * ping);
void ping_enable_isrs(unsigned char enable_bool);
void ping_send_pulse(volatile unsigned long * startTime);
unsigned long ping_busy_wait();
unsigned ping_count_to_cm(ping_data_t * ping, unsigned long ping_count);
unsigned ping_count_to_mm(ping_data_t * ping , unsigned long ping_count);
unsigned ping_cm_busy_wait(ping_data_t * ping);
unsigned ping_mm_busy_wait(ping_data_t * ping); 
#endif /* ___PING_H_*/
