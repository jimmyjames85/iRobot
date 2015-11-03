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
unsigned long ping();
unsigned ping_cm(timer_prescaler_t prescaler);

#endif /* ___PING_H_*/
