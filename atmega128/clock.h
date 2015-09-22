/*
 * clock.h
 *
 *  Created on: Sep 21, 2015
 *      Author: jim
 */
#include <inttypes.h>
#ifndef ATMEGA128_CLOCK_H_
#define ATMEGA128_CLOCK_H_

typedef struct
{
		int8_t hours;
		int8_t minutes;
		int8_t seconds;
} clock_t;

void increaseHours(clock_t * clock);
void decreaseHours(clock_t * clock);
void increaseMinutes(clock_t * clock);
void decreaseMinutes(clock_t * clock);
void increaseSeconds(clock_t * clock);
void decreaseSeconds(clock_t * clock);
void clockTick(clock_t * clock);
void clearClock(clock_t * clock);
inline void initClock(clock_t * clock);
#endif /* ATMEGA128_CLOCK_H_ */
