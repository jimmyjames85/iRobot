/*
 * clock.c
 *
 *  Created on: Sep 21, 2015
 *      Author: jim
 */

#include "clock.h"

void increaseHours(clock_t * clock)
{
	clock->hours = (clock->hours + 1) % 24;
}
void decreaseHours(clock_t * clock)
{
	if (clock->hours>0)
		clock->hours = (clock->hours - 1) % 24;
	else
		clock->hours = 23;
}
void increaseMinutes(clock_t * clock)
{
	clock->minutes = (clock->minutes + 1) % 60;
}
void decreaseMinutes(clock_t * clock)
{
	if (clock->minutes>0)
		clock->minutes = (clock->minutes - 1) % 60;
	else
		clock->minutes = 59;
}
void increaseSeconds(clock_t * clock)
{
	clock->seconds = (clock->seconds + 1) % 60;
}
void decreaseSeconds(clock_t * clock)
{
	if (clock->seconds>0)
		clock->seconds = (clock->seconds - 1) % 60;
	else
		clock->seconds = 59;
}

void clockTick(clock_t * clock)
{
  increaseSeconds(clock);

  // if we've rolled over another minute
  if(clock->seconds == 0)
  {

    increaseMinutes(clock);
    // if we've rolled over another hour
    if(clock->minutes==0)
      increaseHours(clock);
  }
}

void clearClock(clock_t * clock)
{
	clock->hours = 0;
	clock->minutes = 0;
	clock->seconds = 0;
}
inline void initClock(clock_t * clock)
{
	clearClock(clock);
}

