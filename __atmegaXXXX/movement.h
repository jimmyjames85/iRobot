/*
 * movement.h
 *
 *  Created on: Sep 19, 2015
 *      Author: jim
 */

#ifndef ATMEGA128_MOVEMENT_H_
#define ATMEGA128_MOVEMENT_H_


void move_forward_cm(oi_t *sensor, int centimeters);
void turn_CW(oi_t *sensor, int degrees);
void turn_CCW(oi_t *sensor, int degrees);

#endif /* ATMEGA128_MOVEMENT_H_ */
