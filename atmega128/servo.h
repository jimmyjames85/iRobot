/*
 * servo.h
 *
 *  Created on: Oct 26, 2015
 *      Author: jim
 */

#ifndef ATMEGA128_SERVO_H_
#define ATMEGA128_SERVO_H_


typedef struct SERVO_DATA
{
		unsigned int cal_0_degrees;
		unsigned int cal_180_degrees;
		unsigned int cur_pulse_width;
		double pulse_width_single_degree;
		unsigned char ticks_per_ms; //ticks per ms
} servo_data_t;

void servo_calibrate(servo_data_t * servo);
void servo_set_position(servo_data_t * servo, unsigned int degrees);
void servo_increment_degrees(servo_data_t * servo, unsigned int amt);
void servo_decrement_degrees(servo_data_t * servo, unsigned int amt);

#endif /* ATMEGA128_SERVO_H_ */
