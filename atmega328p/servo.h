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
		unsigned char desired_deg; // 0 to 180
} servo_data_t;

//void servo_calibrate(servo_data_t * servo);
void servo_init(servo_data_t * servo);
void servo_calibrate(servo_data_t * servo, unsigned cal_0_degrees, unsigned cal_180_degrees);
void servo_set_position_deg(servo_data_t * servo, int degrees);
unsigned int servo_get_position_deg(servo_data_t * servo);
double servo_calculate_position_deg(servo_data_t * servo);
void servo_increment_degrees(servo_data_t * servo, unsigned int amt);
void servo_decrement_degrees(servo_data_t * servo, unsigned int amt);
#endif /* ATMEGA128_SERVO_H_ */
