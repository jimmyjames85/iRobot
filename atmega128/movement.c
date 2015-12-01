/*
 * movement.c
 *
 *  Created on: Sep 19, 2015
 *      Author: jim
 */

#include <util/delay.h>
#include <stdlib.h>
#include "open_interface.h"

float distance_traveled_mm(oi_encoder_t * wheel_encoder)
{
	//TODO include revolutions
	float distance = 0.444564997642 * wheel_encoder->encoder_count;
	return distance;
}


float fwd_distance_traveled_mm(oi_t * sensor)
{
	float distance_left = distance_traveled_mm(sensor->left_encoder);
	float distance_right = distance_traveled_mm(sensor->right_encoder);
	return (distance_left + distance_right) / 2;
}

void move_forward_cm(oi_t *sensor, int centimeters)
{
	long target_dist_mm = centimeters * 10;

	oi_set_wheels(0, 0);
	oi_tare_encoders(sensor->left_encoder, sensor->right_encoder);
	if (target_dist_mm <= 0)
		return;

	oi_set_wheels(200, 200);
	while (fwd_distance_traveled_mm(sensor) < target_dist_mm)
	{
		oi_load_sensor_data(sensor);
		_delay_ms(20);
	}

	oi_set_wheels(0, 0);
	oi_load_sensor_data(sensor);
}

void turn_CW(oi_t *sensor, int degrees)
{

	uint8_t speed = 100;
    int16_t one_full_rotation_distance_mm=735;

	if(degrees<=0)
	{
		speed *= -1;
		degrees *= -1;
	}

	oi_set_wheels(0, 0);
	oi_tare_encoders(sensor->left_encoder, sensor->right_encoder);

	if(degrees<=0)
		return;

	float target_rotation_mm = ((float)degrees/360)*one_full_rotation_distance_mm;

	oi_set_wheels(speed, -speed);

	float distance_left = abs(distance_traveled_mm(sensor->left_encoder));
	float distance_right = abs(distance_traveled_mm(sensor->right_encoder));
	float avg = (distance_left + distance_right)/2;

	while(avg < target_rotation_mm )
	{
		oi_load_sensor_data(sensor);
		distance_left = distance_traveled_mm(sensor->left_encoder);
		distance_right = -1*distance_traveled_mm(sensor->right_encoder);
		avg = (distance_left + distance_right)/2;

		if(target_rotation_mm/ avg <= 2)
			oi_set_wheels(speed/2,-speed/2);
		//else if(target_rotation_mm/ avg <= 4)
			//oi_set_wheels((speed/4)*3,(-speed/4)*3);
	}

	oi_set_wheels(0, 0);
	oi_load_sensor_data(sensor);

}

void turn_CCW(oi_t *sensor, int degrees)
{
	if(degrees<=0)
		return;

}

