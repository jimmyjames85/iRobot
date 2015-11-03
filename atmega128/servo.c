/*
 * servo.c
 *
 *  Created on: Oct 26, 2015
 *      Author: jim
 */
#include <avr/io.h>
#include "servo.h"
#include "timer.h"
#include "usart/usart.h" //TODO this is used for servo_print_servo.... change it to toString???

// private
void servo_set_pulse_width(servo_data_t * servo, unsigned int pulse_width)
{
	unsigned int top = pulse_width + 20 * servo->ticks_per_ms; // 20ms low time plus the pulse width


#if 0 // old ___atmega328p
			tmr1_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too
			tmr1_set_output_compare_A_value(top);
			tmr1_set_output_compare_B_value(top - pulse_width);//This is OC1B which is PB2...
			tmr1_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
			tmr1_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);

			//update servo_t
			servo->cur_pulse_width = pulse_width;//only update if a board is defined
#elif ___atmega328p
			top = pulse_width + 10* servo->ticks_per_ms; // 10ms low time otherwise we overflow
			tmr0_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too
			tmr0_set_output_compare_A_value(top);
			tmr0_set_output_compare_B_value(top - pulse_width);//This is OC0B which is PD5...
			tmr0_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
			tmr0_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);

			//update servo_t
			servo->cur_pulse_width = pulse_width;//only update if a board is defined
#elif ___atmega128

			tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin while we make updates
			tmr3_set_output_compare_A_value(top);
			tmr3_set_output_compare_B_value(top - pulse_width);
			tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
			tmr3_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);

			//update servo_t
			servo->cur_pulse_width = pulse_width;//only update if a board is defined
#endif
//TODO I think we can migrate the set_mode calls to the servo_init function if needed
}

void servo_init(servo_data_t * servo)
{
	//TODO calculate ticks_per_ms using F_CPU
	//clock 4 us per tick @ 16MHz base
	//clock 8 us per tick @ 8MHz base
	//clock 64us per tick @ 16MHz base on tmr0
#if 0 //OLD ___atmega328p
	tmr1_set_prescaler(TIMER_ONE_64TH);
	servo->ticks_per_ms = 250;	//ticks per ms
	DDRB |= _BV(PB2);// OC1B setup output for servo
#elif ___atmega328p
	tmr0_set_prescaler(TIMER_ONE_1024TH);
	servo->ticks_per_ms = 15;	//ticks per millisecond
	DDRD |= _BV(PD5);// OC0B setup output for servo
#elif ___atmega128
	tmr3_set_prescaler(TIMER_ONE_64TH); //clock 8 us per tick @ 8MHz base
	servo->ticks_per_ms = 125;//ticks per ms //TODO use F_CPU
	DDRE |= _BV(PE4);// OC3B set up output for servo
#endif

//	DDRB |= _BV(PB6);// OC1B set up output for servo
//	DDRE |= _BV(PE4);// OC3B set up output for servo
//	DDRE |= _BV(PE5);//OC3C

}

void servo_calibrate(servo_data_t * servo, unsigned cal_0_degrees, unsigned cal_180_degrees)
{
	servo->cur_pulse_width = 0;
	servo->cal_0_degrees = cal_0_degrees;
	servo->cal_180_degrees = cal_180_degrees;
	servo->pulse_width_single_degree = (cal_180_degrees - cal_0_degrees) / 180.0;
}
/*
 void servo_calibrate(servo_data_t * servo)
 {
 //TODO; write calibration app
 #if ___atmega328p
 servo->cur_pulse_width = 0;
 servo->cal_0_degrees = 100;
 servo->cal_180_degrees = 550;
 servo->pulse_width_single_degree = 2.5;

 #elif ___atmega128
 servo->cur_pulse_width = 0;
 servo->cal_0_degrees = 58;
 servo->cal_180_degrees = 280;
 servo->pulse_width_single_degree = 1.23;

 #endif
 }*/

void servo_increment_degrees(servo_data_t * servo, unsigned int amt)
{
	servo_set_position_deg(servo, servo->desired_deg+amt);
}
void servo_decrement_degrees(servo_data_t * servo, unsigned int amt)
{
	servo_set_position_deg(servo, servo->desired_deg-amt);
}

void servo_set_position_deg(servo_data_t * servo, int degrees)
{

	//min degree=0
	if(degrees<0)
		degrees=0;

	//max degree=180
	if(degrees>180)
		degrees=180;

	servo->desired_deg= degrees;

	servo_set_pulse_width(servo, servo->cal_0_degrees + degrees * servo->pulse_width_single_degree);
}

unsigned int servo_get_position_deg(servo_data_t * servo)
{
	return (servo->cur_pulse_width - servo->cal_0_degrees) / servo->pulse_width_single_degree;
}

double servo_calculate_position_deg(servo_data_t * servo)
{
	return ((double)servo->cur_pulse_width - servo->cal_0_degrees) / servo->pulse_width_single_degree;
}

