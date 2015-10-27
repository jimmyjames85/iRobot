/*
 * servo.c
 *
 *  Created on: Oct 26, 2015
 *      Author: jim
 */
#include <avr/io.h>
#include "servo.h"
#include "timer.h"
#include "usart/usart.h"
// private
void servo_set_pulse_width(servo_data_t * servo, unsigned int pulse_width)
{

#if ___atmega328p
	 tmr1_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too
	 tmr1_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);


	 //pulse_width = 1.5 * servo->ticks_per_ms;
	 unsigned int top = pulse_width + 20 * servo->ticks_per_ms; // 20ms low time plus the pulse width

	 tmr1_set_output_compare_A_value(top);
	 tmr1_set_output_compare_B_value(top - pulse_width); //This is OC1B which is PB2...
	 //tmr1_set_output_compare_C_value(top - pulse_width); //This is OC1B which is PB2...
	 tmr1_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
	 //tmr1_set_output_compare_C_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);

	 //update servo
	 servo->cur_pulse_width = pulse_width;

#endif
//#elif ___atmega128
/*	tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now
	tmr3_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);
	//unsigned int pulse_width = pulse_width_us * servo->ticks_per_ms / 1000;

	//if (pulse_width > servo->max_pulse_width)
		//pulse_width = servo->max_pulse_width;

	unsigned int top = pulse_width + 20 * servo->ticks_per_ms; // 20ms low time plus the pulse width

	tmr3_set_output_compare_A_value(top);
	tmr3_set_output_compare_B_value(top - pulse_width); //This is OC1B which is PB2...
	tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);

	//update servo
	servo->cur_pulse_width = pulse_width;
//#endif*/


	 tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin for now too
	 tmr3_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);


	 //pulse_width = 1.5 * servo->ticks_per_ms;
	 pulse_width = 1.5 * servo->ticks_per_ms; //250 * 4 us per tick = 1000 us... we want 1.5 ms
	 unsigned int top = pulse_width + 20 * servo->ticks_per_ms; // 20ms low time plus the pulse width

	 printf0("pulse_width: %u", pulse_width);

	 tmr3_set_output_compare_A_value(top);
	 tmr3_set_output_compare_B_value(top - pulse_width); //This is OC1B which is PB2...
	 tmr1_set_output_compare_C_value(top - pulse_width); //This is OC1B which is PB2...
	 tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
	 tmr1_set_output_compare_C_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);

	 //update servo
	 servo->cur_pulse_width = pulse_width;



}

void servo_init(servo_data_t * servo)
{

#if ___atmega328p

	tmr1_set_prescaler(ONE_64TH); //clock 4 us per tick @ 16MHz base
	servo->ticks_per_ms = 250;//ticks per ms
	DDRB |= _BV(PB2);// OC1B setup output for servo
#elif ___atmega128
	tmr3_set_prescaler(ONE_64TH); //clock 8 us per tick @ 8MHz base
	servo->ticks_per_ms = 125;//ticks per ms
	DDRE |= _BV(PE4);// OC3B set up output for servo
	//DDRE |= _BV(PE5);//OC3C
#endif

}

void servo_calibrate(servo_data_t * servo)
{
	//TODO; write calibration app
	servo->cur_pulse_width = 0;
	servo->cal_0_degrees = 100;
	servo->cal_180_degrees = 550;
	servo->pulse_width_single_degree = 2.5;

#if ___atmega128
	servo->cur_pulse_width = 0;
	servo->cal_0_degrees = 10;
	servo->cal_180_degrees = 287;
	servo->pulse_width_single_degree = 1.25;

#endif
}


void servo_increment_degrees(servo_data_t * servo, unsigned int amt)
{
	servo_set_pulse_width(servo, servo->cur_pulse_width + amt*servo->pulse_width_single_degree);
}
void servo_decrement_degrees(servo_data_t * servo, unsigned int amt)
{
	servo_set_pulse_width(servo, servo->cur_pulse_width - amt*servo->pulse_width_single_degree);
}

void servo_set_position(servo_data_t * servo, unsigned int degrees)
{
	long math_holder = degrees*servo->pulse_width_single_degree;//((servo->cal_180_degrees-servo->cal_0_degrees)/180);

	printf0("[%ld]\r\n",math_holder);
	servo_set_pulse_width(servo, servo->cal_0_degrees + math_holder);
}

