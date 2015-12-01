/*
 * servo.c
 *
 *  Created on: Oct 26, 2015
 *      Author: jim
 */
#include <avr/io.h>
#include <stdlib.h>
#include "servo.h"
#include "timer.h"

void servo_set_pulse_width(servo_data_t *servo, unsigned int pulse_width)
{
    unsigned int top = pulse_width + 20 * servo->ticks_per_ms; // 20ms low time plus the pulse width

    //tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_DISCONNECTED); //disconnect pin while we make updates
    tmr3_set_output_compare_A_value(top);
    tmr3_set_output_compare_B_value(top - pulse_width);
    tmr3_set_output_compare_B_mode(TIMER_COMPARE_OUTPUT_SET_HIGH);
    tmr3_set_mode(TIMER_MODE_PWM_FAST_OCRnA_TOP);

    //update servo_t
    servo->cur_pulse_width = pulse_width;

//TODO I think we can migrate the set_mode calls to the servo_init function if needed
}

void servo_init(servo_data_t *servo)
{
    tmr3_set_prescaler(TIMER_ONE_64TH);
    //clock 4 us per tick @ 16MHz base
    // -- or --
    //clock 8 us per tick @ 8MHz base
    servo->ticks_per_ms = 64000000L / F_CPU;//ticks per ms
    DDRE |= _BV(PE4);// OC3B set up output for servo

    servo_set_pulse_width(servo, 0);//disconnect
}

inline servo_data_t *newServo()
{
    return malloc(sizeof(servo_data_t));
}
inline void freeServo(servo_data_t * servo)
{
    free(servo);
}
void servo_calibrate(servo_data_t *servo, unsigned cal_0_degrees, unsigned cal_180_degrees)
{
    servo->cur_pulse_width = 0;
    servo->cal_0_degrees = cal_0_degrees;
    servo->cal_180_degrees = cal_180_degrees;
    servo->pulse_width_single_degree = (cal_180_degrees - cal_0_degrees) / 180.0;
}

inline void servo_increment_degrees(servo_data_t *servo, unsigned int amt)
{
    servo_set_position_deg(servo, servo->desired_deg + amt);
}

inline void servo_decrement_degrees(servo_data_t *servo, unsigned int amt)
{
    servo_set_position_deg(servo, servo->desired_deg - amt);
}

void servo_set_position_deg(servo_data_t *servo, int degrees)
{
    servo->desired_deg = degrees;

    //min degree=0     //max degree=180
    if (degrees < 0 || degrees > 180)
        servo_set_pulse_width(servo, 0);//disconnect
    else
        servo_set_pulse_width(servo, servo->cal_0_degrees + degrees * servo->pulse_width_single_degree);
}

inline double servo_calculate_position_deg(servo_data_t *servo)
{
    return ((double) servo->cur_pulse_width - servo->cal_0_degrees) / servo->pulse_width_single_degree;
}

