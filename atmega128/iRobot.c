/*
 * iRobot.c
 *
 *  Created on: Oct 28, 2015
 *      Author: jim
 */
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart/usart.h"
#include <stdlib.h>
#include "timer.h"
#include "list.h"
#include "adc.h"
#include "util.h"
#include "blue_tooth_HC05.h"
#include "servo.h"
#include "ping.h"

void servo_test()
{
	servo_data_t rservo; //fake malloc
	servo_data_t * servo = &rservo;

	servo_init(servo);

	servo_calibrate(servo, 8, 35);
	servo_set_position_deg(servo, 90);

	while (1)
	{
		if (isAvailable0())
		{
			char c = getChar0();
			switch (c)
			{
				case '0':
					servo_set_position_deg(servo, 0);
				break;
				case '4':
					servo_set_position_deg(servo, 45);
				break;
				case '9':
					servo_set_position_deg(servo, 90);
				break;
				case '3':
					servo_set_position_deg(servo, 135);
				break;
				case '8':
					servo_set_position_deg(servo, 180);
				break;
				case '+':
					servo_increment_degrees(servo, 1);
				break;
				case '-':
					servo_decrement_degrees(servo, 1);
				break;
				default:
				break;
			}

			double calcDeg = servo_calculate_position_deg(servo);
			char buff[300];
			ftoa(buff, calcDeg);

			printf0("Cur pulse width: %u\t deg: %u\t calcDeg: %s\r\n", servo->cur_pulse_width, servo->desired_deg, buff);
		}
	}
}

volatile unsigned int first_wheel_hit;
volatile unsigned int second_wheel_hit;
volatile unsigned char hit_count = 0;
volatile unsigned long overflows = 0;
volatile unsigned long overflow_overflows = 0;

void input_capture_test(void)
{

	//setup timer1
	tmr1_set_mode(TIMER_MODE_NORMAL_MAX_TOP);
	tmr1_set_prescaler(TIMER_ONE_64TH); //set timer tick rate to 250KHz each tick = 4us
	tmr1_enable_input_capture_isr(1);
	tmr1_set_input_capture_edge(1);

	tmr1_enable_overflow_isr(1);

	unsigned int rising_edge_time;

	DDRD &= ~_BV(DDD4);
	while (!(PIND |= _BV(DDD4)))
	{
		rising_edge_time = tmr1_read_count();
		while ((PIND |= _BV(DDD4)))
			; //wait for low
	}

	DDRB |= _BV(PB0);
	sei();
	while (1)
	{

		while (2 != hit_count)
		{
			printf0("%d\r\n", hit_count);
			if (hit_count == 1)
			{
				//_delay_ms(500);
				tmr1_enable_input_capture_isr(1);
			}

		}
		tmr1_enable_input_capture_isr(1);

		long diff = ((overflows << 16) | second_wheel_hit) - first_wheel_hit;
		double seconds = (double) diff / 250000;
		double velocity = 1.5 / seconds;

		double velocity_v = velocity / 2;
		double velocity_h = velocity_v;
		double time_in_air = velocity_v / 4.9;
		double distance_h = time_in_air * velocity_h;

		char buff[300];
		ftoa(buff, seconds);
		printf0("(%d) %s seconds\r\n", hit_count, buff);
		hit_count = 0;
		if (overflow_overflows)
			printf0("overflow_overflows = %lu\r\n", overflow_overflows);
		overflows = 0;
		overflow_overflows = 0;
		_delay_ms(1000);
	}
}

void doPingLoop()
{
	timer_prescaler_t prescaler = TIMER_ONE_1024TH;
	ping_init(prescaler);

	while (1)
	{
		unsigned cm = ping_cm(prescaler);
		printf0("%d cm\r\n", cm);
	}
}

list_t * create_jims_ping_sensor_lookup_table()
{
	list_t * lookup_table = lalloc();
	ladd(lookup_table, (void *) ir_new_measurement(118, 360));
	ladd(lookup_table, (void *) ir_new_measurement(146, 300));
	ladd(lookup_table, (void *) ir_new_measurement(169, 245));
	ladd(lookup_table, (void *) ir_new_measurement(199, 215));
	ladd(lookup_table, (void *) ir_new_measurement(215, 195));
	ladd(lookup_table, (void *) ir_new_measurement(224, 175));
	ladd(lookup_table, (void *) ir_new_measurement(245, 165));
	ladd(lookup_table, (void *) ir_new_measurement(250, 155));
	ladd(lookup_table, (void *) ir_new_measurement(265, 145));
	ladd(lookup_table, (void *) ir_new_measurement(279, 135));
	ladd(lookup_table, (void *) ir_new_measurement(303, 125));
	ladd(lookup_table, (void *) ir_new_measurement(327, 115));
	ladd(lookup_table, (void *) ir_new_measurement(370, 105));
	ladd(lookup_table, (void *) ir_new_measurement(394, 95));
	ladd(lookup_table, (void *) ir_new_measurement(409, 85));
	ladd(lookup_table, (void *) ir_new_measurement(457, 75));
	ladd(lookup_table, (void *) ir_new_measurement(522, 65));
	ladd(lookup_table, (void *) ir_new_measurement(562, 55));
	ladd(lookup_table, (void *) ir_new_measurement(696, 45));
	ladd(lookup_table, (void *) ir_new_measurement(912, 35));
	ladd(lookup_table, (void *) ir_new_measurement(1019, 27));
	return lookup_table;
}

void doIrLoop()
{
	//adc_set_vref(ADC_INTERNAL_VREF);//apply 3.1 volts to ADC_AREF (see )
	//TODO setup where to get ref voltage from
	//TODO create method find_good_prescaler_for_adc_based_on_f_cpu

	//TODO this table is for Jim's IR sensor with 3V applied to AREF
	//TODO this table's distance values is in inches * 10 (e.g. 36" = 360)
	//TODO convert to mm or cm
	list_t * lookup_table = create_jims_ping_sensor_lookup_table();

	ir_init(ADC_ONE_64TH, ADC_AREF, 2);
	while (1)
	{
		unsigned voltage = ir_read_voltage_avg(5);
		unsigned calculatedDist = ir_lookup_distance(lookup_table, voltage);
		printf0("%d volts \t\t calculatedDist= %u\r\n", voltage, calculatedDist);
	}
}
void doSweepLoop()
{
	servo_data_t rservo; //fake malloc
	servo_data_t * servo = &rservo;

	servo_init(servo);
	servo_calibrate(servo, 8, 35);
	servo_set_position_deg(servo, 90);

	list_t * lookup_table = create_jims_ping_sensor_lookup_table();

	ir_init(ADC_ONE_64TH, ADC_AREF, 2);

	timer_prescaler_t prescaler = TIMER_ONE_1024TH;
	ping_init(prescaler);

	while (1)
	{
		char c = '\0';
		if (isAvailable0())
		{
			c = getChar0();
			switch (c)
			{
				case '0':
					servo_set_position_deg(servo, 0);
				break;
				case '4':
					servo_set_position_deg(servo, 45);
				break;
				case '9':
					servo_set_position_deg(servo, 90);
				break;
				case '3':
					servo_set_position_deg(servo, 135);
				break;
				case '8':
					servo_set_position_deg(servo, 180);
				break;
				case '+':
					servo_increment_degrees(servo, 4);
				break;
				case '-':
					servo_decrement_degrees(servo, 4);
				break;
				default:
				break;
			}

			_delay_ms(200);//Wait for servo to move before reading
			unsigned voltage = ir_read_voltage_avg(5);
			unsigned calculatedDist = ir_lookup_distance(lookup_table, voltage);

			_delay_ms(20);
			unsigned cm = ping_cm(prescaler);
			double ir_cm = calculatedDist*0.254;
			char buff[20];
			ftoa(buff, ir_cm);
			unsigned curDeg = servo_calculate_position_deg(servo);
			//printf0("%d volts \t\t calculatedDist= %u \r\n", voltage, calculatedDist);
			printf0("%u \t%d p_cm\t  %s ir_cm\r\n",curDeg, cm, buff);
		}

	}
}

int main(void)
{
#if ___atmega328p
	init_USART0(9600, F_CPU);
//	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
#elif ___atmega128
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
#endif

	//input_capture_test();
	doSweepLoop();
	doPingLoop();

	doIrLoop();

	servo_test();

}
