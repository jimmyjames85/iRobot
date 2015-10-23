/*
 * adc_test.c
 *
 *  Created on: Oct 21, 2015
 *      Author: jim
 */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart/usart.h"
#include <stdlib.h>
#include "util.h"
#include "blue_tooth_HC05.h"
#include "adc.h"

volatile unsigned char infra_has_reading;
volatile unsigned long infra_value;

ISR(ADC_vect)
{
	infra_has_reading = 1;
	infra_value = adc_read_data();
}

//assumes adc is enabled and properlly clocked with correct vref source
unsigned long average_n_readings(unsigned char n)
{
	unsigned char is_isr_set = adc_is_enabled_read_conversion_complete_isr();
	adc_enable_conversion_complete_isr(0);

	char total = 0;
	unsigned long sum = 0;

	while (total < n)
	{
		adc_start_conversion();
		while (adc_read_conversion_complete_flag() == 0)
			; //wait for conversion to complete
		adc_clear_conversion_complete_flag();
		sum += adc_read_data();
		total++;
	}

	if (is_isr_set)
		adc_enable_conversion_complete_isr(1);

	return sum / n;
}

int main(void)
{
	//uno input capture pin is PB0 (uno D8)
	//128 input capture pin is PD4

#if ___atmega328p
	init_USART0(9600, F_CPU);
	adc_set_vref(ADC_AREF);
#elif ___atmega128
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
	adc_set_vref(ADC_INTERNAL_VREF); //~2.5
#endif


	adc_select_channel(2);
	adc_set_prescaler(ADC_ONE_64TH);	//TODO create method find_good_prescaler_for_adc_based_on_f_cpu

	adc_enable(1);
	infra_has_reading = 0;
	infra_value = 0;
	_delay_ms(500);
	printf0("BEGINNING ADC TEST\r\n");
	///////////////////////////////////

	unsigned long avg = average_n_readings(5);
	printf0("\r\nFirst reading = %lu\r\n", avg);
	///////////////////////////////////

	adc_enable_conversion_complete_isr(1);
	sei();
	adc_start_conversion();

	printf0("ADCSRA = %02X\r\n", ADCSRA);
	_delay_ms(1000);
	unsigned long old_value = 0;
	while (1)
	{

		unsigned long avg = average_n_readings(100);
		printf0("Avg reading = %lu\r\n", avg);
		//_delay_ms(200);
		if (infra_has_reading)
		{

			/*
			 //adc_clear_conversion_complete_flag();
			 if (old_value != infra_value)
			 printf0("%lu\r\n", infra_value);

			 old_value = infra_value;
			 _delay_ms(30);
			 adc_start_conversion();
			 */
		}

	}

	return 0; /* CODE SHOULD NEVER REACH HERE */
}

