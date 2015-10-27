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
#include <stdlib.h> /* malloc */
#include "util.h"
#include "blue_tooth_HC05.h"
#include "adc.h"
#include "list.h"

volatile unsigned char infra_has_reading;
volatile unsigned long infra_value;

typedef struct
{
	unsigned int voltage :10;
	unsigned int dist_mm;
} adc_dist_measurement_t; //todo rename to ir measurement

//blocks until data and reads until input is not an integer
unsigned long read_long0()
{
	long ret = 0;

	list_t * digitString = lalloc();
	char keepGoing = 1;
	while (keepGoing)
	{
		char digit = getChar0();
		printf0("%c", digit);
		if (digit >= '0' && digit <= '9')
		{
			ladd(digitString, (void *) newChar(digit));
		}
		else
		{
			keepGoing = 0;
			ladd(digitString, (void *) newChar('\0'));
			printf0("\r\n");
		}
	}

	int i = 0;
	char * str = malloc(sizeof(char) * (digitString->length));
	for (i = 0; i < digitString->length; i++)
		*(str + i) = *((char *) (lget(digitString, i)));

	lfreefree(digitString);
	ret = atol(str);
	free(str);
	return ret;
}

adc_dist_measurement_t * new_measurement(unsigned int voltage, unsigned int dist_mm)
{
	adc_dist_measurement_t * ret;
	ret = (adc_dist_measurement_t *) malloc(sizeof(adc_dist_measurement_t) * 1);
	ret->dist_mm = dist_mm;
	ret->voltage = voltage;
	return ret;
}

ISR(ADC_vect)
{
	infra_has_reading = 1;
	infra_value = adc_read_data();
}

//assumes adc is enabled and properlly clocked with correct vref source
unsigned int average_n_readings(unsigned char n)
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
		_delay_ms(50);
	}

	if (is_isr_set)
		adc_enable_conversion_complete_isr(1);

	return sum / n;
}

int compare_measurements(adc_dist_measurement_t * a, adc_dist_measurement_t * b)
{
	return a->voltage - b->voltage;
}

list_t * test_callibrate_ir()
{
	list_t * measurements = lalloc();
	char keepGoing = 1;

	unsigned long averageCount = 5;

	while (keepGoing)
	{
		lmergesort(measurements, 0, measurements->length - 1, (int (*)(const void*, const void*)) compare_measurements);
		int i;
		for (i = 0; i < measurements->length; i++)
		{
			adc_dist_measurement_t * cur = (adc_dist_measurement_t *) lget(measurements, i);
			printf0("%u, %u\r\n", cur->voltage, cur->dist_mm);
		}

		unsigned long reading = average_n_readings(averageCount);
		printf0("Reading is %lu\r\n", reading);
		printf0("(Q)uit (S)kip (E)nter Distance: \r\n");
		char choice = getChar0();
		if (choice == 'e' || choice == 'E')
		{

			printf0("Enter Reading in mm: \r\n");
			unsigned long dist = (unsigned long) read_long0();
			printf0("You entered: %d mm\r\n", dist);
			adc_dist_measurement_t * measure = new_measurement(reading, dist);
			ladd(measurements, measure);
		}
		if (choice == 'q' || choice == 'Q')
			keepGoing = 0;
		//else skip
	}

	return measurements;

}

//uses ir sensor to test
int main(void)
{

#if ___atmega328p
	init_USART0(9600, F_CPU);
	//adc_set_vref(ADC_AREF);
	adc_set_vref(ADC_INTERNAL_VREF);
#elif ___atmega128
	init_USART0(BLUE_TOOTH_BAUD_RATE, F_CPU);
//adc_set_vref(ADC_INTERNAL_VREF); //~2.5
	adc_set_vref(ADC_AREF);//apply 3.1 volts to ADC_AREF (see )
#endif

	adc_select_channel(2);
//uno adc2 is pc2 (uno a2)
//atmega128 adc2 is pf2

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

	list_t * measurements = lalloc();
	/*
	ladd(measurements, new_measurement(423, 300));
	ladd(measurements, new_measurement(480, 240));
	ladd(measurements, new_measurement(540, 210));
	ladd(measurements, new_measurement(593, 190));
	ladd(measurements, new_measurement(707, 160));
	ladd(measurements, new_measurement(759, 140));
	ladd(measurements, new_measurement(858, 120));
	ladd(measurements, new_measurement(930, 110));
	ladd(measurements, new_measurement(997, 100));*/


	ladd(measurements, new_measurement(118, 360));
ladd(measurements, new_measurement(146, 300));
ladd(measurements, new_measurement(169, 245));
ladd(measurements, new_measurement(199, 215));
ladd(measurements, new_measurement(215, 195));
ladd(measurements, new_measurement(224, 175));
ladd(measurements, new_measurement(245, 165));
ladd(measurements, new_measurement(250, 155));
ladd(measurements, new_measurement(265, 145));
ladd(measurements, new_measurement(279, 135));
ladd(measurements, new_measurement(303, 125));
ladd(measurements, new_measurement(327, 115));
ladd(measurements, new_measurement(370, 105));
ladd(measurements, new_measurement(394, 95));
ladd(measurements, new_measurement(409, 85));
ladd(measurements, new_measurement(457, 75));
ladd(measurements, new_measurement(522, 65));
ladd(measurements, new_measurement(562, 55));
ladd(measurements, new_measurement(696, 45));
ladd(measurements, new_measurement(912, 35));
ladd(measurements, new_measurement(1019, 27));


	/*
118, 360
146, 300
169, 245
199, 215
215, 195
224, 175
245, 165
250, 155
265, 145
279, 135
303, 125
327, 115
370, 105
394, 95
409, 85
457, 75
522, 65
562, 55
696, 45
912, 35
1019, 27

	 */

	printf0("measurments->length = %u\r\n", measurements->length);

	lmergesort(measurements, 0, measurements->length - 1, (int (*)(const void *, const void *)) compare_measurements);

	adc_dist_measurement_t reading;
	reading.voltage = 707;
	printf0("searching for 707 at...\r\n");
	printf0("found it at: %d\r\n", lbinsearch(measurements, &reading, 0, measurements->length - 1, (int (*)(const void*, const void*)) compare_measurements));

	while (1)
	{

		reading.voltage = average_n_readings(1);
		int loc = lbinsearch(measurements, &reading, 0, measurements->length - 1, (int (*)(const void*, const void*)) compare_measurements);
		if (loc == -1 || loc >= measurements->length)
			printf0("Error? loc=%d\r\n voltage=%u", loc, reading.voltage);
		else
		{
			adc_dist_measurement_t * ref = lget(measurements, loc);
			unsigned int calculatedDist = ref->dist_mm;

			printf0("readingVolt: %u, refVolt: %u, loc: %d      ", reading.voltage,ref->voltage,loc);
			if (reading.voltage < ref->voltage)
			{
				if (loc == 0)
					calculatedDist = ref->dist_mm;
				else
				{
					adc_dist_measurement_t * secondRef = lget(measurements, loc - 1);
					double percentage = (reading.voltage - secondRef->voltage) / (ref->voltage - secondRef->voltage);
					calculatedDist = ref->dist_mm + (secondRef->dist_mm - ref->dist_mm)*(reading.voltage - secondRef->voltage) / (ref->voltage - secondRef->voltage);
				}
			}
			else if (reading.voltage == ref->voltage)
				calculatedDist = ref->dist_mm;
			else
			{
				if (loc >= measurements->length - 1)
					calculatedDist = ref->dist_mm;
				else
				{
					adc_dist_measurement_t * secondRef = lget(measurements, loc + 1);
					//double percentage = (secondRef->voltage - reading.voltage) / (secondRef->voltage - ref->voltage);
					calculatedDist = ref->dist_mm -  (ref->dist_mm - secondRef->dist_mm)*(secondRef->voltage - reading.voltage) / (secondRef->voltage - ref->voltage);
				}
			}
			printf0("calculatedDist = %u\r\n", calculatedDist);

		}

		if (isAvailable0() && getChar0() == 'r')
		{
			printf0("Recalibrate now:\r\n\r\n");
			lfreefree(measurements);
			measurements = test_callibrate_ir();
		}

		_delay_ms(500);
	}

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

