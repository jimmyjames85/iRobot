/*
 * ir_sensor.c
 *
 *  Created on: Nov 1, 2015
 *      Author: jim
 */

#include <util/delay.h>
#include <stdlib.h> /* malloc */
#include "ir_sensor.h"
#include "usart.h"

#include "util.h" //TODO <----------------------- REMOVE ---------------


//private
int compare_measurements(ir_measurement_t * a, ir_measurement_t * b)
{
	return a->voltage - b->voltage;
}

//blocks until data and reads until input is not an integer
//private
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
/**
 * creates and returns a sorted list of ir_measurement_t readings
 * using usart0 to interact with the user
 *
 * NOTE: the returned list must be freed using lfreefree when finished
 */
list_t * ir_calibrate_lookup_table()
{
	list_t * measurements = lalloc();
	char keepGoing = 1;

	unsigned long averageCount = 5;

	while (keepGoing)
	{
		//We don't "need" to sort every time, but it's nice to display the sorted data during data entry
		lmergesort(measurements, 0, measurements->length - 1, (int (*)(const void*, const void*)) compare_measurements);
		int i;
		for (i = 0; i < measurements->length; i++)
		{
			ir_measurement_t * cur = (ir_measurement_t *) lget(measurements, i);
			printf0("%u, %u\r\n", cur->voltage, cur->dist_mm);
		}

		unsigned long reading = ir_read_voltage_avg(averageCount);
		printf0("Reading is %lu\r\n", reading);
		printf0("(Q)uit (S)kip (E)nter Distance: \r\n");
		char choice = getChar0();
		if (choice == 'e' || choice == 'E')
		{

			printf0("Enter Reading in mm: \r\n");
			unsigned long dist = (unsigned long) read_long0();
			printf0("You entered: %d mm\r\n", dist);
			ir_measurement_t * measure = new_ir_measurement(reading, dist);
			ladd(measurements, measure);
		}
		if (choice == 'q' || choice == 'Q')
			keepGoing = 0;
		//else skip
	}

	return measurements;
}

ir_measurement_t * new_ir_measurement(unsigned int voltage, unsigned int dist_mm)
{
	ir_measurement_t * ret;
	ret = (ir_measurement_t *) malloc(sizeof(ir_measurement_t) * 1);
	ret->dist_mm = dist_mm;
	ret->voltage = voltage;
	return ret;
}

void ir_init(adc_prescaler_t prescaler, adc_vref_t vref, unsigned char channel)
{
	adc_set_prescaler(prescaler); //ADC_ONE_64TH
	adc_set_vref(vref);
	adc_select_channel(channel);
	adc_enable(1);
}

unsigned ir_read_voltage_avg(unsigned char n)
{
	char total = 0;
	unsigned long sum = 0;
	while (total < n)
	{
		sum += ir_read_voltage();
		total++;
		_delay_ms(50);
	}
	return sum / n;
}

unsigned ir_read_voltage()
{
	if(adc_is_enabled_free_running_mode())
		return adc_read_data();

	adc_start_conversion();
	while (adc_read_conversion_complete_flag() == 0)
		; //wait for conversion to complete
	adc_clear_conversion_complete_flag();
	return adc_read_data();
}

void ir_enable_continous_mode()
{
	//adc_enable_conversion_complete_isr(1);
	adc_enable_free_running_mode(1);
	adc_start_conversion();
}

/**
 * lookup_table - should be a list_t of ir_measurement_t values
 */
unsigned int ir_lookup_distance(list_t * lookup_table, unsigned voltage)
{
	unsigned int calculatedDist = -1;
	ir_measurement_t reading; //used for binary search

	reading.voltage = voltage;
	int loc = lbinsearch(lookup_table, &reading, 0, lookup_table->length - 1, (int (*)(const void*, const void*)) compare_measurements);

	if (loc == -1 || loc >= lookup_table->length)
		return calculatedDist;
	else
	{
		ir_measurement_t * ref = lget(lookup_table, loc);
		calculatedDist = ref->dist_mm;

		//printf0("readingVolt: %u, refVolt: %u, loc: %d      ", reading.voltage,ref->voltage,loc);
		if (reading.voltage < ref->voltage)
		{
			if (loc == 0)
				calculatedDist = ref->dist_mm;
			else
			{
				ir_measurement_t * secondRef = lget(lookup_table, loc - 1);
				double percentage = (reading.voltage - secondRef->voltage) / (ref->voltage - secondRef->voltage);
				calculatedDist = ref->dist_mm + (secondRef->dist_mm - ref->dist_mm) * (reading.voltage - secondRef->voltage) / (ref->voltage - secondRef->voltage);
			}
		}
		else if (reading.voltage == ref->voltage)
			calculatedDist = ref->dist_mm;
		else
		{
			if (loc >= lookup_table->length - 1)
				calculatedDist = ref->dist_mm;
			else
			{
				ir_measurement_t * secondRef = lget(lookup_table, loc + 1);
				//double percentage = (secondRef->voltage - reading.voltage) / (secondRef->voltage - ref->voltage);
				calculatedDist = ref->dist_mm - (ref->dist_mm - secondRef->dist_mm) * (secondRef->voltage - reading.voltage) / (secondRef->voltage - ref->voltage);
			}
		}
		//printf0("calculatedDist = %u\r\n", calculatedDist);
	}
	return calculatedDist;
}
