/*
 * ir_sensor.h
 *
 *  Created on: Nov 1, 2015
 *      Author: jim
 */

#ifndef ___IR_SENSOR_H_
#define ___IR_SENSOR_H_
#include "adc.h"
#include "list.h"
typedef struct
{
		unsigned int voltage :10; //max 1023
		unsigned int dist_mm; //TODO remove mm?? keep units arbitrary
} ir_measurement_t;

ir_measurement_t * new_ir_measurement(unsigned int voltage, unsigned int dist_mm);
list_t * ir_calibrate_lookup_table();
unsigned int ir_lookup_distance(list_t * lookup_table, unsigned voltage);

void ir_init(adc_prescaler_t prescaler, adc_vref_t vref, unsigned char channel);
unsigned ir_read_voltage_avg(unsigned char n);
unsigned ir_read_voltage();
void ir_enable_continous_mode();

#endif /* ___IR_SENSOR_H_ */
