/*
 * adc.h
 *
 *  Created on: Oct 21, 2015
 *      Author: jim
 */

#ifndef ATMEGA128_ADC_H_
#define ATMEGA128_ADC_H_

typedef enum ADC_PRESCALER
{
	//note ONE_HALF and ONE_2NDTH are the same prescaler
	ADC_ONE_HALF = 0x00, ADC_ONE_2NDTH=0x01, ADC_ONE_4TH = 0x02, ADC_ONE_8TH = 0x03, ADC_ONE_16TH = 0x04, ADC_ONE_32NDTH = 0x05, ADC_ONE_64TH = 0x06, ADC_ONE_128TH = 0x07
} adc_prescaler_t;

typedef enum ADC_AREF_SELECTION
{
	ADC_AREF, //0 0 AREF, Internal Vref turned off
	ADC_AVCC, //0 1 AVCC with external capacitor at AREF pin
	ADC_INTERNAL_VREF //1 1 Internal Voltage Reference with external capacitor at AREF pin
} adc_vref_t;

void adc_set_vref(adc_vref_t vref_setting);
void adc_select_channel(unsigned char channel);
void adc_set_prescaler(adc_prescaler_t prescaler);
void adc_enable(char enable_bool);
void adc_start_conversion();
void adc_clear_conversion_complete_flag();
unsigned char adc_read_conversion_complete_flag();
void adc_enable_conversion_complete_isr(char enable_bool);
unsigned char adc_is_enabled_read_conversion_complete_isr();
void adc_enable_free_running_mode(char enable_bool);
void adc_enable_left_adjust(char enable_bool);
unsigned int adc_read_data();
#endif /* ATMEGA128_ADC_H_ */
