/*
 * adc.c
 *
 *  Created on: Oct 21, 2015
 *      Author: jim
 */

#include "adc.h"
#include <avr/io.h>
void adc_set_vref(adc_vref_t vref_setting)
{
	switch (vref_setting)
	{
		case ADC_AREF:
			ADMUX &= ~(_BV(REFS1) | _BV(REFS0));
		break;
		case ADC_AVCC:
			ADMUX = (ADMUX & ~(_BV(REFS1))) | _BV(REFS0);
		break;
		case ADC_INTERNAL_VREF_RESERVED: //1 0 (ATMEGA 2560 Internal Voltage Reference 1.1V
			ADMUX =  _BV(REFS1) | (ADMUX & ~(_BV(REFS0)));
		break;
		case ADC_INTERNAL_VREF:
			ADMUX |= _BV(REFS1) | _BV(REFS0);
		break;
		default:
		break;
	}

}

void adc_select_channel(unsigned char channel)
{
	//TODO implement MUX5 for atmega2560
	//no gain options implemented

	if (channel > 7)
		return;
	ADMUX = (ADMUX & 0xE0) | channel;

}

void adc_enable(char enable_bool)
{
	if (enable_bool)
		ADCSRA |= _BV(ADEN);
	else
		ADCSRA &= ~_BV(ADEN);
}

void adc_set_prescaler(adc_prescaler_t prescaler)
{
	ADCSRA = (ADCSRA & 0xF8) | prescaler;
}

void adc_start_conversion()
{
	ADCSRA |= _BV(ADSC);
}

void adc_clear_conversion_complete_flag()
{
	ADCSRA |= _BV(ADIF); /* writing a one to this clear's it*/
}

unsigned char adc_read_conversion_complete_flag()
{
	if (ADCSRA & _BV(ADIF))
		return 1;
	return 0;
}

void adc_enable_conversion_complete_isr(char enable_bool)
{
	if (enable_bool)
		ADCSRA |= _BV(ADIE);
	else
		ADCSRA &= ~_BV(ADIE);
}

unsigned char adc_is_enabled_read_conversion_complete_isr()
{
	return ADCSRA & _BV(ADIE);
}

void adc_enable_free_running_mode(char enable_bool)
{
#if ___atmega2560 // meag adk

	if(enable_bool)
	{
		/*
		 * Bit 7 – Res: Reserved Bit
		 * This bit is reserved for future use.
		 * To ensure compatibility with future devices, this bit
		 * must be written to zero when ADCSRB is written.
		 */

	     //ADTS2:0 = 0b000   ==>  Free running mode

		ADCSRB &= ~(_BV(7) | _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0)); //set auto trigger to free running mode
		ADCSRA |= _BV(ADATE);//enable auto trigger
	}
	else
	ADCSRA &= ~_BV(ADATE); //disable auto trigger


#elif ___atmega328p 	//uno	pg264
	if(enable_bool)
	{
		/*
		 * Bit 7, 5:3 – Res: Reserved Bits
		 * These bits are reserved for future use.
		 * To ensure compatibility with future devices, these bist [sic]
		 * must be written to zero when ADCSRB is written.
		 */
	     //ADTS2:0 = 0b000   ==>  Free running mode

		ADCSRB &= ~(_BV(7) | _BV(5) | _BV(4) | _BV(3) | _BV(ADTS2) | _BV(ADTS1) | _BV(ADTS0)); //set auto trigger to free running mode
		ADCSRA |= _BV(ADATE);//enable auto trigger
	}
	else
	ADCSRA &= ~_BV(ADATE); //disable auto trigger

#elif ___atmega128	//pg 244
	if(enable_bool)
	ADCSRA |= _BV(ADFR); //enable free running mode
	else
	ADCSRA &= ~_BV(ADFR);//disable free running mode
#endif
}
void adc_enable_left_adjust(char enable_bool)
{
	if (enable_bool)
		ADMUX |= _BV(ADLAR);
	else
		ADMUX &= ~_BV(ADLAR);
}

unsigned int adc_read_data()
{
	unsigned int data;
	if ( ADMUX & _BV(ADLAR))
	{
		//left adjusted
		data = ADCL >> 6;
		data |= (ADCH << 2);
	}
	else
	{
		//right adjusted
		data = ADCL;
		data |= (ADCH << 8);
	}
	return data;
}

adc_prescaler_t calculate_prescaler(unsigned long f_cpu)
{
	//TODO

	return -1;
	long max_freq = f_cpu / 50000;
	long min_freq = f_cpu / 200000;

	//max_freq
	/*
	 * AVR ADC must be clocked at frequency between 50 and 200kHz.
	 * So we need to set proper prescaller bits so that scaled system
	 * clock would fit in this range. As our AVR is clocked at 16MHz
	 * we are going to use 128 scaling factor by setting ADPS0, ADPS1
	 * and ADPS2 bits in ADCSRA register.
	 * This gives 16000000/128=125kHz of ADC clock.
	 */
}
