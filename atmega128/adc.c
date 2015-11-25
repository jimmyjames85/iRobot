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
        case ADC_INTERNAL_VREF_RESERVED:
            ADMUX = _BV(REFS1) | (ADMUX & ~(_BV(REFS0)));
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

unsigned char adc_is_enabled_free_running_mode()
{
    //pg 244
    return ADCSRA & _BV(ADFR);
}

void adc_enable_free_running_mode(char enable_bool)
{
    //pg 244
    if (enable_bool)
        ADCSRA |= _BV(ADFR); //enable free running mode
    else
        ADCSRA &= ~_BV(ADFR);//disable free running mode
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
    if (ADMUX & _BV(ADLAR))
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
