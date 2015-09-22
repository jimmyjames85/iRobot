#include <inttypes.h>
#include <avr/io.h>
//#include <stdio.h>

//Pins ( PE0 , PE1 ) = (RX , TX)
//Pins ( PD2 , PD3 ) = (RX , TX)

void setup_onboard_leds(void)
{
	DDRA |= _BV(DDA0); /* set pin 0 of PORTA for output  */
	DDRA |= _BV(DDA1); /* set pin 1 of PORTA for output  (These are on-board leds)*/
}

void set_onboard_led(uint8_t led, uint8_t val)
{
	if (0 == led)
	{
		if (val)
			PORTA |= _BV(PORTA0); /* set pin 0 high to turn led on */
		else
			PORTA &= ~_BV(PORTA0); /* set pin 0 low to turn led off */
	}
	else if (1 == led)
	{
		if (val)
			PORTA |= _BV(PORTA1); /* set pin 0 high to turn led on */
		else
			PORTA &= ~_BV(PORTA1); /* set pin 0 low to turn led off */
	}
}
