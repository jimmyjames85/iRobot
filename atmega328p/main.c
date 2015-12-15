#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

#define BLINK_DELAY_MS 1000
volatile char ch1=0x01;
volatile char ch2=0x02;
volatile char ch3=0x0;

void asm_func();


void setup(void)
{
    usart0_init(0,8,2,USART_PARITY_ODD,1);//2Mbps 0% error
    //usart0_init(207,8,2,USART_PARITY_ODD,1);//9600 0.2% error
}

int main(void)
{
    setup();
    DDRB |= _BV(DDB5);                /* set pin 5 of PORTB for output*/

    while(1)
    {
        ch1=0x01;
        ch2=0x02;
        ch3=0x0;
        //Form feed; new page
        sendChar0(0x0C);//putty clears the terminal when it receives this character :-D

        printf0("ch1=0x%02X ch2=0x%02X ch3=0x%02X \r\n", ch1, ch2, ch3);
        asm_func();
        printf0("ch1=0x%02X ch2=0x%02X ch3=0x%02X \r\n", ch1, ch2, ch3);

        sendString0("ON\r\n");
        //printf0("ON\r\n");
        PORTB |= _BV(PORTB5);        /* set pin 5 high to turn led on */
        _delay_ms(BLINK_DELAY_MS);

        sendString0("\tOFF\r\n");
        //printf0("\tOFF\r\n");
        PORTB &= ~_BV(PORTB5);       /* set pin 5 low to turn led off */
        _delay_ms(BLINK_DELAY_MS);
    }

	return 0; /* CODE SHOULD NEVER REACH HERE */
}
