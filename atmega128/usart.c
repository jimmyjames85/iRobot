/*
 * usart.c
 *
 *  Created on: May 30, 2015
 *      Author: jim
 */

#include <avr/io.h>
#include <stdarg.h>
#include <stdio.h>
#include "usart.h"

void usart0_init(unsigned long UBRR_value, unsigned char frame_size, unsigned char stop_bits,
                 usart_parity_mode_t parity_mode, unsigned char enable_U2X)
{
    /**
     *  See page 183    http://www.atmel.com/Images/doc8161.pdf  Uno
     *  See page 211    http://www.atmel.com/images/doc2549.pdf  Mega_Adk
     *  See page 175    http://www.atmel.com/Images/doc2467s.pdf  ATmega128
     */

    /* Disable receiver and transmitter while we make changes*/
    UCSR0B &= ~(_BV(RXEN0) | _BV(TXEN0));

    if (enable_U2X)
        UCSR0A |= _BV(U2X0);

    /* Set baud rate */
    UBRR0H = (unsigned char) (UBRR_value >> 8);
    UBRR0L = (unsigned char) UBRR_value;

    switch (parity_mode)
    {
        case USART_PARITY_DISABLED:
            UCSR0C &= ~(_BV(UPM01) | _BV(UPM00));
            break;
        case USART_PARITY_RESERVED:
            UCSR0C = (UCSR0C & ~_BV(UPM01)) | _BV(UPM00);
            break;
        case USART_PARITY_EVEN:
            UCSR0C = _BV(UPM01) | (UCSR0C & ~_BV(UPM00));
            break;
        case USART_PARITY_ODD:
            UCSR0C |= _BV(UPM01) | _BV(UPM00);
            break;
        default:
            break;
    }

    if (stop_bits == 2)
        UCSR0C |= _BV(USBS0);
    else
        UCSR0C &= ~_BV(USBS0);

    ///frame size
    UCSR0B &= ~_BV(UCSZ02); //TURN off for all frame sizes except for 9-bit
    switch (frame_size)
    {
        case 5:
            UCSR0C &= ~(_BV(UCSZ01) | _BV(UCSZ00));
            break;
        case 6:
            UCSR0C = (UCSR0C & ~_BV(UCSZ01)) | _BV(UCSZ00);
            break;
        case 7:
            UCSR0C = _BV(UCSZ01) | (UCSR0C & ~_BV(UCSZ00));
            break;
        case 8:
            UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
            break;
        case 9:
            UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
            UCSR0B |= _BV(UCSZ02);
            break;
        default:
            break;
    }

    /* Enable receiver and transmitter */
    UCSR0B |= _BV(RXEN0) | _BV(TXEN0);

}

void usart1_init(unsigned long UBRR_value, unsigned char frame_size, unsigned char stop_bits,
                 usart_parity_mode_t parity_mode, unsigned char enable_U2X)
{

    /**
     *  See page 183    http://www.atmel.com/Images/doc8161.pdf  Uno
     *  See page 211    http://www.atmel.com/images/doc2549.pdf  Mega_Adk
     *  See page 175    http://www.atmel.com/Images/doc2467s.pdf  ATmega128
     */

    /* Disable receiver and transmitter while we make changes*/
    UCSR1B &= ~(_BV(RXEN1) | _BV(TXEN1));

    if (enable_U2X)
        UCSR1A |= _BV(U2X1);

    /* Set baud rate */
    UBRR1H = (unsigned char) (UBRR_value >> 8);
    UBRR1L = (unsigned char) UBRR_value;

    switch (parity_mode)
    {
        case USART_PARITY_DISABLED:
            UCSR1C &= ~(_BV(UPM11) | _BV(UPM10));
            break;
        case USART_PARITY_RESERVED:
            UCSR1C = (UCSR1C & ~_BV(UPM11)) | _BV(UPM10);
            break;
        case USART_PARITY_EVEN:
            UCSR1C = _BV(UPM11) | (UCSR1C & ~_BV(UPM10));
            break;
        case USART_PARITY_ODD:
            UCSR1C |= _BV(UPM11) | _BV(UPM10);
            break;
        default:
            break;
    }

    if (stop_bits == 2)
        UCSR1C |= _BV(USBS1);
    else
        UCSR1C &= ~_BV(USBS1);

    ///frame size
    UCSR1B &= ~_BV(UCSZ12); //TURN off for all frame sizes except for 9-bit
    switch (frame_size)
    {
        case 5:
            UCSR1C &= ~(_BV(UCSZ11) | _BV(UCSZ10));
            break;
        case 6:
            UCSR1C = (UCSR1C & ~_BV(UCSZ11)) | _BV(UCSZ10);
            break;
        case 7:
            UCSR1C = _BV(UCSZ11) | (UCSR1C & ~_BV(UCSZ10));
            break;
        case 8:
            UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10);
            break;
        case 9:
            UCSR1C |= _BV(UCSZ11) | _BV(UCSZ10);
            UCSR1B |= _BV(UCSZ12);
            break;
        default:
            break;
    }

    /* Enable receiver and transmitter */
    UCSR1B |= _BV(RXEN1) | _BV(TXEN1);
}

uint8_t getChar0(void)
{
    /**
     *  USART_Receive(void)
     *  See page 187 - 188		http://www.atmel.com/images/doc8161.pdf  Uno
     *  See page 215 - 216		http://www.atmel.com/images/doc2549.pdf  Mega_Adk
     *  See page 180 - 181      http://www.atmel.com/Images/doc2467s.pdf  ATmega128
     */

    /* Wait for data to be received */
    while (!(UCSR0A & (1 << RXC0)));

    /* Get and return received data from buffer */
    return UDR0;
}

uint8_t getChar1(void)
{
    /**
     *  USART_Receive(void)
     *  See page 187 - 188		http://www.atmel.com/images/doc8161.pdf  Uno
     *  See page 215 - 216		http://www.atmel.com/images/doc2549.pdf  Mega_Adk
     *  See page 180 - 181      http://www.atmel.com/Images/doc2467s.pdf  ATmega128
     */

    /* Wait for data to be received */
    while (!(UCSR1A & (1 << RXC1)));

    /* Get and return received data from buffer */
    return UDR1;
}

uint8_t isAvailable1()
{
    return (UCSR1A & (1 << RXC1));
}

uint8_t isAvailable0()
{
    return (UCSR0A & (1 << RXC0));
}

void sendChar1(uint8_t data)
{
    /**
     *  USART_Transmit(unsigned char data)
     *  See page 184      http://www.atmel.com/Images/doc8161.pdf  Uno
     *  See page 212	  http://www.atmel.com/images/doc2549.pdf  Mega_Adk
     *  See page 177	  http://www.atmel.com/images/doc2467s.pdf  ATmega128 (PE0 = rX , PE1 = tX)
     */

    /* wait for empty transmit buffer */
    while (!(UCSR1A & (1 << UDRE1)));

    /* Put data into buffer, sends the data */
    UDR1 = data;
}

void sendChar0(uint8_t data)
{
    /**
     *  USART_Transmit(unsigned char data)
     *  See page 184      http://www.atmel.com/Images/doc8161.pdf  Uno
     *  See page 212	  http://www.atmel.com/images/doc2549.pdf  Mega_Adk
     *  See page 177	  http://www.atmel.com/images/doc2467s.pdf  ATmega128 (PE0 = rX , PE1 = tX)
     */

    /* wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)));

    /* Put data into buffer, sends the data */
    UDR0 = data;

}

void sendString1(uint8_t *cstr)
{
    unsigned long i = 0;
    while (*cstr != '\0')
    {
        sendChar1(*cstr);
        cstr++;
    }
}

void sendString0(uint8_t *cstr)
{
    unsigned long i = 0;
    while (*cstr != '\0')
    {
        sendChar0(*cstr);
        cstr++;
    }

}

void printf0(const char *fmt, ...)
{
    //TODO detect overflow (is buffer big enough)
    char buffer[1000];
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    sendString0(buffer);
    va_end(args);
}

void printf1(const char *fmt, ...)
{
    //TODO detect overflow (is buffer big enough)
    char buffer[1000];
    va_list args;
    va_start(args, fmt);
    vsprintf(buffer, fmt, args);
    sendString1(buffer);
    va_end(args);
}

void usart0_enable_isr_rx_complete(char enable_bool)
{
    if(enable_bool)
        UCSR0B |= _BV(RXCIE0);
    else
        UCSR0B &= ~_BV(RXCIE0);
}

void usart1_enable_isr_rx_complete(char enable_bool)
{
    if(enable_bool)
        UCSR1B |= _BV(RXCIE1);
    else
        UCSR1B &= ~_BV(RXCIE1);
}

void usart0_enable_isr_tx_complete(char enable_bool)
{
    if(enable_bool)
        UCSR0B |= _BV(TXCIE0);
    else
        UCSR0B &= ~_BV(TXCIE0);
}

void usart1_enable_isr_tx_complete(char enable_bool)
{
    if(enable_bool)
        UCSR1B |= _BV(TXCIE1);
    else
        UCSR1B &= ~_BV(TXCIE1);
}
