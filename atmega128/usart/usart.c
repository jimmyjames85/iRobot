/*
 * usart.c
 *
 *  Created on: May 30, 2015
 *      Author: jim
 */

#include <avr/io.h>
#include <stdarg.h>
#include "usart.h"

void usart0_init(unsigned long UBRR_value, unsigned char frame_size, unsigned char stop_bits, usart_parity_mode_t parity_mode, unsigned char enable_U2X)
{

	/**
	 *  See page 183    http://www.atmel.com/Images/doc8161.pdf  Uno
	 *  See page 211    http://www.atmel.com/images/doc2549.pdf  Mega_Adk
	 *  See page 175    http://www.atmel.com/Images/doc2467s.pdf  ATmega128
	 */

	/* Disable receiver and transmitter while we make changes*/
	UCSR0B &= ~( _BV(RXEN0) | _BV(TXEN0));

	if (enable_U2X)
		UCSR0A |= _BV(U2X0);

	/* Set baud rate */
	UBRR0H = (unsigned char) (UBRR_value >> 8);
	UBRR0L = (unsigned char) UBRR_value;

	switch (parity_mode)
	{
		case USART_PARITY_DISABLED:
			UCSR0C &= ~( _BV(UPM01) | _BV(UPM00));
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

void init_USART0(uint32_t baud, uint32_t f_cpu)
{
	/**
	 *  See page 183    http://www.atmel.com/Images/doc8161.pdf  Uno
	 *  See page 211    http://www.atmel.com/images/doc2549.pdf  Mega_Adk
	 *  See page 175    http://www.atmel.com/Images/doc2467s.pdf  ATmega128
	 */

	/**
	 * 	UBRRn is the USART Baud Rate Register
	 *  Some boards have multiple USARTS (e.g. atmega2560)
	 *  The n in UBRRn points to the USART you are referring to (e.g. UBBR0 or UBBR1)
	 *  This method only uses the 0th USART
	 */

// divide by 16 means Asynchronous Normal Mode
	unsigned long UBRRn_BaudRateCalculation = ((f_cpu / 16 / baud) - 1); //for all n in [0,1,2,3]

//TODO override calculation for variable baud rates and F_CPU according to Table 84 error percentages

//UCSR0A |= (1 << U2X0); //This sets U2X0 = 1

	/* Set baud rate */
	UBRR0H = (unsigned char) (UBRRn_BaudRateCalculation >> 8);
	UBRR0L = (unsigned char) UBRRn_BaudRateCalculation;

	/* Enable receiver and transmitter */
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);

	/* Set frame format: 8data, 2stop bit */
//UCSR0C = (1 << USBS0) | (3 << UCSZ00);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1 << USBS0) | (1 << UCSZ01) | (1 << UCSZ00);
// Keep in mind the data transfer size is determined by three bits.
// UCSZn0, UCSZn1-set in UCSRnC register- and UCSZn2 set in UCSRnB
// in this case we want UCSZn2 to be zero, hence it is not explicitly coded
// when we are setting the UCSRnB register above

}

void init_USART1(uint32_t baud, uint32_t f_cpu)
{
#if ___atmega328p //not for uno
	return;
#else
	/**
	 *  See page 183    http://www.atmel.com/Images/doc8161.pdf  Uno
	 *  See page 211    http://www.atmel.com/images/doc2549.pdf  Mega_Adk
	 *  See page 175    http://www.atmel.com/Images/doc2467s.pdf  ATmega128
	 */

	/**
	 * 	UBRRn is the USART Baud Rate Register
	 *  Some boards have multiple USARTS (e.g. atmega2560)
	 *  The n in UBRRn points to the USART you are referring to (e.g. UBBR0 or UBBR1)
	 *  This method only uses the 1st USART
	 */

// divide by 16 means Asynchronous Normal Mode
	unsigned long UBRRn_BaudRateCalculation = ((f_cpu / 16 / baud) - 1); //for all n in [0,1,2,3]

//TODO override calculation for variable baud rates and F_CPU according to Table 84 error percentages

//UCSR0A |= (1 << U2X0); //This sets U2X0 = 1

	/* Set baud rate */
	UBRR1H = (unsigned char) (UBRRn_BaudRateCalculation >> 8);
	UBRR1L = (unsigned char) UBRRn_BaudRateCalculation;

	/* Enable receiver and transmitter */
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);

	/* Set frame format: 8data, 2stop bit */
//UCSR0C = (1 << USBS0) | (3 << UCSZ00);
	/* Set frame format: 8data, 2stop bit */

//UCSR1C = (1 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10);
//Roomba is only 1-stop bit
	UCSR1C = (0 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10);

// Keep in mind the data transfer size is determined by three bits.
// UCSZn0, UCSZn1-set in UCSRnC register- and UCSZn2 set in UCSRnB
// in this case we want UCSZn2 to be zero, hence it is not explicitly coded
// when we are setting the UCSRnB register above
#endif

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
	while (!(UCSR0A & (1 << RXC0)))
		;

	/* Get and return received data from buffer */
	return UDR0;
}

uint8_t getChar1(void)
{
#if ___atmega328p //not for uno
	return;
#else
	/**
	 *  USART_Receive(void)
	 *  See page 187 - 188		http://www.atmel.com/images/doc8161.pdf  Uno
	 *  See page 215 - 216		http://www.atmel.com/images/doc2549.pdf  Mega_Adk
	 *  See page 180 - 181      http://www.atmel.com/Images/doc2467s.pdf  ATmega128
	 */

	/* Wait for data to be received */
	while (!(UCSR1A & (1 << RXC1)))
		;

	/* Get and return received data from buffer */
	return UDR1;
#endif
}

uint8_t isAvailable1()
{
#if ___atmega328p //not for uno
	return;
#else
	return (UCSR1A & (1 << RXC1));
#endif
}

uint8_t isAvailable0()
{
	return (UCSR0A & (1 << RXC0));
}

void sendChar1(uint8_t data)
{
#if ___atmega328p //not for uno
	return;
#else
	/**
	 *  USART_Transmit(unsigned char data)
	 *  See page 184      http://www.atmel.com/Images/doc8161.pdf  Uno
	 *  See page 212	  http://www.atmel.com/images/doc2549.pdf  Mega_Adk
	 *  See page 177	  http://www.atmel.com/images/doc2467s.pdf  ATmega128 (PE0 = rX , PE1 = tX)
	 */

	/* wait for empty transmit buffer */
	while (!(UCSR1A & (1 << UDRE1)))
		;

	/* Put data into buffer, sends the data */
	UDR1 = data;
#endif
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
	while (!(UCSR0A & (1 << UDRE0)))
		;

	/* Put data into buffer, sends the data */
	UDR0 = data;

}

void sendString1(uint8_t * cstr)
{
#if ___atmega328p //not for uno
	return;
#else
	unsigned long i = 0;
	while (*cstr != '\0')
	{
		sendChar1(*cstr);
		cstr++;
	}
#endif
}

void sendString0(uint8_t * cstr)
{

	unsigned long i = 0;
	/*for(i=0;cstr[i]!='\0';i++)
	 sendChar(cstr[i]);
	 */
	while (*cstr != '\0')
	{
		sendChar0(*cstr);
		cstr++;
	}

}

void printf0(const char * fmt, ...)
{
//TODO detect overflow
	char buffer[1000];
	va_list args;
	va_start(args, fmt);
	vsprintf(buffer, fmt, args);
	sendString0(buffer);
	va_end(args);
}

void printf1(const char * fmt, ...)
{
#if ___atmega328p //not for uno
	return;
#else
//TODO detect overflow
	char buffer[1000];
	va_list args;
	va_start(args, fmt);
	vsprintf(buffer, fmt, args);
	sendString1(buffer);
	va_end(args);
#endif
}

