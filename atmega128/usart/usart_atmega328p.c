/*
 * usart.c
 *
 *  Created on: Oct 15, 2015
 *      Author: jim
 */

#include <avr/io.h>
#include "usart.h"


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


uint8_t getChar0(void)
{
	/**
	 *  USART_Receive(void)
	 *  See page 187 - 188		http://www.atmel.com/images/doc8161.pdf  Uno
	 *  See page 215 - 216		http://www.atmel.com/images/doc2549.pdf  Mega_Adk
	 *  See page 180 - 181      http://www.atmel.com/Images/doc2467s.pdf  ATmega128
	 */

	/* Wait for data to be received */
	while (!(UCSR0A & (1 << RXC0)) );

	/* Get and return received data from buffer */
	return UDR0;
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

void sendString0(uint8_t * cstr)
{
	unsigned long i=0;
	/*for(i=0;cstr[i]!='\0';i++)
		sendChar(cstr[i]);
*/
	while(*cstr != '\0')
	{
		sendChar0(*cstr);
		cstr++;
	}

}


uint8_t isAvailable0()
{
	return (UCSR0A & (1 << RXC0));
}


void init_USART1(uint32_t baud, uint32_t f_cpu)
{
	return;
}


uint8_t getChar1(void)
{
	return 0;
}

uint8_t isAvailable1()
{
	return 0;
}


void sendChar1(uint8_t data)
{
	return;
}


void sendString1(uint8_t * cstr)
{
	return;

}



