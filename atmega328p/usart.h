/*
 * usart.h
 *
 *  Created on: May 30, 2015
 *      Author: jim
 */

#ifndef INCLUDES_COMM_USART_H_
#define INCLUDES_COMM_USART_H_

#include <stdint.h>

typedef enum USART_PARITY_MODE
{
	// @formatter:off
	USART_PARITY_DISABLED = 0x0,
	USART_PARITY_RESERVED = 0x01,
	USART_PARITY_EVEN = 0x02,
	USART_PARITY_ODD = 0x03
// @formatter:on
} usart_parity_mode_t;


void usart0_init(unsigned long UBRR_value, unsigned char frame_size, unsigned char stop_bits, usart_parity_mode_t parity_mode, unsigned char enable_U2X);

/**
 * Wait indefinitely until a char is received
 */
uint8_t getChar0(void);

/**
 * Wait indefinitely for transfer buffer to be empty, then send char
 */
void sendChar0(uint8_t data);
uint8_t isAvailable0();
void sendString0(uint8_t * cstr);


void printf0(const char * fmt, ...);

#endif /* INCLUDES_COMM_USART_H_ */
