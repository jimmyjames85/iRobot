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
// unsigned char USART_Receive_with_timeout(unsigned long millis);

uint8_t getChar1(void);

/**
 * Wait indefinitely for transfer buffer to be empty, then send char
 */
void sendChar0(uint8_t data);

void sendChar1(uint8_t data);
//void USART_Transmit_withDelay(unsigned char data, unsigned long timeoutMillis);

void init_USART0(uint32_t baud, uint32_t f_cpu);//TODO remove
void init_USART1(uint32_t baud, uint32_t f_cpu);

//void USART_Init_withMultipleReg(uint32_t baud, uint8_t txRxReg);

uint8_t isAvailable0();
uint8_t isAvailable1();

void sendString0(uint8_t * cstr);
void sendString1(uint8_t * cstr);

void printf0(const char * fmt, ...);
void printf1(const char * fmt, ...);
#endif /* INCLUDES_COMM_USART_H_ */
