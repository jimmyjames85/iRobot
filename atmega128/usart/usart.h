/*
 * usart.h
 *
 *  Created on: May 30, 2015
 *      Author: jim
 */

#ifndef INCLUDES_COMM_USART_H_
#define INCLUDES_COMM_USART_H_



#include <stdint.h>

/**
 * Wait indefinitely until a char is received
 */
uint8_t getChar(void);
// unsigned char USART_Receive_with_timeout(unsigned long millis);

uint8_t getChar1(void);

/**
 * Wait indefinitely for transfer buffer to be empty, then send char
 */
void sendChar(uint8_t data);

void sendChar1(uint8_t data);
//void USART_Transmit_withDelay(unsigned char data, unsigned long timeoutMillis);

void init_USART(uint32_t baud, uint32_t f_cpu);
void init_USART1(uint32_t baud, uint32_t f_cpu);

//void USART_Init_withMultipleReg(uint32_t baud, uint8_t txRxReg);

uint8_t isAvailable();
uint8_t isAvailable1();

void sendString(uint8_t * cstr);
void sendString1(uint8_t * cstr);
#endif /* INCLUDES_COMM_USART_H_ */
