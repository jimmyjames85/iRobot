/*
 * blue_tooth_hc05.h
 *
 *  Created on: Sep 13, 2015
 *      Author: jim
 */
#include <inttypes.h>
#include <stdarg.h>
#ifndef ATMEGA128_BLUE_TOOTH_HC05_H_
#define ATMEGA128_BLUE_TOOTH_HC05_H_
#define BLUE_TOOTH_BAUD_RATE 38400

uint8_t bt_isAvailable();
uint8_t bgetChar();
void bsendChar(unsigned char data);
void bprintf(const char * fmt, ...);

#endif /* ATMEGA128_BLUE_TOOTH_HC05_H_ */
