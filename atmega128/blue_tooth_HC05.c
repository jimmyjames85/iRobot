#include "blue_tooth_HC05.h"
#include "usart/usart.h"
#include <stdarg.h>
#include <stdio.h>
uint8_t bt_isAvailable()
{
	return isAvailable();
}

uint8_t bt_getChar()
{
	return getChar();
}

void bt_sendChar(unsigned char data)
{
	sendChar(data);
}

void bprintf(const char * fmt, ...)
{
	//TODO detect overflow
	char buffer[1000];
	va_list args;
	va_start(args, fmt);
	vsprintf(buffer, fmt, args);
	sendString(buffer);
	va_end(args);
}
