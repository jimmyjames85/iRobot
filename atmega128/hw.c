/*
 * hw.c
 *
 *  Created on: Sep 17, 2015
 *      Author: jim
 */

#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "hw.h"
#include "clock.h"

#include "blue_tooth_HC05.h"
#include "string.h"
#include "atmega128.h"
//#include "usart/usart.h"
#include "open_interface.h"
#include "blue_tooth_HC05.h"

void function_i(void)
{
	char msg[] = "ATMega128";
	int my_length = 0xFFFF;
	my_length = strlen(msg);
	bprintf("i: my_length= %d\r\n", my_length);

}

void function_ii(void)
{
	char msg[20] = "16MB";
	int my_length = 0xFFFF;
	my_length = strlen(msg);

	bprintf("ii: my_length= %d\r\n", my_length);
}

void function_iii(void)
{
	char msg[] =
	{ 'C', 'P', 'R', 'E', '\0', '2', '8', '8' };
	int my_length = 0xFFFF;
	my_length = strlen(msg);
	bprintf("iii: my_length= %d\r\n", my_length);
}

void function_iv(void)
{
	char msg[] = "CPRE288";
	int my_length = 0xFFFF;
	my_length = strlen(msg + 2);
	bprintf("iv: my_length= %d\r\n", my_length);
}

void function_v(void)
{
	char msg[20];
	int my_length = 0xFFFF;
	int i;
	for (i = 0; i < 20; i++)
	{
		msg[i] = 'a';
	}
	my_length = strlen(msg);
	bprintf("v: my_length= %d\r\n", my_length);
	bprintf("         msg= '%s'\r\n ", msg);
}

void q3(void)
{
	volatile int i = 0;
	volatile char msg[6] =
	{ 'S', 'a', 'y', 0x20, 'h', 'i' };

	/* Question three had long speed declared before int dice
	 * To get the variable addresses to block up in the order
	 *
	 *        {msg[0],...,msg [5],speed[0],dice[0],dice[1]}
	 *
	 * I had to switch the declaration order of dice and speed
	 * as well as declare all variables as volatile*/
	volatile int dice[2] =
	{ 0x30, 0x40 };

	volatile long speed[1] =
	{ 0x38384440 };

	bprintf("dice[0]= 0x%02X\t0x%02X\t0x%02X\t0x%02X\t\r\n", ((uint8_t *) dice)[0], ((uint8_t *) dice)[1], ((uint8_t *) dice)[2], ((uint8_t *) dice)[3]);
	bprintf("dice[1]= 0x%02X\t0x%02X\t0x%02X\t0x%02X\t\r\n", ((uint8_t *) dice)[0], ((uint8_t *) dice)[1], ((uint8_t *) dice)[2], ((uint8_t *) dice)[3]);

	for (i = 0; i < 14; i++)
	{
		uint8_t c = ((uint8_t *) msg)[i];
		if (c == 0x00)
			bprintf("\tmsg[%d] = 0x%02X  '\\0' \r\n", i, c);
		else
			bprintf("\tmsg[%d] = 0x%02X  '%c' \r\n", i, c, c);
	}
	bprintf("\r\n\r\n");

	msg[7] = 0x32;
	speed[1] = 0x33002121;
	bprintf("Q3: %s\r\n", msg);

	for (i = 0; i < 14; i++)
	{
		uint8_t c = ((uint8_t *) msg)[i];
		if (c == 0x00)
			bprintf("\tmsg[%d] = 0x%02X  '\\0' \r\n", i, c);
		else
			bprintf("\tmsg[%d] = 0x%02X  '%c' \r\n", i, c, c);
	}

}

void sizeTest(void)
{

	unsigned long DayOfWeek[7];
	short temperature[10];
	signed char command;
	float distance[10];
	float *time[10];

	bprintf("sizeOf(DayOfWeek) = %d\r\n", sizeof(DayOfWeek));
	bprintf("sizeOf(temperature) = %d\r\n", sizeof(temperature));
	bprintf("sizeOf(char) = %d\r\n", sizeof(char));
	bprintf("sizeOf(distance) = %d\r\n", sizeof(distance));
	bprintf("sizeOf( time) = %d\r\n", sizeof(time));

}

void hw2_q4_a(void)
{
	char msg[100];
	char str1[] = "16 MHZ";
	char str2[] = "held";
	char val_dollar = 0x24;
	char ch1 = 97;

	sprintf(msg, "The ATMega128 runs at %s.", str1);
	bprintf("%s\r\n\r\n", msg);
}
void hw2_q4_b(void)
{
	char msg[100];
	char str1[] = "16 MHZ";
	char str2[] = "held";
	char val_dollar = 0x24;
	char ch1 = 97;

	sprintf(msg, "There are %d bytes in str1.", strlen(str1));
	bprintf("%s\r\n\r\n", msg);
}
void hw2_q4_c(void)
{
	char msg[100];
	char str1[] = "16 MHZ";
	char str2[] = "held";
	char val_dollar = 0x24;
	char ch1 = 97;

	sprintf(msg, "Spend %c%d.", val_dollar, ch1);
	bprintf("%s\r\n\r\n", msg);
}
void hw2_q4_d(void)
{
	char msg[100];
	char str1[] = "16 MHZ";
	char str2[] = "held";
	char val_dollar = 0x24;
	char ch1 = 97;

	sprintf(msg, "%c%s%c, best video game!", str1[5], str2 + 1, ch1);
	bprintf("%s\r\n\r\n", msg);
}

void hw2_q4_e(void)
{
	char msg[100];
	char str1[] = "16 MHZ";
	char str2[] = "held";
	char val_dollar = 0x24;
	char ch1 = 97;
	sprintf(msg, "CPRE 288 makes students %c%c%c", 58, 45, 41);
	bprintf("%s\r\n\r\n", msg);
}

struct point3D
{
	signed long *x;
	unsigned int y;
	char z;
};

union val
{
	char cval;
	char str;
	int *ival[8];
	float fval;
	float *dval;
};

struct compound
{
	char *mystring;
	long *y;
	union
	{
		char *c;
		int *i;
		float *f;
	} u;
};

struct more_compound
{
	char name;
	int age;
	long *pay;
	long *height;

	union
	{
		char short_id;
		int normal_id;
		char my_id;
	} id;

	union
	{
		char *text_data;
		int *numeric_data;
		long *l_numeric_data;
	} data;
};
void hw3_q2(void)
{

	bprintf("\r\n\r\n");
	bprintf("sizeof(point3D) = %d\r\n", sizeof(struct point3D));
	bprintf("sizeof(signed long *) = %d\r\n", sizeof(signed long*));
	bprintf("sizeof(unsigned int) = %d\r\n", sizeof(unsigned int));
	bprintf("sizeof(char) = %d\r\n", sizeof(char));
	bprintf("sizeof(char *) = %d\r\n", sizeof(char *));
	bprintf("sizeof( char cval) = %d\r\n", sizeof(char));
	bprintf("sizeof( char str) = %d\r\n", sizeof(char));
	bprintf("sizeof( int *ival[8]) = %d\r\n", sizeof(int *[8]));
	bprintf("sizeof( float fval) = %d\r\n", sizeof(float));
	bprintf("sizeof( float *dval) = %d\r\n", sizeof(float *));
	bprintf("sizeof(union val) = %d\r\n", sizeof(union val));
	bprintf("sizeof(struct compound) = %d\r\n", sizeof(struct compound));
	bprintf("sizeof(struct more_compound) = %d\r\n", sizeof(struct more_compound));
	bprintf("\r\n\r\n");

}

void hw3_q3_a(void)
{

	char a = 5; //12 10
	char b = 10;
	char *c_ptr; //oxFFEO

	c_ptr = &a;
	*c_ptr = 12;
	*c_ptr = b;
	c_ptr = &b;
	*c_ptr = a;

	bprintf("    &a = %d\r\n", &a);
	bprintf("    &b = %d\r\n", &b);
	bprintf("&c_ptr = %d\r\n\r\n", &c_ptr);
	bprintf("     a = %d\r\n", a);
	bprintf("     b = %d\r\n", b);
	bprintf(" c_ptr = %d\r\n\r\n", c_ptr);

	bprintf("\r\n");
}

void hw3_q3_b(void)
{
	char a = 5;
	char b = 10;
	char *c_ptr = 0;

	c_ptr = &a; //&a
	c_ptr = &b; //&b
	(*c_ptr)++; //b = 11
	c_ptr++;    //&b+1

	bprintf("    &a = %d\r\n", &a);
	bprintf("    &b = %d\r\n", &b);
	bprintf("&c_ptr = %d\r\n\r\n", &c_ptr);
	bprintf("     a = %d\r\n", a);
	bprintf("     b = %d\r\n", b);
	bprintf(" c_ptr = %d\r\n\r\n", c_ptr);

	bprintf("\r\n");
}

void hw3_q3_c(void)
{
	int a = 5;
	int b = 10;
	int *c_ptr = 0;

	c_ptr = &b;       // c-> b
	a = *c_ptr + b;   // a = 20
	(*c_ptr)++;       // b = 11
	c_ptr++;          // c->&b+2

	bprintf("    &a = %d\r\n", &a);
	bprintf("    &b = %d\r\n", &b);
	bprintf("&c_ptr = %d\r\n\r\n", &c_ptr);
	bprintf("     a = %d\r\n", a);
	bprintf("     b = %d\r\n", b);
	bprintf(" c_ptr = %d\r\n\r\n", c_ptr);

	bprintf("\r\n");
}

char read_push_buttons(void)
{
	char ret = 0;

	if (bt_isAvailable())
	{
		ret = bt_getChar();
		if (ret == 0x1B) // <ESC>
			return -1;

		ret = ret - '0';
		if (ret <= 0 || ret > 6)
			ret = 0;
	}
	return ret;
}

#define lprintf bprintf
#define wait_ms _delay_ms

void update_clock_with_input(char input, clock_t * clock)
{
	switch (input)
	{
		case 1:
			decreaseSeconds(clock);
		break;
		case 2:
			increaseSeconds(clock);
		break;
		case 3:
			decreaseMinutes(clock);
		break;
		case 4:
			increaseMinutes(clock);
		break;
		case 5:
			decreaseHours(clock);
		break;
		case 6:
			increaseHours(clock);
		break;
		default:
		break; //do nothing
	}
}



clock_t clock;

void lab4_part1_clock_main(void)
{
	initClock(&clock);
	char lab4_input;
	lab4_input = 0;
	while (1)
	{
		//display time
		lprintf("%02d:%02d:%02d\r\n", clock.hours, clock.minutes, clock.seconds);

		//enter lab4_input loop
		while ((lab4_input = read_push_buttons()))
		{
			if (lab4_input == -1) //TODO remove this
				return;
			update_clock_with_input(lab4_input, &clock);
			lprintf("%02d:%02d:%02d\r\n", clock.hours, clock.minutes, clock.seconds); //display user change

			//The response time of the push button should be 200ms; this means that
			//if the user holds SW2 down for a long period of time, they should see
			//the clock's seconds continually increment higher 5 times every second
			wait_ms(200);
		}

		wait_ms(1000);
		clockTick(&clock);
	}
}

//#warning "CLOCK_COUNT may not be defined correctly" // delete this line after defining CLOCK_COUNT
//#warning "CHECK_COUNT may not be defined correctly" // delete this line after defining CHECK_COUNT


#define CLOCK_COUNT ((F_CPU >> 10)) // Where F_CPU = frequencey of the cpu // TODO - Edit this to be equal to the number of Timer Increments in 1 second
#define CHECK_COUNT CLOCK_COUNT / 500 // TODO - Edit this to be equal to the number of Timer Increments in 200 ms

void timer_init(void)
{
	// set up timer 1: WGM1 bits = 0100, CS = 101, set OCR1A, set TIMSK
	TCCR1A = 0b00000000;		// WGM1[1:0]=00               <---- See Table 61 and WGM1[3:2] below...This sets CTC (clear timer on compare) with MAX_COUNTER_VALUE = OCR1A (so set OCR1A)
	TCCR1B = 0b00001101;		// WGM1[3:2]=01, CS=101       <---- See Table 63 CS=101 means clock prescaler of 1/1024
	OCR1A = CLOCK_COUNT - 1; 	// counter threshold for clock
	TIMSK = _BV(OCIE1A);		// enable OC interrupt, timer 1, channel A

	// set up timer 3: WGM1 bits = 0100, CS = 101, set OCR3A, set TIMSK
	TCCR3A = 0b00000000;		// WGM3[1:0]=00
	TCCR3B = 0b00001101;		// WGM3[3:2]=01, CS=101
	OCR3A = CHECK_COUNT - 1; 	// counter threshold for checking push button
	ETIMSK = _BV(OCIE3A);		// enable OC interrupt, timer 3, channel A

	sei();
}

/**
 * Timer interrupt source 1: the function will be called every one second
 * (you need define CLOCK_COUNT correctly)
 */
ISR( TIMER1_COMPA_vect)
{
	clockTick(&clock);
	// DELETE ME - About the ISR macro
	// ----------------------------------------
	// ISR is a macro defined in interrupt.h.  You should not call an ISR function.
	// The ATmega128 is specifically built to run these functions for you when an
	// event occurs.
	//
	// ISRs (Interrupt Service Routines) are interrupt handlers specific to the platform
	// on which we're working. In this case, this function will be run when the value
	// of Timer1 matches OCR1A.
	//
	// OCR1A = Output Compare Register A for timer1
	//
	// For more information, consult the Atmel User Guide.
	//

	// Be sure to correctly initialize CLOCK_COUNT and CHECK_COUNT so these interrupts
	// get called at the right frequency.

}

/**
 * Timer interrupt source 2: the function will be called every 200 milliseconds
 * (you need define CHECK_COUNT correctly)
 */
ISR( TIMER3_COMPA_vect)
{
	// Insert interrupt handler code for checking push buttons here
	update_clock_with_input(read_push_buttons(),&clock);
}

void lab4_part2_clock_main(void)
{
	timer_init();
	initClock(&clock);
	while (1)
	{
		if(read_push_buttons()==-1)
		{
			//TODO remove this
			cli();
			return;
		}


		//display time
		lprintf("%02d:%02d:%02d\r\n", clock.hours, clock.minutes, clock.seconds);
	}
}

void hw_main(void)
{
	lab4_part2_clock_main();
	/*while (1)
	{
		hw3_q3_c();
		_delay_ms(4000);
	}
	void (*tests[5])(void)=
	{	hw2_q4_a,hw2_q4_b,hw2_q4_c,hw2_q4_d,hw2_q4_e
	};*/
	/*
	 bprintf("\r\nHello World\r\n");
	 for (i = 0; i < 5; i++)
	 {
	 tests[i]();
	 bprintf("---------------------\r\n");
	 }*/

	//sizeTest();
//	tests[3]();
	/*int i;
	 for(i=0;i<5;i++)
	 {
	 tests[i]();
	 bprintf("---------------------\r\n");
	 }*/
/*
	while (1)
	{
		bprintf("hello\r\n");

	}*/
}

