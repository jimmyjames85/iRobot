/*
 * ping.c
 *
 *  Created on: Oct 13, 2015
 *      Author: jim
 */

//#include "ping.h"
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usart/usart.h"
#include <stdlib.h>

volatile unsigned end_time;
volatile unsigned start_time;
volatile short have_reading;
#define uno 1

#if uno
#define set_timer1_normal_mode TCCR1B &= ~(_BV(WGM13)|(_BV(WGM12))); TCCR1A&= ~(_BV(WGM11)|(_BV(WGM10)))
#define set_input_capture_on_falling_edge TCCR1B &= ~_BV(ICES1);
#define enable_input_capture_isr 	TIMSK1 |= _BV(ICIE1);
#define disable_input_capture_isr 	TIMSK1 &= ~_BV(ICIE1);
#define enable_timer1_overflow_isr TIMSK1 |= _BV(TOIE1);
#define clear_timer1_capture_flag TIFR1 |= _BV(ICF1);
#else
#define enable_input_capture_isr 	TIMSK |= _BV(TICIE1);
#define disable_input_capture_isr 	TIMSK &= ~_BV(TICIE1);
#define enable_timer1_overflow_isr 	0;//TIMSK |= _BV(TOIE1);//???
#define clear_timer1_capture_flag TIFR1 |= _BV(ICF1);  //TODO ???
#endif

typedef enum TIMER_PRESCALER
{
	NO_PRESCALING = 0x01, ONE_8TH = 0x02, ONE_64TH = 0x003, ONE_256TH = 0x04, ONE_1024TH = 0x05
} prescaler_t;

#if 0
CS2 CS1 CS0 Function
0 0 0 No clock source (Timer/Counter stopped).
0 0 1 clk I/O /1 (No prescaling)
0 1 0 clk I/O /8 (From prescaler)
0 1 1 clk I/O /64 (From prescaler)
1 0 0 clk I/O /256 (From prescaler)
1 0 1 clk I/O /1024 (From prescaler)
1 1 0 External clock source on T1 pin. Clock on falling edge.
1 1 1 External clock source on T1 pin. Clock on rising edge
#endif

void initPing(void)
{
	//TCCR1A = 0b00000000; // WGM1[1:0]=00 <---- See Table 61 and WGM1[3:2]
	//below...This sets Normal Timer mode // (no clear )

	//TCCR1B = 0b00000101;	// WGM1[3:2]=00, CS=100 [2:0]<---- See Table 63 CS=100
	//means clock prescaler of 1/256

	enable_input_capture_isr
	;
	enable_timer1_overflow_isr
	;

}
volatile unsigned int waiting_for_ping_response;
volatile short timer1_overflow_detected;
ISR(TIMER1_OVF_vect)
{
	if (waiting_for_ping_response)
		timer1_overflow_detected = 1;
}
volatile short sendingPulse;

void ping_send_pulse()
{
	disable_input_capture_isr
	sendingPulse = 1;
#if uno
	DDRB |= _BV(0); // setup for output so we can trigger
	PORTB |= _BV(0); // send high
	_delay_us(5); // tout = 2 us , 5us typical
	PORTB &= ~_BV(0); // send low
	DDRB &= ~_BV(0); // setup for input

//	PORTB &= ~_BV(0);//pull down
#else
	DDRD |= _BV(4); // setup for output so we can trigger */
	PORTD |= _BV(4);//send high */
	_delay_us(2);// tout = 2us
	PORTD &= ~_BV(4);//send low */
	DDRD &= _BV(4);// setup for input*/
#endif

	clear_timer1_capture_flag
	enable_input_capture_isr
	sendingPulse = 0;
}

unsigned int input_capture_reg1_val()
{
	unsigned int read = ICR1L;
	read |= (ICR1H << 8);
	return read;
}
volatile short timer1_input_capture_detected;
volatile unsigned int capVal;
ISR( TIMER1_CAPT_vect)
{
	timer1_input_capture_detected = 1;
	waiting_for_ping_response=0; //WE GOT IT!!!!
	capVal= input_capture_reg1_val();
}

void clear_timer1_val()
{
	TCNT1H = 0x0;
	TCNT1L = 0x0;
}
void stop_timer1()
{
	TCCR1B &= (0xF8);	// stop timer
}
void start_timer1(prescaler_t prescaler)
{
	TCCR1B |= prescaler;
}
unsigned int timer1_val()
{
	unsigned int read = TCNT1L;
	read |= (TCNT1H << 8);
	return read;
}

#define BLINK_DELAY_MS 43

void printUnsignedInt(unsigned int i)
{
	char buffer[30];
	ltoa(i, buffer, 10);
	sendString0(buffer);
}

//TODO create a timer.h and timer.c
// add timer_t to hold timer settings such as current prescaler
// add functions to calculate ticks_to_us, ticks_to_ms, ticks_to_seconds
// add functions to setup the hardware timer

long ticks_to_mm(unsigned long ticks, prescaler_t clock_prescaler)
{
	long ttps;	//timerTicksPerSec

	switch (clock_prescaler)
	{

		//TODO reverse the case order and remove the breaks
		case ONE_8TH:
			ttps = F_CPU / 8;
		break;
		case ONE_64TH:
			ttps = F_CPU / 64;
		break;
		case ONE_256TH:
			ttps = F_CPU / 256;
		break;
		case ONE_1024TH:
			ttps = F_CPU / 1024;
		break;
		case NO_PRESCALING:
		default:
			ttps = F_CPU;
		break;
	}

	return (ticks * (340000L / 2)) / ttps;
}

int main(void)
{

	prescaler_t prescaler = ONE_64TH;
	timer1_overflow_detected = 0;
	init_USART0(9600, F_CPU);
	//initPing();

	set_timer1_normal_mode
	;
	enable_input_capture_isr
	;
	enable_timer1_overflow_isr
	;

	//doPingLoop();
	DDRB |= _BV(DDB5); /* set pin 5 of PORTB for output*/

	unsigned int oldTimerVal = 0;
	unsigned int start_pulse_time = 0;
	waiting_for_ping_response = 0;
	unsigned int continous_pulse = 0;
	unsigned int overflows = 0;
	sei();
	while (1)
	{

		if (timer1_overflow_detected)
		{
			//stop_timer1();
			//clear_timer1_val();
			timer1_overflow_detected = 0;

			overflows++;
		}

		if (timer1_input_capture_detected)
		{
			waiting_for_ping_response = 0;

			long delta = overflows;
			delta = delta*128;
			delta |= capVal;
			delta -= start_pulse_time;
			//		(((long)( overflows << 8)) | capVal) - start_pulse_time;

			long mm = ticks_to_mm(delta, prescaler);
			sendString0("\r\n dist = ");

			printUnsignedInt(mm);
			sendString0(" mm ");
			printUnsignedInt(mm / 10);
			sendString0(" cm ");


			if (overflows)
			{


				sendString0(" <-- over flow ");
				printUnsignedInt(capVal);
				sendString0(" ");
				printUnsignedInt(overflows);
				sendString0(" srt=");
				printUnsignedInt(start_pulse_time);
				sendString0(" delta=");
				printUnsignedInt(delta);
			}

			//printUnsignedInt(delta); //raw
			timer1_input_capture_detected = 0;
			if (continous_pulse)
			{
				overflows = 0;
				ping_send_pulse();
				start_pulse_time = timer1_val();
				waiting_for_ping_response = 1;
			}
		}

		unsigned int timerVal = timer1_val();
		if (timerVal != oldTimerVal)
		{
			//sendString0("\r\ntime: ");
			//printUnsignedInt(timerVal);
			oldTimerVal = timerVal;
		}

		if (isAvailable0())
		{
			char c = getChar0();
			if (c == ' ')
			{
				sendString0("\r\nstarting timer");
				start_timer1(prescaler); //start timer
			}
			else if (c == 's')
			{
				stop_timer1();
				sendString0("user stoped timer\r\n");
			}
			else if (c == 'p')
			{
				overflows = 0;
				ping_send_pulse();
				start_pulse_time = timer1_val();
				waiting_for_ping_response = 1;
				sendString0("\r\n user sent pusle at ");
				printUnsignedInt(start_pulse_time);
				sendString0("\r\n");
			}
			else if (c == 'c')
			{
				continous_pulse = 1;
			}
			else if (c == 'C')
			{
				continous_pulse = 0;
			}
		}

		PORTB |= _BV(PORTB5); /* set pin 5 high to turn led on */
		_delay_ms(BLINK_DELAY_MS);

		PORTB &= ~_BV(PORTB5); /* set pin 5 low to turn led off */
		_delay_ms(BLINK_DELAY_MS);
	}

	return 0; /* CODE SHOULD NEVER REACH HERE */
}
