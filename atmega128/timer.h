/*
 * timer.h
 *
 *  Created on: Oct 15, 2015
 *      Author: jim
 */

#ifndef TIMER_H_
#define TIMER_H_


typedef enum TIMER_PRESCALER
{
	NO_PRESCALING = 0x01, ONE_8TH = 0x02, ONE_64TH = 0x003, ONE_256TH = 0x04, ONE_1024TH = 0x05
} timer_prescaler_t;


void set_mode_timer1(unsigned char);
void set_prescaler_timer1(timer_prescaler_t);
void set_input_capture_edge_timer1(char enableRisingEdge);
void set_enable_input_capture_isr_timer1(char);
void set_enable_overflow_isr_timer1(char);

void clear_timer1(void);
void clear_capture_flag_timer1(void);
void stop_timer1(void);
void start_timer1(timer_prescaler_t);
unsigned read_timer1(void);
unsigned read_input_capture_timer1(void);





#endif /* TIMER_H_ */
