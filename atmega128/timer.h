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
	// @formatter:off
	STOPPED = 0x0,
	NO_PRESCALING = 0x01,
	ONE_8TH = 0x02,
	ONE_64TH = 0x003,
	ONE_256TH = 0x04,
	ONE_1024TH = 0x05
// @formatter:on
} timer_prescaler_t;

typedef enum TIMER_MODE
{
	//uno see table 15-4 pg 136
	// @formatter:off
	TIMER_MODE_NORMAL_MAX_TOP = 0,
	TIMER_MODE_PWM_PHASE_CORRECT_8_BIT_TOP = 1,
	TIMER_MODE_PWM_PHASE_CORRECT_9_BIT_TOP = 2,
	TIMER_MODE_PWM_PHASE_CORRECT_10_BIT_TOP = 3,
	TIMER_MODE_CTC_OCR1A_TOP = 4,
	TIMER_MODE_PWM_FAST_8_BIT_TOP = 5,
	TIMER_MODE_PWM_FAST_9_BIT_TOP = 6,
	TIMER_MODE_PWM_FAST_10_BIT_TOP = 7,
	TIMER_MODE_PWM_PHASE_FREQ_CORRECT_ICRn_TOP = 8,
	TIMER_MODE_PWM_PHASE_FREQ_CORRECT_OCRnA_TOP = 9,
	TIMER_MODE_PWM_PHASE_CORRECT_ICRn_TOP = 10,
	TIMER_MODE_PWM_PHASE_CORRECT_OCRnA_TOP = 11,
	TIMER_MODE_CTC_ICRn_TOP = 12,
	TIMER_MODE_RESERVED = 13,
	TIMER_MODE_PWM_FAST_ICRn_TOP = 14,
	TIMER_MODE_PWM_FAST_OCRnA_TOP = 15
// @formatter:on

} timer_mode_t;

typedef enum TIMER_COMPARE_OUTPUT_MODE
{
	// @formatter:off
	TIMER_COMPARE_OUTPUT_DISCONNECTED = 0,
	TIMER_COMPARE_OUTPUT_TOGGLE = 1,
	TIMER_COMPARE_OUTPUT_SET_LOW = 2,
	TIMER_COMPARE_OUTPUT_SET_HIGH = 3

	// @formatter:on
} timer_compare_output_mode_t;
/**
 * fcpu  - base frequency
 * clock_prescaler -
 */
double ticks_to_secs(unsigned long ticks, timer_prescaler_t clock_prescaler, unsigned long fcpu);

void tmr1_set_mode(unsigned char);
void tmr1_set_prescaler(timer_prescaler_t);
void tmr1_set_input_capture_edge(char true_is_rising_edge);
void tmr1_enable_input_capture_isr(char enable_bool);
void tmr1_enable_overflow_isr(char enable_bool);
void tmr1_clear_count(void);
void tmr1_clear_capture_flag(void);
void tmr1_stop(void);
void tmr1_start(timer_prescaler_t);

void tmr1_enable_output_compare_A_match_isr(char enable_bool);
void tmr1_enable_output_compare_B_match_isr(char enable_bool);
void tmr1_enable_output_compare_C_match_isr(char enable_bool);

void tmr1_set_output_compare_A_value(unsigned int compare_val);
void tmr1_set_output_compare_B_value(unsigned int compare_val);
void tmr1_set_output_compare_C_value(unsigned int compare_val);

void tmr1_set_output_compare_A_mode(timer_compare_output_mode_t);
void tmr1_set_output_compare_B_mode(timer_compare_output_mode_t);
void tmr1_set_output_compare_C_mode(timer_compare_output_mode_t);


unsigned char tmr1_read_capture_flag(void);
unsigned tmr1_read_count(void);
unsigned tmr1_read_input_capture_count(void);


void tmr3_set_mode(unsigned char);
void tmr3_set_prescaler(timer_prescaler_t);
void tmr3_set_input_capture_edge(char true_is_rising_edge);
void tmr3_enable_input_capture_isr(char enable_bool);
void tmr3_enable_overflow_isr(char enable_bool);
void tmr3_clear_count(void);
void tmr3_clear_capture_flag(void);
void tmr3_stop(void);
void tmr3_start(timer_prescaler_t);

void tmr3_enable_output_compare_A_match_isr(char enable_bool);
void tmr3_enable_output_compare_B_match_isr(char enable_bool);
void tmr3_enable_output_compare_C_match_isr(char enable_bool);

void tmr3_set_output_compare_A_value(unsigned int compare_val);
void tmr3_set_output_compare_B_value(unsigned int compare_val);
void tmr3_set_output_compare_C_value(unsigned int compare_val);

void tmr3_set_output_compare_A_mode(timer_compare_output_mode_t);
void tmr3_set_output_compare_B_mode(timer_compare_output_mode_t);
void tmr3_set_output_compare_C_mode(timer_compare_output_mode_t);

unsigned char tmr3_read_capture_flag(void);
unsigned tmr3_read_count(void);
unsigned tmr3_read_input_capture_count(void);

#endif /* TIMER_H_ */
