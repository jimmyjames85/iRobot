/*
 * iRobot.c
 *
 *  Created on: Oct 28, 2015
 *      Author: jim
 */
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#include "usart.h"
#include "timer.h"
#include "list.h"
#include "adc.h"
#include "util.h"
#include "open_interface.h"
#include "servo.h"
#include "ping.h"
#include "ir_sensor.h"

void servo_test()
{
    servo_data_t rservo; //fake malloc
    servo_data_t *servo = &rservo;

    servo_init(servo);

    servo_calibrate(servo, 8, 35);
    servo_set_position_deg(servo, 90);

    while (1)
    {
        if (isAvailable0())
        {
            char c = getChar0();
            switch (c)
            {
                case '0':
                    servo_set_position_deg(servo, 0);
                    break;
                case '4':
                    servo_set_position_deg(servo, 45);
                    break;
                case '9':
                    servo_set_position_deg(servo, 90);
                    break;
                case '3':
                    servo_set_position_deg(servo, 135);
                    break;
                case '8':
                    servo_set_position_deg(servo, 180);
                    break;
                case '+':
                    servo_increment_degrees(servo, 1);
                    break;
                case '-':
                    servo_decrement_degrees(servo, 1);
                    break;
                default:
                    break;
            }

            double calcDeg = servo_calculate_position_deg(servo);
            char buff[300];
            ftoa(buff, calcDeg);

            printf0("Cur pulse width: %u\t deg: %u\t calcDeg: %s\r\n", servo->cur_pulse_width, servo->desired_deg,
                    buff);
        }
    }
}

/*void input_capture_test(void)
 {

 //setup timer1
 tmr1_set_mode(TIMER_MODE_NORMAL_MAX_TOP);
 tmr1_set_prescaler(TIMER_ONE_64TH); //set timer tick rate to 250KHz each tick = 4us
 tmr1_enable_input_capture_isr(1);
 tmr1_set_input_capture_edge(1);

 tmr1_enable_overflow_isr(1);

 unsigned int rising_edge_time;

 DDRD &= ~_BV(DDD4);
 while (!(PIND |= _BV(DDD4)))
 {
 rising_edge_time = tmr1_read_count();
 while ((PIND |= _BV(DDD4)))
 ; //wait for low
 }

 DDRB |= _BV(PB0);
 sei();
 while (1)
 {

 while (2 != hit_count)
 {
 printf0("%d\r\n", hit_count);
 if (hit_count == 1)
 {
 //_delay_ms(500);
 tmr1_enable_input_capture_isr(1);
 }

 }
 tmr1_enable_input_capture_isr(1);

 long diff = ((overflows << 16) | second_wheel_hit) - first_wheel_hit;
 double seconds = (double) diff / 250000;
 double velocity = 1.5 / seconds;

 double velocity_v = velocity / 2;
 double velocity_h = velocity_v;
 double time_in_air = velocity_v / 4.9;
 double distance_h = time_in_air * velocity_h;

 char buff[300];
 ftoa(buff, seconds);
 printf0("(%d) %s seconds\r\n", hit_count, buff);
 hit_count = 0;
 if (overflow_overflows)
 printf0("overflow_overflows = %lu\r\n", overflow_overflows);
 overflows = 0;
 overflow_overflows = 0;
 _delay_ms(1000);
 }
 }*/

void doPingLoop()
{
    timer_prescaler_t prescaler = TIMER_ONE_1024TH;
    ping_init(prescaler);

    while (1)
    {
        unsigned cm = ping_cm_busy_wait(prescaler);
        printf0("%d cm\r\n", cm);
    }

}

list_t *create_jims_ir_sensor_lookup_table()
{
    list_t *lookup_table = lalloc();
    ladd(lookup_table, (void *) new_ir_measurement(118, 360));
    ladd(lookup_table, (void *) new_ir_measurement(146, 300));
    ladd(lookup_table, (void *) new_ir_measurement(169, 245));
    ladd(lookup_table, (void *) new_ir_measurement(199, 215));
    ladd(lookup_table, (void *) new_ir_measurement(215, 195));
    ladd(lookup_table, (void *) new_ir_measurement(224, 175));
    ladd(lookup_table, (void *) new_ir_measurement(245, 165));
    ladd(lookup_table, (void *) new_ir_measurement(250, 155));
    ladd(lookup_table, (void *) new_ir_measurement(265, 145));
    ladd(lookup_table, (void *) new_ir_measurement(279, 135));
    ladd(lookup_table, (void *) new_ir_measurement(303, 125));
    ladd(lookup_table, (void *) new_ir_measurement(327, 115));
    ladd(lookup_table, (void *) new_ir_measurement(370, 105));
    ladd(lookup_table, (void *) new_ir_measurement(394, 95));
    ladd(lookup_table, (void *) new_ir_measurement(409, 85));
    ladd(lookup_table, (void *) new_ir_measurement(457, 75));
    ladd(lookup_table, (void *) new_ir_measurement(522, 65));
    ladd(lookup_table, (void *) new_ir_measurement(562, 55));
    ladd(lookup_table, (void *) new_ir_measurement(696, 45));
    ladd(lookup_table, (void *) new_ir_measurement(912, 35));
    ladd(lookup_table, (void *) new_ir_measurement(1019, 27));
    return lookup_table;
}

//private
int compare_ir_measurements(ir_measurement_t *a, ir_measurement_t *b)
{
    return a->voltage - b->voltage;
}

list_t *create_ir_lookup_table_from_ping(servo_data_t *servo, timer_prescaler_t prescaler)
{

    int maxVoltage = 1023;
    int minVoltage = 130;
    int avg = 5;
    int speed = 50; //50 mm per second
    int delay_for_5_mm = 100;
    oi_set_wheels(0, 0);
    servo_set_position_deg(servo, 90);

    printf0("Put roomba in front of flat surface like a wall. Press 'c' when ready...\r\n");
    while (getChar0() != 'c')
        printf0("Put roomba in front of flat surface like a wall. Press 'c' when ready...\r\n");

    unsigned voltage = ir_read_voltage_avg(avg);
    printf0("start: %u\r\n", voltage);
    while (voltage != maxVoltage)
    {
        printf0("%u\r\n", voltage);
        oi_set_wheels(-speed, -speed);
        _delay_ms(2 * delay_for_5_mm); //1 cm
        oi_set_wheels(0, 0);
        voltage = ir_read_voltage_avg(avg);
    }

    _delay_ms(100);
    unsigned peakVoltage_distance_mm = ping_mm_busy_wait(prescaler);

    while (voltage == maxVoltage)
    {
        printf0("[%u v], %u p_mm \r\n", voltage, peakVoltage_distance_mm);
        oi_set_wheels(-speed, -speed);
        _delay_ms(delay_for_5_mm * 2);
        oi_set_wheels(0, 0);

        _delay_ms(100);
        peakVoltage_distance_mm = ping_mm_busy_wait(prescaler);
        voltage = ir_read_voltage_avg(avg);
    }

    list_t *ret = lalloc();
    ladd(ret, (void *) new_ir_measurement(maxVoltage, peakVoltage_distance_mm));

    unsigned oldMm = peakVoltage_distance_mm;

    while (voltage > minVoltage)
    {

        _delay_ms(100);
        unsigned mm = ping_mm_busy_wait(prescaler);

        printf0("reading [%u v], %u mm   >?     %u oldMm \r\n", voltage, mm, oldMm);
        while (mm <= oldMm)
        {
            printf0("Non increasing ping value: [OLD: %u] [NEW: %u]\r\n", oldMm, mm);

            _delay_ms(100);
            mm = ping_mm_busy_wait(prescaler);
            oi_set_wheels(-speed, -speed);
            _delay_ms(delay_for_5_mm); //5mm
            oi_set_wheels(0, 0);
        }

        printf0(" adding [%u v], %u mm \r\n", voltage, mm);
        ladd(ret, (void *) new_ir_measurement(voltage, mm));
        oi_set_wheels(-speed, -speed);
        _delay_ms(delay_for_5_mm * 2); //1cm
        oi_set_wheels(0, 0);
        voltage = ir_read_voltage_avg(avg);
        oldMm = mm;
    }
    printf0("sorting...\r\n");
    lmergesort(ret, 0, ret->length - 1, (int (*)(const void *, const void *)) compare_ir_measurements);
    printf0("done\r\n");
    return ret;
}

void doIrLoop()
{
    //adc_set_vref(ADC_INTERNAL_VREF);//apply 3.1 volts to ADC_AREF (see )
    //TODO setup where to get ref voltage from
    //TODO create method find_good_prescaler_for_adc_based_on_f_cpu

    //TODO this table is for Jim's IR sensor with 3V applied to AREF
    //TODO this table's distance values is in inches * 10 (e.g. 36" = 360)
    //TODO convert to mm or cm
    list_t *lookup_table = create_jims_ir_sensor_lookup_table();

    ir_init(ADC_ONE_64TH, ADC_AREF, 2);
    while (1)
    {
        unsigned voltage = ir_read_voltage_avg(5);
        unsigned calculatedDist = ir_lookup_distance(lookup_table, voltage);
        printf0("%d volts \t\t calculatedDist= %u\r\n", voltage, calculatedDist);
    }
}

servo_data_t *create_jim_servo()
{
    servo_data_t *servo = malloc(sizeof(servo_data_t));
    servo_init(servo);
    servo_calibrate(servo, 68, 285);
    servo_set_position_deg(servo, 90);
    return servo;
}

void handleInput(servo_data_t *servo, unsigned int *leftWheelVelocity, unsigned int *rightWheelVelocity,
                 int *requestIrCalibration)
{

    *requestIrCalibration = 0;
    char c = '\0';
    if (isAvailable0())
    {
        c = getChar0();
        switch (c)
        {
            case '0':
                servo_set_position_deg(servo, 0);
                break;
            case '4':
                servo_set_position_deg(servo, 45);
                break;
            case '9':
                servo_set_position_deg(servo, 90);
                break;
            case '3':
                servo_set_position_deg(servo, 135);
                break;
            case '8':
                servo_set_position_deg(servo, 180);
                break;
            case '+':
                servo_increment_degrees(servo, 4);
                break;
            case '-':
                servo_decrement_degrees(servo, 4);
                break;
            case 'e':
                *leftWheelVelocity += 50;
                oi_set_wheels(*leftWheelVelocity, *rightWheelVelocity);
                break;
            case 'd':
                *leftWheelVelocity -= 50;
                oi_set_wheels(*leftWheelVelocity, *rightWheelVelocity);
                break;

            case 'r':
                *rightWheelVelocity += 50;
                oi_set_wheels(*leftWheelVelocity, *rightWheelVelocity);
                break;
            case 'f':
                *rightWheelVelocity -= 50;
                oi_set_wheels(*leftWheelVelocity, *rightWheelVelocity);
                break;
            case 'F':
                oi_full_mode();
                printf0("Full Mode \r\n");
                break;
            case ' ':
                *rightWheelVelocity = 0;
                *leftWheelVelocity = 0;
                oi_set_wheels(0, 0);
                break;
            case 'b':
                *rightWheelVelocity = -50;
                *leftWheelVelocity = -50;
                oi_set_wheels(*leftWheelVelocity, *rightWheelVelocity);
                _delay_ms(4000); //
                *rightWheelVelocity = 0;
                *leftWheelVelocity = 0;
                oi_set_wheels(*leftWheelVelocity, *rightWheelVelocity);
                break;
            case 'c':
                *requestIrCalibration = 1;
                break;
            case 's':
                oi_safe_mode();
                printf0("safe mode\r\n");
                break;
            case 'R':
                printf0("reset\r\n");
                oi_reset();
                break;
            default:
                break;
        }
    }
}

volatile unsigned char volatile_timer1_capture_count = 0;
volatile unsigned long volatile_timer1_overflows = 0;
volatile unsigned long volatile_ping_send_pulse_start_time = 0;
volatile char volatile_ping_capture_complete = 0;

//FOR PING
ISR(TIMER1_CAPT_vect)
{
    if (volatile_ping_capture_complete == 0)
        volatile_ping_capture_complete = 1;
}
//FOR PING
ISR(TIMER1_OVF_vect)
{
    volatile_timer1_overflows++;
}

ISR(ADC_vect)
{

}

void sendPing()
{
    ping_enable_isrs(0);
    ping_send_pulse(&volatile_ping_send_pulse_start_time);
    tmr1_clear_overflow_flag(); //We should clear the overflow flag in case it hasn't been done before
    tmr1_clear_capture_flag(); //This triggers the capture flag so we must clear it
    volatile_timer1_overflows = 0;
    volatile_ping_capture_complete = 0;
    ping_enable_isrs(1);
}

void doSweepLoop()
{
    sei();

    servo_data_t *servo = create_jim_servo();

    ir_init(ADC_ONE_64TH, ADC_AREF, 2);
    timer_prescaler_t prescaler = TIMER_ONE_1024TH;
    ping_init(prescaler);
    oi_t *oiSensor = malloc(sizeof(oi_t));
    oi_tare_encoders(&(oiSensor->left_encoder), &(oiSensor->right_encoder));
    int velocity = 0;
    int radius = 0;
    int leftWheelVelocity = 0;
    int rightWheelVelocity = 0;
    oi_full_mode();

    ir_enable_continous_mode();
    sendPing();

    printf0("creating stored ir sensor lookup table...\r\n");
    list_t *lookup_table = create_jims_ir_sensor_lookup_table();
    printf0("done...\r\n");
    while (1)
    {
        char c = '\0';

        char ping_available = 0;
        unsigned p_cm;
        if (volatile_ping_capture_complete)
        {
            ping_available = 1;
            unsigned long end_capture_count = tmr1_read_input_capture_count();
            unsigned long end_time_cap = (volatile_timer1_overflows << 16) | end_capture_count;
            unsigned long delta = end_time_cap - volatile_ping_send_pulse_start_time;
            p_cm = ping_count_to_cm(prescaler, delta);

            //send again
            sendPing();
        }

        int requestIrCalibration = 0;
        handleInput(servo, &leftWheelVelocity, &rightWheelVelocity, &requestIrCalibration);

        if (requestIrCalibration)
        {
            lfreefree(lookup_table);
            cli();
            lookup_table = create_ir_lookup_table_from_ping(servo, prescaler);
            sei();
        }

        unsigned voltage = ir_read_voltage_avg(1);
        unsigned calculatedDist = ir_lookup_distance(lookup_table, voltage);
        double ir_cm = calculatedDist * 0.254;
        ir_cm = calculatedDist / 10;
        char buff[200];
        ftoa(buff, ir_cm);

        //unsigned p_cm; //= ping_cm_busy_wait(prescaler);
        //p_cm = ping_count_to_cm(prescaler, volatile_timer1_capture_count);

        unsigned curDeg = servo_calculate_position_deg(servo);

        //printf0("%d volts \t\t calculatedDist= %u \r\n", voltage, calculatedDist);
        printf0("%uº \t %s ir_cm [%u v]\t", curDeg, buff, voltage);
        printf0(" pw=%u\t", servo->cur_pulse_width);
        if (ping_available)
            printf0("%u p_cm\t", p_cm);
        printf0("\r\n");
    }
}

int main(void)
{

    usart0_init(25, 8, 1, USART_PARITY_EVEN,1); //38400
    usart1_init(25, 8, 1, USART_PARITY_DISABLED,0); //TODO check oi_alternative OI_ALTERNATE_BAUD_RATE

    _delay_ms(333);
    oi_switch_baud_rate();
    _delay_ms(333);
    oi_init();
    oi_full_mode();
    int i;
    int val;
    for (i = 0; i < 10; i++)
    {
        val = i % 2;
        oi_set_leds(val, val, val, val, 0xFF * val, 0xFF);
        _delay_ms(50);
    }
    oi_init();

    //input_capture_test();
    printf0("Hello world!\r\n");
    printf0(" 0x%02X 0x%02X 0x%02X", UCSR0A, UCSR0B, UCSR0C);
    _delay_ms(3000);
    doSweepLoop();
    doPingLoop();
    doIrLoop();
    servo_test();
}
