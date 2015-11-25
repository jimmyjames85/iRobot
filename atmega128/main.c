#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "list.h"
#include "open_interface.h"
#include "movement.h"
#include "util.h"
#include "ping.h"

#define ENCODER_MULTIPLIER (4/9)


float distance_traveled(oi_encoder_t *encoder)
{
    //TODO include revolutions
    float distance = 0.444564997642 * encoder->encoder_count;
    return distance;
}

void commandCenterLoop_main()
{
    oi_t sensor;
    //oi_encoder_t right_encoder = { 0 };
    //oi_encoder_t left_encoder =	{ 0 };

    oi_tare_encoders(&(sensor.left_encoder), &(sensor.right_encoder));

    printf0("Hello world! ! !\r\n");

    int16_t velocity = 0;
    int16_t radius = 0;
    int16_t leftWheelVelocity = 0;
    int16_t rightWheelVelocity = 0;

    //@suppress warning
    while (1)
    {
        char c;
        while (oi_isAvailable())
        {
            sendChar0(oi_getChar());
        }

        if (isAvailable0())
        {
            c = getChar0();
            printf0("\r\n(0x%02X) You pressed: %c\r\n", c, c);

            if (c == 'B')
            {
                printf0("Changing OI Baud Rate to 19200\r\n");
                oi_switch_baud_rate();
                //init_USART1(OI_ALTERNATE_BAUD_RATE, F_CPU);
            }
            else if (c == 'I')
            {
                printf0("Initiating OI..\r\n");
                oi_init();
            }
            else if (c == '~')
            {
                printf0("Reseting..\r\n");
                oi_reset();
                //init_USART1(OI_DEFAULT_BAUD_RATE, F_CPU);
            }
            else if (c == 'l')
            {
                printf0("Led Test\r\n");
                oi_set_leds(1, 1, 1, 1, 0xFF, 0xFF);
            }
            else if (c == 'L')
            {
                printf0("Stop Led Test\r\n");
                oi_set_leds(0, 0, 0, 0, 0x00, 0x00);
            }
            else if (c == 'F')
            {
                printf0("Full Mode\r\n");
                oi_full_mode();
            }
            else if (c == 'e')
            {
                leftWheelVelocity += 50;
                oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
            }
            else if (c == 'd')
            {
                leftWheelVelocity -= 50;
                oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
            }
            else if (c == 'r')
            {
                rightWheelVelocity += 50;
                oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
            }
            else if (c == 'f')
            {
                rightWheelVelocity -= 50;
                oi_set_wheels(leftWheelVelocity, rightWheelVelocity);
            }
            else if (c == 't')
            {
                turn_CW(&sensor, -90);
            }
            else if (c == ' ')
            {
                radius = 0;
                velocity = 0;
                rightWheelVelocity = 0;
                leftWheelVelocity = 0;
                oi_set_wheels(0, 0);
            }
            else if (c == 'H')
            {
                printf0("Shutting down...\r\n\r\n");
                oi_seek_dock();
            }
            else if (c == 'S')
            {
                printf0("Loading Sensor data...\r\n\r\n");
                oi_load_sensor_data(&sensor);
                printf0("distance: 0x%04X\r\n", sensor.distance);
                printf0("angle: ox%04X\r\n", sensor.angle);
                printf0("left requested velocity: 0x%04X\r\n", sensor.left_requested_velocity);
                printf0("right requested velocity: 0x%04X\r\n", sensor.right_requested_velocity);
                printf0("Voltage: %d mV\r\n", sensor.voltage);
                printf0("\r\n\r\n");

                printf0("Left Encoder Count Raw: %d \r\n", sensor.left_encoder.encoder_count_raw);
                printf0("Right Encoder Count Raw: %d \r\n", sensor.right_encoder.encoder_count_raw);

                printf0("\r\n\r\n");
                printf0("Left Encoder Count: %d\t  Left Encoder Rev: %d\r\n", sensor.left_encoder.encoder_count,
                        sensor.left_encoder.encoder_revolution_count);
                printf0("Right Encoder Count: %d\t Right Encoder Rev: %d\r\n", sensor.right_encoder.encoder_count,
                        sensor.right_encoder.encoder_revolution_count);
                printf0("\r\n\r\n");

                char strBuffer[30];
                ftoa(strBuffer, distance_traveled(&(sensor.left_encoder)));
                printf0("Left Distance Traveled: %s\r\n", strBuffer);

                ftoa(strBuffer, distance_traveled(&(sensor.right_encoder)));
                printf0("Right Distance Traveled: %s\r\n", strBuffer);
                printf0("\r\n\r\n");
            }
            else if (c == 'T')
            {
                printf0("Taring...\r\n");
                oi_tare_encoders(&(sensor.left_encoder), &(sensor.right_encoder));
            }
            else if (c == '?')
            {
                uint16_t x = -1;
                int16_t y = -1;

                printf0("x = 0x%04X\ty = 0x%04X \r\n", x, y);
                printf0("x = %d\ty = %d\r\n", x, y);
                /*
                 uint16_t buffer_size = 2000;
                 oi_switch_baud_rate();
                 oi_reset();
                 oi_switch_baud_rate();

                 char iRobot_stats[buffer_size];
                 while(!oi_isAvailable());//wait till we are ready to receive data
                 int i=0;
                 while(oi_isAvailable() && i<buffer_size)
                 iRobot_stats[i++]=oi_getChar();

                 iRobot_stats[i]='\0';
                 printf0("STATS: %s\r\n",iRobot_stats);*/

            }
            else if (c >= '1' && c <= '9')
            {
                int dist_requested_cm = (c - '0') * 10;
                printf0("Traveling Foward %d cm...\r\n", dist_requested_cm);
                move_forward_cm(&sensor, dist_requested_cm);
            }
            /*			else if (c == 'C')	//Class hw lab ..etc
             {
             lab4_part1_clock_main();
             }
             else if (c == 'c')	//Class hw lab ..etc
             {
             lab4_part2_clock_main();
             }*/
        }
    }
}

void setup(void)
{
    usart0_init(25, 8, 1, USART_PARITY_EVEN, 1); //38400
    usart1_init(25, 8, 1, USART_PARITY_DISABLED, 0); //TODO check oi_alternative OI_ALTERNATE_BAUD_RATE

    _delay_ms(333);
    oi_switch_baud_rate();
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
}


void setup_onboard_leds(void)
{
    DDRA |= _BV(DDA0); /* set pin 0 of PORTA for output  */
    DDRA |= _BV(DDA1); /* set pin 1 of PORTA for output  (These are on-board leds)*/
}

void set_onboard_led(uint8_t led, uint8_t val)
{
    if (0 == led)
    {
        if (val)
            PORTA |= _BV(PORTA0); /* set pin 0 high to turn led on */
        else
            PORTA &= ~_BV(PORTA0); /* set pin 0 low to turn led off */
    }
    else if (1 == led)
    {
        if (val)
            PORTA |= _BV(PORTA1); /* set pin 0 high to turn led on */
        else
            PORTA &= ~_BV(PORTA1); /* set pin 0 low to turn led off */
    }
}


volatile char rx_complete;
volatile char tx_complete;
volatile char theChar;

ISR(USART0_RX_vect)
{
    rx_complete = 1;
    theChar = getChar0();
}

ISR(USART0_TX_vect)
{
    set_onboard_led(1,1);
    tx_complete=1;
}

int main(void)
{
    setup();
    setup_onboard_leds();
    printf0("#of consecutive_0s in 0x%0lX is %d\r\n", 213, 223);
    list_t *usartBuffer = lalloc();

    rx_complete = 0;
    tx_complete = 0;
    usart0_enable_isr_rx_complete(1);
    usart0_enable_isr_tx_complete(1);
    sei();


    while (1)
    {
        if (rx_complete)
        {
            ladd(usartBuffer, newChar(theChar));
            printf0(".");
            rx_complete = 0;
        }
        if(tx_complete)
        {
            tx_complete=0;
            set_onboard_led(1,0);
        }

        if (usartBuffer->length != 0)
        {
            if (*((char *) usartBuffer->arr[usartBuffer->length - 1]) == '\r')
            {
                printf0("\r\nYou sent: ");
                int i = 0;
                for (i = 0; i < usartBuffer->length; i++)
                    sendChar0(*(char *) (lget(usartBuffer, i)));

                printf0("\r\n");
                lfreefree(usartBuffer);
                usartBuffer = lalloc();
            }
        }
    }


    return 0; /* CODE SHOULD NEVER REACH HERE */
}
