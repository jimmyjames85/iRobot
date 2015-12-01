//
// Created by jim on 11/24/15.
//
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "cmd_interpreter.h"
#include "usart.h"
#include "util.h"
#include "open_interface.h"
#include "movement.h"
#include "list.h"


#define HASHSIZE 101 //should be prime


unsigned hash(const char *s)
{
    unsigned hashval;

    /**
     * hash only chars before the first space
     * because spaces separate arguments
     */
    for (hashval = 0; *s != '\0'; s++)
        hashval = *s + 31 * hashval;
    return hashval % HASHSIZE;
}

/*
 * grabs the next token using strtok
 * and attempts to parse it as an integer
 *
 * dest - where to put the integer
 * delim - what separates the arguments
 *
 * returns
 *            true if successful
 *            false, otherwise
 *
 * */
int cmdtok_int(int *dest, const char *delim)
{
    char *cmd = strtok(NULL, delim);
    return (cmd != NULL && sscanf(cmd, "%d", dest) == 1);
}


////TODO write a display usage function
////TODO write a display usage function
////TODO write a display usage function
////TODO write a display usage function
////TODO write a display usage function


////TODO MUSIC idea STAR WARS!!!!!!!
////TODO MUSIC idea STAR WARS!!!!!!!
////TODO MUSIC idea STAR WARS!!!!!!!
////TODO MUSIC idea STAR WARS!!!!!!!
////TODO MUSIC idea STAR WARS!!!!!!!
////TODO MUSIC idea STAR WARS!!!!!!!

////TODO enable cmd immediate mode
////TODO enable cmd immediate mode
////TODO enable cmd immediate mode
////TODO enable cmd immediate mode
////TODO enable cmd immediate mode
////TODO enable cmd immediate mode
float distance_traveled(oi_encoder_t *encoder)
{
    //TODO include revolutions
    float distance = 0.444564997642 * encoder->encoder_count;
    return distance;
}

void immediate_command_center(agent_t *agent)
{
    usart0_enable_isr_rx_complete(0);
    printf0("IMMEDIATE MODE COMMAND MODE\r\n");

    char c = 0;
    //@suppress warning
    while (c != 'q')
    {
        if (isAvailable0())
        {
            c = getChar0();
            printf0("\r\n(0x%02X) You pressed: %c\r\n", c, c);

            switch (c)
            {
                case '0':
                    servo_set_position_deg(agent->servo, 0);
                    break;
                case '4':
                    servo_set_position_deg(agent->servo, 45);
                    break;
                case '9':
                    servo_set_position_deg(agent->servo, 90);
                    break;
                case '3':
                    servo_set_position_deg(agent->servo, 135);
                    break;
                case '8':
                    servo_set_position_deg(agent->servo, 180);
                    break;
                case '+':
                    servo_increment_degrees(agent->servo, 4);
                    break;
                case '-':
                    servo_decrement_degrees(agent->servo, 4);
                    break;
                case 'l':;
                    printf0("Lights Off\r\n");
                    oi_set_leds(0, 0, 0, 0, 0x00, 0x00);
                    break;
                case 'L':;
                    printf0("Lights On\r\n");
                    oi_set_leds(1, 1, 1, 1, 0xFF, 0xFF);
                    break;
                case 'F':;
                    printf0("Full Mode\r\n");
                    oi_full_mode();
                    break;
                case 'B':;
                    printf0("Changing OI Baud Rate to 19200\r\n");
                    oi_switch_baud_rate();
                    break;
                case 'I':;
                    printf0("Initiating OI..\r\n");
                    oi_init();
                    break;
                case '~':;
                    printf0("Reseting..\r\n");
                    oi_reset();
                    break;
                case 'e':
                    oi_load_sensor_data(agent->oi_sensor_data);
                    oi_set_wheels(agent->oi_sensor_data->left_requested_velocity+50, agent->oi_sensor_data->right_requested_velocity);
                    break;
                case 'd':
                    oi_load_sensor_data(agent->oi_sensor_data);
                    oi_set_wheels(agent->oi_sensor_data->left_requested_velocity-50, agent->oi_sensor_data->right_requested_velocity);
                    break;
                case 'r':
                    oi_load_sensor_data(agent->oi_sensor_data);
                    oi_set_wheels(agent->oi_sensor_data->left_requested_velocity, agent->oi_sensor_data->right_requested_velocity+50);
                    break;
                case 'f':
                    oi_load_sensor_data(agent->oi_sensor_data);
                    oi_set_wheels(agent->oi_sensor_data->left_requested_velocity, agent->oi_sensor_data->right_requested_velocity-50);
                    break;
                case ' ':;
                    oi_set_wheels(0,0);
                    break;
                case 'S':;
                    printf0("Loading Sensor data...\r\n\r\n");
                    oi_load_sensor_data(agent->oi_sensor_data);
                    printf0("distance: 0x%04X\r\n", agent->oi_sensor_data->distance);
                    printf0("angle: ox%04X\r\n", agent->oi_sensor_data->angle);
                    printf0("left requested velocity: 0x%04X\r\n", agent->oi_sensor_data->left_requested_velocity);
                    printf0("right requested velocity: 0x%04X\r\n", agent->oi_sensor_data->right_requested_velocity);
                    printf0("Voltage: %d mV\r\n", agent->oi_sensor_data->voltage);
                    printf0("\r\n\r\n");

                    printf0("Left Encoder Count Raw: %d \r\n", agent->oi_sensor_data->left_encoder->encoder_count_raw);
                    printf0("Right Encoder Count Raw: %d \r\n", agent->oi_sensor_data->right_encoder->encoder_count_raw);

                    printf0("\r\n\r\n");
                    printf0("Left Encoder Count: %d\t  Left Encoder Rev: %d\r\n",
                            agent->oi_sensor_data->left_encoder->encoder_count,
                            agent->oi_sensor_data->left_encoder->encoder_revolution_count);
                    printf0("Right Encoder Count: %d\t Right Encoder Rev: %d\r\n",
                            agent->oi_sensor_data->right_encoder->encoder_count,
                            agent->oi_sensor_data->right_encoder->encoder_revolution_count);
                    printf0("\r\n\r\n");

                    char strBuffer[30];
                    ftoa(strBuffer, distance_traveled(agent->oi_sensor_data->left_encoder));
                    printf0("Left Distance Traveled: %s\r\n", strBuffer);

                    ftoa(strBuffer, distance_traveled(agent->oi_sensor_data->right_encoder));
                    printf0("Right Distance Traveled: %s\r\n", strBuffer);
                    printf0("\r\n\r\n");
                    break;
                default:
                    break;
            }
        }
    }
    usart0_enable_isr_rx_complete(1);
}


/*
 * uses strtok to retrieve next token
 * */
void execute_servo_command(agent_t *agent)
{
    char *cmd = strtok(NULL, " ");
    int setVal;

    unsigned hashval = hash(cmd);
    switch (hashval)
    {
        case 78:;//setw
            if (cmdtok_int(&setVal, " "))
                servo_set_pulse_width(agent->servo, setVal);
            else
                printf0("Usage: SERVO SETW pulseWidth");
            break;
        case 88:; //set
            if (cmdtok_int(&setVal, " "))
            {
                servo_set_position_deg(agent->servo, setVal);
                printf0("set servo to pw: %d\r\n", agent->servo->cur_pulse_width);
            }
            else
                printf0("Usage: SERVO SET deg");
            break;
        case 89:; //init
            int minPulseWidth;
            int maxPulseWidth;
            if (cmdtok_int(&minPulseWidth, " ") && cmdtok_int(&maxPulseWidth, " "))
            {
                printf0("Initializing Servo: minPulseWidth = %d maxPulseWidth = %d\r\n", minPulseWidth, maxPulseWidth);
                servo_init(agent->servo);
                servo_calibrate(agent->servo, minPulseWidth, maxPulseWidth);
            }
            else
                printf0("Usage: SERVO INIT min_pw max_pw\r\n ");
            break;
        case 63:; //?
            /* no break */
        default:
            printf0("Usage: SERVO <INIT | SET | SETW>\r\n ");
            //TODO display all Usages for SERVO
            break;
    }
}

/*
 * uses strtok to retrieve next token
 * */
void execute_ping_command(agent_t *agent)
{

    char *cmd = strtok(NULL, " ");
    unsigned hashval = hash(cmd);

    switch (hashval)
    {
        case 33:; //CM
            int cm = ping_cm_busy_wait(agent->ping);
            printf0("%u  cm\r\n", cm);
            break;
        case 6:; //PING (with no args)
            /* no break */
        case 40:; //MM
            int mm = ping_mm_busy_wait(agent->ping);
            printf0("%u  mm\r\n", mm);
            break;
        case 89:; //PING INIT TODO implement a timer_prescaler argument ??maybe??
            ping_init(agent->ping);
            break;
        case 63:; //?
            /* no break */
        default:
            printf0("Usage: PING [INIT]\r\n", hashval);
            break;
    }
}

void execute_oi_set_mode_command(agent_t *agent)
{
    char *cmd = strtok(NULL, " ");
    unsigned hashval = hash(cmd);

    switch (hashval)
    {
        case 24:;//SAFE
            oi_safe_mode();
            printf0("SAFE MODE\r\n");
            break;
        case 27:;//PASSIVE
            oi_passive_mode();
            printf0("PASSIVE MODE\r\n");
            break;
        case 32:;//FULL
            oi_full_mode();
            printf0("FULL MODE\r\n");
            break;
        case 89:;//INIT
            oi_init();
            printf0("INIT MODE\r\n");
            break;
        case 63:; //?
            /* no break */
        default:
            printf0("Usage: OI SET MODE < SAFE | PASSIVE | FULL | INIT > \r\n");
            break;
    }
}
void execute_oi_send_command(agent_t *agent)
{
    int cmdList[100];//TODO decrease buffer size if needed
    int i;
    char size=0;
    while(cmdtok_int(&cmdList[size], " ,"))
        size++;

    printf0("Sending [ ");
    for(i=0;i<size;i++)
    {
        char c = ((char)cmdList[i]);
        sendChar1(c);
        printf0(" %d ", cmdList[i]);

    }
    printf0("]\r\nOI: ");

    while(isAvailable1())
        printf0("[ 0x%02 ]",getChar1());

    printf0("\r\n");

    sendChar1(142);
    sendChar1(46);
    printf0("[ 0x%02X%02X ]",getChar1(),getChar1());

 }

void execute_oi_set_command(agent_t *agent)
{
    char *cmd = strtok(NULL, " ");
    unsigned hashval = hash(cmd);

    switch (hashval)
    {
        case 82:;//MODE
            execute_oi_set_mode_command(agent);
            break;
        case 88:; //WHEELS
            int leftWheel, rightWheel;
            if (cmdtok_int(&leftWheel, " ") && cmdtok_int(&rightWheel, " "))
            {
                printf0("setting wheels...%d %d\r\n", leftWheel, rightWheel);
                oi_set_wheels(leftWheel, rightWheel);
            }
            else
                printf0("Usage: OI SET WHEELS <leftWheel> <rightWheel>\t(mm/s)\r\n");
            break;
        case 63:; //?
            /* no break */
        default:
            printf0("Usage: OI SET WHEELS <leftWheel> <rightWheel>\t(mm/s)\r\n");
            break;
    }
}

void execute_oi_command(agent_t *agent)
{
    char *cmd = strtok(NULL, " ");
    unsigned hashval = hash(cmd);

    switch (hashval)
    {
        case 88:; //set
            execute_oi_set_command(agent);
            break;
        case 75:;//send
            execute_oi_send_command(agent);
            break;
        case 63:; //?
            /* no break */
        default:
            printf0("Usage: oi [set] \r\n", hashval);
            break;
    }
}


void execute_cw_command(agent_t *agent)
{
    int deg;
    if (cmdtok_int(&deg, " "))
        turn_CW(agent->oi_sensor_data, deg);
    else
        printf0("Usage: cw <degrees>\r\n");
}

void execute_ccw_command(agent_t *agent)
{
    int deg;
    if (cmdtok_int(&deg, " "))
        turn_CCW(agent->oi_sensor_data, deg);
    else
        printf0("Usage: ccw <degrees>\r\n");
}

void printStatus(agent_t *agent)
{
    printf0("Servo \r\n");
    printf0("\t\tticks per ms = %d\r\n", agent->servo->ticks_per_ms);
}

void execute_command(const char *command, agent_t *agent)
{
    /**
     * immediately clone the incoming command because we use strtok
     * which can't modify the const char *command, but it can modify
     * a clone...
     */
    char *clone = newCStr(command);

    char *cmd = strtok(strupr(clone), " ");//TO UPPER to make it case insensitive

    unsigned hashval = hash(cmd);
    switch (hashval)
    {
        case 0: //empty string
            printf0("EMPTY STRING\r\n");
            break;
        case 2: //stop
            execute_command("oi set wheels 0 0\r\n", agent);
            break;
        case 3: //clear
            //Form feed; new page
            sendChar0(0x0C);//putty clears the terminal when it receives this character :-D
            break;
        case 5://ccw
            execute_ccw_command(agent);
            break;
        case 38: //rev
            printf0("REV!\r\n");
            break;
        case 43://cw
            execute_cw_command(agent);
            break;
        case 48:;//servo
            execute_servo_command(agent);
            break;
        case 55://fwd
            printf0("FWD!\r\n");
            break;
        case 83://fwd
            execute_ping_command(agent);
            break;
        case 84:;//immediate
            immediate_command_center(agent);
            break;
        case 85:;//print
            printStatus(agent);
            break;
        case 98://oi
            execute_oi_command(agent);
            break;

        case 63:;//?
            /* no break */
        default:
            printf0("\t\tUNKOWN COMMAND %s  [ hashval= %u ]\r\n", cmd, hashval); //display all
            break;
    }

    free(clone);


}

