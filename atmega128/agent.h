//
// Created by jim on 11/25/15.
//

#ifndef ATMEGA128_AGENT_H
#define ATMEGA128_AGENT_H


#include "servo.h"
#include "ping.h"
#include "open_interface.h"
#include "llist.h"


typedef struct INPUT_BUFFER
{
    //The actual buffer
    volatile char volatile *buffer;
    volatile char volatile index;
    volatile char volatile buffer_size;
    volatile char volatile rx_complete;
} buffer_t;

typedef struct AGENT_DATA
{
    servo_data_t *servo;
    ping_data_t *ping;
    oi_t *oi_sensor_data;

    volatile buffer_t volatile *buffer0;
    volatile buffer_t volatile *buffer1;


    llist_t *cmd_queue;


} agent_t;

void freeBuffer(buffer_t *buffer);

buffer_t volatile *newBuffer(unsigned size);

agent_t *newAgent(unsigned bufferSize);

void freeAgent(agent_t *agent);

void agent_update_servo_to_jims_servo(agent_t *agent);

#endif //ATMEGA128_AGENT_H
