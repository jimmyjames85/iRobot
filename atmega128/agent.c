//
// Created by jim on 11/25/15.
//

#include <stdlib.h>
#include "agent.h"
#include "open_interface.h"

void agent_update_servo_to_jims_servo(agent_t *agent)
{
    servo_calibrate(agent->servo, 68, 285);
}

buffer_t volatile *newBuffer(unsigned size)
{
    if (size == 0)
        return NULL;

    buffer_t *ret = malloc(sizeof(buffer_t));
    ret->buffer = malloc(size * sizeof(char));
    ret->buffer[0] = '\0';
    ret->index = 0;
    ret->rx_complete = 0;
    return ret;
}

void freeBuffer(buffer_t *buffer)
{
    free((void *) buffer->buffer);
    free(buffer);
}

agent_t *newAgent(unsigned bufferSize)
{
    agent_t *agent = malloc(sizeof(agent_t));
    agent->servo = newServo();

    agent->oi_sensor_data = newOISensorData();
    agent->ping = newPing();
    agent->ping->prescaler = TIMER_ONE_1024TH;
    agent->buffer0 = newBuffer(bufferSize);
    agent->buffer1 = newBuffer(bufferSize);
    agent->cmd_queue = llalloc();

    return agent;
}

void freeAgent(agent_t *agent)
{
    freeServo(agent->servo);
    freeOISensorData(agent->oi_sensor_data);
    freePing(agent->ping);
    freeBuffer((buffer_t *) agent->buffer0);
    freeBuffer((buffer_t *) agent->buffer1);
    llfreefree(agent->cmd_queue);
    free(agent);
}