#include <avr/io.h>
#include <util/delay.h>
#include "llist.h"
#include <avr/interrupt.h>
#include <stdlib.h>
#include "usart.h"
#include "open_interface.h"
#include "util.h"
#include "cmd_interpreter.h"

#define INPUT_BUFFER_SIZE 100
#define CMD_TERMINATOR ';'


volatile char input0_buffer[INPUT_BUFFER_SIZE];
volatile char input0_buffer_i;
volatile char rx_complete;

volatile buffer_t *cmd_buffer;
volatile int start_cmd = 0;

//volatile char cmd_buffer__buffer[INPUT_BUFFER_SIZE];
//volatile char cmd_buffer__index;

agent_t *agent;

void setupInput0BufferAndCmdQueue()
{
    cmd_buffer = newBuffer(INPUT_BUFFER_SIZE);

    start_cmd = 0;
    //cmd_buffer__buffer[0] = '\0';
    //cmd_buffer__index = 0;

    input0_buffer[0] = '\0';
    input0_buffer_i = '\0';
    rx_complete = 0;

    //agent->buffer0->rx_complete = 0;
}

void setupAgent()
{
    agent = newAgent(INPUT_BUFFER_SIZE);
    agent_update_servo_to_jims_servo((agent_t*)agent);
    servo_init(agent->servo);

    agent->ping->prescaler = TIMER_ONE_1024TH;
    ping_init(agent->ping);
}

void setupISRs()
{
    usart0_enable_isr_rx_complete(1);
    usart1_enable_isr_rx_complete(1);
    sei();
}

void setupIndicateReady()
{
///////////////////////TODO Remove
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
///////////////////////TODO Remove
    printf0("\r\n\r\n\r\n      Hello World!\r\n");
}

void setupCommunications()
{
    usart0_init(25, 8, 1, USART_PARITY_EVEN, 1); //38400 8MHz
    usart1_init(51, 8, 1, USART_PARITY_DISABLED, 1); //19200 8MHz
    _delay_ms(333);
    oi_switch_baud_rate();
    oi_init();
}

void setup(void)
{
    setupAgent();
    setupInput0BufferAndCmdQueue();
    setupCommunications();
    setupISRs();
    setupIndicateReady();
}

ISR(USART0_RX_vect)
{
    char c;
    sendChar0(c = getChar0());//parrot

    if (c == '\r')
        sendChar0('\n');

    /*agent->buffer0->buffer[agent->buffer0->index] = c;
    agent->buffer0->index = (agent->buffer0->index + 1) % agent->buffer0->buffer_size;
    agent->buffer0->buffer[agent->buffer0->index] = '\0';
    agent->buffer0->rx_complete = 1;*/


    input0_buffer[input0_buffer_i] = c;
    input0_buffer_i = (input0_buffer_i + 1) % INPUT_BUFFER_SIZE;//Loops around
    input0_buffer[input0_buffer_i] = '\0';
    rx_complete = 1;


    //agent->buffer0->rx_complete = 1;

}

/**
 * Copies data from input buffer to the cmd buffer
 * When it detects the CMD_TERMINATOR it pushes the
 * new command onto the cmd queue.
 */
void parseCommands()
{
    char c;
   /* while ((c = input0_buffer[cmd_buffer__index]) != '\0')
    {
        if (c == CMD_TERMINATOR || c == '\r' || c == '\n') //CR and LF always terminate
        {
            cmd_buffer__buffer[cmd_buffer__index] = '\0';//terminate string
            lladd(agent->cmd_queue,
                  newCStr((const char *) (cmd_buffer__buffer + start_cmd)));//add to our list of commands
            start_cmd = (cmd_buffer__index + 1) % INPUT_BUFFER_SIZE; //update the start pos of next cmd
        }
        else
        {
            cmd_buffer__buffer[cmd_buffer__index] = c;//transfer char to cmd buffer
        }

        cmd_buffer__index = (cmd_buffer__index + 1) % INPUT_BUFFER_SIZE;
        cmd_buffer__buffer[cmd_buffer__index] = '\0';
    }*/

    //while ((c = agent->buffer0->buffer[cmd_buffer->index]) != '\0')
    while ((c = input0_buffer[cmd_buffer->index]) != '\0')
    {
        if (c == CMD_TERMINATOR || c == '\r' || c == '\n') //CR and LF always terminate
        {
            cmd_buffer->buffer[cmd_buffer->index] = '\0';//terminate string
            lladd(agent->cmd_queue,
                  newCStr((const char *) (cmd_buffer->buffer + start_cmd)));//add to our list of commands
            start_cmd = (cmd_buffer->index + 1) % INPUT_BUFFER_SIZE; //update the start pos of next cmd
        }
        else
        {
            cmd_buffer->buffer[cmd_buffer->index] = c;//transfer char to cmd buffer
        }

        cmd_buffer->index = (cmd_buffer->index + 1) % INPUT_BUFFER_SIZE;
        cmd_buffer->buffer[cmd_buffer->index] = '\0';
    }
}


int main(void)
{
    setup();


    while (1)
    {
        //if (agent->buffer0->rx_complete)
        if (rx_complete)
        {
            //agent->buffer0->rx_complete = 0;
            rx_complete = 0;
            parseCommands();
        }

        char *next_cmd = llpop(agent->cmd_queue);
        if (next_cmd != NULL)
        {
            execute_command(next_cmd, agent);
            free(next_cmd);
            printf0("cmd list size = %d\r\n", llsize((llist_t *) agent->cmd_queue));
        }
    }
}
