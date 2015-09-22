#include "open_interface.h"
#include "usart/usart.h"
#include "blue_tooth_HC05.h"
#include <avr/io.h>
#include <util/delay.h>

void oi_load_sensor_data(oi_t * sensor)
{
	uint8_t packet_total = 7;

	oi_sendChar(OI_OPCODE_QUERY_LIST); //OPCODE
	oi_sendChar(packet_total); //Number of pakcets
	oi_sendChar(OI_PACKET_ID_DISTANCE);
	oi_sendChar(OI_PACKET_ID_ANGLE);
	oi_sendChar(OI_PACKET_ID_VOLTAGE);
	oi_sendChar(OI_PACKET_ID_RIGHT_REQUESTED_VELOCITY);
	oi_sendChar(OI_PACKET_ID_LEFT_REQUESTED_VELOCITY);
	oi_sendChar(OI_PACKET_ID_RIGHT_ENCODER_COUNTS);
	oi_sendChar(OI_PACKET_ID_LEFT_ENCODER_COUNTS);

	uint8_t distanceHighByte = oi_getChar(); //This blocks...wait till oi sends us the info
	uint8_t distanceLowByte = oi_getChar();
	uint8_t angleHighByte = oi_getChar();
	uint8_t angleLowByte = oi_getChar();
	uint8_t voltageHighByte = oi_getChar();
	uint8_t voltageLowByte = oi_getChar();
	uint8_t rr_velocity_h = oi_getChar();
	uint8_t rr_velocity_l = oi_getChar();
	uint8_t lr_velocity_h = oi_getChar();
	uint8_t lr_velocity_l = oi_getChar();
	uint8_t r_encoder_count_h = oi_getChar();
	uint8_t r_encoder_count_l = oi_getChar();
	uint8_t l_encoder_count_h = oi_getChar();
	uint8_t l_encoder_count_l = oi_getChar();

	sensor->distance = (distanceHighByte << 8) | distanceLowByte;
	sensor->angle = (angleHighByte << 8) | angleLowByte;
	sensor->voltage = (voltageHighByte << 8) | voltageLowByte;
	sensor->right_requested_velocity = (rr_velocity_h << 8) | rr_velocity_l;
	sensor->left_requested_velocity = (lr_velocity_h << 8) | lr_velocity_l;

	int16_t left_encoder_count = (l_encoder_count_h << 8) | l_encoder_count_l;
	int16_t right_encoder_count = (r_encoder_count_h << 8) | r_encoder_count_l;

	oi_update_encoder(&(sensor->left_encoder), left_encoder_count);
	oi_update_encoder(&(sensor->right_encoder), right_encoder_count);
}

void oi_update_encoder(oi_encoder_t * encoder, int16_t raw_input)
{

	int16_t half = 0x7FFF;

	int16_t tared_input = raw_input - encoder->encoder_count_tare;
	int16_t oldValue = encoder->encoder_count;
	if (oldValue > 0)
	{
		int16_t halfWay = (oldValue - half);
		if (tared_input < halfWay)
			encoder->encoder_revolution_count++;
	}
	else
	{
		int16_t halfWay = (oldValue + half);
		if (tared_input > halfWay )
			encoder->encoder_revolution_count--;
	}
	encoder->encoder_count = tared_input;
	encoder->encoder_count_raw = raw_input;
}

void oi_tare_encoders(oi_encoder_t * left_encoder, oi_encoder_t * right_encoder )
{
	oi_sendChar(OI_OPCODE_QUERY_LIST); //OPCODE
	oi_sendChar(2); //Number of pakcets
	oi_sendChar(OI_PACKET_ID_RIGHT_ENCODER_COUNTS);
	oi_sendChar(OI_PACKET_ID_LEFT_ENCODER_COUNTS);

	uint8_t r_encoder_count_h = oi_getChar();//This blocks...wait till oi sends us the info
	uint8_t r_encoder_count_l = oi_getChar();
	uint8_t l_encoder_count_h = oi_getChar();
	uint8_t l_encoder_count_l = oi_getChar();


	int16_t left_encoder_count = (l_encoder_count_h << 8) | l_encoder_count_l;
	int16_t right_encoder_count = (r_encoder_count_h << 8) | r_encoder_count_l;

	left_encoder->encoder_count_tare = left_encoder_count;
	left_encoder->encoder_count= 0;
	left_encoder->encoder_revolution_count= 0;
	left_encoder->encoder_count_raw= left_encoder_count;

	right_encoder->encoder_count_tare = right_encoder_count;
	right_encoder->encoder_count = 0;
	right_encoder->encoder_revolution_count= 0;
	right_encoder->encoder_count_raw= right_encoder_count;
}


void oi_switch_baud_rate()
{
	DDRD |= _BV(DDD1); /* setup D1 for output BAUD_RATE_CHANGE_PIN */

	PORTD |= _BV(PORTD1); /* set BRC high */
	_delay_ms(2000); /* wait 2 seconds */

	PORTD &= ~_BV(PORTD1); /* set BRC low */
	_delay_ms(100); /* wait 100 ms */

	PORTD |= _BV(PORTD1); /* set BRC high */
	_delay_ms(100); /* wait 100 ms */

	PORTD &= ~_BV(PORTD1); /* set BRC low */
	_delay_ms(100); /* wait 100 ms */

	PORTD |= _BV(PORTD1); /* set BRC high */
	_delay_ms(100); /* wait 100 ms */

	PORTD &= ~_BV(PORTD1); /* set BRC low */
	_delay_ms(100); /* wait 100 ms */

	PORTD |= _BV(PORTD1); /* set BRC high */
	_delay_ms(100); /* wait 100 ms */

}

uint8_t oi_isAvailable()
{
	return isAvailable1();
}

uint8_t oi_getChar()
{
	return getChar1();
}

void oi_sendChar(uint8_t c)
{
	sendChar1(c);
}

void oi_set_wheels(int16_t leftWheelVelocity, int16_t rightWheelVelocity)
{
	unsigned char l_high = leftWheelVelocity >> 8;
	unsigned char l_low = 0xFF & (leftWheelVelocity);

	unsigned char r_high = rightWheelVelocity >> 8;
	unsigned char r_low = 0xFF & (rightWheelVelocity);

	oi_sendChar(OI_OPCODE_DRIVE_WHEELS);
	oi_sendChar(r_high);
	oi_sendChar(r_low);
	oi_sendChar(l_high);
	oi_sendChar(l_low);
}

void oi_init()
{
	oi_sendChar(OI_OPCODE_START);
}

void oi_end()
{
	oi_sendChar(OI_OPCODE_END);
}

void oi_reset()
{
	oi_sendChar(OI_OPCODE_RESET);
}

void oi_safe_mode()
{
	oi_sendChar(OI_OPCODE_SAFE);
}

void oi_full_mode()
{
	oi_sendChar(OI_OPCODE_FULL);
}

void oi_passive_mode()
{
	oi_init();
}

void oi_seek_dock()
{
	oi_passive_mode(); //Set to passive mode for safe charging voltage levels
	oi_sendChar(143);
	oi_end();
}

void oi_set_leds(uint8_t check_robot_led, uint8_t dock_led, uint8_t spot_led, uint8_t debris_led, uint8_t power_color, uint8_t power_intensity)
{
	oi_sendChar(OI_OPCODE_LEDS);	// LED Opcode

	oi_sendChar(check_robot_led << 3 | dock_led << 2 | spot_led << 1 | debris_led);

	// Set the power led color
	oi_sendChar(power_color);

	// Set the power led intensity
	oi_sendChar(power_intensity);
}
