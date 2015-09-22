#include <inttypes.h>
#ifndef _OPEN_INTERFACE_H_
#define _OPEN_INTERFACE_H_

#define OI_DEFAULT_BAUD_RATE 115200
#define OI_ALTERNATE_BAUD_RATE 19200

#define OI_OPCODE_START            128
#define OI_OPCODE_END              173
#define OI_OPCODE_RESET              7
//#define OI_OPCODE_BAUD             129
//#define OI_OPCODE_CONTROL          130
#define OI_OPCODE_SAFE             131
#define OI_OPCODE_FULL             132
//#define OI_OPCODE_POWER            133
//#define OI_OPCODE_SPOT             134
//#define OI_OPCODE_CLEAN            135
//#define OI_OPCODE_MAX              136
//#define OI_OPCODE_DRIVE            137
//#define OI_OPCODE_MOTORS           138
#define OI_OPCODE_LEDS             139
//#define OI_OPCODE_SONG             140
//#define OI_OPCODE_PLAY             141
#define OI_OPCODE_SENSORS          142
#define OI_OPCODE_SEEK_DOCK        143
//
//#define OI_OPCODE_PWM_MOTORS       144
#define OI_OPCODE_DRIVE_WHEELS     145
//#define OI_OPCODE_DRIVE_PWM        146
//#define OI_OPCODE_OUTPUTS          147
//#define OI_OPCODE_STREAM           148
#define OI_OPCODE_QUERY_LIST       149
//#define OI_OPCODE_DO_STREAM        150
//#define OI_OPCODE_SEND_IR_CHAR     151
//#define OI_OPCODE_SCRIPT           152
//#define OI_OPCODE_PLAY_SCRIPT      153
//#define OI_OPCODE_SHOW_SCRIPT      154
//#define OI_OPCODE_WAIT_TIME        155
//#define OI_OPCODE_WAIT_DISTANCE    156
//#define OI_OPCODE_WAIT_ANGLE       157
//#define OI_OPCODE_WAIT_EVENT       158
//
//// Contains Packets 7-26
//#define OI_SENSOR_PACKET_GROUP0 0
//// Contains Packets 7-16
//#define OI_SENSOR_PACKET_GROUP1 1
//// Contains Packets 17-20
//#define OI_SENSOR_PACKET_GROUP2 2
//// Contains Packets 21-26
//#define OI_SENSOR_PACKET_GROUP3 3
//// Contains Packets 27-34
//#define OI_SENSOR_PACKET_GROUP4 4
//// Contains Packets 35-42
//#define OI_SENSOR_PACKET_GROUP5 5
//// Contains Packets 7-42
//#define OI_SENSOR_PACKET_GROUP6 6
#define OI_PACKET_ID_DISTANCE 19
#define OI_PACKET_ID_ANGLE 20

#define OI_PACKET_ID_VOLTAGE 22

#define OI_PACKET_ID_RIGHT_REQUESTED_VELOCITY 41
#define OI_PACKET_ID_LEFT_REQUESTED_VELOCITY 42
#define OI_PACKET_ID_LEFT_ENCODER_COUNTS 43
#define OI_PACKET_ID_RIGHT_ENCODER_COUNTS 44


typedef struct
{
		int16_t encoder_count;
		int16_t encoder_count_raw;
		int16_t encoder_revolution_count;

		int16_t encoder_count_tare;
} oi_encoder_t;



typedef struct
{

		int16_t distance; // in millimeters
		int16_t angle;    // in degrees; counterclockwise is positive; clockwise is negative

		uint16_t voltage; //voltage in millivolts

		int16_t right_requested_velocity;
		int16_t left_requested_velocity;

//		int16_t right_encoder_count;
//		int16_t left_encoder_count;

		oi_encoder_t left_encoder;
		oi_encoder_t right_encoder;

} oi_t;

void oi_update_encoder(oi_encoder_t * encoder, int16_t raw_input);
void oi_tare_encoders(oi_encoder_t * left_encoder, oi_encoder_t * right_encoder );

void oi_load_sensor_data(oi_t * sensor);

uint8_t oi_isAvailable();
uint8_t oi_getChar();
void oi_sendChar(uint8_t c);

void oi_switch_baud_rate();

void oi_init();
void oi_end();
void oi_reset();

void oi_safe_mode();
void oi_full_mode();
void oi_passive_mode();

void oi_seek_dock();

void oi_set_leds(uint8_t check_robot_led, uint8_t dock_led, uint8_t spot_led, uint8_t debris_led, uint8_t power_color, uint8_t power_intensity);

void oi_set_wheels(int16_t left_wheel, int16_t right_wheel);

#endif /* _OPEN_INTERFACE_H_ */
