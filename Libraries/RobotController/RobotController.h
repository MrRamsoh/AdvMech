#include "Arduino.h"

#define NULL 0

#define FOLLOW_DISTANCE 5.5
#define FOLLOW_TOLERANCE 0.6
#define FRONT_STOP_DISTANCE 7
#define STRAIGHT_TOLERANCE 0.7
#define ONE_CELL_DISTANCE 1550

//#define KP 0.0625
#define KP 0.08
#define KI 0
#define KD 0

#define STOP 0
#define FORWARD_SPEED 2000
#define FORWARD_SPEED_SLOW 400
#define TURN_SPEED 200

float front_ir_distance;
float left_ir_distance;
float right_ir_distance;
	
float average_distance;
float both_difference_distance;
float left_difference_distance;
float right_difference_distance;

float increment;

volatile int16_t left_encoder_distance = 0;
volatile int16_t right_encoder_distance = 0;

volatile int16_t left_encoder_velocity = 0;
volatile int16_t right_encoder_velocity = 0;

volatile int16_t left_motor_PWM = 0;
volatile int16_t right_motor_PWM = 0;

volatile int16_t left_motor_setpoint = 0;
volatile int16_t right_motor_setpoint = 0;

unsigned long now = 0;
unsigned long start = 0;

int turn_speed;

// Direction Up = 0, Right = 1, Down = 2, Left = 3
// X Right is positive, Left is negative
// Y Up is positive, Down is negative
int robot_direction;
int robot_x;
int robot_y;
