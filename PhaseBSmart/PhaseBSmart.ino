#include <RobotController.h>

#include <Motor.h>
#include <IRSensor.h>
#include <LightSensor.h>

#include <Encoder.h>
#include <PID.h>

/*****************************************/

// Front facing IR Sensor connected to Pin A2
// Right facing front IR Sensor connected to Pin A3
// Right facing rear IR Sensor connected to Pin A0
IRSensor front_ir(A2);
IRSensor right_front_ir(A3);
// IRSensor right_rear_ir(A0);

// Left Motor. Dir Pin 8, PWM Pin 10, direction not reversed
// Right Motor. Dir Pin 7, PWM Pin 9, direction reversed
Motor left_motor(8, 10, false);
Motor right_motor(7, 9, true);

// Left Encoder. Channel A Pin 2, Channel B Pin 4, direction reversed
// Right Encoder. Channel A Pin 3, Channel B Pin 5, direction not reversed
Encoder left_encoder(2, 4, &left_encoder_distance, &left_encoder_velocity, true);
Encoder right_encoder(3, 5, &right_encoder_distance, &right_encoder_velocity, false);

// Left Motor PID Controller
// Right Motor PID Controller
PID left_PID(&left_encoder_velocity, &left_motor_PWM, &left_motor_setpoint, KP, KI, KD, DIRECT);
PID right_PID(&right_encoder_velocity, &right_motor_PWM, &right_motor_setpoint, KP, KI, KD, DIRECT);

// Front Light Sensor conected to Pin A1
LightSensor light(A1);

void setup() 
{
// Direction Up = 0, Right = 1, Down = 2, Left = 3
// X Right is positive, Left is negative
// Y Up is positive, Down is negative
  robot_direction = 0;
  robot_x = 0;
  robot_y = 0;

  left_encoder.setHighPinA();
  left_encoder.setLowPinB();
  right_encoder.setHighPinA();
  right_encoder.setHighPinB();
  
  left_PID.SetMode(AUTOMATIC);
  left_PID.SetOutputLimits(-255, 255);
  right_PID.SetMode(AUTOMATIC);
  right_PID.SetOutputLimits(-255, 255); 
  
  attachInterrupt(0, updateLeftEncoder, RISING);
  attachInterrupt(1, updateRightEncoder, RISING);
  
//  pinMode(12,INPUT);
//  digitalWrite(12,HIGH);
//  while(digitalRead(12)){}
}

void loop()
{
  driveRobot();      // Main controller
}

void driveRobot()
{
  // light isnt on so must find miner
  if (!light.isOn())
  {
    driveForwardCell();
    delay(500);
    driveForwardCell();
    delay(500);
    driveForwardCell();
    delay(500);
    driveForwardCell();
    delay(500);
    turnRight90EncoderFast();
    delay(500);
    driveForwardCell();
    delay(500);
    driveForwardCell();
    delay(500);
    turnRight90EncoderFast();
    delay(500);
    driveForwardCell();
    delay(500);
    driveForwardCell();
    delay(500);
    turnRight90EncoderFast();
    delay(10000);
  }
  // light is on therefore miner found
  else stop();
}

/* driveForward()
 * Drives the robot continously forward in a straight line using speed control
 *
 * Default values:
 * Left Setpoint = 1001
 * Right Setpoint = 1005
***************************************************************************/
void driveForward()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);  
  
  left_motor_setpoint = 1004;
  right_motor_setpoint = 1005;
  computeEncoderPID();
  setMotors();
}

/* driveForwardCell()
 * Drives the robot continously forward in a straight line using speed control
 * until the next cell is reached.
 *
 * Default values:
 * Left Setpoint = 1001
 * Right Setpoint = 1005
 * Next cell distance = 1550 
***************************************************************************/
void driveForwardCell()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);  
  resetEncoder();
  
  do
  { 
    left_motor_setpoint = 1001;
    right_motor_setpoint = 1005;
    computeEncoderPID();
    setMotors();
  } while(((left_encoder_distance + right_encoder_distance) / 2) < 1550);      
  stop();
  
  if (robot_direction == 0) {robot_y++;}
  else if (robot_direction == 1) {robot_x++;}
  else if (robot_direction == 2) {robot_y--;}
  else if (robot_direction == 3) {robot_x--;}
}

/* turnLeft90EncoderFast()
 * Turns the robot 90 degrees to the left using encoders as the limit of turn.
 * PID is used to keep the motors spinning at constant speed.
 *
 * Default values:
 * Turn limit = 620
 * Left Setpoint = -1000
 * Right Setpoint = 1000
***************************************************************************/
void turnLeft90EncoderFast()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);  
  resetEncoder();
  
  while(abs(left_encoder_distance) < 620 && right_encoder_distance < 620)
  {
    left_motor_setpoint = -800;
    right_motor_setpoint = 800;
    computeEncoderPID();
    setMotors();
  }
  stop();
  
  if (robot_direction == 0) {robot_direction = 3;}
  else if (robot_direction == 1) {robot_direction = 0;}
  else if (robot_direction == 2) {robot_direction = 1;}
  else if (robot_direction == 3) {robot_direction = 2;}
}

/* turnRight90EncoderFast()
 * Turns the robot 90 degrees to the right using encoders as the limit of turn.
 * PID is used to keep the motors spinning at constant speed.
 *
 * Default values:
 * Turn limit = 620
 * Left Setpoint = 1000
 * Right Setpoint = -1000
***************************************************************************/
void turnRight90EncoderFast()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);  
  resetEncoder();
  
  while(abs(left_encoder_distance) < 620 && right_encoder_distance < 620)
  {
    left_motor_setpoint = 800;
    right_motor_setpoint = -800;
    computeEncoderPID();
    setMotors();
  }
  stop();
  
  if (robot_direction == 0) {robot_direction = 1;}
  else if (robot_direction == 1) {robot_direction = 2;}
  else if (robot_direction == 2) {robot_direction = 3;}
  else if (robot_direction == 3) {robot_direction = 0;}
}

/* turnLeft90EncoderSlow()
 * Turns the robot 90 degrees to the left using encoders as the limit to turn.
 * PID is used to keep the motors spinning at the desired speed.
 * Desired speed is decreased as the robot approaches the completion of the
 * 90 degree turn.
 *
 * The desired speed is calculated using the following equation:
 * Turn speed = (Turn limit - average encoder distance) * (Multiplier / Turn limit)
 *
 * Default values:
 * Turn limit = 630
 * Multiplier = 1500
 * Left motor = negative direction
 * Right motor = positive direction
***************************************************************************/
void turnLeft90EncoderSlow()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);  
  resetEncoder();
  
  while(abs(left_encoder_distance) < 630 && right_encoder_distance < 630)
  {
    turn_speed = (630 - ((abs(left_encoder_distance) + right_encoder_distance) / 2)) * (1500 / 630);
    left_motor_setpoint = -turn_speed;
    right_motor_setpoint = turn_speed;
    computeEncoderPID();
    setMotors();
  }
  stop();
}

/* resetEncoder()
 * Resets the distances both the encoders have travelled to avoid overflow.
 * Also useful for programming using the incremental method instead of absolute.
***************************************************************************/
void resetEncoder()
{
  left_encoder_distance = 0;
  right_encoder_distance = 0;
//  left_encoder_velocity = 0;
//  right_encoder_velocity = 0;
}

/* stop()
 * Stops the robot from moving
***************************************************************************/
void stop()
{
  left_PID.SetMode(MANUAL);  
  right_PID.SetMode(MANUAL);  
  
  left_motor_PWM = STOP;
  right_motor_PWM = STOP;
  setMotors();
  delay(20);
}

void setMotors()
{
  left_motor.setSpeed(left_motor_PWM);
  right_motor.setSpeed(right_motor_PWM); 
}

void computeIR()
{
  front_ir_distance = front_ir.getDistance();
  right_front_ir_distance = right_front_ir.getDistance();
//  right_rear_ir_distance = right_rear_ir.getDistance();
  
//  average_distance = (right_front_ir_distance + right_rear_ir_distance) / 2;
//  difference_distance = abs(right_front_ir_distance - right_rear_ir_distance);
}

/* computeEncoderPID()
 * Calls the encoder compute function to calculate the speeds of the two motors.
 * Calls the PID compute function to calculate the correct PWM value to keep
 * the motor spinning at the desired velocity.
***************************************************************************/
void computeEncoderPID()
{
  right_encoder.compute();
  left_encoder.compute();
  
  right_PID.Compute();
  left_PID.Compute();
}

void updateLeftEncoder()
{
  left_encoder.tick();
}
void updateRightEncoder()
{
  right_encoder.tick();
}
