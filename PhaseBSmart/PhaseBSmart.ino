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

int turnCount=0;

void setup() 
{
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
  computeEncoderPID();
  driveRobot();      // Main controller
}

void driveRobot()
{
  // light isnt on so must find miner
  if (!light.isOn())
  {
    driveForward();
  }
  // light is on therefore miner found
  else stop();
}

/* driveForward()
 * Drives the robot continously forward in a straight line using speed control
***************************************************************************/
void driveForward()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);
  left_motor_setpoint = 1001;
  right_motor_setpoint = 1005;
  setMotors();
}

/* stop()
 * Stops the robot from moving and keeps it stopped using speed control
***************************************************************************/
void stop()
{
  left_PID.SetMode(AUTOMATIC);
  right_PID.SetMode(AUTOMATIC);  
  left_motor_PWM = STOP;
  right_motor_PWM = STOP;
  setMotors();
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
