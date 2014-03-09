#include <TaskManager.h>
#include <Gyro.h>
#include <Wire.h>
#include <Encoder.h>
#include <Motor.h>
#include <RobotController.h>
#include <PID.h>

#define GYROSAMPLETIME 10

Gyro gyro;

float maxRate;
float angle;

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

void setup() {
Serial.begin(115200);

gyro=Gyro(ENABLEZ,SCALE250,GYROSAMPLETIME);
TaskInit();
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

Serial.println("calibrating");
gyro.calibrateZ();

TaskRegister(&Gyro::staticComputeZ,(int)&gyro,GYROSAMPLETIME*10,TRUE);
delay(50);
  TaskRegister(&Encoder::staticCompute,(int)&left_encoder,T20MS,TRUE);
  delay(50);
  TaskRegister(&Encoder::staticCompute,(int)&right_encoder,T20MS,TRUE);
  delay(50);
}

void loop() {
delay(50);
turnRight90EncoderFast();
 Serial.print("noise: "); 
 int n = floor((gyro.getNoise() * 100.0) + 0.5);
 Serial.print(n);
 Serial.print("\tangle: "); 
 Serial.print(gyro.getAngle()); 
 Serial.print("\tmaxrate: "); 
 Serial.println(maxRate); 
 while(1){}
 
 
}




void turnLeft90EncoderFast()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);  
  resetEncoder();
  
  while(abs(left_encoder_distance) < 620 && right_encoder_distance < 620)
  {
    left_motor_setpoint = -1200;
    right_motor_setpoint = 1200;
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
  angle =gyro.getAngle()+90;
  while(gyro.getAngle()<angle)
  {
    left_motor_setpoint = 800;
    right_motor_setpoint = -800;
    computeEncoderPID();
    setMotors();
    //if (gyro.getRate()>maxRate) maxRate=gyro.getRate();
  }
  stop();
  
  if (robot_direction == 0) {robot_direction = 1;}
  else if (robot_direction == 1) {robot_direction = 2;}
  else if (robot_direction == 2) {robot_direction = 3;}
  else if (robot_direction == 3) {robot_direction = 0;}
}

void resetEncoder()
{
  left_encoder_distance = 0;
  right_encoder_distance = 0;
}

/* stop()
 * Stops the robot from moving and keeps it stopped using speed control
***************************************************************************/
void stop()
{
  computeEncoderPID();
  left_PID.SetMode(AUTOMATIC);
  right_PID.SetMode(AUTOMATIC);  
  left_motor_PWM = STOP;
  right_motor_PWM = STOP;
  setMotors();
}

void computeEncoderPID()
{
  right_PID.Compute();
  left_PID.Compute();
}

void setMotors()
{
  left_motor.setSpeed(left_motor_PWM);
  right_motor.setSpeed(right_motor_PWM); 
}

void updateLeftEncoder()
{
  left_encoder.tick();
}
void updateRightEncoder()
{
  right_encoder.tick();
}

  
