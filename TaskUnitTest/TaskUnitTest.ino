#include <Encoder.h>
#include <Motor.h>
#include <RobotController.h>
#include <PID.h>
#include <TaskManager.h>

//Motor left_motor(8, 10, false);    // Left Motor, Dir pin 8, PWM pin 10, direction revered
//Motor right_motor(7, 9, true);   // Right Motor, Dir pin 7, PWM pin 9, direction not reversed

Encoder left_encoder(2, 4, &left_encoder_distance, &left_encoder_velocity, false);
Encoder right_encoder(3, 5, &right_encoder_distance, &right_encoder_velocity, true);

//PID left_PID(&left_encoder_velocity, &left_motor_PWM, &left_motor_setpoint, KP, KI, KD, DIRECT);
//PID right_PID(&right_encoder_velocity, &right_motor_PWM, &right_motor_setpoint, KP, KI, KD, DIRECT);

unsigned long last=0;
void setup() 
{
  left_encoder.setHighPinA();
  left_encoder.setLowPinB();
  right_encoder.setHighPinA();
  right_encoder.setHighPinB();
//  
//  left_PID.SetMode(AUTOMATIC);
//  left_PID.SetOutputLimits(-255, 255);
//  right_PID.SetMode(AUTOMATIC);
//  right_PID.SetOutputLimits(-255, 255);  
  
//  attachInterrupt(0, updateLeftEncoder, CHANGE);
//  attachInterrupt(1, updateRightEncoder, CHANGE);

TaskInit();
TaskRegister(&test,-1,T1S,TRUE);
TaskStop();

  Serial.begin(9600);
  pinMode(12,INPUT);
  digitalWrite(12,HIGH);
  while(digitalRead(12)){}
//   left_motor_setpoint=-4000;
//   time=millis()-10000;
TaskStart();
}

void loop()
{
delay(1000);
}

void updateLeftEncoder()
{
  left_encoder.tick();
}
void updateRightEncoder()
{
  right_encoder.tick();
}
void test (int garbage)
{
  
  Serial.println((int)(millis()-last-1000));
  last=millis();
}
