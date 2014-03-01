#include <Encoder.h>
#include <Motor.h>
#include <RobotController.h>

Motor left_motor(8, 10, false);    // Left Motor, Dir pin 8, PWM pin 10, direction revered
Motor right_motor(7, 9, true);   // Right Motor, Dir pin 7, PWM pin 9, direction not reversed

Encoder left_encoder(2, 4, &left_encoder_distance, &left_encoder_velocity, true);
Encoder right_encoder(3, 5, &right_encoder_distance, &right_encoder_velocity, false);

void setup() 
{
  left_encoder.setHighPinA();
  left_encoder.setLowPinB();
  right_encoder.setHighPinA();
  right_encoder.setHighPinB();
  
  attachInterrupt(0, updateLeftEncoder, CHANGE);
  attachInterrupt(1, updateRightEncoder, CHANGE);
  
  Serial.begin(9600);
  
  pinMode(12,INPUT);
  digitalWrite(12,HIGH);
  while(digitalRead(12)){}
}

void loop()
{
  if (!digitalRead(12))
  {
    left_encoder_distance = 0;
    right_encoder_distance = 0;
  }

  left_encoder.compute();
  right_encoder.compute();
  
  if (left_encoder_distance < 894 - 400) left_motor_PWM = 180;
  else if (left_encoder_distance < 894 - 200) left_motor_PWM = 120;
  else if (left_encoder_distance < 894 - 110) left_motor_PWM = 20;
  else left_motor_PWM = 0;
  
  if (right_encoder_distance < 894 - 400) right_motor_PWM = 180;
  else if(right_encoder_distance < 894 - 200) right_motor_PWM = 120;
  else if(right_encoder_distance < 894 - 110) right_motor_PWM = 20;
  else right_motor_PWM = 0;
  
  setMotors();  
  
  Serial.print(left_encoder_distance);
  Serial.print(" ");
  Serial.print(left_encoder_velocity);
  Serial.print(" ");
  Serial.print(right_encoder_distance);
  Serial.print(" ");
  Serial.println(right_encoder_velocity);  
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
