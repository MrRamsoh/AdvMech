#include <Encoder.h>
#include <RobotController.h>
#include <IRSensor.h>
#include <Motor.h>
#include <LightSensor.h>

IRSensor front_ir(A2);            // IR Sensor located at the front facing front. Analog 2
IRSensor right_front_ir(A3);      // IR Sensor located at the front facing right. Analog 3
//IRSensor right_rear_ir(A0);       // IR Sensor located at the rear facing right. Analog 0

Motor left_motor(8, 10, false);    // Left Motor, Dir pin 8, PWM pin 10, direction revered
Motor right_motor(7, 9, true);   // Right Motor, Dir pin 7, PWM pin 9, direction not reversed

LightSensor light(A1);

int turnCount=0;

void setup() 
{
  pinMode(12,INPUT);
  digitalWrite(12,HIGH);
  while(digitalRead(12)){}
}

void loop()
{
  computeIR();
  driveRobot();      // Main controller
}
