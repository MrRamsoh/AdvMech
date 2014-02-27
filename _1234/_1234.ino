#include <RobotController.h>
#include <IRSensor.h>
#include <Motor.h>
#include <LightSensor.h>

IRSensor front_ir(A2);            // IR Sensor located at the front facing front. Analog 2
IRSensor right_front_ir(A3);      // IR Sensor located at the front facing right. Analog 3
//IRSensor right_rear_ir(A0);       // IR Sensor located at the rear facing right. Analog 0

Motor left_motor(8, 10, true);    // Left Motor, Dir pin 8, PWM pin 10, direction revered
Motor right_motor(7, 9, false);   // Right Motor, Dir pin 7, PWM pin 9, direction not reversed

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

void computeIR()
{
  front_ir_distance = front_ir.getDistance();
  right_front_ir_distance = right_front_ir.getDistance();
//  right_rear_ir_distance = right_rear_ir.getDistance();
  
//  average_distance = (right_front_ir_distance + right_rear_ir_distance) / 2;
//  difference_distance = abs(right_front_ir_distance - right_rear_ir_distance);
}

void driveRobot()
{
  // light isnt on so must find miner
  if (!light.isOn())
  {
    if(turnCount>4)
    {
      turnBack();
    }
    else if (right_front_ir_distance < 13 && right_front_ir_distance != NULL)
    {
      if(front_ir_distance<FRONT_STOP_DISTANCE && front_ir_distance!=NULL)
      {
         turnLeft90(); 
      }
      else
      {
      followRightWall();
      }
    }
    else
    {
      turnRight90();
    }
  }
  // light is on therefore miner found
  else stop();
}

void setMotors()
{
  left_motor.setSpeed(left_motor_PWM);
  right_motor.setSpeed(right_motor_PWM); 
}

void followRightWall()
{
  if (right_front_ir_distance < 6 && right_front_ir_distance != NULL)
  {
    turnLeft();
  }
  else
  {
    turnRight();
  }
}

void driveForward()
{
  left_motor_PWM = 115;
  right_motor_PWM = 100;
  setMotors();
}

void stop()
{
  left_motor_PWM = STOP;
  right_motor_PWM = STOP;
  setMotors();
  delay(50);
}

void turnLeft()
{
  turnCount=0;
  left_motor_PWM = 115 + 10 + (-40);
  right_motor_PWM = 100 + 10 + 40;
  setMotors();
}

void turnRight()
{
  turnCount=0;
  left_motor_PWM = 115 + 10 + 40;
  right_motor_PWM = 100 + 10 + (-40);
  setMotors();
}

void turnLeft90()
{
  left_motor_PWM = 115 + 10 + (-255);
  right_motor_PWM = 100 + 10 + 70;
  setMotors();
  delay(750);
  stop();
}
void turnRight90()
{
  turnCount++;
  left_motor_PWM = 115 + 10 + 70;
  right_motor_PWM = 100 + 10 + (-90);
  setMotors();
  delay(1000);
  stop();
}
void turnBack()
{
  left_motor_PWM = 115 + 10 + (-90)-255;
  right_motor_PWM = 100 + 10 + (70)-255;
  setMotors();
  delay(1000);
  stop();
}
