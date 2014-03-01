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
//IRSensor right_rear_ir(A0);

// Left Motor. Dir Pin 8, PWM Pin 10, direction not reversed
// Right Motor. Dir Pin 7, PWM Pin 9, direction reversed
Motor left_motor(8, 10, false);
Motor right_motor(7, 9, true);

// Left Encoder. Channel A Pin 2, Channel B Pin 4, direction reversed
// Right Encoder. Channel A Pin 3, Channel B Pin 5, direction not reversed
Encoder left_encoder(2, 4, &left_encoder_distance, &left_encoder_velocity, true);
Encoder right_encoder(3, 5, &right_encoder_distance, &right_encoder_velocity, false);

// Front Light Sensor conected to Pin A1
LightSensor light(A1);

int turnCount=0;

void setup() 
{
  left_encoder.setHighPinA();
  left_encoder.setLowPinB();
  right_encoder.setHighPinA();
  right_encoder.setHighPinB();
  
  attachInterrupt(0, updateLeftEncoder, CHANGE);
  attachInterrupt(1, updateRightEncoder, CHANGE);
  
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
  left_motor_PWM = 215;
  right_motor_PWM = 200;
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
  left_motor_PWM = 215 + 10 + (-40);
  right_motor_PWM = 200 + 10 + 40;
  setMotors();
}

void turnRight()
{
  turnCount=0;
  left_motor_PWM = 215 + 10 + 40;
  right_motor_PWM = 200 + 10 + (-40);
  setMotors();
}

void turnLeft90()
{
  left_motor_PWM = 215 + 10 + (-255);
  right_motor_PWM = 200 + 10 + 70;
  setMotors();
  delay(750);
  stop();
}
void turnRight90()
{
  turnCount++;
  left_motor_PWM = 215 + 10 + 70;
  right_motor_PWM = 200 + 10 + (-90);
  setMotors();
  delay(1000);
  stop();
}
void turnBack()
{
  left_motor_PWM = 215 + 10 + (-90)-255;
  right_motor_PWM = 200 + 10 + (70)-255;
  setMotors();
  delay(1000);
  stop();
}

void updateLeftEncoder()
{
  left_encoder.tick();
}
void updateRightEncoder()
{
  right_encoder.tick();
}
