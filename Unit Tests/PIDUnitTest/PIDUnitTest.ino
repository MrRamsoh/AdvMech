#include <TaskManager.h>
#include <Encoder.h>
#include <Motor.h>
#include <RobotController.h>
#include <PID.h>

Motor left_motor(8, 10, false);    // Left Motor, Dir pin 8, PWM pin 10, direction revered
Motor right_motor(7, 9, true);   // Right Motor, Dir pin 7, PWM pin 9, direction not reversed

Encoder left_encoder(2, 4, &left_encoder_distance, &left_encoder_velocity, true);
Encoder right_encoder(3, 5, &right_encoder_distance, &right_encoder_velocity, false);

PID left_PID(&left_encoder_velocity, &left_motor_PWM, &left_motor_setpoint, KP, KI, KD, DIRECT);
PID right_PID(&right_encoder_velocity, &right_motor_PWM, &right_motor_setpoint, KP, KI, KD, DIRECT);

unsigned long time=0;
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
  
  TaskInit();
  TaskRegister(&Encoder::staticCompute,(int)&left_encoder,T20MS,TRUE);
  delay(50);
  TaskRegister(&Encoder::staticCompute,(int)&right_encoder,T20MS,TRUE);
  delay(50);
  TaskRegister(&PID::staticCompute,(int)&left_PID,T100MS,TRUE);
  delay(50);
  TaskRegister(&PID::staticCompute,(int)&right_PID,T100MS,TRUE);
  TaskStop();
  
  Serial.begin(9600);
  pinMode(12,INPUT);
  digitalWrite(12,HIGH);
  while(digitalRead(12)){}
  TaskStart();
   left_motor_setpoint=-1000;
   time=millis()-5000;
}

void loop()
{
  

  Serial.print(left_motor_setpoint);
  Serial.print(",");
  Serial.print(left_encoder_velocity);
  Serial.print(",");
  Serial.println(left_motor_PWM);
  delay (10);
  left_motor.setSpeed(left_motor_PWM);
  if(millis()>time+5000) {left_motor_setpoint+=200; time=millis();}
  //if(left_encoder_distance >= 2980) {left_motor.setSpeed(0); delay(500); left_motor.setSpeed(-100);}
  //if(left_encoder_distance <0) {left_motor.setSpeed(0); delay(500);  left_motor.setSpeed(100);}
}

void updateLeftEncoder()
{
  left_encoder.tick();
}
void updateRightEncoder()
{
  right_encoder.tick();
}
