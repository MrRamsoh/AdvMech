#include <TaskManager.h>
#include <RobotController.h>

#include <Motor.h>
#include <IRSensor.h>
// #include <LightSensor.h>

#include <Encoder.h>
#include <PID.h>

/*****************************************/

// Left Motor. Dir Pin 8, PWM Pin 10, direction not reversed
// Right Motor. Dir Pin 7, PWM Pin 9, direction reversed
Motor left_motor(8, 10, false);
Motor right_motor(7, 9, true);

// Front facing IR Sensor connected to Pin A2
// Left facing front IR Sensor connected to Pin A0
// Right facing rear IR Sensor connected to Pin A3
IRSensor front_ir(A2);
IRSensor left_ir(A0);
IRSensor right_ir(A3);

// Left Encoder. Channel A Pin 2, Channel B Pin 4, direction reversed
// Right Encoder. Channel A Pin 3, Channel B Pin 5, direction not reversed
Encoder left_encoder(2, 4, &left_encoder_distance, &left_encoder_velocity, true);
Encoder right_encoder(3, 5, &right_encoder_distance, &right_encoder_velocity, false);

// Left Motor PID Controller
// Right Motor PID Controller
PID left_PID(&left_encoder_velocity, &left_motor_PWM, &left_motor_setpoint, KP, KI, KD, DIRECT);
PID right_PID(&right_encoder_velocity, &right_motor_PWM, &right_motor_setpoint, KP, KI, KD, DIRECT);

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
  
//  pinMode(12,INPUT);
//  digitalWrite(12,HIGH);
//  while(digitalRead(12)){}
}

void loop()
{
  driveForwardCell();
  stop();
  delay(1000);
}

void driveForwardCell()
{
  resetEncoder();
  do
  {
    computeIR();
    left_motor_setpoint = 1001;
    right_motor_setpoint = 1005;
    computeEncoderPID();
    setMotors();
  } while(((left_encoder_distance + right_encoder_distance) / 2) < 1550);      
}

void computeEncoderPID()
{
//  right_encoder.compute();
//  left_encoder.compute();
  
  right_PID.Compute();
  left_PID.Compute();
}

void computeIR()
{
  front_ir_distance = front_ir.getDistance();
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

/* stop()
 * Stops the robot from moving and keeps it stopped using speed control
***************************************************************************/
void stop()
{
//  left_PID.SetMode(AUTOMATIC);
//  right_PID.SetMode(AUTOMATIC);  
  left_motor_PWM = STOP;
  right_motor_PWM = STOP;
  setMotors();
}

/* resetEncoder()
 * Resets the distances both the encoders have travelled to avoid overflow.
 * Also useful for programming using the incremental method instead of absolute.
***************************************************************************/
void resetEncoder()
{
  left_encoder_distance = 0;
  right_encoder_distance = 0;
}

void computeIR()
{
  front_ir_distance = front_ir.getDistance(); 
  left_ir_distance = left_ir.getDistance(); 
  right_ir_distance = right_ir.getDistance(); 
}
