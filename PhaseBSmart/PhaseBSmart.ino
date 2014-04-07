#include <TaskManager.h>
#include <RobotController.h>

#include <Motor.h>
#include <IRSensor.h>
#include <LightSensor.h>

#include <Encoder.h>
#include <PID.h>

#include <Segment.h>

/*****************************************/

// Front facing IR Sensor connected to Pin A2
// Left facing front IR Sensor connected to Pin A0
// Right facing rear IR Sensor connected to Pin A3
IRSensor front_ir(A2);
IRSensor left_ir(A0);
IRSensor right_ir(A3);

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

Segment cell_display(A4,A5);

void setup() 
{
// Direction Up = 0, Right = 1, Down = 2, Left = 3
// X Right is positive, Left is negative
// Y Up is positive, Down is negative
  robot_direction = 0;
  robot_x = 1;
  robot_y = 0;
  
  cell_display.print(robot_x, robot_y);

// Initialize the encoder PinA and PinB
  left_encoder.setHighPinA();
  left_encoder.setLowPinB();
  right_encoder.setHighPinA();
  right_encoder.setHighPinB();

// Initialize PID Controller for motor PWM  
  left_PID.SetMode(AUTOMATIC);
  left_PID.SetOutputLimits(-255, 255);
  right_PID.SetMode(AUTOMATIC);
  right_PID.SetOutputLimits(-255, 255); 

// Create interrupts for encoder ticks  
  attachInterrupt(0, updateLeftEncoder, RISING);
  attachInterrupt(1, updateRightEncoder, RISING);

// Initialize Task Manager  
  TaskInit();
// Register a task to automatically calculate encoder distance travelled and velocity
  TaskRegister(&Encoder::staticCompute,(int)&left_encoder,T20MS,TRUE);
  delay(50);
  TaskRegister(&Encoder::staticCompute,(int)&right_encoder,T20MS,TRUE);
  delay(50);
// Register a task to automatically calculate all the IR distances
  TaskRegister(&computeIR,-1,T100MS,TRUE);
  delay(50);
}

void loop()
{
  driveRobot();      // Main function to drive the robot
}

void driveRobot()
{
  for(int i = 0; i < 4; i++)
  {
    driveForwardCell();
  } 
  stop(60);  
  turnLeft90EncoderFast();
  stop(60);
  for(int i = 0; i < 4; i++)
  {
    driveForwardCell();
  } 
  stop(60);
  turnLeft90EncoderFast();
  stop(60);
  turnLeft90EncoderFast();
  stop(60);
  for(int i = 0; i < 4; i++)
  {
    driveForwardCell();
  } 
  stop(60);
  turnRight90EncoderFast();
  stop(60);
  for(int i = 0; i < 4; i++)
  {
    driveForwardCell();
  } 
  stop(60);    

  delay(100000);
}

/* driveForwardCell()
 * Drives the robot continously forward in a straight line using speed control
 * until the next cell is reached.
 *
 * Default values:
 * Left Setpoint = 1001
 * Right Setpoint = 1005
 * One cell distance = 1550 
***************************************************************************/
void driveForwardCell()
{
  left_PID.SetMode(AUTOMATIC);  
  right_PID.SetMode(AUTOMATIC);
  
  int x = 0;

  correct:

  resetEncoder();
  
  do
  { 
    if (left_ir_distance != NULL && right_ir_distance == NULL)
    {
      if (left_difference_distance < 0)
      {
        left_motor_setpoint = UPPER;
        right_motor_setpoint = LOWER;
      }
      else
      {
        left_motor_setpoint = LOWER;
        right_motor_setpoint = UPPER;
      }
    }
    else if (left_ir_distance == NULL && right_ir_distance != NULL)
    {
      if (right_difference_distance < 0)
      {
        left_motor_setpoint = LOWER;
        right_motor_setpoint = UPPER;
      }
      else
      {
        left_motor_setpoint = UPPER;
        right_motor_setpoint = LOWER;
      }
    }
    else
    {
      if (left_difference_distance < 0)
      {
        left_motor_setpoint = UPPER;
        right_motor_setpoint = LOWER;
      }
      else
      {
        left_motor_setpoint = LOWER;
        right_motor_setpoint = UPPER;
      }
    }

    computeEncoderPID();
    setMotors();
  } while((((left_encoder_distance + right_encoder_distance) / 2) < ONE_CELL_DISTANCE) && (front_ir_distance > FRONT_STOP_DISTANCE || front_ir_distance == NULL));      
//  stop();
  
  if (front_ir_distance > FRONT_STOP_DISTANCE && front_ir_distance < 12 && x == 0)
  {
    x = 1;
    goto correct;
  }
//  stop();
//  
//  if (robot_direction == 0) {robot_y++;}
//  else if (robot_direction == 1) {robot_x++;}
//  else if (robot_direction == 2) {robot_y--;}
//  else if (robot_direction == 3) {robot_x--;}
//  
//  cell_display.print(robot_x, robot_y);
}

void followLeft()
{
  increment = left_different_distance
}

void followRight()
{
  
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
  
  while(abs(left_encoder_distance) < 630 && right_encoder_distance < 630)
  {
    left_motor_setpoint = -800;
    right_motor_setpoint = 800;
    computeEncoderPID();
    setMotors();
  }
//  stop();
//  
//  if (robot_direction == 0) {robot_direction = 3;}
//  else if (robot_direction == 1) {robot_direction = 0;}
//  else if (robot_direction == 2) {robot_direction = 1;}
//  else if (robot_direction == 3) {robot_direction = 2;}
//  
//  cell_display.print(robot_x, robot_y);
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
//  stop();
//  
//  if (robot_direction == 0) {robot_direction = 1;}
//  else if (robot_direction == 1) {robot_direction = 2;}
//  else if (robot_direction == 2) {robot_direction = 3;}
//  else if (robot_direction == 3) {robot_direction = 0;}
  
//  cell_display.print(robot_x, robot_y);
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
void stop(int time)
{
  left_PID.SetMode(MANUAL);  
  right_PID.SetMode(MANUAL);  
  
  left_motor_PWM = STOP;
  right_motor_PWM = STOP;
  setMotors();
  delay(time);
}

void setMotors()
{
  left_motor.setSpeed(left_motor_PWM);
  right_motor.setSpeed(right_motor_PWM); 
}

/* computeEncoderPID()
 * Calls the encoder compute function to calculate the speeds of the two motors.
 * Calls the PID compute function to calculate the correct PWM value to keep
 * the motor spinning at the desired velocity.
***************************************************************************/
void computeEncoderPID()
{
  right_PID.Compute();
  left_PID.Compute();
}

void computeIR(int garbage)
{
  front_ir_distance = front_ir.getDistance(); 
  left_ir_distance = left_ir.getDistance(); 
  right_ir_distance = right_ir.getDistance();
  
  both_difference_distance = left_ir_distance - right_ir_distance;
  left_difference_distance = left_ir_distance - FOLLOW_DISTANCE;
  right_difference_distance = right_ir_distance - FOLLOW_DISTANCE;  
}

void updateLeftEncoder()
{
  left_encoder.tick();
}
void updateRightEncoder()
{
  right_encoder.tick();}
