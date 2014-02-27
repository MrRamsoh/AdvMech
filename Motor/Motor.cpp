#include "Arduino.h"
#include "Motor.h"

/* Constructor(Direction Pin, PWM Pin, Direction Flip)
 * Direction Pin = Direction connection pin on the Arduino
 * PWM Pin = PWM connection pin on the Arduino
 * Direction Flip = Decides whether to spin clockwise or counter-clockwise as positive movement
***************************************************************************/
Motor::Motor(int8_t PinDir, int8_t PinPWM, boolean Flip)
{
  pinDir = PinDir;
  pinPWM = PinPWM;
  flip = Flip;
  
  pinMode(pinDir, OUTPUT); 
  pinMode(pinPWM, OUTPUT);
}

/* setSpeed()
 * This is the first of the two setSpeed() functions. It takes in a direction
 * and a PWM value from the user and sets the speed and direction of the motor
 * accordingly. This is not the desired way to control the motors if using PID.
***************************************************************************/
void Motor::setSpeed(int Dir, int PWM)
{
  PWM = testPWM(PWM);
  
  if (!flip) digitalWrite(pinDir, Dir);
  else if (flip && Dir) digitalWrite(pinDir, LOW);
  else if (flip && !Dir) digitalWrite(pinDir, HIGH);

  analogWrite(pinPWM, PWM);
}

/* setSpeed()
 * This is the second of the two setSpeed() functions. It takes in a value
 * that is between -255 and 255. A positive value means that the user wants
 * to drive forward. A negative value means that the user wants to drive
 * backwards. A value of 0 means they want to stop. This is the desired method
 * for setting speed if using PID.
***************************************************************************/
void Motor::setSpeed(int PWM)
{
  PWM = testPWM(PWM);
  
  if (PWM > 0)
  {
    if (!flip) digitalWrite(pinDir, HIGH);
    else digitalWrite(pinDir, LOW);
  }
  else
  {
    if (!flip) digitalWrite(pinDir, LOW);
    else digitalWrite(pinDir, HIGH);
  }
  analogWrite(pinPWM, abs(PWM));
}

/* testPWM()
 * Responsible for making sure that the PWM value sent to Arduino is valid.
 * Checks to see if the value is between -255 and 255.
***************************************************************************/
int Motor::testPWM(int PWM)
{
  if (PWM > 255) PWM = 255;
  if (PWM < -255) PWM = -255;
  
  return PWM;
}
