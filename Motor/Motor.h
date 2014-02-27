#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

#define IR_UPPER_LIMIT 600
#define IR_LOWER_LIMIT 110
#define NULL 0

class Motor
{
  public:
    Motor(int8_t PinDir, int8_t PinPWM, boolean Flip);
    void setSpeed(int Dir, int PWM);
    void setSpeed(int PWM);
    int testPWM(int PWM);
    
  private:
    int8_t pinDir;
    int8_t pinPWM;
    boolean flip;
};

#endif
