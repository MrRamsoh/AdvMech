#ifndef IRSensor_h
#define IRSensor_h

#include "Arduino.h"

#define IR_UPPER_LIMIT 600
#define IR_LOWER_LIMIT 110
#define NULL 0

class IRSensor
{
  public:
    IRSensor(uint8_t Pin);
    int getValue();
    double getDistance();
    
  private:
    uint8_t pin;
};

#endif
