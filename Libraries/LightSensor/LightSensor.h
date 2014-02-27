#ifndef LightSensor_h
#define LightSensor_h

#include "Arduino.h"

#define LIGHT_LEVEL 650

class LightSensor
{
  public:
    LightSensor(int8_t Pin);
    boolean isOn();
    
  private:
    int8_t pin;
};

#endif
