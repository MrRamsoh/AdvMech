#include "Arduino.h"
#include "LightSensor.h"

/* Constructor(Pin)
 * Pin = Digital connection pin on the Arduino
***************************************************************************/
LightSensor::LightSensor(int8_t Pin)
{
  pin = Pin;
  
  //pinMode(pin, INPUT);
  //digitalWrite(pin, HIGH);  
}

/* isOn()
 * Checks to see whether there is light near the light sensor or not. If there
 * is light, the function returns 1. If there is no light, it returns 0.
 * These values are reverse when they are read from the sensor. In other words,
 * when there is light present, the sensor reads 0 instead of 1. But we return 1.
***************************************************************************/
boolean LightSensor::isOn()
{
  int light = analogRead(pin);
  
  if (light>LIGHT_LEVEL) return true;
  else return false;
}

