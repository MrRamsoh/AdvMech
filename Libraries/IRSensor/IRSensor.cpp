#include "Arduino.h"
#include "IRSensor.h"

/* Constructor(Pin)
 * Pin = Analog connection pin on the Arduino
***************************************************************************/
IRSensor::IRSensor(uint8_t Pin)
{
  pin = Pin;  
  pinMode(pin, INPUT); 
}

/* getValue()
 * Returns the raw value read from the IR Sensor after checking for validity.
 * If the raw value is between the Upper and Lower bounds of validity, then
 * the raw value is returned. If the value falls outside the bounds, 0 is returned.
***************************************************************************/
int IRSensor::getValue()
{
  int read_val = analogRead(pin);
  
//  delay(20);
  
  if (read_val > IR_LOWER_LIMIT && read_val < IR_UPPER_LIMIT) return read_val;
  else return NULL; 
}

/* getDistance()
 * Returns a distance to target read from the IR Sensor after checking for validity.
 * The raw value to distance conversion factor was determined by using the data sheet
 * and by taking several samples from the sensor. The data from the sensor was used to
 * generate an exponential regression where the input is the raw data and the output
 * is the distance to target in cm.
 * If the raw value is between the correct bounds, it is converted to cm and sent back.
 * If it falls outside the bounds, 0 is returned.
***************************************************************************/
float IRSensor::getDistance()
{
  int read_val = analogRead(pin);
  
//  delay(20);
  
  if (read_val > IR_LOWER_LIMIT && read_val < IR_UPPER_LIMIT) return (pow(read_val, -1.174) * 6069.2);
  else return NULL; 
}


