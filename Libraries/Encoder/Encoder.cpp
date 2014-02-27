#include "Arduino.h"
#include "Encoder.h"

/* Constructor(PinA, PinB, Pointer to Distance, Pointer to Velocity, Direction Flip)
 * PinA = Encoder OutA connection pin on the Arduino
 * PinB = Encoder OutB connection pin on the Arduino
 * Pointer to Distance = Memory location where the distance is stored
 * Pointer to Velocity = Memory location where the velocity is stored
 * Direction Flip = Decides whether to count clockwise as positive or negative movement
***************************************************************************/
Encoder::Encoder(int8_t PinA, int8_t PinB, volatile int32_t *Ticks, volatile int32_t *Velocity, boolean Flip)
{
  pin_a = PinA;
  pin_b = PinB;
  position = Ticks;
  velocity = Velocity;
  flip = Flip;
  
  pinMode(pin_a, INPUT); 
  pinMode(pin_b, INPUT);
}

/* tick()
 * Responsible for calculating the position of the encoder.
 * It looks at the change in the two pins to decide whether the encoder turned
 * clockwise or counter-clockwise. Based on that, the position is either incremented
 * or decremented. If the flip direction boolean is enabled, then it increments
 * instead of decrement, and vice versa. This is needed because the two motors
 * turn in opposite directions to each other in order to go forward or reverse.
***************************************************************************/
void Encoder::tick()
{
  // Direction is flipped
  if (flip)
  {
    if (digitalRead(pin_a)) digitalRead(pin_b) ? (*position)-- : (*position)++;
    else digitalRead(pin_b) ? (*position)++ : (*position)--;
  }
  // Direction is not flipped
  else if (!flip)
  {
    if (digitalRead(pin_a)) digitalRead(pin_b) ? (*position)++ : (*position)--;
    else digitalRead(pin_b) ? (*position)-- : (*position)++;
  }
}

/* compute()
 * Responsible for calculating the velocity of the encoder.
 * It looks at how many ticks have gone by in 100ms. Then multiplies that number
 * by 10 to get how many ticks per 1000ms, which gives you ticks/s.
***************************************************************************/
void Encoder::compute()
{
  unsigned long now = millis();
  unsigned long time_change = (now - last_time);
  
  if(time_change >= 100)
  {
    float timeDialate = 100 / time_change;
	*velocity= (int)((*position - last_pos) * 10 * timeDialate);
    last_pos = *position;
    last_time = millis();
  } 
}

/* getPosition()
 * Returns the displacement of the encoder since the start.
***************************************************************************/
int32_t Encoder::getPosition() 
{ 
  return *position; 
}

/* setLowPinA(), setHighPinA, setLowPinB, setHighPinB
 * The following four functions set the encoder output pins to either the
 * pull up or pull down configuration. This was necessary for us because the two
 * encoders we are using are behaving differently even though they are the same.
 * They do not count properly if these values are not configured properly.
***************************************************************************/
void Encoder::setLowPinA()
{
  digitalWrite(pin_a, LOW);
}

void Encoder::setHighPinA()
{
  digitalWrite(pin_a, HIGH);
}

void Encoder::setLowPinB()
{
  digitalWrite(pin_b, LOW);
}

void Encoder::setHighPinB()
{
  digitalWrite(pin_b, HIGH);
}
