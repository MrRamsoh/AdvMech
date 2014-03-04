#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

class Encoder
{

  public:
    Encoder(int8_t PinA, int8_t PinB, volatile int16_t *Ticks, volatile int16_t *Velocity, boolean Flip);
    void tick();
    void compute();
	static void staticCompute(int objectPointer);
    int32_t getPosition();
    void setPosition(const int32_t p);
    void setLowPinA();
    void setHighPinA();
    void setLowPinB();
    void setHighPinB();

  private:
    volatile int16_t *position; 
    volatile int16_t *velocity;	
    int8_t pin_a;
    int8_t pin_b;
    boolean flip;
    unsigned long last_time;
    uint16_t last_pos;
};

#endif
