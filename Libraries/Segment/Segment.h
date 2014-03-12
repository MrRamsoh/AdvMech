#ifndef Segment_h
#define Segment_h

#include "Arduino.h"


class Segment
{
  public:
    Segment(uint8_t data, uint8_t clock);
    void print(int a, int b);    
  private:
    void shiftOut(int myDataPin, int myClockPin, byte myDataOut);
    byte getByte(int input);
    uint8_t sdt, sck;
};

#endif
