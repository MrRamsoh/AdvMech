#include "Arduino.h"
#include "Segment.h"

Segment::Segment(uint8_t data, uint8_t clock)
{
	sdt=data;
	sck=clock;
}

void Segment::print(int a, int b)
{
	byte byteA,byteB;
	byteA = this->getByte(a);
	byteB = this->getByte(b);
	this->shiftOut(sdt,sck,byteB);
	this->shiftOut(sdt,sck,byteA);
}

byte Segment::getByte(int input)
{
	switch (input){
		case 0://gfedcba
		return 0b10000000;
		break;
		case 1:
		return 0b11110010;
		break;
		case 2:
		return 0b01001000;
		break;
		case 3:
		return 0b01100000;
		break;
		case 4:
		return 0b00110010;
		break;
		case 5:
		return 0b00100100;
		break;
		case 6:
		return 0b00000100;
		break;
		case 7:
		return 0b11110000;
		break;
		case 8:
		return 0b00000000;
		break;
		case 9:
		return 0b00110000;
		break;
	}
}

		
	


void Segment::shiftOut(int myDataPin, int myClockPin, byte myDataOut) {
  // This shifts 8 bits out MSB first, 
  //on the rising edge of the clock,
  //clock idles low

  //internal function setup
  int i=0;
  int pinState;
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, OUTPUT);

  //clear everything out just in case to
  //prepare shift register for bit shifting
  digitalWrite(myDataPin, 0);
  digitalWrite(myClockPin, 0);

  //for each bit in the byte myDataOut�
  //NOTICE THAT WE ARE COUNTING DOWN in our for loop
  //This means that %00000001 or "1" will go through such
  //that it will be pin Q0 that lights. 
  for (i=7; i>=0; i--)  {
    digitalWrite(myClockPin, 0);

    //if the value passed to myDataOut and a bitmask result 
    // true then... so if we are at i=6 and our value is
    // %11010100 it would the code compares it to %01000000 
    // and proceeds to set pinState to 1.
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {	
      pinState= 0;
    }

    //Sets the pin to HIGH or LOW depending on pinState
    digitalWrite(myDataPin, pinState);
    //register shifts bits on upstroke of clock pin  
    digitalWrite(myClockPin, 1);
	
	delay(1);
    //zero the data pin after shift to prevent bleed through
    digitalWrite(myDataPin, 0);
	
	
	
  }
  

  //stop shifting
  digitalWrite(myClockPin, 0);
}
