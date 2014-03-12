#include <Segment.h>

Segment test(A4,A5);
void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly: 
  for(int i=0; i<10;i++)
  {
  test.print(i,i);
  delay(1000);
  }

}
