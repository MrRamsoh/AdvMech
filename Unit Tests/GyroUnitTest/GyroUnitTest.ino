#include <Gyro.h>
#include <Wire.h>

#define GYROSAMPLETIME 10

Gyro gyro;
unsigned long time=0;
void setup() {
  Serial.begin(115200);
 
gyro=Gyro(ENABLEZ,SCALE500,GYROSAMPLETIME);

 Serial.println("calibrating");
 //gyro returns hundredths of degrees/sec

//print dc offset and noise level 
gyro.calibrateZ();

}

void loop() {
  // Every 10 ms take a sample from the gyro 
if(millis() - time > GYROSAMPLETIME) 
{ 
 time = millis(); // update the time to get the next sample 
 Gyro::staticComputeZ((int)&gyro); 
 
 Serial.print("noise: "); 
 Serial.print(gyro.getNoise());
 Serial.print("\tangle: "); 
 Serial.print(gyro.getAngle()); 
 Serial.print("\trate: "); 
 Serial.println(gyro.getRate()); 
}

  
}
