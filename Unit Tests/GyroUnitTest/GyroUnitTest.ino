#include <TaskManager.h>
#include <Gyro.h>
#include <Wire.h>

#define GYROSAMPLETIME 10

Gyro gyro;

void setup() {
Serial.begin(115200);

gyro=Gyro(ENABLEZ,SCALE500,GYROSAMPLETIME);
TaskInit();

Serial.println("calibrating");
gyro.calibrateZ();

TaskRegister(&Gyro::staticComputeZ,(int)&gyro,GYROSAMPLETIME*10,TRUE);
}

void loop() {
delay(50);
 Serial.print("noise: "); 
 int n = floor((gyro.getNoise() * 100.0) + 0.5);
 Serial.print(n);
 Serial.print("\tangle: "); 
 Serial.print(gyro.getAngle()); 
 Serial.print("\trate: "); 
 Serial.println(gyro.getRate()); 
}

  
