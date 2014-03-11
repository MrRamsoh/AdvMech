#include <TaskManager.h>
#include <RobotController.h>

#include <IRSensor.h>

/*****************************************/

// Front facing IR Sensor connected to Pin A2
// Left facing front IR Sensor connected to Pin A0
// Right facing rear IR Sensor connected to Pin A3
IRSensor front_ir(A2);
IRSensor left_ir(A0);
IRSensor right_ir(A3);

void setup() 
{
  Serial.begin(9600);
}

void loop()
{
  computeIR();
  Serial.print(front_ir_distance);
  Serial.print("\t");
  Serial.print(left_ir_distance);
  Serial.print("\t");
  Serial.print(right_ir_distance);
  Serial.print("\t\t");
  Serial.print(both_difference_distance);
  Serial.print("\t");
  Serial.print(left_difference_distance);
  Serial.print("\t");
  Serial.println(right_difference_distance);
}

void computeIR()
{
  front_ir_distance = front_ir.getDistance(); 
  left_ir_distance = left_ir.getDistance(); 
  right_ir_distance = right_ir.getDistance();
  
  both_difference_distance = left_ir_distance - right_ir_distance;
  left_difference_distance = left_ir_distance - FOLLOW_DISTANCE;
  right_difference_distance = right_ir_distance - FOLLOW_DISTANCE;
}
