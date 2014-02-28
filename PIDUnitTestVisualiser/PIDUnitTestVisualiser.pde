int[] numbers = new int[1600];
int[] numbers2 = new int[1600];
int[] numbers3 = new int[1600];
 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
  float inByteA=0;
 float inByteB=0;
 float inByteC=0;
  String[] input;
  
void setup(){
  size(1600,960);
   // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, Serial.list()[0], 9600);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(200);

}

void draw(){
  background(200);
  
  
  
  fill(255);
  stroke(255);
  beginShape();
  for(int i = 0; i<numbers.length;i++){
    vertex(i,960-numbers[i]);
  }
  vertex(width,height/2);
  vertex(0,height/2);
  endShape();
  
  fill(127,221,0,100);
  stroke(127,221,0);
  beginShape();
  for(int i = 0; i<numbers2.length;i++){
    vertex(i,960-numbers2[i]);
  }
  vertex(width,height/2);
  vertex(0,height/2);
  endShape();

  fill(127,34,255,0);
  stroke(127,34,255);
  beginShape();
  for(int i = 0; i<numbers3.length;i++){
    vertex(i,960-numbers3[i]);
  }
  vertex(width,height);
  vertex(0,height);
  endShape();

}

 void serialEvent (Serial myPort) {
 // get the ASCII string:


 String inString = myPort.readStringUntil('\n');
inString = trim(inString);
 input = split(inString, ',');
 //println(input);
 
  if (inString != null&& input.length==3 ) {
 // trim off any whitespace:

 // convert to an int and map to the screen height:
 



println(input);

  inByteA = Float.parseFloat(input[0]);
inByteB = Float.parseFloat(input[1]);
inByteC = Float.parseFloat(input[2]);


 inByteA = map(inByteA, -4500, 4500, 0, height);
 inByteB = map(inByteB, -4500, 4500, 0, height);
 inByteC = map(inByteC, -255, 255, 0, height);
   for(int i = 1; i<numbers.length;i++){
   numbers[i-1] = numbers[i];
   }
numbers[numbers.length-1]=(int) inByteA;


   for(int i = 1; i<numbers2.length;i++){
   numbers2[i-1] = numbers2[i];
   }
numbers2[numbers2.length-1]=(int) inByteB;

   for(int i = 1; i<numbers3.length;i++){
   numbers3[i-1] = numbers3[i];
   }
numbers3[numbers3.length-1]=(int) inByteC;

 }
 }



