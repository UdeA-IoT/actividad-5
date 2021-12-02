#include <Servo.h>

Servo myservo; // create servo object to control a servo

void setup(){
  myservo.attach(D8); // attach the servo on D8 = GPIO15
}

void loop(){
  int pos; // holds the position the servo should move to
  // goes from 0 degrees to 180 degrees
  // in steps of 1 degree
  for(pos = 0; pos <= 180; pos += 1){
    myservo.write(pos); // move servo to position in var pos
    delay(15); // waits 15ms to reach the position
  }
  // goes from 180 degrees to 0 degrees
  // in steps of 1 degree
  for(pos = 180; pos>=0; pos-=1) {
    myservo.write(pos); // move servo to position in var pos
    delay(15); // waits 15ms to reach the position
  }
}
