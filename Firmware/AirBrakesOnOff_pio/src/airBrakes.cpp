#include <Arduino.h>
#include <Servo.h>

#define VERBOSE false

// Offset to center servos about fins
//const int offset_s1 = 82;
//const int offset_s2 = 87;
//const int offset_s3 = 90;
// Offset to center servos about brakes
const int offset_b1 = 57-5;
const int offset_b2 = 55-5;
const int offset_b3 = 62-5;

const int bangle = 80;
const int angle = 25;

//Servo servo_1;
//Servo servo_2;
//Servo servo_3;
Servo servo_b1;
Servo servo_b2;
Servo servo_b3;
void setup(){

  //servo_1.attach(9);
  //servo_2.attach(10);
  //servo_3.attach(11);
  servo_b1.attach(3);
  servo_b2.attach(5);
  servo_b3.attach(6);

  pinMode(0, INPUT);
  // FIXME maybe a conflict with digitalRead on the pin 1

  //servo_1.write(offset_s1);
  //servo_2.write(offset_s2);
  //servo_3.write(offset_s3);
  servo_b1.write(offset_b1); //50 TO 70 G00D
  servo_b2.write(offset_b2);
  servo_b3.write(offset_b3);
  delay(500);
}


void loop() {
  if (digitalRead(0)){
    servo_b1.write(offset_b1+bangle);
    servo_b2.write(offset_b2+bangle);
    servo_b3.write(offset_b3+bangle);
  } else{
    servo_b1.write(offset_b1); //50 TO 70 G00D
    servo_b2.write(offset_b2);
    servo_b3.write(offset_b3);
  }
}
