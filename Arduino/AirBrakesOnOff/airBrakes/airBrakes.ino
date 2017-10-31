
#include <Servo.h>

//const int magSwitch = 2;
//const int ledStandby = 7;

#define VERBOSE true

// Offset to center servos about fins;
const int offset_s1 = 82;
const int offset_s2 = 87;
const int offset_s3 = 90;

const int offset_b1 = 57;
const int offset_b2 = 55;
const int offset_b3 = 62;

const int period = 20; // refresh rate [ms]

unsigned long Time, standbyTime;
//Assign signal application points


unsigned long stepSignal_time1 = 1500;
unsigned long stepSignal_time2 = 3500;
unsigned long brakeSignal_time = 5000;
unsigned long brakeSignal_height = 300;

enum State{
  READY,
  MOTOR,
  PHASE1,
  BRAKES,
  PHASE2
};
float h0;
// brake angles
int bangle = 80;
int angle = 25;
Servo servo_1;
Servo servo_2;
Servo servo_3;
Servo servo_b1;
Servo servo_b2;
Servo servo_b3;
void setup(){
  //brakes

  servo_1.attach(9);
  servo_2.attach(10);
  servo_3.attach(11);
  servo_b1.attach(3);
  servo_b2.attach(5);
  servo_b3.attach(6);

  pinMode(13, OUTPUT);
  Serial.begin(115200);

  //Time in millisecons

  servo_1.write(offset_s1);
  servo_2.write(offset_s2);
  servo_3.write(offset_s3);
  servo_b1.write(offset_b1); //50 TO 70 G00D
  servo_b2.write(offset_b2);
  servo_b3.write(offset_b3);
  standbyTime = millis();
  delay(500);

  h0 = 347; // what ever the preasure sensor gives
  #if VERBOSE
  Serial.println("Ready");
  #endif

  //Serial.print(1);


}


void loop() {
  static char state = READY;
  static long last = 0;
  static long lastState = 0;
  static String buffAcc = "";
  static String buffAlt = "";
  // to do the loop periodically, maybe not mandatory

  //while(millis()-last<period);
  //last = millis();
  char c;
  // get the altitude and the acceleration from the module in the nose cone

  /*
  while(c = Serial.read()!='a');
  float a = Serial.parseFloat()/208.77;
  Serial.read();
  float altitude = Serial.parseFloat();*/

  //while(c = (char)Serial.read()!='h') buffAcc += c;
  //while(c = (char)Serial.read()!='\n') buffAlt += c;<

  //float a = Serial.parseFloat()/208.77; //7 = 2048/9.81
  //float altitude = Serial.parseFloat();
  //Serial.println(a);
  //Serial.println(altitude);
  //Serial.println(Serial.read());
  //Serial.print(1);

  switch(state){
    case READY:

      digitalWrite(13, HIGH);

      // détection d'accélération
      // TODO ajputer un temps d'accélération, peut etre avec un état intermédiaire
      // TODO conversion de l'acceleration
      // WARN sens de l'accélération positive

      if (Serial.read() == '1'){
        state = MOTOR;
        #if VERBOSE
        Serial.println("MOTOR");
        #endif
        lastState = millis();
      }
      break;

    case MOTOR:
      digitalWrite(13, LOW);

      // timer ou fin de l'accélération
      if (Serial.read() == '0'){
        if(millis()-lastState < 500){///////////////////////////////////////////////////////77
          #if VERBOSE
          Serial.println("faux départ");
          #endif
          state = READY;
          break;
        }

        lastState = millis();
        state = BRAKES;
        #if VERBOSE
        Serial.println("BRAKES");
        #endif
      }
      if(millis()-lastState > 2200){
        lastState = millis();
        state = BRAKES;
        #if VERBOSE
        Serial.println("BRAKES");
        #endif
      }
      break;
    case PHASE1:
      // timer ou altitude; ou les deux
      if(millis()-lastState > 500){
        state = BRAKES;
        #if VERBOSE
        Serial.println("BRAKES");
        #endif
      }
      break;

    case BRAKES:
      servo_b1.write(offset_s1+bangle);
      servo_b2.write(offset_s2+bangle);
      servo_b3.write(offset_s3+bangle);
      state = PHASE2;
      #if VERBOSE
      Serial.println("PHASE2");
      #endif
      break;

    case PHASE2:
      break;
  }
}
