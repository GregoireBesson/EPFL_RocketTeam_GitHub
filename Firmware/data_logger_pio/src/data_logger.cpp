#include <avr/pgmspace.h>
#include <Arduino.h>
#include "data_logger.h"

//select the chip (depend on the SD shield, put 10 for Adafruit)
const int chipSelect = 10;
const int button = 3;
const int led = 8;
const int error_led = 7;
const unsigned long period = 20 * 1000; // period in microseconds
bool writeToSD = false;

Adafruit_BMP280 bmp; // I2C
File myFile;
SoftwareSerial *telemSerial;

String DELIMITER;

// Initializations
void setup() {

    DELIMITER = F(",\t");

    //Xbee serial: RX = digital pin 5, TX = digital pin 6
    telemSerial = new SoftwareSerial(5, 6);
    telemSerial->begin(57600);

    // Arduino initialization
    Wire.begin();
    Serial.begin(115200);

    // Configure gyroscope range
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
    // Configure accelerometers range
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_8_G);
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

    if (!bmp.begin()) {
#if VERBOSE
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring! (or adress!!!)"));
#endif
        while (1);
    }

    //seaLevelhPa = bmp.readPressure() / (100 * pow(1 - (altInit / 44330), 1 / 0.1903));

#if VERBOSE
    Serial.print("\nInitializing SD card... ");
#endif
    pinMode(chipSelect, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(error_led, OUTPUT);
    pinMode(button, INPUT_PULLUP);
    pinMode(BUZZER, OUTPUT);

    if (!SD.begin(chipSelect)) {
        digitalWrite(error_led, HIGH);
        delay(100);
        digitalWrite(error_led, LOW);
        delay(500);
    } else {
        // open the file. note that only one file can be opened at a time,
        // so you have to close this one before opening another.

        myFile = SD.open("data/test.txt", FILE_WRITE);
        if (myFile) {
            writeToSD = true;
#if VERBOSE
            Serial.println("initialization of SD card file succeeded!");
#endif
        }
    }

    if (!writeToSD) {
#if VERBOSE
        Serial.println("Error during the initialization of SD Card.");
#endif
        digitalWrite(error_led, HIGH);
    }


#if VERBOSE
    Serial.println("Press the button to start data logging");
#endif
    while(digitalRead(button));
    delay(10);
    while(!digitalRead(button));

    // send visual and audio feedback when the logging starts
    digitalWrite(led, HIGH);
    bip(800);

    // if the file opened okay, write to it:
    myFile.println("START");
    String temp = "t[ms],\t\tax,\tay,\taz,\tgx,\tgy,\tgz,\tmx,\tmy,\tmz,\tpres[hPa]";
    myFile.println(temp);
#if VERBOSE
    Serial.println("START");
    Serial.println(temp);
#endif
    //telemSerial->println("Telemetry starts now");

    //request for the first mag measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

#if VERBOSE
    Serial.println("Ready");
#endif
}


long int cpt = 0;

// Main loop, read and display data
void loop() {
    static unsigned long measurement_time = 0;

    // Stop logging and switch off the LED when button is pushed
    if (!digitalRead(button) & writeToSD) {
        myFile.println("STOP");
        // close the file:
        myFile.close();
#if VERBOSE
        Serial.println("File correctly saved.");
#endif
        writeToSD = false;
        digitalWrite(led, LOW);
        bip(500);
        delay(500);
        bip(500);

        // infinite loop to stop the sampling after closing the file
        while(true);
    }

    // ____________________________________
    // :::  accelerometer and gyroscope :::

    uint8_t bufIMU[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, bufIMU);

    // array containing the data (acc, gyro, mag)
    int16_t tab_data[9] PROGMEM;

    tab_data[0] = bufIMU[0] << 8 | bufIMU[1]; // ax
    tab_data[1] = bufIMU[2] << 8 | bufIMU[3];  // ay
    tab_data[2] = bufIMU[4] << 8 | bufIMU[5];  // az

    tab_data[3] = bufIMU[8] << 8 | bufIMU[9];  // gx
    tab_data[4] = bufIMU[10] << 8 | bufIMU[11];// gy
    tab_data[5] = bufIMU[12] << 8 | bufIMU[13];// gz

    // _____________________
    // :::  Magnetometer :::

    // Read register Status 1 and wait for the DRDY: Data Ready
    uint8_t ST1;
    do {
        I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
    } while (!(ST1 & 0x01));

    // Read magnetometer data
    uint8_t bufMag[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, bufMag);
    // request for the following mag measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

    tab_data[6] = bufMag[3] << 8 | bufMag[2]; // mx
    tab_data[7] = bufMag[1] << 8 | bufMag[0]; // my
    tab_data[8] = bufMag[5] << 8 | bufMag[4]; // mz

    float pressure = bmp.readPressure();
    //float altitude = 44330 * (1.0 - pow((pressure / 100) / seaLevelhPa, 0.1903));

    measurement_time = static_cast<uint32_t>(micros());

    uint8_t i = 0;
    if (writeToSD) {
      myFile.print(measurement_time + DELIMITER);
      for(i=0;i<9;i++){
        myFile.print(tab_data[i] + DELIMITER);
      }
      myFile.println(pressure);
    }

#if VERBOSE
  Serial.print(measurement_time + DELIMITER);
  for(i=0;i<9;i++){
    Serial.print(tab_data[i] + DELIMITER);
  }
  Serial.println(pressure);
#endif

    // start the Finite State Machine
    static char state = READY;
    static long lastState = 0;

    switch(state){
      case READY:

        //digitalWrite(13, HIGH);

        // détection d'accélération
        // TODO ajputer un temps d'accélération, peut etre avec un état intermédiaire
        // TODO conversion de l'acceleration
        // WARN sens de l'accélération positive

        if (abs(tab_data[1]) > ACC_THRESHOLD){
          state = MOTOR;
          myFile.println("M");
        #if VERBOSE
          Serial.println("MOTOR");
        #endif
          lastState = millis();
        }
        break;

      case MOTOR:
        //digitalWrite(13, LOW);

        // timer ou fin de l'accélération
        if (abs(tab_data[1]) < ACC_THRESHOLD){
          if(millis()-lastState < 500){
            myFile.println("FS");
          #if VERBOSE
            Serial.println("False start");
          #endif
            state = READY;
            break;
          }
        }
        if(millis()-lastState > 2500){ // increase this delay
          lastState = millis();
          state = BRAKES;
          myFile.println("B");
        #if VERBOSE
          Serial.println("BRAKES");
        #endif
        }
        break;

      case BRAKES:
        Serial.print('1');
        state = PHASE2;
      #if VERBOSE
        Serial.println("PHASE2");
      #endif
        break;

      case PHASE2:
        break;
    }
}

// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data) {
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();

    // Read Nbytes
    Wire.requestFrom(Address, Nbytes);
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
    // Set register address
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

void telem_write_uint32(uint32_t val) {
    for (int8_t i = 3; i >= 0; --i) {
        telemSerial->write(val >> 8 * i);
    }
}

void telem_write_uint16(uint32_t val) {
    for (int8_t i = 3; i >= 0; --i) {
        telemSerial->write(val >> 8 * i);
    }
}

void bip(int duration) {
  // send a bip when the logging starts
  int period = 100;
  int count = duration*5;
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(period);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(period);
  }
}
