#include <Arduino.h>
#include "data_logger.h"

//select the chip (depend on the SD shield, put 10 for Adafruit)
const int chipSelect = 10;
const int button = 3;
const int led = 8;
const int error_led = 7;
const unsigned long period = 20 * 1000; // period in microseconds
const float altInit = 750;
float seaLevelhPa = 1013.25;
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
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
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

    seaLevelhPa = bmp.readPressure() / (100 * pow(1 - (altInit / 44330), 1 / 0.1903));
#if VERBOSE
    Serial.print("computed sea level pressure:\t");
    Serial.println(seaLevelhPa);
#endif


#if VERBOSE
    Serial.print("\nInitializing SD card... ");
#endif
    pinMode(chipSelect, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(error_led, OUTPUT);
    pinMode(button, INPUT_PULLUP);

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
            Serial.println("initialization of sd card file succeeded!");
#endif
        }
    }

    if (!writeToSD) {
#if VERBOSE
        Serial.println("Error during the initialization of SD Card.");
#endif
        digitalWrite(error_led, HIGH);
    }

    /*
#if VERBOSE
    Serial.println("Press the button to start data logging");
#endif
    while(digitalRead(button));
    delay(10);
    while(!digitalRead(button));
    digitalWrite(led,HIGH);
     */

    // if the file opened okay, write to it:
    myFile.println("START");
    String temp = "t[ms],\tax,\tay,\taz,\tgx,\tgy,\tgz,\tmx,\tmy,\tmz,\ttemp[C],\tpres[hPa]";
    myFile.println(temp);
#if VERBOSE
    Serial.println(temp);
#endif
    telemSerial->println("Telemetry starts now");

    digitalWrite(led, HIGH);

    //request for the first mag measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

}


long int cpt = 0;

// Main loop, read and display data
void loop() {
    static unsigned long measurement_time = 0;
    while (micros() - measurement_time < period);
    // Stop logging and switch off the LED when button is pushed

    if (!digitalRead(button) & writeToSD) {
        digitalWrite(led, LOW);
        myFile.println("STOP");
        // close the file:
        myFile.close();
#if VERBOSE
        Serial.println("File correctly saved.");
#endif
        writeToSD = false;
    }

    // ____________________________________
    // :::  accelerometer and gyroscope :::

    uint8_t bufIMU[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, bufIMU);

    int16_t ax = bufIMU[0] << 8 | bufIMU[1];
    int16_t ay = bufIMU[2] << 8 | bufIMU[3];
    int16_t az = bufIMU[4] << 8 | bufIMU[5];

    int16_t gx = bufIMU[8] << 8 | bufIMU[9];
    int16_t gy = bufIMU[10] << 8 | bufIMU[11];
    int16_t gz = bufIMU[12] << 8 | bufIMU[13];

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

    int16_t mx = bufMag[3] << 8 | bufMag[2];
    int16_t my = bufMag[1] << 8 | bufMag[0];
    int16_t mz = bufMag[5] << 8 | bufMag[4];

    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    float altitude = 44330 * (1.0 - pow((pressure / 100) / seaLevelhPa, 0.1903));

    measurement_time = static_cast<uint32_t>(micros());

    /*
    telemSerial->write(TS);
    telem_write_uint32(measurement_time);
    telemSerial->write(AX);
    telem_write_uint16(ax);
    telemSerial->write(AY);
    telem_write_uint16(ay);
    telemSerial->write(AZ);
    telem_write_uint16(az);
    // */
    String dataStringSD = measurement_time + DELIMITER
                          + ax + DELIMITER
                          + ay + DELIMITER
                          + az + DELIMITER
                          + gx + DELIMITER
                          + gy + DELIMITER
                          + gz + DELIMITER;

    if (writeToSD) {
        myFile.print(dataStringSD);
    }
    telemSerial->print(dataStringSD);
    dataStringSD.remove(0, dataStringSD.length());

    dataStringSD = mx + DELIMITER
                          + my + DELIMITER
                          + mz + DELIMITER
                          + temperature
                          + DELIMITER + pressure
                          + DELIMITER + altitude;

    if (writeToSD) {
        myFile.println(dataStringSD);
    }
    telemSerial->println(dataStringSD);

    /*
#if VERBOSE
    if (abs((float) ay / 208.77) > 20)Serial.print('1');
    else Serial.print('0');
#endif
     */
    //String dataStringArduino= 'a'+String(ay)+'h'+String(altitude);
    //Serial.println(dataStringArduino);
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
