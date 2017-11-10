#include <avr/pgmspace.h>
#include <Arduino.h>
#include <CRC/SimpleCRC.h>
#include "data_logger.h"
#include "DatagramSpec.h"
//#include "pitot/pitot.h"

//select the chip (depend on the SD shield, put 10 for Adafruit)
const int chipSelect = 10;
const int button = 3;
const int led = 8;
const int error_led = 7;
const int brakes_pin = 4;
const unsigned long period = 10 * 1000; // period in microseconds
bool writeToSD = false;
uint32_t sequenceNumber;


Adafruit_BMP280 bmp; // I2C
File myFile;
SoftwareSerial *telemSerial;

String DELIMITER;

// Initializations
void setup() {

    DELIMITER = F(",\t");

    //Xbee serial: RX = digital pin 5, TX = digital pin 6
    telemSerial = new SoftwareSerial(5, 6);
    telemSerial->begin(115200);
    sequenceNumber = 0;

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
        while (true) {
            bip(50);
        }
    }

#if VERBOSE
    Serial.print(F("\nInitializing SD card... "));
#endif
    pinMode(chipSelect, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(brakes_pin, OUTPUT);
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

        myFile = SD.open(F("data/test.txt"), FILE_WRITE);
        if (myFile) {
            writeToSD = true;
#if VERBOSE
            Serial.println(F("initialization of SD card file succeeded!"));
#endif
        }
    }

    if (!writeToSD) {
#if VERBOSE
        Serial.println(F("Error during the initialization of SD Card."));
#endif
        digitalWrite(error_led, HIGH);
    }


#if VERBOSE
    Serial.println(F("Press the button to start data logging"));
#endif
    /*
    while (digitalRead(button));
    delay(10);
    while (!digitalRead(button));
*/
    // send visual and audio feedback when the logging starts
    digitalWrite(led, HIGH);
    bip(800);

    // if the file opened okay, write to it:
    myFile.println(F("START"));
    String temp = F("t[ms],\t\tax,\tay,\taz,\tgx,\tgy,\tgz,\tmx,\tmy,\tmz,\tpres[hPa]");
    myFile.println(temp);
#if VERBOSE
    Serial.println(F("START"));
    Serial.println(temp);
#endif
    //telemSerial->println("Telemetry starts now");

    //request for the first mag measurement
    I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

#if VERBOSE
    Serial.println(F("Ready"));
#endif
}


long int cpt = 0;

// Main loop, read and display data
void loop() {
    static unsigned long measurement_time = 0;
    while (micros() - measurement_time < period) {
        delayMicroseconds(100);
    }
    measurement_time = static_cast<uint32_t>(micros());

    static uint8_t counter = 0;

    // Stop logging and switch off the LED when button is pushed
    if (!digitalRead(button) & writeToSD) {
        myFile.println(F("STOP"));
        // close the file:
        myFile.close();
#if VERBOSE
        Serial.println(F("File correctly saved."));
#endif
        writeToSD = false;
        digitalWrite(led, LOW);
        bip(500);
        delay(500);
        bip(500);
    }

    uint16_t remainder = SimpleCRC::CRC_16_GENERATOR_POLY.initialValue;
    // Beginning of telemetry transmission:
    for (size_t i = 0; i < PREAMBLE_SIZE; ++i) {
        telemSerial->write(HEADER_PREAMBLE_FLAG);
    }
    telem_write_uint32(sequenceNumber++, &remainder);
    telem_write_uint8(static_cast<uint8_t>(DatagramPayloadType::TELEMETRY), &remainder);
    telemSerial->write(CONTROL_FLAG);

    // Datagram PAYLOAD
    telem_write_uint32(static_cast<uint32_t>(measurement_time), &remainder);

    // ____________________________________
    // :::  accelerometer and gyroscope :::

    uint8_t bufIMU[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, bufIMU);

    // array containing the data (acc, gyro, mag)

    measurements data{};

    data.ax = bufIMU[0] << 8 | bufIMU[1];   // ax
    data.ay = bufIMU[2] << 8 | bufIMU[3];  // ay
    data.az = bufIMU[4] << 8 | bufIMU[5];  // az

    telem_write_uint16(data.ax, &remainder);
    telem_write_uint16(data.ay, &remainder);
    telem_write_uint16(data.az, &remainder);

    data.gx = bufIMU[8] << 8 | bufIMU[9];  // gx
    data.gy = bufIMU[10] << 8 | bufIMU[11];// gy
    data.gz = bufIMU[12] << 8 | bufIMU[13];// gz

    telem_write_uint16(data.gx, &remainder);
    telem_write_uint16(data.gy, &remainder);
    telem_write_uint16(data.gz, &remainder);

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

    data.mx = bufMag[3] << 8 | bufMag[2]; // mx
    data.my = bufMag[1] << 8 | bufMag[0]; // my
    data.mz = bufMag[5] << 8 | bufMag[4]; // mz


    telem_write_uint16(data.mx, &remainder);
    telem_write_uint16(data.my, &remainder);
    telem_write_uint16(data.mz, &remainder);

    auto pressureTemperature = bmp.readPressureTemperature();
    float_cast pressure = {.fl = pressureTemperature.pressure};
    float_cast temperature = {.fl = pressureTemperature.temperature};

    telem_write_uint32(temperature.uint32, &remainder);
    telem_write_uint32(pressure.uint32, &remainder);



    auto crc = SimpleCRC::Finalize(remainder);
    telemSerial->write(crc >> 8);
    telemSerial->write(crc);


    String dataStringSD = measurement_time + DELIMITER
                          + data.ax + DELIMITER
                          + data.ay + DELIMITER
                          + data.az + DELIMITER
                          + data.gx + DELIMITER
                          + data.gy + DELIMITER
                          + data.gz + DELIMITER
                          + data.mx + DELIMITER
                          + data.my + DELIMITER
                          + data.mz + DELIMITER
                          + pressure.fl;

    if (writeToSD) {
        myFile.println(dataStringSD);
    }

#if VERBOSE
    Serial.println(dataStringSD);
#endif

    // start the Finite State Machine
    static char state = READY;
    static long lastState = 0;

    switch (state) {
        case READY:

            //digitalWrite(13, HIGH);

            // détection d'accélération
            // TODO ajputer un temps d'accélération, peut etre avec un état intermédiaire
            // TODO conversion de l'acceleration
            // WARN sens de l'accélération positive

            if (abs(data.ay) > ACC_THRESHOLD) {
                state = MOTOR;
                myFile.println(F("M"));
#if VERBOSE
                Serial.println(F("MOTOR"));
#endif
                lastState = millis();
            }
            break;

        case MOTOR:
            //digitalWrite(13, LOW);

            // timer ou fin de l'accélération
            if (abs(data.ay) < ACC_THRESHOLD) {
                if (millis() - lastState < 500) {
                    myFile.println(F("FS"));
#if VERBOSE
                    Serial.println(F("False start"));
#endif
                    state = READY;
                    break;
                }
            }
            if (millis() - lastState > 2500) { // increase this delay
                lastState = millis();
                state = OPEN;
                myFile.println(F("O"));
#if VERBOSE
                Serial.println(F("OPEN"));
#endif
            }
            break;

        case OPEN:
            digitalWrite(brakes_pin, HIGH);

            if (millis() - lastState > 800 && counter < 3) {
                counter++;
                state = CLOSE;
                myFile.println(F("C"));
#if VERBOSE
                Serial.println(F("CLOSE"));
#endif
                lastState = millis();
            }
            break;
        case CLOSE:
            digitalWrite(brakes_pin, LOW);

            if (millis() - lastState > 800) {
                state = OPEN;
                myFile.println(F("O"));
#if VERBOSE
                Serial.println(F("OPEN"));
#endif
                lastState = millis();
            }
            break;
#if VERBOSE
            Serial.println("PHASE2");
#endif
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

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"

void telem_write_uint32(uint32_t val, uint16_t *remainder) {
    for (int8_t i = sizeof(uint32_t) - 1; i >= 0; --i) {
        uint8_t byte = val >> 8 * i;
        *remainder = SimpleCRC::CalculateRemainderFromTable(byte, *remainder);
        telemSerial->write(byte);
    }
}

void telem_write_uint16(uint16_t val, uint16_t *remainder) {
    for (int8_t i = sizeof(uint16_t) - 1; i >= 0; --i) {
        uint8_t byte = val >> 8 * i;
        *remainder = SimpleCRC::CalculateRemainderFromTable(byte, *remainder);
        telemSerial->write(byte);
    }
}

void telem_write_uint8(uint8_t val, uint16_t *remainder) {
    *remainder = SimpleCRC::CalculateRemainderFromTable(val, *remainder);
    telemSerial->write(val);
}

#pragma clang diagnostic pop


void bip(int duration) {
    // send a bip when the logging starts
    int period = 100;
    int count = duration * 5;
    for (int i = 0; i < count; i++) {
        digitalWrite(BUZZER, HIGH);
        delayMicroseconds(period);
        digitalWrite(BUZZER, LOW);
        delayMicroseconds(period);
    }
}
