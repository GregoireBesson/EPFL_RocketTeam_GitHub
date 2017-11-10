//
// Created by clement on 18/10/2017.
//

#ifndef DATA_LOGGER_PIO_DATA_LOGGER_H
#define DATA_LOGGER_PIO_DATA_LOGGER_H


#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>

struct measurements {
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
};

enum State {
    READY,
    MOTOR,
    OPEN,
    CLOSE
};


void telem_write_uint32(uint32_t val, uint16_t *remainder);

void telem_write_uint16(uint16_t val, uint16_t *remainder);

void telem_write_uint8(uint8_t val, uint16_t *remainder);

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t *Data);

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

void bip(int duration);

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

// TODO demander l'accélération maximale de la fusée
// faut sûrement mettre 16g
#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    VERBOSE                   true

#define    TS                        0x40
#define    AX                        0x20
#define    AY                        0x21
#define    AZ                        0x22

#define    ACC_THRESHOLD             1000 //4175 //=20*208,77

#define    BUZZER                    A0



#endif //DATA_LOGGER_PIO_DATA_LOGGER_H
