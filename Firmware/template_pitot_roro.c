#include <stdint.h>
#include <Wire.h>

#define DIFF_PRESS_HIGH_RANGE_SENSOR_ADDR   0x58
#define DIFF_PRESS_LOW_RANGE_SENSOR_ADDR    0x28

#define PRESSURE_SENSOR_STATUS_NORMAL       0b00
#define PRESSURE_SENSOR_STATUS_COMMAND      0b01
#define PRESSURE_SENSOR_STATUS_STALE_DATA   0b10
#define PRESSURE_SENSOR_STATUS_DIAGNOSTIC   0b11

/* HSCMRRN001PD2A3, +/- 1psi */
#define PRESSURE_SENSOR1_MAX     6894.76f   /* [Pa] */
#define PRESSURE_SENSOR1_MIN     -6894.76f  /* [Pa] */

/* SSCMRNN015PG5A3 0 - 15 psi*/
#define PRESSURE_SENSOR2_MAX     103421.f    /* [Pa] */
#define PRESSURE_SENSOR2_MIN     0.f         /* [Pa] */

int diff_pressure_hi_res(float *p_press)
{
    uint8_t buf[2];
    Wire.begin();
    Wire.beginTransmission((uint8_t) DIFF_PRESS_HIGH_RANGE_SENSOR_ADDR);
    //Wire.write((uint8_t) reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t) DIFF_PRESS_HIGH_RANGE_SENSOR_ADDR, (uint8_t) 2);
    for (int i=0;i<2;i++){
      buf[i] = Wire.read();
    }
    uint16_t press = (((uint16_t)buf[0]  << 8) | (uint16_t)buf[1]) & 0x3fff;

    //msg_t ret;
    //i2cAcquireBus(&I2CD2);
    //ret = i2cMasterReceiveTimeout(&I2CD2, DIFF_PRESS_HIGH_RANGE_SENSOR_ADDR,
    //                                 buf, sizeof(buf), TIME_INFINITE);
    //i2cReleaseBus(&I2CD2);

    //if (ret != MSG_OK) {
    //    return -1;
    //}
    //uint16_t press = (((uint16_t)buf[0] << 8) | (uint16_t)buf[1]) & 0x3fff;
    //uint16_t temp = (((uint16_t)buf[2] << 3) | ((uint16_t)buf[1] >> 5)) & 0x07ff;
    uint8_t status = buf[0] >> 6;

    if (status != PRESSURE_SENSOR_STATUS_NORMAL && status != PRESSURE_SENSOR_STATUS_STALE_DATA) {
        return -1;
    }

    /* Pressure transfer function */
    *p_press = ((float)press - 1638) * (PRESSURE_SENSOR2_MAX - PRESSURE_SENSOR2_MIN)
                / (14745 - 1638) + PRESSURE_SENSOR2_MIN;

    /* Temperature transfer function */
    //*p_temp = ((float)temp / 2047 * 200) - 50;

    return 0;
}
