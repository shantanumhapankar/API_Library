/*
 * BNO055.c
 *
 *  Created on: Mar 11, 2019
 *      Author: Shantanu
 */

#include "BNO055.h"
#include "../HWI/I2C.h"
#include <string.h>
#include "math.h"

void delay_ms(unsigned int delay)
{
    while (delay--)
    {
        __delay_cycles(1000);
    }
}

void bno055WriteReg(uint8_t reg, uint8_t data) {
    i2cSendStartTx();
    i2cWrite(reg);
    i2cWrite(data);
    i2cSendStop();
}

uint8_t bno055ReadReg(uint8_t reg) {
    uint8_t val = 0;
    // Send the address of the reg to read.
    i2cSendStartTx();
    i2cWrite(reg);
    // Read value now.
    i2cSendStartRx();
    // Send stop before reading as we are reading just one byte.
    i2cSendStop();

    val = i2cRead();
    return val;
}


void bno055SetMode(uint8_t mode) {
    bno055WriteReg(BNO055_OPR_MODE_ADDR, mode);
}

void bno055SetPower(uint8_t mode) {
    bno055SetMode(OPERATION_MODE_CONFIG);
    delay_ms(10);
    bno055WriteReg(BNO055_PWR_MODE_ADDR, mode);
    delay_ms(10);
    bno055SetMode(OPERATION_MODE_NDOF);
    delay_ms(10);
}

void bno055Init(void) {
    i2cInit(BNO055_ADDRESS_A);
    // Set mode to change config.
    bno055SetMode(OPERATION_MODE_CONFIG);
    delay_ms(10);
    // Reset the board.
    bno055WriteReg(BNO055_SYS_TRIGGER_ADDR, 0x20);
    delay_ms(600);

    // Make sure it has booted up.
    while(bno055ReadReg(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        delay_ms(1000);
    }

    // Now that the board has booted up, set to normal power mode.
    bno055WriteReg(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    delay_ms(10);

    // Now to change to operation mode NDOF, change page ID
    bno055WriteReg(BNO055_PAGE_ID_ADDR, 0);
    delay_ms(10);
    bno055SetMode(OPERATION_MODE_NDOF);
    delay_ms(10);
}

void bno055SetExtCrystalUse(uint8_t usextal) {

  /* Switch to config mode (just in case since this is the default) */
  bno055SetMode(OPERATION_MODE_CONFIG);
  delay_ms(10);
  bno055WriteReg(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) {
      bno055WriteReg(BNO055_SYS_TRIGGER_ADDR, 0x80);
  } else {
      bno055WriteReg(BNO055_SYS_TRIGGER_ADDR, 0x00);
  }
  delay_ms(10);
//  bno055SetMode(OPERATION_MODE_COMPASS);
  bno055SetMode(OPERATION_MODE_NDOF);

  delay_ms(10);
}

//void bno055GetAccData(imu_data * data) {
//    /* set the accelerometer x lsb */
//    data->accel_x_lsb= bno055ReadReg(BNO055_ACCEL_DATA_X_LSB_ADDR);
//
//    /* set the accelerometer x msb */
//    data->accel_x_msb = bno055ReadReg(BNO055_ACCEL_DATA_X_MSB_ADDR);
//
//    /* set the accelerometer y lsb */
//    data->accel_y_lsb= bno055ReadReg(BNO055_ACCEL_DATA_Y_LSB_ADDR);
//
//    /* set the accelerometer y msb */
//    data->accel_y_msb = bno055ReadReg(BNO055_ACCEL_DATA_Y_MSB_ADDR);
//
//    /* set the accelerometer z lsb */
//    data->accel_z_lsb= bno055ReadReg(BNO055_ACCEL_DATA_Z_LSB_ADDR);
//
//    /* set the accelerometer z msb */
//    data->accel_z_msb = bno055ReadReg(BNO055_ACCEL_DATA_Z_MSB_ADDR);
//}

uint8_t bno055GetCalStat(void) {
    if (bno055ReadReg(BNO055_CALIB_STAT_ADDR) == 0xff) return 1;
    else return 0;
}

int16_t bno055GetData(uint8_t msb_reg, uint8_t lsb_reg) {
    uint8_t msb = bno055ReadReg(msb_reg);
    delay_ms(50);
    uint8_t lsb = bno055ReadReg(lsb_reg);
    delay_ms(50);
    return (((int16_t)msb << 8) | (int16_t)lsb);
}

void bno055GetCalData(offset_data* data) {
    delay_ms(500);
    bno055SetMode(OPERATION_MODE_CONFIG);
    delay_ms(500);
    data->accel_offset_x = bno055GetData(ACCEL_OFFSET_X_MSB_ADDR, ACCEL_OFFSET_X_LSB_ADDR);
    delay_ms(50);
    data->accel_offset_y = bno055GetData(ACCEL_OFFSET_Y_MSB_ADDR, ACCEL_OFFSET_Y_LSB_ADDR);
    delay_ms(50);
    data->accel_offset_z = bno055GetData(ACCEL_OFFSET_Z_MSB_ADDR, ACCEL_OFFSET_Z_LSB_ADDR);
    delay_ms(50);

    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    data->mag_offset_x = bno055GetData(MAG_OFFSET_X_MSB_ADDR, MAG_OFFSET_X_LSB_ADDR);
    delay_ms(50);
    data->mag_offset_y = bno055GetData(MAG_OFFSET_Y_MSB_ADDR, MAG_OFFSET_Y_LSB_ADDR);
    delay_ms(50);
    data->mag_offset_z = bno055GetData(MAG_OFFSET_Z_MSB_ADDR, MAG_OFFSET_Z_LSB_ADDR);
    delay_ms(50);

    /* Gyro offset range depends on the DPS range:
      2000 dps = +/- 32000 LSB
      1000 dps = +/- 16000 LSB
       500 dps = +/- 8000 LSB
       250 dps = +/- 4000 LSB
       125 dps = +/- 2000 LSB
       ... where 1 DPS = 16 LSB */
    data->gyro_offset_x = bno055GetData(GYRO_OFFSET_X_MSB_ADDR, GYRO_OFFSET_X_LSB_ADDR);
    delay_ms(50);
    data->gyro_offset_y = bno055GetData(GYRO_OFFSET_Y_MSB_ADDR, GYRO_OFFSET_Y_LSB_ADDR);
    delay_ms(50);
    data->gyro_offset_z = bno055GetData(GYRO_OFFSET_Z_MSB_ADDR, GYRO_OFFSET_Z_LSB_ADDR);
    delay_ms(50);

    /* Accelerometer radius = +/- 1000 LSB */
    data->accel_radius = bno055GetData(ACCEL_RADIUS_MSB_ADDR, ACCEL_RADIUS_LSB_ADDR);
    delay_ms(50);

    /* Magnetometer radius = +/- 960 LSB */
    data->mag_radius = bno055GetData(MAG_RADIUS_MSB_ADDR, MAG_RADIUS_LSB_ADDR);

    delay_ms(500);
    bno055SetMode(OPERATION_MODE_NDOF);
    delay_ms(500);
}


void bno055SetCalData(offset_data* data) {
    delay_ms(500);
    bno055SetMode(OPERATION_MODE_CONFIG);
    delay_ms(500);

    /* Note: Configuration will take place only when user writes to the last
     byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
     Therefore the last byte must be written whenever the user wants to
     changes the configuration. */

    bno055WriteReg(ACCEL_OFFSET_X_LSB_ADDR, (data->accel_offset_x) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(ACCEL_OFFSET_X_MSB_ADDR, (data->accel_offset_x >> 8) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(ACCEL_OFFSET_Y_LSB_ADDR, (data->accel_offset_y) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(ACCEL_OFFSET_Y_MSB_ADDR, (data->accel_offset_y >> 8) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(ACCEL_OFFSET_Z_LSB_ADDR, (data->accel_offset_z) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(ACCEL_OFFSET_Z_MSB_ADDR, (data->accel_offset_z >> 8) & 0x0FF);
    delay_ms(50);

    bno055WriteReg(MAG_OFFSET_X_LSB_ADDR, (data->mag_offset_x) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(MAG_OFFSET_X_MSB_ADDR, (data->mag_offset_x >> 8) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(MAG_OFFSET_Y_LSB_ADDR, (data->mag_offset_y) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(MAG_OFFSET_Y_MSB_ADDR, (data->mag_offset_y >> 8) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(MAG_OFFSET_Z_LSB_ADDR, (data->mag_offset_z) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(MAG_OFFSET_Z_MSB_ADDR, (data->mag_offset_z >> 8) & 0x0FF);
    delay_ms(50);

    bno055WriteReg(GYRO_OFFSET_X_LSB_ADDR, (data->gyro_offset_x) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(GYRO_OFFSET_X_MSB_ADDR, (data->gyro_offset_x >> 8) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(GYRO_OFFSET_Y_LSB_ADDR, (data->gyro_offset_y) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(GYRO_OFFSET_Y_MSB_ADDR, (data->gyro_offset_y >> 8) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(GYRO_OFFSET_Z_LSB_ADDR, (data->gyro_offset_z) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(GYRO_OFFSET_Z_MSB_ADDR, (data->gyro_offset_z >> 8) & 0x0FF);
    delay_ms(50);

    bno055WriteReg(ACCEL_RADIUS_LSB_ADDR, (data->accel_radius) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(ACCEL_RADIUS_MSB_ADDR, (data->accel_radius >> 8) & 0x0FF);
    delay_ms(50);

    bno055WriteReg(MAG_RADIUS_LSB_ADDR, (data->mag_radius) & 0x0FF);
    delay_ms(50);
    bno055WriteReg(MAG_RADIUS_MSB_ADDR, (data->mag_radius >> 8) & 0x0FF);

    delay_ms(500);
    bno055SetMode(OPERATION_MODE_NDOF);
    delay_ms(500);
}

void bno055GetEuler(euler_data *data) {
    double x = 0, y = 0, z = 0;

    x = (double)bno055GetData(BNO055_EULER_H_MSB_ADDR, BNO055_EULER_H_LSB_ADDR);
    y = (double)bno055GetData(BNO055_EULER_R_MSB_ADDR, BNO055_EULER_R_LSB_ADDR);
    z = (double)bno055GetData(BNO055_EULER_P_MSB_ADDR, BNO055_EULER_P_LSB_ADDR);
    data->yaw   = x/16.0;
    data->roll  = y/16.0;
    data->pitch = z/16.0;
}

void bno055GetQuat(quat_data *data) {
    double w = 0.0, x = 0.0, y = 0.0, z = 0.0;
    const double scale = (1.0 / (1 << 14));

    w = bno055GetData(BNO055_QUATERNION_DATA_W_MSB_ADDR, BNO055_QUATERNION_DATA_W_LSB_ADDR);
    x = bno055GetData(BNO055_QUATERNION_DATA_X_MSB_ADDR, BNO055_QUATERNION_DATA_X_LSB_ADDR);
    y = bno055GetData(BNO055_QUATERNION_DATA_Y_MSB_ADDR, BNO055_QUATERNION_DATA_Y_LSB_ADDR);
    z = bno055GetData(BNO055_QUATERNION_DATA_Z_MSB_ADDR, BNO055_QUATERNION_DATA_Z_LSB_ADDR);
    w = w*scale;
    x = x*scale;
    y = y*scale;
    z = z*scale;

    data->w = w;
    data->x = x;
    data->y = y;
    data->z = z;

    data->pitch = - RADTODEG * atan2(2 * (x * z + y * w), 1 - 2 * (x * x + y * y));
    data->roll  = RADTODEG * asin(2 * y * z - 2 * x * w);
    data->yaw   = RADTODEG * atan2(2 * (x * y + z * w), 1 - 2 * (y * y + z * z));
    if (data->yaw < 0) data->yaw = -(data->yaw);
    else data->yaw = (180 - data->yaw) + 180;
}

void bno055GetLinearAcc(acc_data *data) {
    int16_t x, y, z;

    x = bno055GetData(BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR , BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR);
    y = bno055GetData(BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR , BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR);
    z = bno055GetData(BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR , BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR);

    data->x = ((double)x)/100.0;
    data->y = ((double)y)/100.0;
    data->z = ((double)z)/100.0;
}

void bno055GetRawAcc(acc_data *data) {
    int16_t x, y, z;

    x = bno055GetData(BNO055_ACCEL_DATA_X_MSB_ADDR , BNO055_ACCEL_DATA_X_LSB_ADDR);
    y = bno055GetData(BNO055_ACCEL_DATA_Y_MSB_ADDR, BNO055_ACCEL_DATA_Y_LSB_ADDR);
    z = bno055GetData(BNO055_ACCEL_DATA_Z_MSB_ADDR, BNO055_ACCEL_DATA_Z_LSB_ADDR);

    data->x = ((double)x)/100.0;
    data->y = ((double)y)/100.0;
    data->z = ((double)z)/100.0;
}

void bno055GetRefAcc(quat_data quat, acc_data linearaccel, acc_data *data) {
    data->x = (1-2*(quat.y*quat.y + quat.z*quat.z))*linearaccel.x +   (2*(quat.x*quat.y + quat.w*quat.z))*linearaccel.y +   (2*(quat.x*quat.z - quat.w*quat.y))*linearaccel.z;  // rotate linearaccel by quaternion
    data->y =   (2*(quat.x*quat.y - quat.w*quat.z))*linearaccel.x + (1-2*(quat.x*quat.x + quat.z*quat.z))*linearaccel.y +   (2*(quat.y*quat.z + quat.w*quat.x))*linearaccel.z;
    data->z =   (2*(quat.x*quat.z + quat.w*quat.y))*linearaccel.x +   (2*(quat.y*quat.z - quat.w*quat.x))*linearaccel.y + (1-2*(quat.x*quat.x + quat.y*quat.y))*linearaccel.z;
}
