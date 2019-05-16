/*
 * BNO055.h
 *
 *  Created on: Mar 11, 2019
 *      Author: Shantanu
 */

#ifndef BNO055_H_
#define BNO055_H_

#include "stdint.h"


#define DEGTORAD     0.0174532925199432957f
#define RADTODEG     57.295779513082320876f

#define BNO055_SAMPLERATE_DELAY_MS (100)
/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

/** Offsets registers **/
#define NUM_BNO055_OFFSET_REGISTERS (22)


//Register definitions

/* Page id register definition */
#define BNO055_PAGE_ID_ADDR          0x07
/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR          0x00
#define BNO055_ACCEL_REV_ID_ADDR     0x01
#define BNO055_MAG_REV_ID_ADDR       0x02
#define BNO055_GYRO_REV_ID_ADDR      0x03
#define BNO055_SW_REV_ID_LSB_ADDR    0x04
#define BNO055_SW_REV_ID_MSB_ADDR    0x05
#define BNO055_BL_REV_ID_ADDR        0x06
/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D
/* Mag data register */
#define BNO055_MAG_DATA_X_LSB_ADDR   0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR   0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR   0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR   0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR   0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR   0x13
/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR  0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR  0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR  0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR  0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR  0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR  0x19
/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR      0x1A
#define BNO055_EULER_H_MSB_ADDR      0x1B
#define BNO055_EULER_R_LSB_ADDR      0x1C
#define BNO055_EULER_R_MSB_ADDR      0x1D
#define BNO055_EULER_P_LSB_ADDR      0x1E
#define BNO055_EULER_P_MSB_ADDR      0x1F
/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR  0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR  0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR  0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR  0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR  0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR  0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR  0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR  0x27
/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D
/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR      0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR      0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR      0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR      0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR      0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR      0x33
/* Temperature data register */
#define BNO055_TEMP_ADDR                    0x34
/* Status registers */
#define BNO055_CALIB_STAT_ADDR              0x35
#define BNO055_SELFTEST_RESULT_ADDR         0x36
#define BNO055_INTR_STAT_ADDR               0x37
#define BNO055_SYS_CLK_STAT_ADDR            0x38
#define BNO055_SYS_STAT_ADDR                0x39
#define BNO055_SYS_ERR_ADDR                 0x3A
/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR                0x3B
#define BNO055_DATA_SELECT_ADDR             0x3C
/* Mode registers */
#define BNO055_OPR_MODE_ADDR                0x3D
#define BNO055_PWR_MODE_ADDR                0x3E
#define BNO055_SYS_TRIGGER_ADDR             0x3F
#define BNO055_TEMP_SOURCE_ADDR             0x40
/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR         0x41
#define BNO055_AXIS_MAP_SIGN_ADDR           0x42
/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR             0x55
#define ACCEL_OFFSET_X_MSB_ADDR             0x56
#define ACCEL_OFFSET_Y_LSB_ADDR             0x57
#define ACCEL_OFFSET_Y_MSB_ADDR             0x58
#define ACCEL_OFFSET_Z_LSB_ADDR             0x59
#define ACCEL_OFFSET_Z_MSB_ADDR             0x5A
/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR               0x5B
#define MAG_OFFSET_X_MSB_ADDR               0x5C
#define MAG_OFFSET_Y_LSB_ADDR               0x5D
#define MAG_OFFSET_Y_MSB_ADDR               0x5E
#define MAG_OFFSET_Z_LSB_ADDR               0x5F
#define MAG_OFFSET_Z_MSB_ADDR               0x60
/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR              0x61
#define GYRO_OFFSET_X_MSB_ADDR              0x62
#define GYRO_OFFSET_Y_LSB_ADDR              0x63
#define GYRO_OFFSET_Y_MSB_ADDR              0x64
#define GYRO_OFFSET_Z_LSB_ADDR              0x65
#define GYRO_OFFSET_Z_MSB_ADDR              0x66
/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR               0x67
#define ACCEL_RADIUS_MSB_ADDR               0x68
#define MAG_RADIUS_LSB_ADDR                 0x69
#define MAG_RADIUS_MSB_ADDR                 0x6A

/* Page 1 registers */
#define BNO055_UNIQUE_ID_ADDR               0x50
#define BNO055_INT_MASK_ADDR                0X0F
#define BNO055_INT_EN_ADDR                  0X10

//Definitions for unit selection
#define MPERSPERS   0x00
#define MILLIG      0x01
#define DEG_PER_SEC 0x00
#define RAD_PER_SEC 0x02
#define DEGREES     0x00
#define RADIANS     0x04
#define CENTIGRADE  0x00
#define FAHRENHEIT  0x10
#define WINDOWS     0x00
#define ANDROID     0x80

//Definitions for power mode
#define POWER_MODE_NORMAL   0x00
#define POWER_MODE_LOWPOWER 0x01
#define POWER_MODE_SUSPEND  0x02

//Definitions for operating mode
#define OPERATION_MODE_CONFIG        0x00
#define OPERATION_MODE_ACCONLY       0x01
#define OPERATION_MODE_MAGONLY       0x02
#define OPERATION_MODE_GYRONLY       0x03
#define OPERATION_MODE_ACCMAG        0x04
#define OPERATION_MODE_ACCGYRO       0x05
#define OPERATION_MODE_MAGGYRO       0x06
#define OPERATION_MODE_AMG           0x07
#define OPERATION_MODE_IMUPLUS       0x08
#define OPERATION_MODE_COMPASS       0x09
#define OPERATION_MODE_M4G           0x0A
#define OPERATION_MODE_NDOF_FMC_OFF  0x0B
#define OPERATION_MODE_NDOF          0x0C


// Struct to store yaw, roll and pitch.
typedef struct {
    double x;
    double y;
    double z;
    double w;
    double yaw;
    double roll;
    double pitch;
} quat_data;

typedef struct {
    double yaw;
    double roll;
    double pitch;
} euler_data;


typedef struct {
    double x;
    double y;
    double z;
} acc_data;

//typedef struct {
//    double x; //east
//    double y; //north
//    double z; //up
//} ref_acc_data;

// A structure to represent offsets
typedef struct {
  int16_t accel_offset_x; /**< x acceleration offset */
  int16_t accel_offset_y; /**< y acceleration offset */
  int16_t accel_offset_z; /**< z acceleration offset */

  int16_t mag_offset_x; /**< x magnetometer offset */
  int16_t mag_offset_y; /**< y magnetometer offset */
  int16_t mag_offset_z; /**< z magnetometer offset */

  int16_t gyro_offset_x; /**< x gyroscrope offset */
  int16_t gyro_offset_y; /**< y gyroscrope offset */
  int16_t gyro_offset_z; /**< z gyroscrope offset */

  int16_t accel_radius; /**< acceleration radius */

  int16_t mag_radius; /**< magnetometer radius */
} offset_data;

void bno055WriteReg(uint8_t reg, uint8_t data);
uint8_t bno055ReadReg(uint8_t reg);
void bno055SetMode(uint8_t mode);
void bno055Init(void);
void bno055SetExtCrystalUse(uint8_t usextal);
uint8_t bno055GetCalStat(void);
void bno055GetCalData(offset_data* data);
void bno055SetCalData(offset_data* data);
void bno055GetEuler(euler_data *data);
void bno055GetQuat(quat_data *data);
void bno055GetLinearAcc(acc_data *data);
void bno055GetRefAcc(quat_data data, acc_data linearaccel, acc_data *ref_data);
void bno055GetRawAcc(acc_data *data);
#endif /* BNO055_H_ */
