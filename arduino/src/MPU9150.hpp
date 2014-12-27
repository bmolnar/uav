#ifndef _MPU9150_H
#define _MPU9150_H

#include "I2C.hpp"

// Bit definition
#define BIT0 0
#define BIT1 1
#define BIT2 2
#define BIT3 3
#define BIT4 4
#define BIT5 5
#define BIT6 6
#define BIT7 7


// I2C Address
#define MPU9150_I2C_ADDRESS        0x68

// Magnetometer Registers
#define MPU9150_WIA                0x00 // R
#define MPU9150_INFO               0x01 // R
#define MPU9150_ST1                0x02 // R
#define MPU9150_HXL                0x03 // R
#define MPU9150_HXH                0x04 // R
#define MPU9150_HYL                0x05 // R
#define MPU9150_HYH                0x06 // R
#define MPU9150_HZL                0x07 // R
#define MPU9150_HZH                0x08 // R
#define MPU9150_ST2                0x09 // R
#define MPU9150_CNTL               0x0A // R/W
#define MPU9150_RSV                0x0B // R/W
#define MPU9150_ASTC               0x0C // R/W
#define MPU9150_TS1                0x0D // R/W
#define MPU9150_TS2                0x0E // R/W
#define MPU9150_I2CDIS             0x0F // R/W
#define MPU9150_ASAX               0x10 // R
#define MPU9150_ASAY               0x11 // R
#define MPU9150_ASAZ               0x12 // R


// Gyroscope and Accelerometer Registers
#define MPU9150_SELF_TEST_X        0x0D // R/W
#define MPU9150_SELF_TEST_Y        0x0E // R/W
#define MPU9150_SELF_TEST_Z        0x0F // R/W
#define MPU9150_SELF_TEST_A        0x10 // R/W
#define MPU9150_SMPLRT_DIV         0x19 // R/W
#define MPU9150_CONFIG             0x1A // R/W
#define MPU9150_GYRO_CONFIG        0x1B // R/W
#define MPU9150_ACCEL_CONFIG       0x1C // R/W
#define MPU9150_FF_THR             0x1D // R/W
#define MPU9150_FF_DUR             0x1E // R/W
#define MPU9150_MOT_THR            0x1F // R/W
#define MPU9150_MOT_DUR            0x20 // R/W
#define MPU9150_ZRMOT_THR          0x21 // R/W
#define MPU9150_ZRMOT_DUR          0x22 // R/W
#define MPU9150_FIFO_EN            0x23 // R/W
#define MPU9150_I2C_MST_CTRL       0x24 // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25 // R/W
#define MPU9150_I2C_SLV0_REG       0x26 // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27 // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28 // R/W
#define MPU9150_I2C_SLV1_REG       0x29 // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B // R/W
#define MPU9150_I2C_SLV2_REG       0x2C // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E // R/W
#define MPU9150_I2C_SLV3_REG       0x2F // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30 // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31 // R/W
#define MPU9150_I2C_SLV4_REG       0x32 // R/W
#define MPU9150_I2C_SLV4_DO        0x33 // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34 // R/W
#define MPU9150_I2C_SLV4_DI        0x35 // R  
#define MPU9150_I2C_MST_STATUS     0x36 // R
#define MPU9150_INT_PIN_CFG        0x37 // R/W
#define MPU9150_INT_ENABLE         0x38 // R/W
#define MPU9150_INT_STATUS         0x3A // R  
#define MPU9150_ACCEL_XOUT_H       0x3B // R  
#define MPU9150_ACCEL_XOUT_L       0x3C // R  
#define MPU9150_ACCEL_YOUT_H       0x3D // R  
#define MPU9150_ACCEL_YOUT_L       0x3E // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F // R  
#define MPU9150_ACCEL_ZOUT_L       0x40 // R  
#define MPU9150_TEMP_OUT_H         0x41 // R  
#define MPU9150_TEMP_OUT_L         0x42 // R  
#define MPU9150_GYRO_XOUT_H        0x43 // R  
#define MPU9150_GYRO_XOUT_L        0x44 // R  
#define MPU9150_GYRO_YOUT_H        0x45 // R  
#define MPU9150_GYRO_YOUT_L        0x46 // R  
#define MPU9150_GYRO_ZOUT_H        0x47 // R  
#define MPU9150_GYRO_ZOUT_L        0x48 // R  
#define MPU9150_EXT_SENS_DATA_00   0x49 // R  
#define MPU9150_EXT_SENS_DATA_01   0x4A // R  
#define MPU9150_EXT_SENS_DATA_02   0x4B // R  
#define MPU9150_EXT_SENS_DATA_03   0x4C // R  
#define MPU9150_EXT_SENS_DATA_04   0x4D // R  
#define MPU9150_EXT_SENS_DATA_05   0x4E // R  
#define MPU9150_EXT_SENS_DATA_06   0x4F // R  
#define MPU9150_EXT_SENS_DATA_07   0x50 // R  
#define MPU9150_EXT_SENS_DATA_08   0x51 // R  
#define MPU9150_EXT_SENS_DATA_09   0x52 // R  
#define MPU9150_EXT_SENS_DATA_10   0x53 // R  
#define MPU9150_EXT_SENS_DATA_11   0x54 // R  
#define MPU9150_EXT_SENS_DATA_12   0x55 // R  
#define MPU9150_EXT_SENS_DATA_13   0x56 // R  
#define MPU9150_EXT_SENS_DATA_14   0x57 // R  
#define MPU9150_EXT_SENS_DATA_15   0x58 // R  
#define MPU9150_EXT_SENS_DATA_16   0x59 // R  
#define MPU9150_EXT_SENS_DATA_17   0x5A // R  
#define MPU9150_EXT_SENS_DATA_18   0x5B // R  
#define MPU9150_EXT_SENS_DATA_19   0x5C // R  
#define MPU9150_EXT_SENS_DATA_20   0x5D // R  
#define MPU9150_EXT_SENS_DATA_21   0x5E // R  
#define MPU9150_EXT_SENS_DATA_22   0x5F // R  
#define MPU9150_EXT_SENS_DATA_23   0x60 // R  
#define MPU9150_MOT_DETECT_STATUS  0x61 // R  
#define MPU9150_I2C_SLV0_DO        0x63 // R/W
#define MPU9150_I2C_SLV1_DO        0x64 // R/W
#define MPU9150_I2C_SLV2_DO        0x65 // R/W
#define MPU9150_I2C_SLV3_DO        0x66 // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67 // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68 // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69 // R/W
#define MPU9150_USER_CTRL          0x6A // R/W
#define MPU9150_PWR_MGMT_1         0x6B // R/W
#define MPU9150_PWR_MGMT_2         0x6C // R/W
#define MPU9150_FIFO_COUNTH        0x72 // R/W
#define MPU9150_FIFO_COUNTL        0x73 // R/W
#define MPU9150_FIFO_R_W           0x74 // R/W
#define MPU9150_WHO_AM_I           0x75 // R


// CNTL
#define MPU9150_MODE0              BIT0
#define MPU9150_MODE1              BIT1
#define MPU9150_MODE2              BIT2
#define MPU9150_MODE3              BIT3

#define MPU9150_MODE_PWRDOWN       (0)
#define MPU9150_MODE_SNGMEAS       (bit(MPU9150_MODE0))
#define MPU9150_MODE_SELFTEST      (bit(MPU9150_MODE3))
#define MPU9150_MODE_FUSEROM       (bit(MPU9150_MODE3)|bit(MPU9150_MODE2)|bit(MPU9150_MODE1)|bit(MPU9150_MODE0))


// GYRO_CONFIG
#define MPU9150_FS_SEL0            BIT3
#define MPU9150_FS_SEL1            BIT4
#define MPU9150_ZG_ST              BIT5
#define MPU9150_YG_ST              BIT6
#define MPU9150_XG_ST              BIT7

#define MPU9150_FS_SEL_0           (0)
#define MPU9150_FS_SEL_1           (bit(MPU9150_FS_SEL0))
#define MPU9150_FS_SEL_2           (bit(MPU9150_FS_SEL1))
#define MPU9150_FS_SEL_3           (bit(MPU9150_FS_SEL1)|bit(MPU9150_FS_SEL0))

#define MPU9150_FS_SEL_250         (0)
#define MPU9150_FS_SEL_500         (bit(MPU9150_FS_SEL0))
#define MPU9150_FS_SEL_1000        (bit(MPU9150_FS_SEL1))
#define MPU9150_FS_SEL_2000        (bit(MPU9150_FS_SEL1)|bit(MPU9150_FS_SEL0))


// ACCEL_CONFIG
#define MPU9150_ACCEL_HPF0         BIT0
#define MPU9150_ACCEL_HPF1         BIT1
#define MPU9150_ACCEL_HPF2         BIT2
#define MPU9150_AFS_SEL0           BIT3
#define MPU9150_AFS_SEL1           BIT4
#define MPU9150_ZA_ST              BIT5
#define MPU9150_YA_ST              BIT6
#define MPU9150_XA_ST              BIT7

#define MPU9150_ACCEL_HPF_0        (0)
#define MPU9150_ACCEL_HPF_1        (bit(MPU9150_ACCEL_HPF0))
#define MPU9150_ACCEL_HPF_2        (bit(MPU9150_ACCEL_HPF1))
#define MPU9150_ACCEL_HPF_3        (bit(MPU9150_ACCEL_HPF1)|bit(MPU9150_ACCEL_HPF0))
#define MPU9150_ACCEL_HPF_4        (bit(MPU9150_ACCEL_HPF2))
#define MPU9150_ACCEL_HPF_7        (bit(MPU9150_ACCEL_HPF2)|bit(MPU9150_ACCEL_HPF1)|bit(MPU9150_ACCEL_HPF0))

#define MPU9150_ACCEL_HPF_RESET    MPU9150_ACCEL_HPF_0
#define MPU9150_ACCEL_HPF_5HZ      MPU9150_ACCEL_HPF_1
#define MPU9150_ACCEL_HPF_2_5HZ    MPU9150_ACCEL_HPF_2
#define MPU9150_ACCEL_HPF_1_25HZ   MPU9150_ACCEL_HPF_3
#define MPU9150_ACCEL_HPF_0_63HZ   MPU9150_ACCEL_HPF_4
#define MPU9150_ACCEL_HPF_HOLD     MPU9150_ACCEL_HPF_7

#define MPU9150_AFS_SEL_0          (0)
#define MPU9150_AFS_SEL_1          (bit(MPU9150_AFS_SEL0))
#define MPU9150_AFS_SEL_2          (bit(MPU9150_AFS_SEL1))
#define MPU9150_AFS_SEL_3          (bit(MPU9150_AFS_SEL1)|bit(MPU9150_AFS_SEL0))

#define MPU9150_AFS_SEL_2G         MPU9150_AFS_SEL_0
#define MPU9150_AFS_SEL_4G         MPU9150_AFS_SEL_1
#define MPU9150_AFS_SEL_8G         MPU9150_AFS_SEL_2
#define MPU9150_AFS_SEL_16G        MPU9150_AFS_SEL_3


enum MPU9150AccelRange
{
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
};


struct MPU9150Config
{
  uint8_t dlpf_cfg:3;
  uint8_t ext_sync_set:3;
  uint8_t unused0:2;
};

struct MPU9150GyroConfig
{
  uint8_t unused0:3;
  uint8_t fs_sel:2;
  uint8_t zg_st:1;
  uint8_t yg_st:1;
  uint8_t xg_st:1;
};

struct MPU9150AccelConfig
{
  uint8_t accel_hpf:3;
  uint8_t afs_sel:2;
  uint8_t za_st:1;
  uint8_t ya_st:1;
  uint8_t xa_st:1;
};




struct MPU9150Data
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t mag_x;
  int16_t mag_y;
  int16_t mag_z;
};

class MPU9150 : public I2CDevice
{
public:
  MPU9150(I2CBus& bus);

  bool setPwrMgmt1(bool device_reset, bool sleep, bool cycle, bool temp_dis, uint8_t clksel);
  bool getWhoAmI(uint8_t *reg);

  bool resetDevice();

  bool setConfig(MPU9150Config config);
  bool getConfig(MPU9150Config* config);

  bool setGyroConfig(MPU9150GyroConfig config);
  bool getGyroConfig(MPU9150GyroConfig* config);

  bool setAccelConfig(MPU9150AccelConfig config);
  bool getAccelConfig(MPU9150AccelConfig* config);


  bool setGyroRange(uint8_t value);
  bool getGyroRange(uint8_t* value);
  bool setGyroSelfTest(bool x, bool y, bool z);
  bool getGyroSelfTest(bool* x, bool* y, bool* z);


  bool setAccelRange(MPU9150AccelRange range);
  bool getAccelRange(MPU9150AccelRange* range);


  bool getData(MPU9150Data* data);
};

#endif // _MPU9150_H
