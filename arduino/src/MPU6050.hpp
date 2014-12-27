#ifndef _MPU6050_H
#define _MPU6050_H

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
#define MPU6050_I2C_ADDRESS        0x68

// Registers
#define MPU6050_SELF_TEST_X        0x0D // R/W
#define MPU6050_SELF_TEST_Y        0x0E // R/W
#define MPU6050_SELF_TEST_Z        0x0F // R/W
#define MPU6050_SELF_TEST_A        0x10 // R/W
#define MPU6050_SMPLRT_DIV         0x19 // R/W
#define MPU6050_CONFIG             0x1A // R/W
#define MPU6050_GYRO_CONFIG        0x1B // R/W
#define MPU6050_ACCEL_CONFIG       0x1C // R/W
#define MPU6050_FF_THR             0x1D // R/W
#define MPU6050_FF_DUR             0x1E // R/W
#define MPU6050_MOT_THR            0x1F // R/W
#define MPU6050_MOT_DUR            0x20 // R/W
#define MPU6050_ZRMOT_THR          0x21 // R/W
#define MPU6050_ZRMOT_DUR          0x22 // R/W
#define MPU6050_FIFO_EN            0x23 // R/W
#define MPU6050_I2C_MST_CTRL       0x24 // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25 // R/W
#define MPU6050_I2C_SLV0_REG       0x26 // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27 // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28 // R/W
#define MPU6050_I2C_SLV1_REG       0x29 // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B // R/W
#define MPU6050_I2C_SLV2_REG       0x2C // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E // R/W
#define MPU6050_I2C_SLV3_REG       0x2F // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30 // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31 // R/W
#define MPU6050_I2C_SLV4_REG       0x32 // R/W
#define MPU6050_I2C_SLV4_DO        0x33 // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34 // R/W
#define MPU6050_I2C_SLV4_DI        0x35 // R  
#define MPU6050_I2C_MST_STATUS     0x36 // R
#define MPU6050_INT_PIN_CFG        0x37 // R/W
#define MPU6050_INT_ENABLE         0x38 // R/W
#define MPU6050_INT_STATUS         0x3A // R  
#define MPU6050_ACCEL_XOUT_H       0x3B // R  
#define MPU6050_ACCEL_XOUT_L       0x3C // R  
#define MPU6050_ACCEL_YOUT_H       0x3D // R  
#define MPU6050_ACCEL_YOUT_L       0x3E // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F // R  
#define MPU6050_ACCEL_ZOUT_L       0x40 // R  
#define MPU6050_TEMP_OUT_H         0x41 // R  
#define MPU6050_TEMP_OUT_L         0x42 // R  
#define MPU6050_GYRO_XOUT_H        0x43 // R  
#define MPU6050_GYRO_XOUT_L        0x44 // R  
#define MPU6050_GYRO_YOUT_H        0x45 // R  
#define MPU6050_GYRO_YOUT_L        0x46 // R  
#define MPU6050_GYRO_ZOUT_H        0x47 // R  
#define MPU6050_GYRO_ZOUT_L        0x48 // R  
#define MPU6050_EXT_SENS_DATA_00   0x49 // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F // R  
#define MPU6050_EXT_SENS_DATA_07   0x50 // R  
#define MPU6050_EXT_SENS_DATA_08   0x51 // R  
#define MPU6050_EXT_SENS_DATA_09   0x52 // R  
#define MPU6050_EXT_SENS_DATA_10   0x53 // R  
#define MPU6050_EXT_SENS_DATA_11   0x54 // R  
#define MPU6050_EXT_SENS_DATA_12   0x55 // R  
#define MPU6050_EXT_SENS_DATA_13   0x56 // R  
#define MPU6050_EXT_SENS_DATA_14   0x57 // R  
#define MPU6050_EXT_SENS_DATA_15   0x58 // R  
#define MPU6050_EXT_SENS_DATA_16   0x59 // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F // R  
#define MPU6050_EXT_SENS_DATA_23   0x60 // R  
#define MPU6050_MOT_DETECT_STATUS  0x61 // R  
#define MPU6050_I2C_SLV0_DO        0x63 // R/W
#define MPU6050_I2C_SLV1_DO        0x64 // R/W
#define MPU6050_I2C_SLV2_DO        0x65 // R/W
#define MPU6050_I2C_SLV3_DO        0x66 // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67 // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68 // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69 // R/W
#define MPU6050_USER_CTRL          0x6A // R/W
#define MPU6050_PWR_MGMT_1         0x6B // R/W
#define MPU6050_PWR_MGMT_2         0x6C // R/W
#define MPU6050_FIFO_COUNTH        0x72 // R/W
#define MPU6050_FIFO_COUNTL        0x73 // R/W
#define MPU6050_FIFO_R_W           0x74 // R/W
#define MPU6050_WHO_AM_I           0x75 // R


// CNTL
#define MPU6050_MODE0              BIT0
#define MPU6050_MODE1              BIT1
#define MPU6050_MODE2              BIT2
#define MPU6050_MODE3              BIT3

#define MPU6050_MODE_PWRDOWN       (0)
#define MPU6050_MODE_SNGMEAS       (bit(MPU6050_MODE0))
#define MPU6050_MODE_SELFTEST      (bit(MPU6050_MODE3))
#define MPU6050_MODE_FUSEROM       (bit(MPU6050_MODE3)|bit(MPU6050_MODE2)|bit(MPU6050_MODE1)|bit(MPU6050_MODE0))


// GYRO_CONFIG
#define MPU6050_FS_SEL0            BIT3
#define MPU6050_FS_SEL1            BIT4
#define MPU6050_ZG_ST              BIT5
#define MPU6050_YG_ST              BIT6
#define MPU6050_XG_ST              BIT7

#define MPU6050_FS_SEL_0           (0)
#define MPU6050_FS_SEL_1           (bit(MPU6050_FS_SEL0))
#define MPU6050_FS_SEL_2           (bit(MPU6050_FS_SEL1))
#define MPU6050_FS_SEL_3           (bit(MPU6050_FS_SEL1)|bit(MPU6050_FS_SEL0))

#define MPU6050_FS_SEL_250         (0)
#define MPU6050_FS_SEL_500         (bit(MPU6050_FS_SEL0))
#define MPU6050_FS_SEL_1000        (bit(MPU6050_FS_SEL1))
#define MPU6050_FS_SEL_2000        (bit(MPU6050_FS_SEL1)|bit(MPU6050_FS_SEL0))


// ACCEL_CONFIG
#define MPU6050_ACCEL_HPF0         BIT0
#define MPU6050_ACCEL_HPF1         BIT1
#define MPU6050_ACCEL_HPF2         BIT2
#define MPU6050_AFS_SEL0           BIT3
#define MPU6050_AFS_SEL1           BIT4
#define MPU6050_ZA_ST              BIT5
#define MPU6050_YA_ST              BIT6
#define MPU6050_XA_ST              BIT7

#define MPU6050_ACCEL_HPF_0        (0)
#define MPU6050_ACCEL_HPF_1        (bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_2        (bit(MPU6050_ACCEL_HPF1))
#define MPU6050_ACCEL_HPF_3        (bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))
#define MPU6050_ACCEL_HPF_4        (bit(MPU6050_ACCEL_HPF2))
#define MPU6050_ACCEL_HPF_7        (bit(MPU6050_ACCEL_HPF2)|bit(MPU6050_ACCEL_HPF1)|bit(MPU6050_ACCEL_HPF0))

#define MPU6050_ACCEL_HPF_RESET    MPU6050_ACCEL_HPF_0
#define MPU6050_ACCEL_HPF_5HZ      MPU6050_ACCEL_HPF_1
#define MPU6050_ACCEL_HPF_2_5HZ    MPU6050_ACCEL_HPF_2
#define MPU6050_ACCEL_HPF_1_25HZ   MPU6050_ACCEL_HPF_3
#define MPU6050_ACCEL_HPF_0_63HZ   MPU6050_ACCEL_HPF_4
#define MPU6050_ACCEL_HPF_HOLD     MPU6050_ACCEL_HPF_7

#define MPU6050_AFS_SEL_0          (0)
#define MPU6050_AFS_SEL_1          (bit(MPU6050_AFS_SEL0))
#define MPU6050_AFS_SEL_2          (bit(MPU6050_AFS_SEL1))
#define MPU6050_AFS_SEL_3          (bit(MPU6050_AFS_SEL1)|bit(MPU6050_AFS_SEL0))

#define MPU6050_AFS_SEL_2G         MPU6050_AFS_SEL_0
#define MPU6050_AFS_SEL_4G         MPU6050_AFS_SEL_1
#define MPU6050_AFS_SEL_8G         MPU6050_AFS_SEL_2
#define MPU6050_AFS_SEL_16G        MPU6050_AFS_SEL_3


struct MPU6050Reg_SelfTestX
{
  uint8_t xg_test:5;
  uint8_t xa_test:3;
};

struct MPU6050Reg_SelfTestY
{
  uint8_t yg_test:5;
  uint8_t ya_test:3;
};

struct MPU6050Reg_SelfTestZ
{
  uint8_t zg_test:5;
  uint8_t za_test:3;
};

struct MPU6050Reg_SelfTestA
{
  uint8_t za_test:2;
  uint8_t ya_test:2;
  uint8_t xa_test:2;
  uint8_t unused:2;
};

struct MPU6050Reg_SmplrtDiv
{
  uint8_t smplrt_div:8;
};

struct MPU6050Reg_Config
{
  uint8_t dlpf_cfg:3;
  uint8_t ext_sync_set:3;
  uint8_t unused0:2;
};

struct MPU6050Reg_GyroConfig
{
  uint8_t unused0:3;
  uint8_t fs_sel:2;
  uint8_t zg_st:1;
  uint8_t yg_st:1;
  uint8_t xg_st:1;
};

struct MPU6050Reg_AccelConfig
{
  uint8_t unused0:3;
  uint8_t afs_sel:2;
  uint8_t za_st:1;
  uint8_t ya_st:1;
  uint8_t xa_st:1;
};

struct MPU6050Reg_PwrMgmt1
{
  uint8_t clksel:3;
  uint8_t temp_dis:1;
  uint8_t unused0:1;
  uint8_t cycle:1;
  uint8_t sleep:1;
  uint8_t device_reset:1;
};

struct MPU6050Reg_PwrMgmt2
{
  uint8_t stby_zg:1;
  uint8_t stby_yg:1;
  uint8_t stby_xg:1;
  uint8_t stby_za:1;
  uint8_t stby_ya:1;
  uint8_t stby_xa:1;
  uint8_t lp_wake_ctrl:2;
};

struct MPU6050Reg_WhoAmI
{
  uint8_t who_am_i:8;
};

typedef union
{
  MPU6050Reg_SelfTestX   self_test_x;
  MPU6050Reg_SelfTestY   self_test_y;
  MPU6050Reg_SelfTestZ   self_test_z;
  MPU6050Reg_SelfTestA   self_test_a;
  MPU6050Reg_SmplrtDiv   smplrt_div;
  MPU6050Reg_Config      config;
  MPU6050Reg_GyroConfig  gyro_config;
  MPU6050Reg_AccelConfig accel_config;
  MPU6050Reg_PwrMgmt1    pwr_mgmt_1;
  MPU6050Reg_PwrMgmt2    pwr_mgmt_2;
  MPU6050Reg_WhoAmI      who_am_i;
  uint8_t                bits;
} MPU6050Reg;

#define MPU6050REG_INIT { .bits = 0 }

enum MPU6050DlpfCfg
{
  MPU6050_DLPF_CFG_0 = 0,
  MPU6050_DLPF_CFG_1 = 1,
  MPU6050_DLPF_CFG_2 = 2,
  MPU6050_DLPF_CFG_3 = 3,
  MPU6050_DLPF_CFG_4 = 4,
  MPU6050_DLPF_CFG_5 = 5,
  MPU6050_DLPF_CFG_6 = 6,
  MPU6050_DLPF_CFG_7 = 7
};

enum MPU6050ExtSyncSet
{
  MPU6050_EXT_SYNC_SET_0 = 0,
  MPU6050_EXT_SYNC_SET_1 = 1,
  MPU6050_EXT_SYNC_SET_2 = 2,
  MPU6050_EXT_SYNC_SET_3 = 3,
  MPU6050_EXT_SYNC_SET_4 = 4,
  MPU6050_EXT_SYNC_SET_5 = 5,
  MPU6050_EXT_SYNC_SET_6 = 6,
  MPU6050_EXT_SYNC_SET_7 = 7,
};

enum MPU6050GyroRange
{
  // Value, Range (+/- deg/s), Sensitivity (LSB/(deg/s))
  // 0x00,  250,               131
  // 0x01,  500,               65.5
  // 0x02,  1000,              32.8
  // 0x03,  2000,              16.4

  MPU6050_GYRO_RANGE_250DPS  = 0,
  MPU6050_GYRO_RANGE_500DPS  = 1,
  MPU6050_GYRO_RANGE_1000DPS = 2,
  MPU6050_GYRO_RANGE_2000DPS = 3
};

static inline float MPU6050GyroRangeSensitivity(MPU6050GyroRange range)
{
  switch (range) {
  case MPU6050_GYRO_RANGE_250DPS:
    return 131.0;
  case MPU6050_GYRO_RANGE_500DPS:
    return 65.5;
  case MPU6050_GYRO_RANGE_1000DPS:
    return 32.8;
  case MPU6050_GYRO_RANGE_2000DPS:
    return 16.4;
  default:
    return 0.0;
  }
}

enum MPU6050AccelRange
{
  // Value, Range (+/- g), Sensitivity (LSB/g)
  // 0x00,  2,             16384
  // 0x01,  4,             8192
  // 0x02,  8,             4096
  // 0x03,  16,            2048

  MPU6050_ACCEL_RANGE_2G     = 0,
  MPU6050_ACCEL_RANGE_4G     = 1,
  MPU6050_ACCEL_RANGE_8G     = 2,
  MPU6050_ACCEL_RANGE_16G    = 3
};

static inline float MPU6050AccelRangeSensitivity(MPU6050AccelRange range)
{
  switch (range) {
  case MPU6050_ACCEL_RANGE_2G:
    return 16384.0;
  case MPU6050_ACCEL_RANGE_4G:
    return 8192.0;
  case MPU6050_ACCEL_RANGE_8G:
    return 4096.0;
  case MPU6050_ACCEL_RANGE_16G:
    return 2048.0;
  default:
    return 0.0;
  }
}

enum MPU6050Clksel
{
  MPU6050_CLKSEL_0 = 0,
  MPU6050_CLKSEL_1 = 1,
  MPU6050_CLKSEL_2 = 2,
  MPU6050_CLKSEL_3 = 3,
  MPU6050_CLKSEL_4 = 4,
  MPU6050_CLKSEL_5 = 5,
  MPU6050_CLKSEL_6 = 6,
  MPU6050_CLKSEL_7 = 7
};

enum MPU6050LpWakeCtrl
{
  MPU6050_LP_WAKE_CTRL_1_25_HZ = 0,
  MPU6050_LP_WAKE_CTRL_5_HZ = 1,
  MPU6050_LP_WAKE_CTRL_20_HZ = 2,
  MPU6050_LP_WAKE_CTRL_40_HZ = 3
};

#define MPU6050_GYRO_X    (1 << 0)
#define MPU6050_GYRO_Y    (1 << 1)
#define MPU6050_GYRO_Z    (1 << 2)
#define MPU6050_GYRO_ALL  (MPU6050_GYRO_X|MPU6050_GYRO_Y|MPU6050_GYRO_Z)
#define MPU6050_ACCEL_X   (1 << 3)
#define MPU6050_ACCEL_Y   (1 << 4)
#define MPU6050_ACCEL_Z   (1 << 5)
#define MPU6050_ACCEL_ALL (MPU6050_ACCEL_X|MPU6050_ACCEL_Y|MPU6050_ACCEL_Z)
#define MPU6050_TEMP      (1 << 6)
#define MPU6050_ALL       (MPU6050_GYRO_ALL|MPU6050_ACCEL_ALL|MPU6050_TEMP)

struct MPU6050Params
{
  uint8_t           smplrt_div;

  MPU6050DlpfCfg    dlpf_cfg;
  MPU6050ExtSyncSet ext_sync_set;

  MPU6050GyroRange  fs_sel;
  MPU6050AccelRange afs_sel;
  uint8_t           self_test;

  bool              device_reset;
  bool              sleep;
  bool              cycle;
  bool              temp_dis;
  MPU6050Clksel     clksel;

  uint8_t           stby;
  MPU6050LpWakeCtrl lp_wake_ctrl;
};

#define PARAMS_INIT { \
    .smplrt_div   = 0, \
    .dlpf_cfg     = MPU6050_DLPF_CFG_0, \
    .ext_sync_set = MPU6050_EXT_SYNC_SET_0, \
    .fs_sel       = MPU6050_GYRO_RANGE_250DPS, \
    .afs_sel      = MPU6050_ACCEL_RANGE_2G, \
    .self_test    = 0, \
    .device_reset = 0, \
    .sleep        = 0, \
    .cycle        = 0, \
    .temp_dis     = 0, \
    .clksel       = MPU6050_CLKSEL_0, \
    .stby         = 0, \
    .lp_wake_ctrl = MPU6050_LP_WAKE_CTRL_1_25_HZ \
}

struct MPU6050Data3u8
{
  uint8_t           x;
  uint8_t           y;
  uint8_t           z;
};
struct MPU6050Data3i
{
  int16_t           x;
  int16_t           y;
  int16_t           z;
};
static inline void MPU6050Data3i_Subtract(MPU6050Data3i& a, MPU6050Data3i& b, MPU6050Data3i* result)
{
  result->x = a.x - b.x;
  result->y = a.y - b.y;
  result->z = a.z - b.z;
}
struct MPU6050Data3f
{
  float             x;
  float             y;
  float             z;
};



struct MPU6050SelfTestData
{
  MPU6050Data3u8    gyro;
  MPU6050Data3u8    accel;
};
struct MPU6050SelfTestMeasurement
{
  MPU6050Data3i     gyro;
  MPU6050Data3i     accel;
};
static inline void MPU6050SelfTestMeasurement_Subtract(MPU6050SelfTestMeasurement& a, MPU6050SelfTestMeasurement& b, MPU6050SelfTestMeasurement* result)
{
  MPU6050Data3i_Subtract(a.gyro, b.gyro, &result->gyro);
  MPU6050Data3i_Subtract(a.accel, b.accel, &result->accel);
}
struct MPU6050SelfTestResult
{
  MPU6050Data3f     gyro;
  MPU6050Data3f     accel;
};




struct MPU6050Data
{
  MPU6050Data3i     gyro;
  MPU6050Data3i     accel;
  int16_t           temp;
};

struct MPU6050Measurement
{
  MPU6050Data3f     gyro;
  MPU6050Data3f     accel;
  float             temp;
};


class MPU6050 : public I2CDevice
{
public:
  MPU6050(I2CBus& bus);

  bool getReg(uint8_t addr, MPU6050Reg* reg);
  bool getRegs(uint8_t addr, MPU6050Reg* reg, uint8_t count);
  bool setReg(uint8_t addr, MPU6050Reg reg);
  bool setRegs(uint8_t addr, const MPU6050Reg* reg, uint8_t count);
  bool modReg(uint8_t addr, uint8_t reg, uint8_t mask);

  bool getGyroRange(MPU6050GyroRange* range);
  bool setGyroRange(MPU6050GyroRange range);
  bool getAccelRange(MPU6050AccelRange* range);
  bool setAccelRange(MPU6050AccelRange range);

  bool enableReset();
  bool enableSleep();
  bool disableSleep();
  bool enableCycle();
  bool disableCycle();
  bool enableTemp();
  bool disableTemp();

  bool getParams(MPU6050Params* params);
  bool setParams(MPU6050Params& params);

  bool init();

  bool getData(MPU6050Data* data);
  bool getMeasurement(MPU6050Measurement* meas);

  bool enableSelfTest(uint8_t mask);
  bool disableSelfTest(uint8_t mask);
  bool getSelfTestData(MPU6050SelfTestData* st);
  bool getFactoryTrim(MPU6050SelfTestMeasurement* ft);
  bool getSelfTestMeasurement(MPU6050SelfTestMeasurement *res);
  bool getSelfTestResponse(uint8_t mask, MPU6050SelfTestMeasurement* res);
  bool selfTest(uint8_t mask, MPU6050SelfTestResult* result);
};

#endif // _MPU6050_H
