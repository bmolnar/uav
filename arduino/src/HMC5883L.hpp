#ifndef _HMC5883L_H
#define _HMC5883L_H

#include "I2C.hpp"

/*
 * HMC5883L Three-Axis Digital Compass
 */

#define HMC5883L_CONF_A      0x00
#define HMC5883L_CONF_B      0x01
#define HMC5883L_MODE        0x02
#define HMC5883L_DATA_XOUT_H 0x03
#define HMC5883L_DATA_XOUT_L 0x04
#define HMC5883L_DATA_YOUT_H 0x05
#define HMC5883L_DATA_YOUT_L 0x06
#define HMC5883L_DATA_ZOUT_H 0x07
#define HMC5883L_DATA_ZOUT_L 0x08
#define HMC5883L_STATUS      0x09
#define HMC5883L_IDENT_A     0x0A
#define HMC5883L_IDENT_B     0x0B
#define HMC5883L_IDENT_C     0x0C

//#define HMC5883L_I2C_ADDRESS 0x3C
#define HMC5883L_I2C_ADDRESS 0x1E

#define HMC5883L_MODE_CONTINUOUS 0x00
#define HMC5883L_MODE_SINGLESHOT 0x01
#define HMC5883L_MODE_IDLE 0x03


#define HMC5883L_D0 0
#define HMC5883L_D1 1
#define HMC5883L_D2 2
#define HMC5883L_D3 3
#define HMC5883L_D4 4
#define HMC5883L_D5 5
#define HMC5883L_D6 6
#define HMC5883L_D7 7


//
// Configuration Register A
//
#define HMC5883L_MS0  HMC5883L_D0
#define HMC5883L_MS1  HMC5883L_D1
#define HMC5883L_DO0  HMC5883L_D2
#define HMC5883L_DO1  HMC5883L_D3
#define HMC5883L_DO2  HMC5883L_D4
#define HMC5883L_MA0  HMC5883L_D5
#define HMC5883L_MA1  HMC5883L_D6
#define HMC5883L_CRA7 HMC5883L_D7
#define HMC5883L_CRA_UNUSED (bit(HMC5883L_CRA7))

#define HMC5883L_MS_0 (0)
#define HMC5883L_MS_1 (bit(HMC5883L_MS0))
#define HMC5883L_MS_2 (bit(HMC5883L_MS1))
#define HMC5883L_MS_3 (bit(HMC5883L_MS1)|bit(HMC5883L_MS0))

#define HMC5883L_DO_0 (0)
#define HMC5883L_DO_1 (bit(HMC5883L_DO0))
#define HMC5883L_DO_2 (bit(HMC5883L_DO1))
#define HMC5883L_DO_3 (bit(HMC5883L_DO1)|bit(HMC5883L_DO0))
#define HMC5883L_DO_4 (bit(HMC5883L_DO2))
#define HMC5883L_DO_5 (bit(HMC5883L_DO2)|bit(HMC5883L_DO0))
#define HMC5883L_DO_6 (bit(HMC5883L_DO2)|bit(HMC5883L_DO1))
#define HMC5883L_DO_7 (bit(HMC5883L_DO2)|bit(HMC5883L_DO1)|bit(HMC5883L_DO0))

#define HMC5883L_MA_0 (0)
#define HMC5883L_MA_1 (bit(HMC5883L_MA0))
#define HMC5883L_MA_2 (bit(HMC5883L_MA1))
#define HMC5883L_MA_3 (bit(HMC5883L_MA1)|bit(HMC5883L_MA0))

//
// Configuration Register B
//
#define HMC5883L_CRB0 HMC5883L_D0
#define HMC5883L_CRB1 HMC5883L_D1
#define HMC5883L_CRB2 HMC5883L_D2
#define HMC5883L_CRB3 HMC5883L_D3
#define HMC5883L_CRB4 HMC5883L_D4
#define HMC5883L_GN0  HMC5883L_D5
#define HMC5883L_GN1  HMC5883L_D6
#define HMC5883L_GN2  HMC5883L_D7
#define HMC5883L_CRB_UNUSED \
    (bit(HMC5883L_CRB4)|bit(HMC5883L_CRB3)|bit(HMC5883L_CRB2)|bit(HMC5883L_CRB1)|bit(HMC5883L_CRB0))

#define HMC5883L_GN_0 (0)
#define HMC5883L_GN_1 (bit(HMC5883L_GN0))
#define HMC5883L_GN_2 (bit(HMC5883L_GN1))
#define HMC5883L_GN_3 (bit(HMC5883L_GN1)|bit(HMC5883L_GN0))
#define HMC5883L_GN_4 (bit(HMC5883L_GN2))
#define HMC5883L_GN_5 (bit(HMC5883L_GN2)|bit(HMC5883L_GN0))
#define HMC5883L_GN_6 (bit(HMC5883L_GN2)|bit(HMC5883L_GN1))
#define HMC5883L_GN_7 (bit(HMC5883L_GN2)|bit(HMC5883L_GN1)|bit(HMC5883L_GN0))

//
// Mode Register
//
#define HMC5883L_MD0  HMC5883L_D0
#define HMC5883L_MD1  HMC5883L_D1
#define HMC5883L_MR2  HMC5883L_D2
#define HMC5883L_MR3  HMC5883L_D3
#define HMC5883L_MR4  HMC5883L_D4
#define HMC5883L_MR5  HMC5883L_D5
#define HMC5883L_MR6  HMC5883L_D6
#define HMC5883L_MR7  HMC5883L_D7
#define HMC5883L_MR_UNUSED \
    (bit(HMC5883L_MR7)|bit(HMC5883L_MR6)|bit(HMC5883L_MR5)|bit(HMC5883L_MR4)|bit(HMC5883L_MR3)|bit(HMC5883L_MR2))

#define HMC5883L_MD_0 (0)
#define HMC5883L_MD_1 (bit(HMC5883L_MD0))
#define HMC5883L_MD_2 (bit(HMC5883L_MD1))
#define HMC5883L_MD_3 (bit(HMC5883L_MD1)|bit(HMC5883L_MD0))


struct HMC5883LRegConfA
{
  uint8_t MS:2;
  uint8_t DO:3;
  uint8_t MA:2;
  uint8_t unused0:1;
};

struct HMC5883LRegConfB
{
  uint8_t unused0:5;
  uint8_t GN:3;
};

struct HMC5883LRegMode
{
  uint8_t MD:2;
  uint8_t unused0:5;
  uint8_t HS:1;
};

struct HMC5883LRegStatus
{
  uint8_t RDY:1;
  uint8_t LOCK:1;
  uint8_t unused0:6;
};

typedef union {
  HMC5883LRegConfA  conf_a;
  HMC5883LRegConfB  conf_b;
  HMC5883LRegMode   mode;
  HMC5883LRegStatus status;
  uint8_t           bits;
} HMC5883LReg;


#define HMC5883L_MAG_X (1 << 0)
#define HMC5883L_MAG_Y (1 << 1)
#define HMC5883L_MAG_Z (1 << 2)

enum HMC5883LMsMode
{
  // 0x00: Normal (Default)
  // 0x01: Positive Bias
  // 0x02: Negative Bias
  // 0x03: (Reserved)

  HMC5883L_MSMODE_NORMAL  = 0,
  HMC5883L_MSMODE_POSBIAS = 1,
  HMC5883L_MSMODE_NEGBIAS = 2,
  HMC5883L_MSMODE_UNUSED  = 3
};

enum HMC5883LOutputRate
{
  // 0x00: 0.75 Hz
  // 0x01: 1.5 Hz
  // 0x02: 3 Hz
  // 0x03: 7.5 Hz
  // 0x04: 15 Hz (Default)
  // 0x05: 30 Hz
  // 0x06: 75 Hz
  // 0x07: (Not Used)

  HMC5883L_OUTPRT_0_75_HZ = 0,
  HMC5883L_OUTPRT_1_5_HZ  = 1,
  HMC5883L_OUTPRT_3_HZ    = 2,
  HMC5883L_OUTPRT_7_5_HZ  = 3,
  HMC5883L_OUTPRT_15_HZ   = 4,
  HMC5883L_OUTPRT_30_HZ   = 5,
  HMC5883L_OUTPRT_75_HZ   = 6,
  HMC5883L_OUTPRT_UNUSED  = 7
};

enum HMC5883LSampleAvg
{
  // 0x00: 1 (Default)
  // 0x01: 2
  // 0x02: 4
  // 0x03: 8

  HMC5883L_SAMPLE_AVG_1  = 0,
  HMC5883L_SAMPLE_AVG_2  = 1,
  HMC5883L_SAMPLE_AVG_4  = 2,
  HMC5883L_SAMPLE_AVG_8  = 3
};

enum HMC5883LRange
{
  // Value: Range (+/- Ga), Gain (LSB/Ga), Resolution (mGa/LSB)
  // 0x00:  0.88,           1370,          0.73
  // 0x01:  1.3,            1090,          0.92 (Default)
  // 0x02:  1.9,            820,           1.22
  // 0x03:  2.5,            660,           1.52
  // 0x04:  4.0,            440,           2.27
  // 0x05:  4.7,            390,           2.56
  // 0x06:  5.6,            330,           3.03
  // 0x07:  8.1,            230,           4.35

  HMC5883L_RANGE_0_88 = 0,
  HMC5883L_RANGE_1_3  = 1,
  HMC5883L_RANGE_1_9  = 2,
  HMC5883L_RANGE_2_5  = 3,
  HMC5883L_RANGE_4_0  = 4,
  HMC5883L_RANGE_4_7  = 5,
  HMC5883L_RANGE_5_6  = 6,
  HMC5883L_RANGE_8_1  = 7
};

static inline uint16_t HMC5883LRangePrecision(HMC5883LRange range)
{
  switch (range) {
  case HMC5883L_RANGE_0_88:
    return 1370;
  case HMC5883L_RANGE_1_3:
    return 1090;
  case HMC5883L_RANGE_1_9:
    return 820;
  case HMC5883L_RANGE_2_5:
    return 660;
  case HMC5883L_RANGE_4_0:
    return 440;
  case HMC5883L_RANGE_4_7:
    return 390;
  case HMC5883L_RANGE_5_6:
    return 330;
  case HMC5883L_RANGE_8_1:
    return 230;
  default:
    return 0.0;
  }
}

enum HMC5883LOpMode
{
  // 0x00: Continuous Mode
  // 0x01: Single-Measurement Mode (Default)
  // 0x02: Idle Mode
  // 0x03: Idle Mode

  HMC5883L_OPMODE_CONT   = 0,
  HMC5883L_OPMODE_SINGLE = 1,
  HMC5883L_OPMODE_IDLE1  = 2,
  HMC5883L_OPMODE_IDLE2  = 3
};


struct HMC5883LParams
{
  // CONFA
  HMC5883LMsMode     msmode;
  HMC5883LOutputRate outprt;
  HMC5883LSampleAvg  smpavg;

  // CONFB
  HMC5883LRange      range;

  // MODE
  HMC5883LOpMode     opmode;
  bool               hs;
};

#define HMC5883LParams_INIT {       \
  .msmode = HMC5883L_MSMODE_NORMAL, \
  .outprt = HMC5883L_OUTPRT_15_HZ,  \
  .smpavg = HMC5883L_SAMPLE_AVG_1,  \
  .range  = HMC5883L_RANGE_1_3,     \
  .opmode = HMC5883L_OPMODE_SINGLE, \
  .hs     = false                   \
}


struct HMC5883LData3i
{
  int16_t           x;
  int16_t           y;
  int16_t           z;
};
struct HMC5883LData3f
{
  float             x;
  float             y;
  float             z;
};

#define HMC5883L_XY_EXCITATION 1160
#define HMC5883L_Z_EXCITATION  1080

struct HMC5883LCalibration
{
  HMC5883LData3f    gain_error;
  HMC5883LData3f    offset;
  HMC5883LData3f    min;
  HMC5883LData3f    max;
};

struct HMC5883LData
{
  HMC5883LData3i    mag;
};
struct HMC5883LMeasurement
{
  HMC5883LData3f    mag;
};

static inline uint8_t checkSelfTestLimits(HMC5883LData& data, HMC5883LRange range)
{
  uint16_t prec = HMC5883LRangePrecision(range);
  int16_t lower = 243 * prec / 390;
  int16_t upper = 575 * prec / 390;

  return ((data.mag.x < lower || upper < data.mag.x) ? HMC5883L_MAG_X : 0)
    | ((data.mag.y < lower || upper < data.mag.y) ? HMC5883L_MAG_Y : 0)
    | ((data.mag.z < lower || upper < data.mag.z) ? HMC5883L_MAG_Z : 0);
}

class HMC5883L : public I2CDevice
{
public:
  HMC5883L(I2CBus& bus);

  bool getReg(uint8_t addr, HMC5883LReg* reg);
  bool setReg(uint8_t addr, HMC5883LReg reg);

  bool getParams(HMC5883LParams* params);
  bool setParams(HMC5883LParams& params);
  bool init();

  bool getMsMode(HMC5883LMsMode* msmode);
  bool setMsMode(HMC5883LMsMode msmode);

  bool getRange(HMC5883LRange* range);
  bool setRange(HMC5883LRange range);

  bool getData(HMC5883LData* data);
  bool updateCalibration(HMC5883LCalibration* calib);
  bool getCalibration(HMC5883LCalibration* calib);
  bool getMeasurement(HMC5883LCalibration& calib, HMC5883LMeasurement* meas);

  uint8_t selfTestAcquireAndCheck(HMC5883LRange range);
  bool selfTestAcquireLoop(HMC5883LRange range, int max_count);
  bool selfTestAtRange(HMC5883LRange range);
  bool selfTest();
};

#endif /* _HMC5883L_H */
