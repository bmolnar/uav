#include "HMC5883L.hpp"

#include "I2C.hpp"

#define UINT16(n) ((uint16_t)(n))
#define UINT16_HL(h,l) ((UINT16(h) << 8) | UINT16(l))
#define INT16(n) ((int16_t)(n))
#define INT16_HL(h,l) (INT16(UINT16_HL((h), (l))))
#define FLOAT(n) ((float)(n))

HMC5883L::HMC5883L(I2CBus& bus)
  : I2CDevice(bus, HMC5883L_I2C_ADDRESS)
{
}

bool HMC5883L::getReg(uint8_t addr, HMC5883LReg* reg)
{
  int rv = readReg(addr, &reg->bits);
  return (rv >= 0);
}
bool HMC5883L::setReg(uint8_t addr, HMC5883LReg reg)
{
  int rv = writeReg(addr, reg.bits);
  return (rv >= 0);
}


bool HMC5883L::getParams(HMC5883LParams* params)
{
  HMC5883LReg reg;

  if (!getReg(HMC5883L_CONF_A, &reg))
    return false;
  params->msmode = (HMC5883LMsMode) reg.conf_a.MS;
  params->outprt = (HMC5883LOutputRate) reg.conf_a.DO;
  params->smpavg = (HMC5883LSampleAvg) reg.conf_a.MA;

  if (!getReg(HMC5883L_CONF_B, &reg))
    return false;
  params->range = (HMC5883LRange) reg.conf_b.GN;

  if (!getReg(HMC5883L_MODE, &reg))
    return false;
  params->opmode = (HMC5883LOpMode) reg.mode.MD;
  params->hs     = reg.mode.HS;

  return true;
}

bool HMC5883L::setParams(HMC5883LParams& params)
{
  HMC5883LReg reg;

  reg.bits      = 0;
  reg.conf_a.MS = params.msmode;
  reg.conf_a.DO = params.outprt;
  reg.conf_a.MA = params.smpavg;
  if (!setReg(HMC5883L_CONF_A, reg))
    return false;

  reg.bits      = 0;
  reg.conf_b.GN = params.range;
  if (!setReg(HMC5883L_CONF_B, reg))
    return false;

  reg.bits      = 0;
  reg.mode.MD   = params.opmode;
  reg.mode.HS   = params.hs;
  if (!setReg(HMC5883L_MODE, reg))
    return false;

  return true;
}

bool HMC5883L::init()
{
  HMC5883LParams params = HMC5883LParams_INIT;
  if (!setParams(params))
    return false;
  return true;
}

bool HMC5883L::getMsMode(HMC5883LMsMode* msmode)
{
  HMC5883LParams params;

  if (!getParams(&params))
    return false;
  *msmode = params.msmode;
  return true;
}
bool HMC5883L::setMsMode(HMC5883LMsMode msmode)
{
  HMC5883LParams params;

  if (!getParams(&params))
    return false;
  params.msmode = msmode;
  if (!setParams(params))
    return false;
  return true;
}

bool HMC5883L::getRange(HMC5883LRange* range)
{
  HMC5883LParams params;

  if (!getParams(&params))
    return false;
  *range = (HMC5883LRange) params.range;
  return true;
}
bool HMC5883L::setRange(HMC5883LRange range)
{
  HMC5883LParams params;

  if (!getParams(&params))
    return false;
  params.range = range;
  if (!setParams(params))
    return false;
  return true;
}

bool HMC5883L::getData(HMC5883LData* data)
{
  uint8_t regs[6];
  int rv;

  rv = readRegs(HMC5883L_DATA_XOUT_H, regs, 6);
  if (rv < 0)
    return false;
  rv = sendAddr(HMC5883L_DATA_XOUT_H);
  if (rv < 0)
    return false;

  data->mag.x = INT16_HL(regs[0], regs[1]);
  data->mag.z = INT16_HL(regs[2], regs[3]);
  data->mag.y = INT16_HL(regs[4], regs[5]);
  return true;
}

bool HMC5883L::updateCalibration(HMC5883LCalibration* calib)
{
  HMC5883LMeasurement meas;
  if (!getMeasurement(*calib, &meas))
    return false;

  calib->min.x = min(calib->min.x, (meas.mag.x - calib->offset.x));
  calib->min.y = min(calib->min.y, (meas.mag.y - calib->offset.y));
  calib->min.z = min(calib->min.z, (meas.mag.z - calib->offset.z));

  calib->max.x = max(calib->max.x, (meas.mag.x - calib->offset.x));
  calib->max.y = max(calib->max.y, (meas.mag.y - calib->offset.y));
  calib->max.z = max(calib->max.z, (meas.mag.z - calib->offset.z));

  calib->offset.x = ((calib->max.x - calib->min.x) / 2.0) - calib->max.x;
  calib->offset.y = ((calib->max.y - calib->min.y) / 2.0) - calib->max.y;
  calib->offset.z = ((calib->max.z - calib->min.z) / 2.0) - calib->max.z;

  return true;
}

bool HMC5883L::getCalibration(HMC5883LCalibration* calib)
{
  HMC5883LData3f gain_error;

  calib->gain_error.x = calib->gain_error.y = calib->gain_error.z = 1.0;
  calib->offset.x     = calib->offset.y     = calib->offset.z     = 0.0;

  // POSBIAS
  if (!setMsMode(HMC5883L_MSMODE_POSBIAS))
    return false;
  delay(100);
  HMC5883LMeasurement meas;
  meas.mag.x = meas.mag.y = meas.mag.z = -4096.0;
  while ((meas.mag.x < 0.1) | (meas.mag.y < 0.1) | (meas.mag.z < 0.1)) {
    if (!getMeasurement(*calib, &meas))
      return false;
  }

  gain_error.x = FLOAT(HMC5883L_XY_EXCITATION) / meas.mag.x;
  gain_error.y = FLOAT(HMC5883L_XY_EXCITATION) / meas.mag.y;
  gain_error.z = FLOAT(HMC5883L_Z_EXCITATION) / meas.mag.z;

  // NEGBIAS
  if (!setMsMode(HMC5883L_MSMODE_NEGBIAS))
    return false;
  delay(100);
  meas.mag.x = meas.mag.y = meas.mag.z = 4096.0;
  while ((meas.mag.x > -0.1) | (meas.mag.y > -0.1) | (meas.mag.z > -0.1)) {
    if (!getMeasurement(*calib, &meas))
      return false;
  }

  calib->gain_error.x = ((FLOAT(HMC5883L_XY_EXCITATION) / fabsf(meas.mag.x)) + gain_error.x) / 2.0;
  calib->gain_error.y = ((FLOAT(HMC5883L_XY_EXCITATION) / fabsf(meas.mag.y)) + gain_error.y) / 2.0;
  calib->gain_error.z = ((FLOAT(HMC5883L_Z_EXCITATION) / fabsf(meas.mag.z)) + gain_error.z) / 2.0;

  // NORMAL
  if (!setMsMode(HMC5883L_MSMODE_NORMAL))
    return false;
  delay(100);
  if (!getMeasurement(*calib, &meas))
    return false;

  calib->min.x = calib->max.x = meas.mag.x;
  calib->min.y = calib->max.y = meas.mag.y;
  calib->min.z = calib->max.z = meas.mag.z;
  return true;
}


bool HMC5883L::getMeasurement(HMC5883LCalibration& calib, HMC5883LMeasurement* meas)
{
  HMC5883LRange range;
  if (!getRange(&range))
    return false;

  HMC5883LData data;
  if (!getData(&data))
    return false;

  float prec = (float) HMC5883LRangePrecision(range);
  meas->mag.x = (FLOAT(data.mag.x) / prec) * calib.gain_error.x + calib.offset.x;
  meas->mag.y = (FLOAT(data.mag.y) / prec) * calib.gain_error.y + calib.offset.y;
  meas->mag.z = (FLOAT(data.mag.z) / prec) * calib.gain_error.z + calib.offset.z;
  return true;
}



uint8_t HMC5883L::selfTestAcquireAndCheck(HMC5883LRange range)
{
  HMC5883LData data;
  if (!getData(&data))
    return -1;
  return checkSelfTestLimits(data, range);
}

bool HMC5883L::selfTestAcquireLoop(HMC5883LRange range, int max_count)
{
  while (max_count-- > 0) {
    if (selfTestAcquireAndCheck(range) == 0)
      return true;
    delay(67);
  }
  return false;
}

bool HMC5883L::selfTestAtRange(HMC5883LRange range)
{
  HMC5883LParams params = HMC5883LParams_INIT;
  params.smpavg = HMC5883L_SAMPLE_AVG_8;
  params.outprt = HMC5883L_OUTPRT_15_HZ;
  params.msmode = HMC5883L_MSMODE_POSBIAS;
  params.range  = range;
  params.opmode = HMC5883L_OPMODE_CONT;
  if (!setParams(params))
    return false;
  delay(6);
  return selfTestAcquireLoop(range, 128);  
}

bool HMC5883L::selfTest()
{
  for (int range = HMC5883L_RANGE_4_7; range < HMC5883L_RANGE_8_1; range++) {
    if (selfTestAtRange((HMC5883LRange) range))
      return true;
  }
  return false;
}

