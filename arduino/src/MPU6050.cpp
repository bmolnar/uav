#include "MPU6050.hpp"

#include "I2C.hpp"

#define UINT16(n) ((uint16_t)(n))
#define UINT16_HL(h,l) ((UINT16(h) << 8) | UINT16(l))
#define INT16(n) ((int16_t)(n))
#define INT16_HL(h,l) (INT16(UINT16_HL((h), (l))))
#define FLOAT(n) ((float)(n))

MPU6050::MPU6050(I2CBus& bus)
    : I2CDevice(bus, MPU6050_I2C_ADDRESS)
{
}

bool MPU6050::getReg(uint8_t addr, MPU6050Reg* reg)
{
  int rv = readReg(addr, &reg->bits);
  return (rv >= 0);
}
bool MPU6050::getRegs(uint8_t addr, MPU6050Reg* reg, uint8_t count)
{
  int rv = readRegs(addr, &reg->bits, count);
  return (rv >= 0);
}

bool MPU6050::setReg(uint8_t addr, MPU6050Reg reg)
{
  int rv = writeReg(addr, reg.bits);
  return (rv >= 0);
}
bool MPU6050::setRegs(uint8_t addr, const MPU6050Reg* reg, uint8_t count)
{
  int rv = writeRegs(addr, &reg->bits, count);
  return (rv >= 0);
}

bool MPU6050::modReg(uint8_t addr, uint8_t reg, uint8_t mask)
{
  int rv = updateReg(addr, reg, mask);
  return (rv >= 0);
}


bool MPU6050::getGyroRange(MPU6050GyroRange* range)
{
  MPU6050Reg reg = MPU6050REG_INIT;
  if (!getReg(MPU6050_GYRO_CONFIG, &reg))
    return false;
  *range = (MPU6050GyroRange) reg.gyro_config.fs_sel;
  return true;
}
bool MPU6050::setGyroRange(MPU6050GyroRange range)
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.gyro_config.fs_sel = range;
  return modReg(MPU6050_GYRO_CONFIG, reg.bits, MPU6050_FS_SEL_3);
}


bool MPU6050::getAccelRange(MPU6050AccelRange* range)
{
  MPU6050Reg reg = MPU6050REG_INIT;
  if (!getReg(MPU6050_ACCEL_CONFIG, &reg))
    return false;
  *range = (MPU6050AccelRange) reg.accel_config.afs_sel;
  return true;
}
bool MPU6050::setAccelRange(MPU6050AccelRange range)
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.accel_config.afs_sel = range;
  return modReg(MPU6050_ACCEL_CONFIG, reg.bits, MPU6050_AFS_SEL_3);
}


// PWR_MGMT_1
bool MPU6050::enableReset()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.device_reset = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, ~0, reg.bits);
}
bool MPU6050::enableSleep()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.sleep = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, ~0, reg.bits);
}
bool MPU6050::disableSleep()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.sleep = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, 0, reg.bits);
}
bool MPU6050::enableCycle()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.cycle = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, ~0, reg.bits);
}
bool MPU6050::disableCycle()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.cycle = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, 0, reg.bits);
}
bool MPU6050::enableTemp()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.temp_dis = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, 0, reg.bits);
}
bool MPU6050::disableTemp()
{
  MPU6050Reg reg = MPU6050REG_INIT;
  reg.pwr_mgmt_1.temp_dis = 0x01;
  return modReg(MPU6050_PWR_MGMT_1, ~0, reg.bits);
}





bool MPU6050::init()
{
  MPU6050Params params = PARAMS_INIT;

  if (!enableReset())
    return false;
  if (!disableSleep())
    return false;
  if (!setParams(params))
    return false;
  return true;
}


#define ADD_FLAG(v,f) ((v) ? (f) : 0)

bool MPU6050::getParams(MPU6050Params* params)
{
  MPU6050Reg reg;

  if (!getReg(MPU6050_SMPLRT_DIV, &reg))
    return false;
  params->smplrt_div   = reg.smplrt_div.smplrt_div;

  if (!getReg(MPU6050_CONFIG, &reg))
    return false;
  params->dlpf_cfg     = (MPU6050DlpfCfg) reg.config.dlpf_cfg;
  params->ext_sync_set = (MPU6050ExtSyncSet) reg.config.ext_sync_set;

  if (!getReg(MPU6050_GYRO_CONFIG, &reg))
    return false;
  params->fs_sel       = (MPU6050GyroRange) reg.gyro_config.fs_sel;
  params->self_test    = 0;
  params->self_test   |= ADD_FLAG(reg.gyro_config.zg_st, MPU6050_GYRO_Z);
  params->self_test   |= ADD_FLAG(reg.gyro_config.yg_st, MPU6050_GYRO_Y);
  params->self_test   |= ADD_FLAG(reg.gyro_config.xg_st, MPU6050_GYRO_X);

  if (!getReg(MPU6050_ACCEL_CONFIG, &reg))
    return false;
  params->afs_sel      = (MPU6050AccelRange) reg.accel_config.afs_sel;
  params->self_test   |= ADD_FLAG(reg.accel_config.za_st, MPU6050_ACCEL_Z);
  params->self_test   |= ADD_FLAG(reg.accel_config.ya_st, MPU6050_ACCEL_Y);
  params->self_test   |= ADD_FLAG(reg.accel_config.xa_st, MPU6050_ACCEL_X);

  if (!getReg(MPU6050_PWR_MGMT_1, &reg))
    return false;
  params->clksel       = (MPU6050Clksel) reg.pwr_mgmt_1.clksel;
  params->temp_dis     = reg.pwr_mgmt_1.temp_dis;
  params->cycle        = reg.pwr_mgmt_1.cycle;
  params->sleep        = reg.pwr_mgmt_1.sleep;
  params->device_reset = reg.pwr_mgmt_1.device_reset;

  if (!getReg(MPU6050_PWR_MGMT_2, &reg))
    return false;
  params->lp_wake_ctrl = (MPU6050LpWakeCtrl) reg.pwr_mgmt_2.lp_wake_ctrl;
  params->stby         = ADD_FLAG(reg.pwr_mgmt_2.stby_xa, MPU6050_ACCEL_X)
                         | ADD_FLAG(reg.pwr_mgmt_2.stby_ya, MPU6050_ACCEL_Y)
                         | ADD_FLAG(reg.pwr_mgmt_2.stby_za, MPU6050_ACCEL_Z)
                         | ADD_FLAG(reg.pwr_mgmt_2.stby_xg, MPU6050_GYRO_X)
                         | ADD_FLAG(reg.pwr_mgmt_2.stby_yg, MPU6050_GYRO_Y)
                         | ADD_FLAG(reg.pwr_mgmt_2.stby_zg, MPU6050_GYRO_Z);


  return true;
}

#define CHECK_FLAG(n,f) (((n) & (f)) != 0)

bool MPU6050::setParams(MPU6050Params& params)
{
  MPU6050Reg reg;

  reg.bits                    = 0;
  reg.smplrt_div.smplrt_div   = params.smplrt_div;
  if (!setReg(MPU6050_SMPLRT_DIV, reg))
    return false;

  reg.bits                    = 0;
  reg.config.dlpf_cfg         = params.dlpf_cfg;
  reg.config.ext_sync_set     = params.ext_sync_set;
  if (!setReg(MPU6050_CONFIG, reg))
    return false;

  reg.bits                    = 0;
  reg.gyro_config.fs_sel      = params.fs_sel;
  reg.gyro_config.zg_st       = CHECK_FLAG(params.self_test, MPU6050_GYRO_Z);
  reg.gyro_config.yg_st       = CHECK_FLAG(params.self_test, MPU6050_GYRO_Y);
  reg.gyro_config.xg_st       = CHECK_FLAG(params.self_test, MPU6050_GYRO_X);
  if (!setReg(MPU6050_GYRO_CONFIG, reg))
    return false;

  reg.bits                    = 0;
  reg.accel_config.afs_sel    = params.afs_sel;
  reg.accel_config.za_st      = CHECK_FLAG(params.self_test, MPU6050_ACCEL_Z);
  reg.accel_config.ya_st      = CHECK_FLAG(params.self_test, MPU6050_ACCEL_Y);
  reg.accel_config.xa_st      = CHECK_FLAG(params.self_test, MPU6050_ACCEL_X);
  if (!setReg(MPU6050_ACCEL_CONFIG, reg))
    return false;

  reg.bits                    = 0;
  reg.pwr_mgmt_1.clksel       = params.clksel;
  reg.pwr_mgmt_1.temp_dis     = params.temp_dis;
  reg.pwr_mgmt_1.cycle        = params.cycle;
  reg.pwr_mgmt_1.sleep        = params.sleep;
  reg.pwr_mgmt_1.device_reset = params.device_reset;
  if (!setReg(MPU6050_PWR_MGMT_1, reg))
    return false;

  reg.bits                    = 0;
  reg.pwr_mgmt_2.lp_wake_ctrl = params.lp_wake_ctrl;
  reg.pwr_mgmt_2.stby_xa      = CHECK_FLAG(params.stby, MPU6050_ACCEL_X);
  reg.pwr_mgmt_2.stby_ya      = CHECK_FLAG(params.stby, MPU6050_ACCEL_Y);
  reg.pwr_mgmt_2.stby_za      = CHECK_FLAG(params.stby, MPU6050_ACCEL_Z);
  reg.pwr_mgmt_2.stby_xg      = CHECK_FLAG(params.stby, MPU6050_GYRO_X);
  reg.pwr_mgmt_2.stby_yg      = CHECK_FLAG(params.stby, MPU6050_GYRO_Y);
  reg.pwr_mgmt_2.stby_zg      = CHECK_FLAG(params.stby, MPU6050_GYRO_Z);
  if (!setReg(MPU6050_PWR_MGMT_2, reg))
    return false;

  return true;
}



bool MPU6050::getData(MPU6050Data* data)
{
  uint8_t regdata[14];
  int rv;

  rv = readRegs(MPU6050_ACCEL_XOUT_H, regdata, 14);
  if (rv < 0)
    return false;

  data->accel.x = INT16_HL(regdata[0], regdata[1]);
  data->accel.y = INT16_HL(regdata[2], regdata[3]);
  data->accel.z = INT16_HL(regdata[4], regdata[5]);
  data->temp    = INT16_HL(regdata[6], regdata[7]);
  data->gyro.x  = INT16_HL(regdata[8], regdata[9]);
  data->gyro.y  = INT16_HL(regdata[10], regdata[11]);
  data->gyro.z  = INT16_HL(regdata[12], regdata[13]);

  return true;
}

bool MPU6050::getMeasurement(MPU6050Measurement* meas)
{
  MPU6050GyroRange gyroRange;
  if (!getGyroRange(&gyroRange))
    return false;
  float gyroSens = MPU6050GyroRangeSensitivity(gyroRange);

  MPU6050AccelRange accelRange;
  if (!getAccelRange(&accelRange))
    return false;
  float accelSens = MPU6050AccelRangeSensitivity(accelRange);

  MPU6050Data data;
  if (!getData(&data))
    return false;

  meas->gyro.x  = FLOAT(data.gyro.x) / gyroSens;
  meas->gyro.y  = FLOAT(data.gyro.y) / gyroSens;
  meas->gyro.z  = FLOAT(data.gyro.z) / gyroSens;
  meas->accel.x = FLOAT(data.accel.x) / accelSens;
  meas->accel.y = FLOAT(data.accel.y) / accelSens;
  meas->accel.z = FLOAT(data.accel.z) / accelSens;
  meas->temp    = FLOAT(data.temp + 12420) / 340.0;
  return true;
}




bool MPU6050::enableSelfTest(uint8_t mask)
{
  MPU6050Params params;
  if (!getParams(&params))
    return false;
  params.self_test |= mask;
  if (!setParams(params))
    return false;
  return true;
}

bool MPU6050::disableSelfTest(uint8_t mask)
{
  MPU6050Params params;
  if (!getParams(&params))
    return false;
  params.self_test &= ~mask;
  if (!setParams(params))
    return false;
  return true;
}

bool MPU6050::getSelfTestData(MPU6050SelfTestData* data)
{
  struct {
    MPU6050Reg_SelfTestX self_test_x;
    MPU6050Reg_SelfTestY self_test_y;
    MPU6050Reg_SelfTestZ self_test_z;
    MPU6050Reg_SelfTestA self_test_a;
  } regs;

  if (!getRegs(MPU6050_SELF_TEST_X, (MPU6050Reg*) &regs.self_test_x, 4))
    return false;
  data->gyro.x  = regs.self_test_x.xg_test;
  data->gyro.y  = regs.self_test_y.yg_test;
  data->gyro.z  = regs.self_test_z.zg_test;
  data->accel.x = ((regs.self_test_x.xa_test & 0x07) << 2) | (regs.self_test_a.xa_test & 0x03);
  data->accel.y = ((regs.self_test_y.ya_test & 0x07) << 2) | (regs.self_test_a.ya_test & 0x03);
  data->accel.z = ((regs.self_test_z.za_test & 0x07) << 2) | (regs.self_test_a.za_test & 0x03);
  return true;
}

#define GYRO_FACTORY_TRIM(n)  ((n > 0) ? INT16(25.0 * 131.0 * powf(1.046, FLOAT(n-1))) : 0)
#define ACCEL_FACTORY_TRIM(n) ((n > 0) ? INT16(4096.0 * 0.34 * powf((0.92/0.34), (FLOAT(n-1) / 30.0))) : 0)

bool MPU6050::getFactoryTrim(MPU6050SelfTestMeasurement* ft)
{
  MPU6050SelfTestData st;

  if (!getSelfTestData(&st))
    return false;
  ft->gyro.x  = GYRO_FACTORY_TRIM(st.gyro.x);
  ft->gyro.y  = -GYRO_FACTORY_TRIM(st.gyro.y);
  ft->gyro.z  = GYRO_FACTORY_TRIM(st.gyro.z);
  ft->accel.x = ACCEL_FACTORY_TRIM(st.accel.x);
  ft->accel.y = ACCEL_FACTORY_TRIM(st.accel.y);
  ft->accel.z = ACCEL_FACTORY_TRIM(st.accel.z);
  return true;
}

bool MPU6050::getSelfTestMeasurement(MPU6050SelfTestMeasurement *res)
{
  uint8_t regdata[14];
  int rv;

  rv = readRegs(MPU6050_ACCEL_XOUT_H, regdata, 14);
  if (rv < 0)
    return false;
  res->accel.x = INT16_HL(regdata[0], regdata[1]);
  res->accel.y = INT16_HL(regdata[2], regdata[3]);
  res->accel.z = INT16_HL(regdata[4], regdata[5]);
  res->gyro.x  = INT16_HL(regdata[8], regdata[9]);
  res->gyro.y  = INT16_HL(regdata[10], regdata[11]);
  res->gyro.z  = INT16_HL(regdata[12], regdata[13]);
  return true;
}

bool MPU6050::getSelfTestResponse(uint8_t mask, MPU6050SelfTestMeasurement* res)
{
  if (!enableSelfTest(mask))
    return false;
  MPU6050SelfTestMeasurement dataSTEnabled;
  if (!getSelfTestMeasurement(&dataSTEnabled))
    return false;

  if (!disableSelfTest(mask))
    return false;
  MPU6050SelfTestMeasurement dataSTDisabled;
  if (!getSelfTestMeasurement(&dataSTDisabled))
    return false;

  MPU6050SelfTestMeasurement_Subtract(dataSTEnabled, dataSTDisabled, res);
  return true;
}

bool MPU6050::selfTest(uint8_t mask, MPU6050SelfTestResult* result)
{
  MPU6050Params params;
  if (!getParams(&params))
    return false;
  params.fs_sel  = MPU6050_GYRO_RANGE_250DPS;
  params.afs_sel = MPU6050_ACCEL_RANGE_8G;
  if (!setParams(params))
    return false;

  MPU6050SelfTestMeasurement str;
  if (!getSelfTestResponse(mask, &str))
    return false;

  MPU6050SelfTestMeasurement ft;
  if (!getFactoryTrim(&ft))
    return false;

  result->gyro.x  = (FLOAT(str.gyro.x) -  FLOAT(ft.gyro.x))  / FLOAT(ft.gyro.x);
  result->gyro.y  = (FLOAT(str.gyro.y) -  FLOAT(ft.gyro.y))  / FLOAT(ft.gyro.y);
  result->gyro.z  = (FLOAT(str.gyro.z) -  FLOAT(ft.gyro.z))  / FLOAT(ft.gyro.z);
  result->accel.x = (FLOAT(str.accel.x) - FLOAT(ft.accel.x)) / FLOAT(ft.accel.x);
  result->accel.y = (FLOAT(str.accel.y) - FLOAT(ft.accel.y)) / FLOAT(ft.accel.y);
  result->accel.z = (FLOAT(str.accel.z) - FLOAT(ft.accel.z)) / FLOAT(ft.accel.z);
  return true;
}
