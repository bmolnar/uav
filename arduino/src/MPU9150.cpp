#include "MPU9150.hpp"

#include "I2C.hpp"

#define UINT16(n) ((uint16_t)(n))
#define UINT16_HL(h,l) ((UINT16(h) << 8) | UINT16(l))
#define INT16(n) ((int16_t)(n))
#define INT16_HL(h,l) (INT16(UINT16_HL((h), (l))))

MPU9150::MPU9150(I2CBus& bus)
    : I2CDevice(bus, MPU9150_I2C_ADDRESS)
{
}

bool MPU9150::setPwrMgmt1(bool device_reset, bool sleep, bool cycle, bool temp_dis, uint8_t clksel)
{
  uint8_t reg = (device_reset ? (0x01 << 7) : 0x00)
                | (sleep ? (0x01 << 6) : 0x00)
                | (cycle ? (0x01 << 5) : 0x00)
                | (temp_dis ? (0x01 << 3) : 0x00)
                | (clksel & 0x07);
  int rv = writeReg(MPU9150_PWR_MGMT_1, reg);
  return (rv >= 0) ? true : false;
}

bool MPU9150::getWhoAmI(uint8_t *reg)
{
  int rv = readReg(MPU9150_WHO_AM_I, reg);
  return (rv >= 0) ? true : false;
}




bool MPU9150::resetDevice()
{
  return setPwrMgmt1(1, 0, 0, 0, 0);
}




bool MPU9150::setConfig(MPU9150Config config)
{
  int rv = writeReg(MPU9150_CONFIG, *(uint8_t*) &config);
  return (rv >= 0);
}
bool MPU9150::getConfig(MPU9150Config* config)
{
  int rv = readReg(MPU9150_CONFIG, (uint8_t*) &config);
  return (rv >= 0);
}


bool MPU9150::setGyroConfig(MPU9150GyroConfig config)
{
  int rv = writeReg(MPU9150_GYRO_CONFIG, *(uint8_t*) &config);
  return (rv >= 0);
}
bool MPU9150::getGyroConfig(MPU9150GyroConfig* config)
{
  int rv = readReg(MPU9150_GYRO_CONFIG, (uint8_t*) &config);
  return (rv >= 0);
}


bool MPU9150::setAccelConfig(MPU9150AccelConfig config)
{
  int rv = writeReg(MPU9150_ACCEL_CONFIG, *(uint8_t*) &config);
  return (rv >= 0);
}
bool MPU9150::getAccelConfig(MPU9150AccelConfig* config)
{
  int rv = readReg(MPU9150_ACCEL_CONFIG, (uint8_t*) &config);
  return (rv >= 0);
}





bool MPU9150::setAccelRange(MPU9150AccelRange range)
{
  uint8_t value;

  switch (range) {
  case ACCEL_RANGE_2G:
    value = MPU9150_AFS_SEL_2G;
    break;
  case ACCEL_RANGE_4G:
    value = MPU9150_AFS_SEL_4G;
    break;
  case ACCEL_RANGE_8G:
    value = MPU9150_AFS_SEL_8G;
    break;
  case ACCEL_RANGE_16G:
    value = MPU9150_AFS_SEL_16G;
    break;
  default:
    return false;
  }

  int rv = updateReg(MPU9150_ACCEL_CONFIG, value, MPU9150_AFS_SEL_3);
  return (rv >= 0);
}

bool MPU9150::getAccelRange(MPU9150AccelRange* range)
{
  uint8_t value;
  int rv;

  rv = readReg(MPU9150_ACCEL_CONFIG, &value);
  if (rv < 0) {
    return false;
  }

  switch (value) {
  case MPU9150_AFS_SEL_2G:
    *range = ACCEL_RANGE_2G;
    break;
  case MPU9150_AFS_SEL_4G:
    *range = ACCEL_RANGE_4G;
    break;
  case MPU9150_AFS_SEL_8G:
    *range = ACCEL_RANGE_8G;
    break;
  case MPU9150_AFS_SEL_16G:
    *range = ACCEL_RANGE_16G;
    break;
  default:
    return false;
  }
  return true;
}

bool MPU9150::setGyroRange(uint8_t value)
{
  int rv = updateReg(MPU9150_GYRO_CONFIG, (value << MPU9150_FS_SEL0), MPU9150_FS_SEL_3);
  return (rv >= 0);
}

bool MPU9150::getGyroRange(uint8_t* value)
{
  uint8_t reg;
  int rv;

  rv = readReg(MPU9150_GYRO_CONFIG, &reg);
  if (rv < 0) {
    return false;
  }
  *value = (reg & MPU9150_FS_SEL_3) >> MPU9150_FS_SEL0;
  return true;
}

bool MPU9150::getData(MPU9150Data* data)
{
  uint8_t regdata[14];
  int rv;

  rv = readRegs(MPU9150_ACCEL_XOUT_H, regdata, 14);
  if (rv < 0)
    return false;

  data->accel_x = INT16_HL(regdata[0], regdata[1]);
  data->accel_y = INT16_HL(regdata[2], regdata[3]);
  data->accel_z = INT16_HL(regdata[4], regdata[5]);
  data->temp    = INT16_HL(regdata[6], regdata[7]);
  data->gyro_x  = INT16_HL(regdata[8], regdata[9]);
  data->gyro_y  = INT16_HL(regdata[10], regdata[11]);
  data->gyro_z  = INT16_HL(regdata[12], regdata[13]);

  rv = readRegs(MPU9150_HXL, regdata, 6);
  if (rv < 0)
    return false;

  data->mag_x   = INT16_HL(regdata[1], regdata[0]);
  data->mag_y   = INT16_HL(regdata[3], regdata[2]);
  data->mag_z   = INT16_HL(regdata[5], regdata[4]);

  return true;
}
