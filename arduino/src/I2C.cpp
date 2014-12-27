#include "I2C.hpp"

#include <Arduino.h>
#include <Wire.h>

//
// I2CBus
//
I2CBus::I2CBus()
{
  Wire.begin();
}

void I2CBus::beginTransmission(uint8_t addr)
{
  Wire.beginTransmission(addr);
}

uint8_t I2CBus::endTransmission()
{
  return Wire.endTransmission();
}

uint8_t I2CBus::requestFrom(uint8_t addr, uint8_t nbytes)
{
  return Wire.requestFrom(addr, nbytes);
}

uint8_t I2CBus::write(const uint8_t *buf, uint8_t nbytes)
{
  return Wire.write(buf, nbytes);
}

uint8_t I2CBus::read(uint8_t *buf, uint8_t nbytes)
{
  uint8_t pos;

  for (pos = 0; Wire.available() && pos < nbytes; pos++) {
    buf[pos] = Wire.read();
  }
  return pos;
}

uint8_t I2CBus::checkAddress(uint8_t addr)
{
  beginTransmission(addr);
  return endTransmission();
}

uint8_t I2CBus::scan(void (*foundfn)(uint8_t, void *), void *priv)
{
  uint8_t devices = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    uint8_t rv = checkAddress(addr);
    if (rv == I2C_SUCCESS) {
      devices++;
      if (foundfn != NULL) {
        (*foundfn)(addr, priv);
      }
    }
  }
  return devices;
}


//
// I2CDevice
//
I2CDevice::I2CDevice(I2CBus& bus, uint8_t addr)
  : bus_(bus), addr_(addr)
{
}

int I2CDevice::sendAddr(uint8_t regaddr)
{
  uint8_t rv;

  // Send initial register address
  bus_.beginTransmission(addr_);

  rv = bus_.write(&regaddr, 1);
  if (rv != 1)
    return -1;

  rv = bus_.endTransmission();
  if (rv != 0)
    return (int) -rv;

  return 1;
}

int I2CDevice::readRegs(uint8_t regaddr, uint8_t *data, uint8_t count)
{
  uint8_t rv;

  // Send initial register address
  bus_.beginTransmission(addr_);

  rv = bus_.write(&regaddr, 1);
  if (rv != 1)
    return -1;

  rv = bus_.endTransmission();
  if (rv != 0)
    return (int) -rv;

  // Perform read
  rv = bus_.requestFrom(addr_, count);
  if (rv != count)
    return -1;

  rv = bus_.read(data, count);
  if (rv != count)
    return -1;

  return (int) rv;
}

int I2CDevice::readReg(uint8_t regaddr, uint8_t *value)
{
  return readRegs(regaddr, value, 1);
}

int I2CDevice::writeRegs(uint8_t regaddr, const uint8_t *data, uint8_t count)
{
  uint8_t rv;

  bus_.beginTransmission(addr_);

  rv = bus_.write(&regaddr, 1);
  if (rv != 1)
    return -1;

  rv = bus_.write(data, count);
  if (rv != count)
    return -1;

  rv = bus_.endTransmission();
  if (rv != 0)
    return (int) -rv;

  return (int) count;
}

int I2CDevice::writeReg(uint8_t regaddr, uint8_t value)
{
  return writeRegs(regaddr, &value, 1);
}

int I2CDevice::updateReg(uint8_t regaddr, uint8_t value, uint8_t mask)
{
  uint8_t reg;
  int rv;

  rv = readReg(regaddr, &reg);
  if (rv < 0)
    return rv;

  reg = (reg & ~mask) | (value & mask);
  return writeReg(regaddr, reg);
}


//
// I2CManager
//
I2CManager::I2CManager()
{
}

I2CBus& I2CManager::getBus(uint8_t bus_id)
{
  return bus_;
}

struct ScanInfo
{
  uint8_t bus_id;
  void (*foundfn)(I2CDeviceInfo&, void*);
  void *priv;
};

static void found_bus_device(uint8_t addr, void *priv)
{
  struct ScanInfo *scan_info = (ScanInfo *) priv;

  I2CDeviceInfo dev_info;
  dev_info.bus_id = scan_info->bus_id;
  dev_info.addr = addr;

  if (scan_info->foundfn != NULL) {
    (*scan_info->foundfn)(dev_info, scan_info->priv);
  }
}

uint8_t I2CManager::scanDevices(void (*foundfn)(I2CDeviceInfo&, void*), void *priv)
{
  ScanInfo scan_info;
  scan_info.bus_id = 0;
  scan_info.foundfn = foundfn;
  scan_info.priv = priv;

  return bus_.scan(found_bus_device, &scan_info);
}


I2CManager i2c;
