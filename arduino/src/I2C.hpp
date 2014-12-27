#ifndef _I2C_HPP
#define _I2C_HPP

#include <Arduino.h>
#include <Wire.h>

#define I2C_SUCCESS      0
#define I2C_ERR_TOOLONG  1
#define I2C_ERR_ADDRNACK 2
#define I2C_ERR_DATANACK 3
#define I2C_ERR_OTHER    4

struct I2CDeviceInfo
{
  uint8_t bus_id;
  uint8_t addr;
};

class I2CBus
{
public:
  I2CBus();
  void beginTransmission(uint8_t addr);
  uint8_t endTransmission(void);
  uint8_t requestFrom(uint8_t addr, uint8_t nbytes);
  uint8_t write(const uint8_t *buf, uint8_t nbytes);
  uint8_t read(uint8_t *buf, uint8_t nbytes);
  uint8_t checkAddress(uint8_t addr);
  uint8_t scan(void (*foundfn)(uint8_t, void *), void *priv);
};

class I2CDevice
{
 private:
    I2CBus& bus_;
    uint8_t addr_;

 public:
    I2CDevice(I2CBus& bus, uint8_t addr);

    int sendAddr(uint8_t regaddr);

    int readRegs(uint8_t regaddr, uint8_t *data, uint8_t count);
    int readReg(uint8_t regaddr, uint8_t *value);
    int writeRegs(uint8_t regaddr, const uint8_t *data, uint8_t count);
    int writeReg(uint8_t regaddr, uint8_t value);
    int updateReg(uint8_t regaddr, uint8_t value, uint8_t mask);
};

class I2CManager
{
private:
  I2CBus bus_;

public:
  I2CManager();
  I2CBus& getBus(uint8_t bus_id);
  uint8_t scanDevices(void (*foundfn)(I2CDeviceInfo&, void*), void *priv);
};

extern I2CManager i2c;

#endif /* _I2C_HPP */
