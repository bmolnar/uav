#include <Arduino.h>

#include "Logger.hpp"
#include "I2C.hpp"
#include "MPU6050.hpp"
#include "MPU9150.hpp"
#include "HMC5883L.hpp"

PrintLogger print_logger(Serial);


class I2CScan
{
public:
  static void foundDevice(I2CDeviceInfo& info, void* priv)
  {
    logger.printf("foundDevice: bus=0x%02x, addr=0x%02x", info.bus_id, info.addr);
  }

  static void run()
  {
    i2c.scanDevices(I2CScan::foundDevice, NULL);
  }
};


class MPU6050Acquire
{
public:
  MPU6050Acquire()
  {
  }

  static void printGyroConfig(MPU6050Reg_GyroConfig& reg)
  {
    logger.printf("GYRO_CONFIG:  XG_ST=%u, YG_ST=%u, ZG_ST=%u, FS_SEL=0x%02x",
                  reg.xg_st, reg.yg_st, reg.zg_st, reg.fs_sel);
  }
  static void printAccelConfig(MPU6050Reg_AccelConfig& reg)
  {
    logger.printf("ACCEL_CONFIG: XA_ST=%u, YA_ST=%u, ZA_ST=%u, AFS_SEL=0x%02x",
                  reg.xa_st, reg.ya_st, reg.za_st, reg.afs_sel);
  }
  static void printPwrMgmt1(MPU6050Reg_PwrMgmt1& reg)
  {
    logger.printf("PWR_MGMT_1:   DEVICE_RESET=%u, SLEEP=%u, CYCLE=%u, TEMP_DIS=%u, CLKSEL=0x%02x",
                  reg.device_reset, reg.sleep, reg.cycle, reg.temp_dis, reg.clksel);
  }
  static void printPwrMgmt2(MPU6050Reg_PwrMgmt2& reg)
  {
    logger.printf("PWR_MGMT_2:   LP_WAKE_CTRL=0x%02x, STBY_XA=%u, STBY_YA=%u, STBY_ZA=%u, STBY_XG=%u, STBY_YG=%u, STBY_ZG=%u",
                  reg.lp_wake_ctrl, reg.stby_xa, reg.stby_ya, reg.stby_za, reg.stby_xg, reg.stby_yg, reg.stby_zg);
  }

  static void printRegisters(MPU6050& dev)
  {
    MPU6050Reg reg;

    logger.printf("REGISTERS:");
    if (dev.getReg(MPU6050_GYRO_CONFIG, &reg)) {
      printGyroConfig(reg.gyro_config);
    }
    if (dev.getReg(MPU6050_ACCEL_CONFIG, &reg)) {
      printAccelConfig(reg.accel_config);
    }
    if (dev.getReg(MPU6050_PWR_MGMT_1, &reg)) {
      printPwrMgmt1(reg.pwr_mgmt_1);
    }
    if (dev.getReg(MPU6050_PWR_MGMT_2, &reg)) {
      printPwrMgmt2(reg.pwr_mgmt_2);
    }
  }




  static void printSelfTestData(MPU6050SelfTestData& data)
  {
    logger.printf("accel=(%u, %u, %u), gyro=(%u, %u, %u)",
                  data.accel.x, data.accel.y, data.accel.z, data.gyro.x, data.gyro.y, data.gyro.z);
  }

  static void printSelfTestMeasurement(MPU6050SelfTestMeasurement& data)
  {
    logger.printf("accel=(%08d, %08d, %08d), gyro=(%08d, %08d, %08d)",
                  data.accel.x, data.accel.y, data.accel.z, data.gyro.x, data.gyro.y, data.gyro.z);
  }

  static void printSelfTestResult(MPU6050SelfTestResult& data)
  {
    logger.printf("accel=(%.5f, %.5f, %.5f), gyro=(%.5f, %.5f, %.5f)",
                  data.accel.x, data.accel.y, data.accel.z, data.gyro.x, data.gyro.y, data.gyro.z);
  }

  static void printData(MPU6050Data& data)
  {
    logger.printf("accel=(%08d, %08d, %08d), gyro=(%08d, %08d, %08d), temp=%d",
                  data.accel.x, data.accel.y, data.accel.z,
                  data.gyro.x, data.gyro.y, data.gyro.z, data.temp);
  }

  static void printMeasurement(MPU6050Measurement& meas)
  {
    logger.printf("accel=(%.5f, %.5f, %.5f), gyro=(%.5f, %.5f, %.5f), temp=%.5f",
                  meas.accel.x, meas.accel.y, meas.accel.z, meas.gyro.x, meas.gyro.y, meas.gyro.z, meas.temp);
  }

  static void getFactoryTrim()
  {
    MPU6050 dev(i2c.getBus(0));



    MPU6050SelfTestData st;
    if (!dev.getSelfTestData(&st)) {
      return;
    }
    printSelfTestData(st);

    MPU6050SelfTestMeasurement ft;
    if (!dev.getFactoryTrim(&ft)) {
      return;
    }
    printSelfTestMeasurement(ft);

  }


  static void getSelfTestResponse()
  {


  }


  static void selfTest()
  {
    MPU6050 dev(i2c.getBus(0));

    if (!dev.init()) {
      logger.printf("init() failed");
      return;
    }

    MPU6050SelfTestResult result;
    if (!dev.selfTest(MPU6050_ALL, &result)) {
      logger.printf("selfTest() failed");
      return;
    }
    printSelfTestResult(result);
  }


  static void loopAcquireData(MPU6050& dev)
  {
    MPU6050Data data;

    while (true) {
      if (!dev.getData(&data)) {
        logger.printf("getData() failed");
      } else {
        printData(data);
      }
    }
  }

  static void loopAcquireMeas(MPU6050& dev)
  {
    MPU6050Measurement meas;

    while (true) {
      if (!dev.getMeasurement(&meas)) {
        logger.printf("getMeasurement() failed");
      } else {
        printMeasurement(meas);
      }
    }
  }

  static void getData()
  {
    MPU6050 dev(i2c.getBus(0));

    logger.printf("#1:");
    printRegisters(dev);

    if (!dev.enableReset()) {
      logger.printf("reset() failed");
      return;
    }

    logger.printf("#2:");
    printRegisters(dev);

    if (!dev.disableSleep()) {
      logger.printf("disableSleep() failed");
      return;
    }

    logger.printf("#3:");
    printRegisters(dev);

    MPU6050Params params;
    if (!dev.getParams(&params)) {
      logger.printf("getParams() failed");
      return;
    }
    params.fs_sel = MPU6050_GYRO_RANGE_1000DPS;
    params.afs_sel = MPU6050_ACCEL_RANGE_4G;
    if (!dev.setParams(params)) {
      logger.printf("setParams() failed");
      return;
    }

    logger.printf("#4:");
    printRegisters(dev);

    loopAcquireMeas(dev);
  }
};



class HMC5883LTest
{
public:
  static void printMeasurement(HMC5883LMeasurement& meas)
  {
    logger.printf("mag=(%.5f, %.5f, %.5f)", meas.mag.x, meas.mag.y, meas.mag.z);
  }
  static void printData(HMC5883LData& data)
  {
    logger.printf("mag=(%08d, %08d, %08d)", data.mag.x, data.mag.y, data.mag.z);
  }

  static void printCalibration(HMC5883LCalibration& calib)
  {
    logger.printf("offset=(%.5f, %.5f, %.5f), gain_error=(%.5f, %.5f, %.5f), min=(%.5f, %.5f, %.5f), max=(%.5f, %.5f, %.5f)",
                  calib.offset.x, calib.offset.y, calib.offset.z,
                  calib.gain_error.x, calib.gain_error.y, calib.gain_error.z,
                  calib.min.x, calib.min.y, calib.min.z,
                  calib.max.x, calib.max.y, calib.max.z);
  }



  static bool selfTestAcquireAndCheck(HMC5883L& dev, HMC5883LRange range)
  {
    HMC5883LData data;
    if (!dev.getData(&data))
      return false;

    uint16_t prec = HMC5883LRangePrecision(range);
    int32_t lower = 243 * ((int32_t) prec) / 390;
    int32_t upper = 575 * ((int32_t) prec) / 390;

    logger.printf("lower=%ld, upper=%ld", lower, upper);
    printData(data);

    if ((data.mag.x < lower || upper < data.mag.x) || (data.mag.y < lower || upper < data.mag.y) || (data.mag.z < lower || upper < data.mag.z)) {
      return false;
    } else {
      return true;
    }
  }

  static bool selfTestAtRange(HMC5883L& dev, HMC5883LRange range)
  {
    int attempts = 128;

    HMC5883LParams params = HMC5883LParams_INIT;
    params.smpavg = HMC5883L_SAMPLE_AVG_8;
    params.outprt = HMC5883L_OUTPRT_15_HZ;
    params.msmode = HMC5883L_MSMODE_POSBIAS;
    params.range  = range;
    params.opmode = HMC5883L_OPMODE_CONT;
    if (!dev.setParams(params))
      return false;
    delay(6);

    while (attempts-- > 0) {
      if (selfTestAcquireAndCheck(dev, range)) {
        return true;
      }
      delay(67);
    }
    return false;
  }

  static bool selfTest(HMC5883L& dev)
  {
    bool result = false;

    for (int range = HMC5883L_RANGE_4_7; range < HMC5883L_RANGE_8_1; range++) {
      bool rv = selfTestAtRange(dev, (HMC5883LRange) range);
      if (rv) {
        result = true;
        break;
      }
    }

    HMC5883LParams params = HMC5883LParams_INIT;
    params.smpavg = HMC5883L_SAMPLE_AVG_8;
    params.outprt = HMC5883L_OUTPRT_15_HZ;
    params.msmode = HMC5883L_MSMODE_NORMAL;
    params.range  = HMC5883L_RANGE_4_7;
    params.opmode = HMC5883L_OPMODE_CONT;
    if (!dev.setParams(params))
      return false;

    return result;
  }



  static void getData()
  {
    HMC5883L dev(i2c.getBus(0));

    dev.init();

    HMC5883LParams params;
    if (!dev.getParams(&params)) {
      return;
    }
    params.msmode = HMC5883L_MSMODE_NORMAL;
    params.outprt = HMC5883L_OUTPRT_15_HZ;
    params.smpavg = HMC5883L_SAMPLE_AVG_1;
    params.range  = HMC5883L_RANGE_4_7;
    params.opmode = HMC5883L_OPMODE_CONT;
    if (!dev.setParams(params)) {
      return;
    }


    HMC5883LCalibration calib;
    if (!dev.getCalibration(&calib))
      return;

    int cycles = 1024;
    while (cycles-- > 0) {
      logger.printf("updateCalibration:");
      if (!dev.updateCalibration(&calib))
        return;
      printCalibration(calib);
    }


    HMC5883LMeasurement meas;
    while (true) {
      if (!dev.getMeasurement(calib, &meas)) {
        logger.printf("getMeasurement() failed");
      } else {
        printMeasurement(meas);
      }
    }
  }


};


void setup(void)
{
  Serial.begin(38400);
  logger.setImpl(&print_logger);
}

void loop(void)
{
  //MPU6050Acquire::getData();
  //MPU6050Acquire::selfTest();
  //MPU6050Acquire::getFactoryTrim();

  HMC5883LTest::getData();
}
