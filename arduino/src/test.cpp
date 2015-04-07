#include <Arduino.h>

#include "Logger.hpp"
#include "I2C.hpp"
#include "MPU6050.hpp"
#include "MPU9150.hpp"
#include "HMC5883L.hpp"

struct UAVData3f
{
  float x;
  float y;
  float z;
};

struct UAVSensorData
{
  UAVData3f gyro;
  UAVData3f accel;
  UAVData3f mag;
};




class Remote
{
public:
  virtual bool sendData(const uint8_t* buf, size_t count)
  {
    return true;
  }
  bool sendUAVSensorData(const UAVSensorData& data)
  {
    return sendData((const uint8_t *) &data, sizeof(data));
  }
};





class PrintRemote : public Remote
{
private:
  Print& print_;

public:
  PrintRemote(Print& print) : print_(print)
  {
  }

  bool sendNibble(const uint8_t b)
  {
    if (b < 10)
      print_.print((char)('0' + b));
    else
      print_.print((char)('A' + (b - 10)));
    return true;
  }

  bool sendByte(const uint8_t b)
  {
    if (!sendNibble((b >> 4) & 0x0F))
      return false;
    if (!sendNibble((b >> 0) & 0x0F))
      return false;
    return true;
  }

  bool newLine()
  {
    print_.println();
    return true;
  }

  bool sendData(const uint8_t* buf, size_t count)
  {
    for (size_t i = 0; i < count; i++) {
      if (!sendByte(buf[i]))
        return false;
    }
    newLine();
    return true;
  }
};








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

    int cycles = 128;
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


class UAV
{
private:
  Remote&             remote_;
  Logger&             logger_;

  MPU6050             mpu6050_;
  HMC5883L            hmc5883l_;
  HMC5883LCalibration hmc5883l_calib_;

public:
  UAV(Remote& remote, Logger& logger)
      : remote_(remote), logger_(logger),
        mpu6050_(i2c.getBus(0)), hmc5883l_(i2c.getBus(0))
  {
  }

  bool getData(UAVSensorData* data)
  {
    MPU6050Measurement mpu6050meas;
    if (!mpu6050_.getMeasurement(&mpu6050meas)) {
      logger_.printf("mpu6050::getMeasurement() failed");
      return false;
    }

    HMC5883LMeasurement hmc5883lmeas;
    if (!hmc5883l_.getMeasurement(hmc5883l_calib_, &hmc5883lmeas)) {
      logger_.printf("hmc5883l::getMeasurement() failed");
      return false;
    }

    data->gyro.x  = mpu6050meas.gyro.x;
    data->gyro.y  = mpu6050meas.gyro.y;
    data->gyro.z  = mpu6050meas.gyro.z;
    data->accel.x = mpu6050meas.accel.x;
    data->accel.y = mpu6050meas.accel.y;
    data->accel.z = mpu6050meas.accel.z;
    data->mag.x   = hmc5883lmeas.mag.x;
    data->mag.y   = hmc5883lmeas.mag.y;
    data->mag.z   = hmc5883lmeas.mag.z;
    return true;
  }

  void logData(const UAVSensorData& data)
  {
    logger_.printf("A=(%.5f, %.5f, %.5f)", data.accel.x, data.accel.y, data.accel.z);
    logger_.printf("G=(%.5f, %.5f, %.5f)", data.gyro.x, data.gyro.y, data.gyro.z);
    logger_.printf("M=(%.5f, %.5f, %.5f)", data.mag.x, data.mag.y, data.mag.z);
  }


  void setup()
  {
    if (!mpu6050_.init()) {
      logger_.printf("mpu6050::init() failed");
    }
    if (!mpu6050_.enableReset()) {
      logger_.printf("reset() failed");
      return;
    }
    if (!mpu6050_.disableSleep()) {
      logger_.printf("disableSleep() failed");
      return;
    }




    if (!hmc5883l_.init()) {
      logger_.printf("hmc5883l::init() failed");
    }

    HMC5883LParams params;
    if (!hmc5883l_.getParams(&params)) {
      return;
    }
    params.msmode = HMC5883L_MSMODE_NORMAL;
    params.outprt = HMC5883L_OUTPRT_15_HZ;
    params.smpavg = HMC5883L_SAMPLE_AVG_1;
    params.range  = HMC5883L_RANGE_4_7;
    params.opmode = HMC5883L_OPMODE_CONT;
    if (!hmc5883l_.setParams(params)) {
      return;
    }

    if (!hmc5883l_.getCalibration(&hmc5883l_calib_))
      return;

    int cycles = 1000;
    while (cycles-- > 0) {
      logger_.printf("updateCalibration:");
      if (!hmc5883l_.updateCalibration(&hmc5883l_calib_)) {
        return;
      }
      delay(5);
    }
  }

  void loop()
  {
    UAVSensorData uav_data;
    if (!getData(&uav_data)) {
      logger_.printf("getData() failed");
      return;
    }

#if 1
    if (!remote_.sendUAVSensorData(uav_data)) {
      logger_.printf("sendUAVSensorData() failed");
      return;
    }
#else
    logData(uav_data);
#endif
  }
};



class UAVTest
{
private:
  PrintRemote remote_;
  PrintLogger logger_;
  UAV uav_;

public:
  UAVTest() : remote_(Serial), logger_(Serial), uav_(remote_, logger_)
  {
  }

  void setup()
  {
    uav_.setup();
  }

  void loop()
  {
    uav_.loop();
  }
};




UAVTest uav_test;

PrintLogger print_logger(Serial);

void setup(void)
{
  Serial.begin(38400);
  setLogger(print_logger);

  uav_test.setup();
}

void loop(void)
{
  uav_test.loop();

  //MPU6050Acquire::getData();
  //MPU6050Acquire::selfTest();
  //MPU6050Acquire::getFactoryTrim();

  //HMC5883LTest::getData();
}
