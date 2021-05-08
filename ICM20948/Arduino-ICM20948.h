#ifndef __Arduino_ICM20948_H__
#define __Arduino_ICM20948_H__

/*************************************************************************
  Defines
*************************************************************************/

typedef struct {
  int i2c_speed;
  bool is_SPI;
  int cs_pin;
  int spi_speed;
  int mode;
  bool enable_gyroscope;
  bool enable_accelerometer;
  bool enable_magnetometer;
  bool enable_gravity;
  bool enable_linearAcceleration;
  bool enable_quaternion6;
  bool enable_quaternion9;
  bool enable_har;
  bool enable_steps;
  int gyroscope_frequency;
  int accelerometer_frequency;
  int magnetometer_frequency;
  int gravity_frequency;
  int linearAcceleration_frequency;
  int quaternion6_frequency;
  int quaternion9_frequency;
  int har_frequency;
  int steps_frequency;

} ArduinoICM20948Settings;

/*************************************************************************
  Class
*************************************************************************/

class ArduinoICM20948
{
  public:

    ArduinoICM20948();

    //void init(TwoWire *theWire = &Wire, JTICM20948Settings settings);
    void init(ArduinoICM20948Settings settings);
    void task();

    bool gyroDataIsReady();
    bool accelDataIsReady();
    bool magDataIsReady();
    bool gravDataIsReady();
    bool linearAccelDataIsReady();
    bool quat6DataIsReady();
    bool euler6DataIsReady();
    bool quat9DataIsReady();
    bool euler9DataIsReady();
    bool harDataIsReady();
    bool stepsDataIsReady();

    void readGyroData(float *x, float *y, float *z);
    void readAccelData(float *x, float *y, float *z);
    void readMagData(float *x, float *y, float *z);
    void readGravData(float* x, float* y, float* z);
    void readLinearAccelData(float* x, float* y, float* z);
    void readQuat6Data(float *w, float *x, float *y, float *z);
    void readEuler6Data(float *roll, float *pitch, float *yaw);
    void readQuat9Data(float* w, float* x, float* y, float* z);
    void readEuler9Data(float* roll, float* pitch, float* yaw);
    void readHarData(char* activity);
    void readStepsData(unsigned long* steps_count);
};


#endif