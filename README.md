Arduino ICM20948 DMP library
======================
This repository contains a modified version of the ICM20948 library with extra functionality that InvenSense did not complete. It also includes an example project for running the ICM-20948 with Arduino or other Arduino compatible microcontrollers.

## Functionality

* Communication over SPI and i2c
* Original DMP firmware with 9DOF sensor fusion
* Easy to set DMP settings
* Provides Accel, Gyro, Mag as Calib, Uncalib and Raw 
* Provides Linear Accel, Gravity Vector, 6&9DoF Quaternion and Euler, Activity Recognition and Step Counter
* The sensor is implemented as an Object by using the ArduinoICM20948 class (Check example)

## Settings

```
ArduinoICM20948 icm20948;
ArduinoICM20948Settings icmSettings =
{
  .i2c_speed = 400000,                // i2c clock speed
  .is_SPI = false,                    // Enable SPI, if disable use i2c
  .cs_pin = 10,                       // SPI chip select pin
  .spi_speed = 7000000,               // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                          // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,           // Enables gyroscope output
  .enable_accelerometer = true,       // Enables accelerometer output
  .enable_magnetometer = true,        // Enables magnetometer output // Enables quaternion output
  .enable_gravity = true,             // Enables gravity vector output
  .enable_linearAcceleration = true,  // Enables linear acceleration output
  .enable_quaternion6 = true,         // Enables quaternion 6DOF output
  .enable_quaternion9 = true,         // Enables quaternion 9DOF output
  .enable_har = true,                 // Enables activity recognition
  .enable_steps = true,               // Enables step counter
  .gyroscope_frequency = 1,           // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,       // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,        // Max frequency = 70, min frequency = 1 
  .gravity_frequency = 1,             // Max frequency = 225, min frequency = 1
  .linearAcceleration_frequency = 1,  // Max frequency = 225, min frequency = 1
  .quaternion6_frequency = 50,        // Max frequency = 225, min frequency = 50
  .quaternion9_frequency = 50,        // Max frequency = 225, min frequency = 50
  .har_frequency = 50,                // Max frequency = 225, min frequency = 50
  .steps_frequency = 50               // Max frequency = 225, min frequency = 50
  
};

icm20948.init(icmSettings);
icm20948.task();
```
Feel free give feedback and suggest changes.

Contact at jsouriadakis@outlook.com
