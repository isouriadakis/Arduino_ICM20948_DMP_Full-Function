#include "Arduino-ICM20948.h"
#include <Wire.h>


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

const uint8_t number_i2c_addr = 2;
uint8_t poss_addresses[number_i2c_addr] = {0X69, 0X68};
uint8_t ICM_address;
bool ICM_found = false;


void i2c_scan(){
    uint8_t error;
    for(uint8_t add_int = 0; add_int < number_i2c_addr; add_int++ ){
        Serial.printf("Scanning 0x%02X for slave...", poss_addresses[add_int]);
        Wire.beginTransmission(poss_addresses[add_int]);
        error = Wire.endTransmission();
        if (error == 0){
            Serial.println("found.");
            if(poss_addresses[add_int] == 0x69 || poss_addresses[add_int] == 0x68){
                Serial.println("\t- address is ICM.");
                ICM_address = poss_addresses[add_int];
                ICM_found = true;
            }
        }
    }
}
void run_icm20948_quat6_controller(bool inEuler = false)
{
    float quat_w, quat_x, quat_y, quat_z;
    float roll, pitch, yaw;
    char sensor_string_buff[128];
    if (inEuler)
    {
        if (icm20948.euler6DataIsReady())
        {
            icm20948.readEuler6Data(&roll, &pitch, &yaw);
            sprintf(sensor_string_buff, "Euler6 roll, pitch, yaw(deg): [%f,%f,%f]", roll, pitch, yaw);
            Serial.println(sensor_string_buff);
        }
    }
    else
    {
        if (icm20948.quat6DataIsReady())
        {
            icm20948.readQuat6Data(&quat_w, &quat_x, &quat_y, &quat_z);
            sprintf(sensor_string_buff, "Quat6 w, x, y, z (deg): [%f,%f,%f,%f]", quat_w, quat_x, quat_y, quat_z);
            Serial.println(sensor_string_buff);
        }
    }
    
}
void run_icm20948_quat9_controller(bool inEuler = false)
{
    float quat_w, quat_x, quat_y, quat_z;
    float roll, pitch, yaw;
    char sensor_string_buff[128];
    if (inEuler)
    {
        if (icm20948.euler9DataIsReady())
        {
            icm20948.readEuler9Data(&roll, &pitch, &yaw);
            sprintf(sensor_string_buff, "Euler9 roll, pitch, yaw(deg): [%f,%f,%f]", roll, pitch, yaw);
            Serial.println(sensor_string_buff);
        }
    }
    else
    {
        if (icm20948.quat9DataIsReady())
        {
            icm20948.readQuat9Data(&quat_w, &quat_x, &quat_y, &quat_z);
            sprintf(sensor_string_buff, "Quat9 w, x, y, z (deg): [%f,%f,%f,%f]", quat_w, quat_x, quat_y, quat_z);
            Serial.println(sensor_string_buff);
        }
    }
    
}
void run_icm20948_accel_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.accelDataIsReady())
    {
        icm20948.readAccelData(&x, &y, &z);
        sprintf(sensor_string_buff, "Acceleration x, y, z (g): [%f,%f,%f]", x, y, z);
        Serial.println(sensor_string_buff);
    }
    
}
void run_icm20948_gyro_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.gyroDataIsReady())
    {
        icm20948.readGyroData(&x, &y, &z);
        sprintf(sensor_string_buff, "Gyroscope x, y, z (rad/s): [%f,%f,%f]", x, y, z);
        Serial.println(sensor_string_buff);
    }
    
}
void run_icm20948_mag_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.magDataIsReady())
    {
        icm20948.readMagData(&x, &y, &z);
        sprintf(sensor_string_buff, "Magnetometer x, y, z (mT): [%f,%f,%f]", x, y, z);
        Serial.println(sensor_string_buff);
    }
    
}
void run_icm20948_linearAccel_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.linearAccelDataIsReady())
    {
        icm20948.readLinearAccelData(&x, &y, &z);
        sprintf(sensor_string_buff, "Linear Acceleration x, y, z (g): [%f,%f,%f]", x, y, z);
        Serial.println(sensor_string_buff);
    }
    
}
void run_icm20948_grav_controller()
{
    float x, y, z;
    char sensor_string_buff[128];
    if (icm20948.gravDataIsReady())
    {
        icm20948.readGravData(&x, &y, &z);
        sprintf(sensor_string_buff, "Gravity Vector x, y, z (g): [%f,%f,%f]", x, y, z);
        Serial.println(sensor_string_buff);
    }
    
}
void run_icm20948_har_controller()
{
    char activity;
    char sensor_string_buff[128];
    if (icm20948.harDataIsReady())
    {
        icm20948.readHarData(&activity);
        sprintf(sensor_string_buff, "Current Activity : %c", activity);
        Serial.println(sensor_string_buff);
    }
    
}
void run_icm20948_steps_controller()
{
    unsigned long steps;
    char sensor_string_buff[128];
    if (icm20948.stepsDataIsReady())
    {
        icm20948.readStepsData(&steps);
        sprintf(sensor_string_buff, "Steps Completed : %lu", steps);
        Serial.println(sensor_string_buff);
    }
    
}

void setup() 
{
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Starting ICM");
  delay(10);
  i2c_scan();
  if (ICM_found)
  {
      icm20948.init(icmSettings);
  }


}
void loop() 
{
  if (ICM_found)
  {
    icm20948.task();
    //run_icm20948_accel_controller();
    //run_icm20948_gyro_controller();
    //run_icm20948_mag_controller();
    //run_icm20948_linearAccel_controller();
    //run_icm20948_grav_controller();
    //run_icm20948_quat6_controller(true);
    //run_icm20948_quat9_controller(true);
    run_icm20948_har_controller();
    run_icm20948_steps_controller();
  }
  delay(10);

}
