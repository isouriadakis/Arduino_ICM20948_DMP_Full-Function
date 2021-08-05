/*************************************************************************
  Includes
*************************************************************************/

#include "Arduino-ICM20948.h"
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

// InvenSense drivers and utils
#include "Icm20948.h"
#include "SensorTypes.h"
#include "Icm20948MPUFifoControl.h"

/*************************************************************************
  Variables
*************************************************************************/

int chipSelectPin;
int com_speed;

uint8_t I2C_Address = 0x69;

float gyro[3];
bool gyro_data_ready = false;

float accel[3];
bool accel_data_ready = false;

float mag[3];
bool mag_data_ready = false;

float grav[3];
bool grav_data_ready = false;

float lAccel[3];
bool linearAccel_data_ready = false;

float quat6[4];
bool quat6_data_ready = false;

float euler6[3];
bool euler6_data_ready = false;

float quat9[4];
bool quat9_data_ready = false;

float euler9[3];
bool euler9_data_ready = false;

int har;
bool har_data_ready = false;

unsigned long steps;
bool steps_data_ready = false;

/*************************************************************************
  HAL Functions for Arduino
*************************************************************************/
int spi_master_read_register(uint8_t reg, uint8_t* rbuffer, uint32_t rlen)
{
    //return spi_master_transfer_rx(NULL, reg, rbuffer, rlen);
  
    SPI.beginTransaction(SPISettings(com_speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(0x00);
    SPI.endTransaction();

    digitalWrite(chipSelectPin, LOW);
    SPI.beginTransaction(SPISettings(com_speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(((reg & 0x7F) | 0x80));
    for (uint32_t indi = 0; indi < rlen; indi++)
    {
        *(rbuffer + indi) = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(chipSelectPin, HIGH);

    return 0;
}

int spi_master_write_register(uint8_t reg, const uint8_t* wbuffer, uint32_t wlen)
{
    //return spi_master_transfer_tx(NULL, reg, wbuffer, wlen);

    SPI.beginTransaction(SPISettings(com_speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(0x00);
    SPI.endTransaction();

    digitalWrite(chipSelectPin, LOW);
    SPI.beginTransaction(SPISettings(com_speed, MSBFIRST, SPI_MODE0));
    SPI.transfer((reg & 0x7F) | 0x00);
    for (uint32_t indi = 0; indi < wlen; indi++)
    {
        SPI.transfer(*(wbuffer + indi));
    }
    SPI.endTransaction();
    digitalWrite(chipSelectPin, HIGH);

    return 0;
}

int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data)
{
  if (address != 0x69)
  {

    Serial.print("Odd address:");
    Serial.println(address);
  }
  //Serial.print("write address ");
  //Serial.println(address);
  //Serial.print("register ");
  //Serial.println(reg);
  //Serial.print("length = ");
  //Serial.println(len);
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, len);
  Wire.endTransmission();
  return 0;
}

int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff)
{
  if (address != 0x69)
  {

    Serial.print("Odd read address:");
    Serial.println(address);
  }
  //Serial.print("read address ");
  //Serial.println(address);
  //Serial.print("register ");
  //Serial.println(reg);
  //Serial.print("length = ");
  //Serial.println(len);

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // Send repeated start

  uint32_t offset = 0;
  uint32_t num_received = Wire.requestFrom(address, len);
  //Serial.print("received = ");
  //Serial.println(num_received);
  //Serial.println(buff[0]);
  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      buff[i] = Wire.read();
    }
    return 0;
  }
  else
  {
    return -1;
  }
}


/*************************************************************************
  Invensense Variables
*************************************************************************/

inv_icm20948_t icm_device;
int rc = 0;
static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
#define AK0991x_DEFAULT_I2C_ADDR  0x0C
#define AK0991x_SECONDARY_I2C_ADDR  0x0E  /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA      0x68  /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB     0x69  /* I2C slave address for INV device on Rev B board */

#define AD0_VAL   1     // The value of the last bit of the I2C address.


#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];

static const float cfg_mounting_matrix[9] = {
  1.f, 0, 0,
  0, 1.f, 0,
  0, 0, 1.f
};

int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

/*************************************************************************
  Invensense Functions
*************************************************************************/

void check_rc(int rc, const char * msg_context) 
{
  if (rc < 0) {
    Serial.println("ICM20948 ERROR!");
    while (1);
  }
}

int load_dmp3(void) 
{
  int rc = 0;
  rc = inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));
  return rc;
}

void inv_icm20948_sleep_us(int us) 
{
  delayMicroseconds(us);
}

void inv_icm20948_sleep(int ms) 
{
  delay(ms);
}

uint64_t inv_icm20948_get_time_us(void) 
{
  return micros();
}


void initiliaze_SPI(void)
{
    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, HIGH);
    SPI.begin();

    SPI.beginTransaction(SPISettings(com_speed, MSBFIRST, SPI_MODE0));
    SPI.transfer(0x00);
    SPI.endTransaction();
}
void initiliaze_I2C(void)
{
    Wire.begin();
    Wire.setClock(com_speed);
}

bool is_interface_SPI = false;
void set_comm_interface(ArduinoICM20948Settings settings)
{
    chipSelectPin = settings.cs_pin;
    is_interface_SPI = settings.is_SPI;
    if (is_interface_SPI)
    {
        com_speed = settings.spi_speed;
        initiliaze_SPI();
    }
    else
    {
        com_speed = settings.i2c_speed;
        //initiliaze_I2C();
    }

}
inv_bool_t interface_is_SPI(void)
{
  return is_interface_SPI;
}

//---------------------------------------------------------------------
int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    if (interface_is_SPI())
    {
        return spi_master_read_register(reg, rbuffer, rlen);
    }
    return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
}

//---------------------------------------------------------------------

int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
    if (interface_is_SPI())
    {
        return spi_master_write_register(reg, wbuffer, wlen);
    }
    return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
}

static void icm20948_apply_mounting_matrix(void) 
{
  int ii;

  for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
    inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
  }
}

static void icm20948_set_fsr(void) 
{
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

int icm20948_sensor_setup(void)
{

  int rc;
  uint8_t i, whoami = 0xff;

  inv_icm20948_soft_reset(&icm_device);

  // Get whoami number
  rc = inv_icm20948_get_whoami(&icm_device, &whoami);

  // Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
  for (i = 0; i < sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0]); ++i) {

    if (whoami == EXPECTED_WHOAMI[i]) {
      break;
    }
  }

  if (i == sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0])) {
    Serial.print("Bad WHOAMI value = 0x");
    Serial.println(whoami, HEX);
    return rc;
  }

  // Setup accel and gyro mounting matrix and associated angle for current board
  inv_icm20948_init_matrix(&icm_device);

  // set default power mode
  rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
  if (rc != 0) {
    Serial.println("Icm20948 Initialization failed.");
    return rc;
  }

  // Configure and initialize the ICM20948 for normal use

  // Initialize auxiliary sensors
  inv_icm20948_register_aux_compass( &icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
  rc = inv_icm20948_initialize_auxiliary(&icm_device);
  if (rc == -1) {
    Serial.println("Compass not detected...");
  }

  icm20948_apply_mounting_matrix();

  icm20948_set_fsr();

  // re-initialize base state structure
  inv_icm20948_init_structure(&icm_device);

  return 0;
}

static uint8_t icm20948_get_grv_accuracy(void) 
{
  uint8_t accel_accuracy;
  uint8_t gyro_accuracy;

  accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
  gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
  return (min(accel_accuracy, gyro_accuracy));
}

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
  INV_SENSOR_TYPE_ACCELEROMETER,
  INV_SENSOR_TYPE_GYROSCOPE,
  INV_SENSOR_TYPE_RAW_ACCELEROMETER,
  INV_SENSOR_TYPE_RAW_GYROSCOPE,
  INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
  INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
  INV_SENSOR_TYPE_BAC,
  INV_SENSOR_TYPE_STEP_DETECTOR,
  INV_SENSOR_TYPE_STEP_COUNTER,
  INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
  INV_SENSOR_TYPE_ROTATION_VECTOR,
  INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
  INV_SENSOR_TYPE_MAGNETOMETER,
  INV_SENSOR_TYPE_SMD,
  INV_SENSOR_TYPE_PICK_UP_GESTURE,
  INV_SENSOR_TYPE_TILT_DETECTOR,
  INV_SENSOR_TYPE_GRAVITY,
  INV_SENSOR_TYPE_LINEAR_ACCELERATION,
  INV_SENSOR_TYPE_ORIENTATION,
  INV_SENSOR_TYPE_B2S
};

void build_sensor_event_data(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg) 
{
  float raw_bias_data[6];
  inv_sensor_event_t event;
  (void)context;
  uint8_t sensor_id = convert_to_generic_ids[sensortype];

  memset((void *)&event, 0, sizeof(event));
  event.sensor = sensor_id;
  event.timestamp = timestamp;
  switch (sensor_id) 
  {
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
      memcpy(raw_bias_data, data, sizeof(raw_bias_data));
      memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
      memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
      memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
      break;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
      memcpy(raw_bias_data, data, sizeof(raw_bias_data));
      memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
      memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
      memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
      break;
    case INV_SENSOR_TYPE_GYROSCOPE:
      memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
      memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));

      // WE WANT THIS
      gyro[0] = event.data.gyr.vect[0];
      gyro[1] = event.data.gyr.vect[1];
      gyro[2] = event.data.gyr.vect[2];
      gyro_data_ready = true;
      break;

    case INV_SENSOR_TYPE_GRAVITY:
      memcpy(event.data.grav.vect, data, sizeof(event.data.grav.vect));
      event.data.grav.accuracy_flag = inv_icm20948_get_accel_accuracy();

      grav[0] = event.data.grav.vect[0];
      grav[1] = event.data.grav.vect[1];
      grav[2] = event.data.grav.vect[2];
      grav_data_ready = true;
      //add gravity
      break;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        memcpy(event.data.linAcc.vect, data, sizeof(event.data.linAcc.vect));
        memcpy(&(event.data.linAcc.accuracy_flag), arg, sizeof(event.data.linAcc.accuracy_flag));

        // WE WANT THIS
      lAccel[0] = event.data.linAcc.vect[0];
      lAccel[1] = event.data.linAcc.vect[1];
      lAccel[2] = event.data.linAcc.vect[2];
      linearAccel_data_ready = true;
      break;
        //add linear acceleration
    case INV_SENSOR_TYPE_ACCELEROMETER:
      memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
      memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));

      // WE WANT THIS
      accel[0] = event.data.acc.vect[0];
      accel[1] = event.data.acc.vect[1];
      accel[2] = event.data.acc.vect[2];
      accel_data_ready = true;
      break;

    case INV_SENSOR_TYPE_MAGNETOMETER:
      memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
      memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));

      // WE WANT THIS
      mag[0] = event.data.mag.vect[0];
      mag[1] = event.data.mag.vect[1];
      mag[2] = event.data.mag.vect[2];
      mag_data_ready = true;
      break;

    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
      memcpy(&(event.data.quaternion9DOF.accuracy), arg, sizeof(event.data.quaternion9DOF.accuracy));
      memcpy(event.data.quaternion9DOF.quat, data, sizeof(event.data.quaternion9DOF.quat));
      quat9[0] = event.data.quaternion9DOF.quat[0];
      quat9[1] = event.data.quaternion9DOF.quat[1];
      quat9[2] = event.data.quaternion9DOF.quat[2];
      quat9[3] = event.data.quaternion9DOF.quat[3];
      quat9_data_ready = true;
      euler9_data_ready = true;
      break;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
      memcpy(event.data.quaternion6DOF.quat, data, sizeof(event.data.quaternion6DOF.quat));
      event.data.quaternion6DOF.accuracy_flag = icm20948_get_grv_accuracy();
      
      // WE WANT THIS
      quat6[0] = event.data.quaternion6DOF.quat[0];
      quat6[1] = event.data.quaternion6DOF.quat[1];
      quat6[2] = event.data.quaternion6DOF.quat[2];
      quat6[3] = event.data.quaternion6DOF.quat[3];
      quat6_data_ready = true;
      euler6_data_ready = true;
      break;

    case INV_SENSOR_TYPE_BAC:
      memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));

      har = event.data.bac.event;
      har_data_ready = true;
      break;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
    case INV_SENSOR_TYPE_TILT_DETECTOR:
    case INV_SENSOR_TYPE_STEP_DETECTOR:
    case INV_SENSOR_TYPE_SMD:
      event.data.event = true;
      break;
    case INV_SENSOR_TYPE_B2S:
      event.data.event = true;
      memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
      break;
    case INV_SENSOR_TYPE_STEP_COUNTER:
      memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
      
      steps = event.data.step.count;
      steps_data_ready = true;
      break;
    case INV_SENSOR_TYPE_ORIENTATION:
      //we just want to copy x,y,z from orientation data
      memcpy(&(event.data.orientation), data, 3 * sizeof(float));
      break;
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
      memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
      break;
    default:
      return;
  }
}

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
  switch (sensor)
  {
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
      return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
      return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
    case INV_SENSOR_TYPE_ACCELEROMETER:
      return INV_ICM20948_SENSOR_ACCELEROMETER;
    case INV_SENSOR_TYPE_GYROSCOPE:
      return INV_ICM20948_SENSOR_GYROSCOPE;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
      return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
      return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
    case INV_SENSOR_TYPE_BAC:
      return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
    case INV_SENSOR_TYPE_STEP_DETECTOR:
      return INV_ICM20948_SENSOR_STEP_DETECTOR;
    case INV_SENSOR_TYPE_STEP_COUNTER:
      return INV_ICM20948_SENSOR_STEP_COUNTER;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
      return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
      return INV_ICM20948_SENSOR_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
      return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_MAGNETOMETER:
      return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
    case INV_SENSOR_TYPE_SMD:
      return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
      return INV_ICM20948_SENSOR_FLIP_PICKUP;
    case INV_SENSOR_TYPE_TILT_DETECTOR:
      return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
    case INV_SENSOR_TYPE_GRAVITY:
      return INV_ICM20948_SENSOR_GRAVITY;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
      return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
    case INV_SENSOR_TYPE_ORIENTATION:
      return INV_ICM20948_SENSOR_ORIENTATION;
    case INV_SENSOR_TYPE_B2S:
      return INV_ICM20948_SENSOR_B2S;
    default:
      return INV_ICM20948_SENSOR_MAX;
  }
}

/*************************************************************************
  Class Functions
*************************************************************************/

ArduinoICM20948::ArduinoICM20948()
{
}

void ArduinoICM20948::init(ArduinoICM20948Settings settings)
{
  set_comm_interface(settings);
  Serial.println("Initializing ICM-20948...");

  // Initialize icm20948 serif structure
  struct inv_icm20948_serif icm20948_serif;
  icm20948_serif.context   = 0; // no need
  icm20948_serif.read_reg  = idd_io_hal_read_reg;
  icm20948_serif.write_reg = idd_io_hal_write_reg;
  icm20948_serif.max_read  = 1024 * 16; // maximum number of bytes allowed per serial read
  icm20948_serif.max_write = 1024 * 16; // maximum number of bytes allowed per serial write
  icm20948_serif.is_spi = interface_is_SPI();

  // Reset icm20948 driver states
  inv_icm20948_reset_states(&icm_device, &icm20948_serif);
  inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

  // Setup the icm20948 device
  rc = icm20948_sensor_setup();

  if (icm_device.selftest_done && !icm_device.offset_done)
  {
    // If we've run self test and not already set the offset.
    inv_icm20948_set_offset(&icm_device, unscaled_bias);
    icm_device.offset_done = 1;
  }

  // Now that Icm20948 device is initialized, we can proceed with DMP image loading
  // This step is mandatory as DMP image is not stored in non volatile memory
  rc += load_dmp3();
  check_rc(rc, "Error sensor_setup/DMP loading.");

  // Set mode
  inv_icm20948_set_lowpower_or_highperformance(&icm_device, settings.mode);

  // Set frequency
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1000 / settings.gyroscope_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1000 / settings.accelerometer_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), 1000 / settings.magnetometer_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1000 / settings.quaternion6_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR), 1000 / settings.quaternion9_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), 1000 / settings.gravity_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_LINEAR_ACCELERATION), 1000 / settings.linearAcceleration_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_BAC), 1000 / settings.har_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_STEP_COUNTER), 1000 / settings.steps_frequency);
  


  // Enable / disable
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), settings.enable_gyroscope);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), settings.enable_accelerometer);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), settings.enable_magnetometer);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), settings.enable_quaternion6);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR), settings.enable_quaternion9);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), settings.enable_gravity);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_LINEAR_ACCELERATION), settings.enable_linearAcceleration);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_BAC), settings.enable_har);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_STEP_COUNTER), settings.enable_steps);
}



void ArduinoICM20948::task()
{
  inv_icm20948_poll_sensor(&icm_device, (void*)0, build_sensor_event_data);
}

bool ArduinoICM20948::gyroDataIsReady()
{
  return gyro_data_ready;
}

bool ArduinoICM20948::accelDataIsReady()
{
  return accel_data_ready;
}


bool ArduinoICM20948::magDataIsReady()
{
  return mag_data_ready;
}

bool ArduinoICM20948::linearAccelDataIsReady()
{
  return linearAccel_data_ready;
}

bool ArduinoICM20948::gravDataIsReady()
{
  return grav_data_ready;
}

bool ArduinoICM20948::quat6DataIsReady()
{
  return quat6_data_ready;
}

bool ArduinoICM20948::euler6DataIsReady()
{
  return euler6_data_ready;
}

bool ArduinoICM20948::quat9DataIsReady()
{
    return quat9_data_ready;
}

bool ArduinoICM20948::euler9DataIsReady()
{
    return euler9_data_ready;
}

bool ArduinoICM20948::harDataIsReady()
{
    return har_data_ready;
}

bool ArduinoICM20948::stepsDataIsReady()
{
    return steps_data_ready;
}

void ArduinoICM20948::readGyroData(float *x, float *y, float *z)
{
  *x = gyro[0];
  *y = gyro[1];
  *z = gyro[2];
  gyro_data_ready = false;
}

void ArduinoICM20948::readAccelData(float *x, float *y, float *z)
{
  *x = accel[0];
  *y = accel[1];
  *z = accel[2];
  accel_data_ready = false;
}

void ArduinoICM20948::readMagData(float *x, float *y, float *z)
{
  *x = mag[0];
  *y = mag[1];
  *z = mag[2];
  mag_data_ready = false;
}

void ArduinoICM20948::readLinearAccelData(float* x, float* y, float* z)
{
    *x = lAccel[0];
    *y = lAccel[1];
    *z = lAccel[2];
    linearAccel_data_ready = false;
}

void ArduinoICM20948::readGravData(float* x, float* y, float* z)
{
    *x = grav[0];
    *y = grav[1];
    *z = grav[2];
    grav_data_ready = false;
}

void ArduinoICM20948::readQuat6Data(float *w, float *x, float *y, float *z)
{
  *w = quat6[0];
  *x = quat6[1];
  *y = quat6[2];
  *z = quat6[3];
  quat6_data_ready = false;
}

void ArduinoICM20948::readEuler6Data(float *roll, float *pitch, float *yaw)
{
    *roll = (atan2f(quat6[0]*quat6[1] + quat6[2]*quat6[3], 0.5f - quat6[1]*quat6[1] - quat6[2]*quat6[2]))* 57.29578f;
    *pitch = (asinf(-2.0f * (quat6[1]*quat6[3] - quat6[0]*quat6[2])))* 57.29578f;
	*yaw = (atan2f(quat6[1]*quat6[2] + quat6[0]*quat6[3], 0.5f - quat6[2]*quat6[2] - quat6[3]*quat6[3]))* 57.29578f + 180.0f;
    euler6_data_ready = false;
}

void ArduinoICM20948::readQuat9Data(float* w, float* x, float* y, float* z)
{
    *w = quat9[0];
    *x = quat9[1];
    *y = quat9[2];
    *z = quat9[3];
    quat9_data_ready = false;
}

void ArduinoICM20948::readEuler9Data(float* roll, float* pitch, float* yaw)
{
    *roll = (atan2f(quat9[0] * quat9[1] + quat9[2] * quat9[3], 0.5f - quat9[1] * quat9[1] - quat9[2] * quat9[2])) * 57.29578f;
    *pitch = (asinf(-2.0f * (quat9[1] * quat9[3] - quat9[0] * quat9[2]))) * 57.29578f;
    *yaw = (atan2f(quat9[1] * quat9[2] + quat9[0] * quat9[3], 0.5f - quat9[2] * quat9[2] - quat9[3] * quat9[3])) * 57.29578f + 180.0f;
    euler9_data_ready = false;
}

void ArduinoICM20948::readHarData(char* activity)
{
    
    char temp = 'n';
    switch (har)
    {
        case 1:
            temp = 'd';
            break;
        case 2:
            temp = 'w';
            break;
        case 3:
            temp = 'r';
            break;
        case 4:
            temp = 'b';
            break;
        case 5:
            temp = 't';
            break;
        case 6:
            temp = 's';
            break;
    }
    *activity = temp;
    har_data_ready = false;
}

void ArduinoICM20948::readStepsData(unsigned long* step_count)
{
    *step_count = steps;
    steps_data_ready = false;
}