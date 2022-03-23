#include "Arduino.h"
#include <Wire.h>
#include <limits.h>
#include "Driver_LSM303.h"



/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/*!
    @brief  Reads the raw data from the sensor
*/
void MyLSM303::read(uint8_t i2c_address) {

Adafruit_I2CDevice **i2c_dev=NULL;
uint16_t out_addr=0;
struct Data_struct *RawData=NULL;
switch(i2c_address){
    case ADDRESS_ACC:
      i2c_dev = &i2c_dev_acc;
      out_addr = 0xA8;    //LSM303_REG_ACC_OUT_X_L_A + 1MSB for autoincrement
      RawData = &acc_raw;
    break;
    case ADDRESS_MAG:
      i2c_dev = &i2c_dev_mag;
      out_addr = LSM303_REG_MAG_OUTX_L_REG;   //not sure, why 1MSB is not necessary here
      RawData = &mag_raw;
    break;
  }

  Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      *i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, out_addr, 6);

  uint16_t buffer[3];
  data_reg.read((uint8_t *)buffer, 6);
  
  (*RawData).x = buffer[0];
  (*RawData).y = buffer[1];
  (*RawData).z = buffer[2];
}



/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/*!
    @brief  Instantiates a new Adafruit_LIS2MDL class
    @param sensorID an option ID to differentiate the sensor from others
*/
MyLSM303::MyLSM303(int32_t sensorID) {
  _sensorID = sensorID;

  mag_raw.x = 0;
  mag_raw.y = 0;
  mag_raw.z = 0;

  acc_raw.x = 0;
  acc_raw.y = 0;
  acc_raw.z = 0;
}



/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  frq
 *            The I2C frequency to be used. Standard = 100000, FastMode = 400000
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool MyLSM303::begin(uint8_t i2c_address, uint32_t frq, TwoWire *wire) {
  
  Adafruit_I2CDevice **i2c_dev=NULL;
  switch(i2c_address){
    case ADDRESS_ACC:
      i2c_dev = &i2c_dev_acc;
    break;
    case ADDRESS_MAG:
      i2c_dev = &i2c_dev_mag;
    break;
  }
  
  if (!*i2c_dev) {
    *i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);
  }
  if (!(*i2c_dev)->begin()) {
    return false;
  }

  if(!(*i2c_dev)->setSpeed(frq)){
    Serial.println(F("Not able to set I2C clk frq"));
  }

  return _init(i2c_address);
}


/*!
 *    @brief  checks connection & ID, resets & enables sensors
 */
bool MyLSM303::_init(uint8_t i2c_address) {
  
  Adafruit_I2CDevice **i2c_dev=NULL;
  Adafruit_BusIO_Register **config_a=NULL;
  uint16_t id_addr=0, cfg_a_addr=0;
  uint32_t sensor_id=0;

  switch(i2c_address){
    case ADDRESS_ACC:
      i2c_dev = &i2c_dev_acc;
      config_a = &config_reg1_acc;
      id_addr = LSM303_REG_ACC_WHO_AM_I;
      cfg_a_addr = LSM303_REG_ACC_CTRL_REG1_A;
      sensor_id = CHIP_ID_ACC;
    break;
    case ADDRESS_MAG:
      i2c_dev = &i2c_dev_mag;
      config_a = &config_regA_mag;
      id_addr = LSM303_REG_MAG_WHO_AM_I;
      cfg_a_addr = LSM303_REG_MAG_CFG_REG_A;
      sensor_id = CHIP_ID_MAG;
    break;
  }

  // Check connection
  Adafruit_BusIO_Register chip_id = Adafruit_BusIO_Register(
      *i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, id_addr, 1);

  // make sure we're talking to the right chip
  if (chip_id.read() != sensor_id) {
    // No sensor detected ... return false
    return false;
  }

  *config_a = new Adafruit_BusIO_Register(*i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, cfg_a_addr, 1);

  // enable int latching -> BDU (?)
  reset(i2c_address);

  return true;
}


/*!
 *    @brief  Resets the sensor to an initial state
 */
void MyLSM303::reset(uint8_t i2c_address) {

  switch(i2c_address){
    case ADDRESS_ACC: {
      //Pass-filter
      Adafruit_BusIO_Register config_reg2 = Adafruit_BusIO_Register(i2c_dev_acc, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_ACC_CTRL_REG2_A, 1);
      config_reg2.write(0x00);
            
      //Interrupts
      Adafruit_BusIO_Register config_reg3 = Adafruit_BusIO_Register(i2c_dev_acc, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_ACC_CTRL_REG3_A, 1);
      config_reg3.write(0x00);
            
      //BDU, BLE, Scale 2-16g, selftest, SPI
      Adafruit_BusIO_Register config_reg4 = Adafruit_BusIO_Register(i2c_dev_acc, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_ACC_CTRL_REG4_A, 1);
      config_reg4.write(0x80);
            
      //Data rate, PWR, Enable Axis
      (*config_reg1_acc).write(0x57);

      _accrange = getAccRange();
      _accmode = getAccMode();
    }break;

    case ADDRESS_MAG:{
    //page 26, Magnetometer Self-test procedure
      //temp compensation, reboot, reset, PWR, ODR, Mode
      (*config_regA_mag).write(0x8C);
      delay(100);
      //Serial.print("REG A:"); Serial.println((*config_regA_mag).read(), HEX);

      //REG B test Read
      //Adafruit_BusIO_Register config_b = Adafruit_BusIO_Register(i2c_dev_mag, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_MAG_CFG_REG_B, 1);
      //Serial.print("REG B:"); Serial.println(config_b.read(), HEX);


      //BDU-BlockDataUpdate, avoiding of reading of incorrect data, when readings occur asynchronously
      Adafruit_BusIO_Register config_c = Adafruit_BusIO_Register(i2c_dev_mag, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_MAG_CFG_REG_C, 1);
      config_c.write(0x10);
      //Serial.print("REG BDU:"); Serial.println(config_c.read(), HEX);
    }break;
  }
}



/*!
    @brief  Gets the most recent sensor event
    @param event The `sensors_event_t` to fill with event data
    @returns true, always
*/
bool MyLSM303::getEvent(uint8_t i2c_address, sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Parse data into sensor_event */
  switch(i2c_address){
    case ADDRESS_ACC: {
      float lsb = getAccLSB();
      uint8_t shift = getAccShift();

      event->version = sizeof(sensors_event_t);
      event->sensor_id = _sensorID;
      event->type = SENSOR_TYPE_ACCELEROMETER;
      event->acceleration.x = (float)(acc_raw.x >> shift) * lsb * SENSORS_GRAVITY_STANDARD;
      event->acceleration.y = (float)(acc_raw.y >> shift) * lsb * SENSORS_GRAVITY_STANDARD;
      event->acceleration.z = (float)(acc_raw.z >> shift) * lsb * SENSORS_GRAVITY_STANDARD;
    }break;

    case ADDRESS_MAG:{
      event->version = sizeof(sensors_event_t);
      event->sensor_id = _sensorID;
      event->type = SENSOR_TYPE_MAGNETIC_FIELD;
      event->magnetic.x = (float)mag_raw.x * MAG_LSB * MG_TO_MUT;
      event->magnetic.y = (float)mag_raw.y * MAG_LSB * MG_TO_MUT;
      event->magnetic.z = (float)mag_raw.z * MAG_LSB * MG_TO_MUT;
    }break;
  }
  return true;
}


/*!
    @brief  DummyEvent due to library restrictions, gets the most recent sensor event
    @param event The `sensors_event_t` to fill with event data
    @returns true, always
*/
bool MyLSM303::getEvent(sensors_event_t *event) {
  return getEvent(ADDRESS_ACC, event);
}


/*!
    @brief  Gets the sensor_t data
*/
void MyLSM303::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "LIS2MDL", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 5000;  // 50 gauss = 5000 uTesla
  sensor->min_value = -5000; // -50 gauss = -5000 uTesla
  sensor->resolution = 0.15; // 1.65 gauss = 0.15 uTesla
}


/**
 * @brief Gets the status of the axis, if new data set is available
 * @param i2c_address address of sensor to check from
 */
bool MyLSM303::checkStatus(uint8_t i2c_address) {
  
  switch(i2c_address){
    case ADDRESS_ACC: {
      Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(i2c_dev_acc, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_ACC_STATUS_REG_A, 1);
      Adafruit_BusIO_RegisterBits xyz = Adafruit_BusIO_RegisterBits(&status_reg, 1, 5);
      return (bool)xyz.read();
    }break;

    case ADDRESS_MAG: {
      Adafruit_BusIO_Register status_reg = Adafruit_BusIO_Register(i2c_dev_mag, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM303_REG_MAG_STATUS_REG, 1);
      Adafruit_BusIO_RegisterBits xyz = Adafruit_BusIO_RegisterBits(&status_reg, 1, 5);
      return (bool)xyz.read();
    }break;
  }
  return false;
}


/***************************************************************************
 Acceleration specific methods
 ***************************************************************************/

/*!
    @brief Sets the accelerometer's range
    @param new_range an `lsm303_accel_range_t` representing the range of
    measurements in +/-G. The smaller the range, the more accurate.
*/
void MyLSM303::setAccRange(lsm303_accel_range_t new_range) {
  Adafruit_BusIO_Register ctrl_4 = Adafruit_BusIO_Register(i2c_dev_acc, LSM303_REG_ACC_CTRL_REG4_A, 1);

  Adafruit_BusIO_RegisterBits range_reg = Adafruit_BusIO_RegisterBits(&ctrl_4, 2, 4);

  range_reg.write(new_range);

  _accrange = getAccRange();
}


/*!
    @brief Gets the accelerometer's range
    @returns The `lsm303_accel_range_t` representing the range of
    measurements in +/-G.
*/
lsm303_accel_range_t MyLSM303::getAccRange(void) {
  Adafruit_BusIO_Register ctrl_4 = Adafruit_BusIO_Register(i2c_dev_acc, LSM303_REG_ACC_CTRL_REG4_A, 1);
  Adafruit_BusIO_RegisterBits range = Adafruit_BusIO_RegisterBits(&ctrl_4, 2, 4);

  return (lsm303_accel_range_t)range.read();
}


/*!
    @brief Get the accelerometer's power mode
    @returns The `lsm303_mode_t` representing the power mode.
*/
lsm303_mode_t MyLSM303::getAccMode(void) {

  Adafruit_BusIO_Register ctrl_1 =
      Adafruit_BusIO_Register(i2c_dev_acc, LSM303_REG_ACC_CTRL_REG1_A, 1);
  Adafruit_BusIO_Register ctrl_4 =
      Adafruit_BusIO_Register(i2c_dev_acc, LSM303_REG_ACC_CTRL_REG4_A, 1);

  Adafruit_BusIO_RegisterBits low_power =
      Adafruit_BusIO_RegisterBits(&ctrl_1, 1, 3);
  Adafruit_BusIO_RegisterBits hi_res =
      Adafruit_BusIO_RegisterBits(&ctrl_4, 1, 3);

  uint8_t low_power_bit = low_power.read();
  uint8_t hi_res_bit = hi_res.read();
  return (lsm303_mode_t)(low_power_bit << 1 | hi_res_bit);
}



/*!
    @brief Sets the accelerometer's power mode
    @param new_mode an `lsm303_mode_t` representing the power mode.
    The mode effects the precision of the sensor's readings
    High resolution is 12-bit
    Normal mode is 10-bit
    Low power is 8-bit
*/
void MyLSM303::setAccMode(lsm303_mode_t new_mode) {

  Adafruit_BusIO_Register ctrl_1 =
      Adafruit_BusIO_Register(i2c_dev_acc, LSM303_REG_ACC_CTRL_REG1_A, 1);

  Adafruit_BusIO_Register ctrl_4 =
      Adafruit_BusIO_Register(i2c_dev_acc, LSM303_REG_ACC_CTRL_REG4_A, 1);

  Adafruit_BusIO_RegisterBits low_power =
      Adafruit_BusIO_RegisterBits(&ctrl_1, 1, 3);

  Adafruit_BusIO_RegisterBits hi_res =
      Adafruit_BusIO_RegisterBits(&ctrl_4, 1, 3);

  hi_res.write(new_mode & 0b01);
  delay(20);
  low_power.write((new_mode & 0b10) >> 1);
  delay(20);

  _accmode = getAccMode();
}



/*!
    @brief  Gets the Least Significant Bit value for the current mode
    @param mode the current mode, used to determind the appropriate lsb value
    in concert with the current range setting.
*/
float MyLSM303::getAccLSB() {
  float lsb=0;
  lsm303_accel_range_t range = _accrange;
  lsm303_mode_t mode = _accmode;
  if (mode == LSM303_MODE_NORMAL) {
    switch (range) {
    case LSM303_RANGE_2G:
      lsb = 0.0039;
      break;
    case LSM303_RANGE_4G:
      lsb = 0.00782;
      break;
    case LSM303_RANGE_8G:
      lsb = 0.01563;
      break;
    case LSM303_RANGE_16G:
      lsb = 0.0469;
      break;
    }
  }

  else if (mode == LSM303_MODE_HIGH_RESOLUTION) {
    switch (range) {
    case LSM303_RANGE_2G:
      lsb = 0.00098;
      break;
    case LSM303_RANGE_4G:
      lsb = 0.00195;
      break;
    case LSM303_RANGE_8G:
      lsb = 0.0039;
      break;
    case LSM303_RANGE_16G:
      lsb = 0.01172;
      break;
    }
  } else if (mode == LSM303_MODE_LOW_POWER) {
    switch (range) {
    case LSM303_RANGE_2G:
      lsb = 0.01563;
      break;
    case LSM303_RANGE_4G:
      lsb = 0.03126;
      break;
    case LSM303_RANGE_8G:
      lsb = 0.06252;
      break;
    case LSM303_RANGE_16G:
      lsb = 0.18758;
      break;
    }
  }

  return lsb;
}




/*!
    @brief  Gets the bit shift amount for the current mode
    @param mode the current mode, used to determind the appropriate shift
    amount based on the bitdepth of the mode
*/
uint8_t MyLSM303::getAccShift() {
  lsm303_mode_t mode = _accmode;
  uint8_t shift=0;
  switch (mode) {
  case LSM303_MODE_HIGH_RESOLUTION:
    shift = 4;
    break;
  case LSM303_MODE_NORMAL:
    shift = 6;
    break;
  case LSM303_MODE_LOW_POWER:
    shift = 8;
    break;
  }

  return shift;
}


/***************************************************************************
 Magnetometer specific methods
 ***************************************************************************/


/*!
    @brief Get the magnetometer's power mode
    @returns The `lsm303_mode_t` representing the power mode.
*/
lsm303_mode_t MyLSM303::getMagMode(void) {
  Adafruit_BusIO_RegisterBits power =
      Adafruit_BusIO_RegisterBits(config_regA_mag, 1, 4);

  uint8_t power_bit = power.read();
  return (lsm303_mode_t)(power_bit+1);
}



/*!
    @brief Sets the magnetometer's power mode
    @param new_mode an `lsm303_mode_t` representing the power mode.
    The mode effects the precision of the sensor's readings
    High resolution is 12-bit
    Low power is 8-bit
*/
void MyLSM303::setMagMode(lsm303_mode_t new_mode) { 

  Adafruit_BusIO_RegisterBits power =
      Adafruit_BusIO_RegisterBits(config_regA_mag, 1, 4);

  power.write((new_mode-1) & 0b01);
  delay(20);

  _magmode = getMagMode();
}