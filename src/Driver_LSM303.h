#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/**
 * Default I2C Address & IDs
 */
#define ADDRESS_ACC 0b0011001 //page 39, I2C operation, default adress
#define ADDRESS_MAG 0b0011110
#define CHIP_ID_ACC 0x33      //page 43, register mapping, default value
#define CHIP_ID_MAG 0x40 

/**
 * Constants
 */
#define MAG_LSB 1.5 //!< Sensitivity
#define MG_TO_MUT 0.1 //!< Conversion rate of Milligauss to Microtesla


/*
 * @brief Register addresses
 */
typedef enum {                               // DEFAULT    TYPE
    LSM303_REG_ACC_WHO_AM_I = 0x0F,     // 00000111   rw
    LSM303_REG_ACC_CTRL_REG1_A = 0x20,  // 00000111   rw
    LSM303_REG_ACC_CTRL_REG2_A = 0x21,  // 00000000   rw
    LSM303_REG_ACC_CTRL_REG3_A = 0x22,  // 00000000   rw
    LSM303_REG_ACC_CTRL_REG4_A = 0x23,  // 00000000   rw
    LSM303_REG_ACC_CTRL_REG5_A = 0x24,  // 00000000   rw
    LSM303_REG_ACC_CTRL_REG6_A = 0x25,  // 00000000   rw
    LSM303_REG_ACC_REFERENCE_A = 0x26,  // 00000000   r
    LSM303_REG_ACC_STATUS_REG_A = 0x27, // 00000000   r
    LSM303_REG_ACC_OUT_X_L_A = 0x28,
    LSM303_REG_ACC_OUT_X_H_A = 0x29,
    LSM303_REG_ACC_OUT_Y_L_A = 0x2A,
    LSM303_REG_ACC_OUT_Y_H_A = 0x2B,
    LSM303_REG_ACC_OUT_Z_L_A = 0x2C,
    LSM303_REG_ACC_OUT_Z_H_A = 0x2D,
    LSM303_REG_ACC_FIFO_CTRL_REG_A = 0x2E,
    LSM303_REG_ACC_FIFO_SRC_REG_A = 0x2F,
    LSM303_REG_ACC_INT1_CFG_A = 0x30,
    LSM303_REG_ACC_INT1_SOURCE_A = 0x31,
    LSM303_REG_ACC_INT1_THS_A = 0x32,
    LSM303_REG_ACC_INT1_DURATION_A = 0x33,
    LSM303_REG_ACC_INT2_CFG_A = 0x34,
    LSM303_REG_ACC_INT2_SOURCE_A = 0x35,
    LSM303_REG_ACC_INT2_THS_A = 0x36,
    LSM303_REG_ACC_INT2_DURATION_A = 0x37,
    LSM303_REG_ACC_CLICK_CFG_A = 0x38,
    LSM303_REG_ACC_CLICK_SRC_A = 0x39,
    LSM303_REG_ACC_CLICK_THS_A = 0x3A,
    LSM303_REG_ACC_TIME_LIMIT_A = 0x3B,
    LSM303_REG_ACC_TIME_LATENCY_A = 0x3C,
    LSM303_REG_ACC_TIME_WINDOW_A = 0x3D,
    
    LSM303_REG_MAG_OFFSET_X_REG_L = 0x45,
    LSM303_REG_MAG_OFFSET_X_REG_H = 0x46,
    LSM303_REG_MAG_OFFSET_Y_REG_L = 0x47,
    LSM303_REG_MAG_OFFSET_Y_REG_H = 0x48,
    LSM303_REG_MAG_OFFSET_Z_REG_L = 0x49,
    LSM303_REG_MAG_OFFSET_Z_REG_H = 0x4A,
    LSM303_REG_MAG_WHO_AM_I = 0x4F,
    LSM303_REG_MAG_CFG_REG_A = 0x60,
    LSM303_REG_MAG_CFG_REG_B = 0x61,
    LSM303_REG_MAG_CFG_REG_C = 0x62,
    LSM303_REG_MAG_INT_CRTL_REG = 0x63,
    LSM303_REG_MAG_INT_SOURCE_REG = 0x64,
    LSM303_REG_MAG_INT_THS_L_REG = 0x65,
    LSM303_REG_MAG_STATUS_REG = 0x67,
    LSM303_REG_MAG_OUTX_L_REG = 0x68,
    LSM303_REG_MAG_OUTX_H_REG = 0x69,
    LSM303_REG_MAG_OUTY_L_REG = 0x6A,
    LSM303_REG_MAG_OUTY_H_REG = 0x6B,
    LSM303_REG_MAG_OUTZ_L_REG = 0x6C,
    LSM303_REG_MAG_OUTZ_H_REG = 0x6D,
} lsm303Registers_t;


/**
 * @brief: Data type for raw values
 */
typedef struct Data_struct{
  int16_t x; ///< x-axis data
  int16_t y; ///< y-axis data
  int16_t z; ///< z-axis data
} RawData;


/*!
 * @brief Set of linear acceleration measurement ranges
 */
typedef enum range {
  LSM303_RANGE_2G,  ///< Measurement range from +2G to -2G (19.61 m/s^2)
  LSM303_RANGE_4G,  ///< Measurement range from +4G to -4G (39.22 m/s^2)
  LSM303_RANGE_8G,  ///< Measurement range from +8G to -8G (78.45 m/s^2)
  LSM303_RANGE_16G, ///< Measurement range from +16G to -16G (156.9 m/s^2)
} lsm303_accel_range_t;


/*!
 * @brief Set of different modes that can be used. Normal, high resolution, and
 * low power
 */
typedef enum mode {
  LSM303_MODE_NORMAL,          ///< Normal measurement mode; 10-bit
  LSM303_MODE_HIGH_RESOLUTION, ///< High resolution mode; 12-bit
  LSM303_MODE_LOW_POWER,       ///< Low power mode; 8-bit
} lsm303_mode_t;




class MyLSM303 : public Adafruit_Sensor {
    public:
      RawData mag_raw;
      RawData acc_raw;

      MyLSM303(int32_t sensorID = -1);

      bool begin(uint8_t i2c_addr, uint32_t frq=100000, TwoWire *wire = &Wire); //ADDRESS_ACC or ADDRESS_MAG
      void reset(uint8_t i2c_addr);
      void read(uint8_t i2c_addr);
      bool checkStatus(uint8_t i2c_address);

      bool getEvent(sensors_event_t *event);//Dummy Event
      bool getEvent(uint8_t i2_addr, sensors_event_t *);
      void getSensor(sensor_t *);

      //ACC specific methods
      void setAccRange(lsm303_accel_range_t new_range);
      lsm303_accel_range_t getAccRange(void);
      void setAccMode(lsm303_mode_t new_mode);
      lsm303_mode_t getAccMode(void);

      //MAG specific methods
      void setMagMode(lsm303_mode_t new_mode);
      lsm303_mode_t getMagMode(void);

    private:
      int32_t _sensorID;
      
      Adafruit_BusIO_Register *config_reg1_acc, //ODR, PWR, Enable Axis
                              *config_regA_mag; //TempComp, Reboot, Reset, PWR, ODR, Mode
      Adafruit_I2CDevice  *i2c_dev_mag = NULL, 
                          *i2c_dev_acc = NULL;
      Adafruit_SPIDevice  *spi_dev = NULL;

      bool _init(uint8_t i2c_address);

      //ACC specific methods
      uint8_t getAccShift(void);
      float getAccLSB(void);
      lsm303_accel_range_t _accrange;
      lsm303_mode_t _accmode;
      lsm303_mode_t _magmode;
};