#include <Arduino.h>
const byte IO_Battery     = 1;  // Define the IO port for Battery detection
const byte IO_Long_Button = 2;  // Define the IO port for Long press button
const byte IO_Button0     = 6;  // Define the IO port for Button 0
const byte IO_Button1     = 7;  // Define the IO port for Button 1
const byte IO_Button2     = 0;  // Define the IO port for Button 2
const byte IO_Button3     = 40; // Define the IO port for Button 3
const byte IO_Button4     = 42; // Define the IO port for Button 4
const byte IO_EN          = 18; // Define a generic IO port 'EN'
const byte IO_LIGHT_RED   = 20;
const byte IO_LIGHT_GREEN = 19; 
#ifdef HARDWARE_2_0
const byte IO_Button_LED  =  21; // Define the IO port for Button LED
#else
const byte IO_Button_LED  =  16; // Define the IO port for Button LED
#endif
const byte IO_11V         = 37;  // Define the IO port related to 11V
const byte IO_IMU_RX      = 38;  // Define the IO port for IMU Receivers
const byte IO_IMU_TX      = 39;  // Define the IO port for IMU Transmitter

enum METER_TYPE_DEFINE {
    TYPE_0_5 = 1,
    TYPE_1_0 = 11,
    TYPE_2_0 = 12
};

enum PAGE_DEFINE {
    PAGE_HOME,
    PAGE_BLE,
    PAGE_LIGHT_SWITCH,
    PAGE_ZERO_MENU,
    PAGE_ZERO_HOME,
    PAGE_ZERO_ANGLE,
    PAGE_ZERO_FLAT,
    PAGE_ZERO_RESET,
    PAGE_INFO,
    PAGE_CALI_FLAT,
    PAGE_CALI_ANGLE,
    PAGE_IMU_FACTORY_ZERO,
    PAGE_FLAT_FACTORY_ZERO
};

enum HOME_MODE{
  HOME_ANGLE = 0,
  HOME_SLOPE,
  HOME_FLATNESS,
  HOME_AUTO,
  HOME_AUTO_SLOPE,
  HOME_AUTO_FLATNESS
};

// state
enum MEASURE_FSM_DEFINE {
  M_IDLE,
  M_UNSTABLE,
  M_MEASURE_ING,
  M_MEASURE_DONE,
  M_UPLOAD_DONE
};

enum IMU_FSM_DEFINE
{
    IMU_COMMON,
    IMU_CALI_ZERO,
    IMU_COMPLETE,
    IMU_FACTORY_ZERO,
};

enum FLAT_FSM_DEFINE {
    FLAT_COMMON,
    FLAT_CALI_ZERO,
    FLAT_CALI_COMPLETE,
    FLAT_FACTORY_ZERO,
    FLAT_LCD_CALI,
    FLAT_APP_CALI,
    FLAT_ROBOT_ARM_CALI,
};

enum WARN_MODE_DEFINE {
    WARN_OFF,
    WARN_ON_LIGHT_OFF,
    WARN_ON_LIGHT_ON
};

enum SPEED_MODE_DEFINE {
    SPEED_MODE_STANDARD,
    SPEED_MODE_QUICK,
    SPEED_MODE_AUTO
};

enum CALI_STEP{
  SAVE  = 11,
  ECHO  = 12,
  RESET = 13,
};

struct Measure{
  byte state = 0;
  byte progress = 0;
};

struct Calibration{
  byte step = 0;
  byte status = 0;
  byte rr = 0;
  float peak_avg = 0;
};

struct ClinoMeter{
  float angle_live = 0.0f;
  float angle_hold = 0.0f;
  float slope_live = 0.0f;
  float slope_hold = 0.0f;
  byte  arrow_live = 0;
  byte  arrow_hold = 0;
  Measure measure;
};

struct FlatnessMeter{
  float flat_live = 0.0f;
  float flat_hold = 0.0f;
  byte  arrow_live = 0;
  byte  arrow_hold = 0;
  bool  adc_online[4] = {0};
  byte  online_block = 0;
  byte  ready[8] = {0};
  byte state = 0;
  byte progress = 0;
  Measure measure;
  Calibration cali;
};

enum BLE_COM_DEFINE{

  BATTERY_BASE = 0,//电量，小于1000的值就是电量
  METER_TYPE_BASE = 2,//靠尺类型
  METER_TYPE_500 = 2001,
  METER_TYPE_600 = 2002,
  METER_TYPE_1000 = 2011,
  METER_TYPE_2000 = 2012,

  HOME_MODE_BASE = 3,
  HOME_MODE_ANGLE = 3000,//测量模式
  HOME_MODE_SLOPE = 3001,
  HOME_MODE_FLAT  = 3002,
  HOME_MODE_FLAT_SLOPE = 3003,

  SLOPE_STD_BASE = 4,
  SLOPE_STD_1000 = 4000,//坡度标准
  SLOPE_STD_1200 = 4001,
  SLOPE_STD_2000 = 4002,

  ANGLE_SEPPD_BASE  = 5,
  ANGLE_SEPPD_STD   = 5000,// 标准
  ANGLE_SEPPD_QUICK = 5001,// 快速
  ANGLE_SEPPD_AUTO  = 5002,// 自动:极快免按

  WARN_BASE = 6,
  WARN_DISABLE = 6000,//无预警
  WARN_ENABLE_NO_LIGHT = 6001,//有预警无灯
  WARN_ENABLE_LIGHT = 6002,// 有预警有灯

  CALIBRATION_BASE = 7,

  VERSION_SOFTWARE_BASE = 8,
  VERSION_HARDWARE_BASE = 9,
};

