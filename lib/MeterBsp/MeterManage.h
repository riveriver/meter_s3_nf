/**
 * @file Meterh
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-05
 * @note 1.解耦角度和平整度测量，为后续多规格尺一份代码的实现打基础，为后续双核数据传递（订阅发布模型）打基础
 * 具体变化：旧框架由管理层先接收传感器数据再进行计算，新框架各模块单独实现采样和计算，只把结果传给管理层
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef Meter_H_
#define Meter_H_
#include <Arduino.h>
#include <Preferences.h>
#include "SLED.h"

// Define various IO port numbers

const byte IO_Battery = 1;      // Define the IO port for Battery detection
const byte IO_Long_Button = 2;  // Define the IO port for Long press button
const byte IO_Button0 = 6;      // Define the IO port for Button 0
const byte IO_Button1 = 7;      // Define the IO port for Button 1
const byte IO_Button2 = 0;      // Define the IO port for Button 2
const byte IO_Button3 = 40;      // Define the IO port for Button 3
const byte IO_Button4 = 42;      // Define the IO port for Button 4
const byte IO_EN = 18;          // Define a generic IO port 'EN'
const byte IO_LIGHT_RED = 20;
const byte IO_LIGHT_GREEN = 19; 
#ifdef HARDWARE_2_0
const byte IO_Button_LED =  21;  // Define the IO port for Button LED
#else
const byte IO_Button_LED =  16;  // Define the IO port for Button LED
#endif
const byte IO_11V = 37;         // Define the IO port related to 11V
const byte IO_IMU_RX = 38;      // Define the IO port for IMU Receivers
const byte IO_IMU_TX = 39;      // Define the IO port for IMU Transmitter

enum PAGE_DEFINE {
    PAGE_HOME,
    PAGE_BLE,
    PAGE_ZERO_MENU,
    PAGE_ZERO_HOME,
    PAGE_ZERO_ANGLE,
    PAGE_ZERO_FLAT,
    PAGE_ZERO_RESET,
    PAGE_LIGHT_SWITCH,
    PAGE_INFO,
    PAGE_IMU_CALI_INFO,
    PAGE_CALI_FLAT,
    PAGE_IMU_FACTORY_ZERO,
    PAGE_FLAT_FACTORY_ZERO
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

enum MEASURE_FSM_DEFINE {
  M_IDLE,
  M_UNSTABLE,
  M_MEASURING,
  M_MEASURE_DONE,
  M_UPLOAD_DONE
};

/*
state:
idle 0x00
request motion 0x01
wait for motion 0x02
collect data 0x03
collect complete 0x04

*/
enum ROBOT_CALI_FSM_DEFINE
{
    ROBOT_CALI_IDLE,
    REQUEST_MOTION,
    WAIT_MOTION,
    COLLECT_DATA,
};


enum SPEED_MODE_DEFINE {
    SPEED_MODE_STANDARD,
    SPEED_MODE_QUICK,
    SPEED_MODE_AUTO
};

enum WARN_MODE_DEFINE {
    WARN_OFF,
    WARN_ON_LIGHT_OFF,
    WARN_ON_LIGHT_ON
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

enum HOME_MODE{
  HOME_ANGLE,
  HOME_SLOPE,
  HOME_FLATNESS,
  HOME_SLOPE_FLATNESS,
  HOME_AUTO_SLOPE,
  HOME_AUTO_FLATNESS
};

enum IMU_FSM_DEFINE
{
    IMU_COMMON,
    IMU_CALI_ZERO,
    IMU_COMPLETE,
    IMU_FACTORY_ZERO,
};

enum CALI_STEP{
  RESET= 11,
  ECHO = 12,
  SAVE = 13,
};

class Meter{
private:
struct Calibration{
  byte step = 0;
  byte status = 0;
  byte rr = 0;
  float peak_avg = 0;
};
struct Measure{
  byte state = 0;
  byte progress = 0;
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
  byte  sub_online = 0;
  byte  online_block = 0;
  byte  ready[8] = {0};
  byte state = 0;
  byte progress = 0;
  Measure measure;
  Calibration cali;
};
Preferences   pref;
public:
  Measure measure;
  ClinoMeter  clino;
  FlatnessMeter flat;

#ifdef HARDWARE_0_0
  int ver_software = 001;
#else
  int ver_software = 105;
#endif

#ifdef HARDWARE_1_0
  int ver_hardware = 100;
#elif defined(HARDWARE_2_0)
  int ver_hardware = 205;
#elif defined(HARDWARE_3_0)
  int ver_hardware = 306;
#else
  int ver_hardware = 000;
#endif

#ifdef TYPE_500
byte meter_type = 1;
#elif defined(TYPE_1000)
byte meter_type = 11;
#elif defined(TYPE_2000)
byte meter_type = 12;
#endif
#ifdef FACTORY_TEST
  byte flat_debug = 0;
#else
  byte flat_debug = 0;
#endif
  int imu_version = 0;

  // MeterUI *p_display;
  SLED led;
  byte cursor = 0;
  byte page = 0;
  byte pre_page = 0;
  byte minor_page = 0;
  byte pre_minor_page = 0;
  byte home_size = 2;
  byte home_mode = 0;
  byte auto_mode_select = 0;
  byte pre_home_mode = 0;
  byte flat_cali_cmd = 0;
  Calibration imu_cali;
  
  int     block_time;
  int     cali_count = 0;
  byte    speed_mode  = SPEED_MODE_QUICK;
  byte warn_light_onoff  = true;
  bool flat_abs = true;
  // byte    warrning_mode = WARN_ON_LIGHT_ON;
  byte  dash_num = 0;
  float flat_th = 50.0f;
  
  byte  adjust_num = 0;
  float warn_angle = 0;
  float warn_flat  = 0;
  float slope_standard = 1000.0f;
  int  sleep_time = 15.0f;
  float auto_angle = 0.0f;
  int  battery;
  bool has_update_dist = false;
  bool has_home_change = false;
  bool has_imu_forward = false;
  bool has_flat_forward = false;
  bool has_angle_forward = false;
  bool if_ble_switch    = false;
  byte reset_state = 0; // 0:normal, 1:reset, 2:reset_done
  bool reset_yesno = false;
  float reset_progress = 0;
  String  cali_forward_str = "";
  String  flat_cali_str = "";
  String  to_app_str = "";
  String  angle_info = "";
  int8_t  flat_height_level = -1; 
  int8_t  robot_cali_height = 0;
  int8_t  robot_cali_state = 0; 

  void initMeter(){

    pref.begin("Meter",false);
    meter_type = pref.getInt("Type",11);
    home_mode  = pref.getInt("Home",HOME_ANGLE);
    speed_mode = pref.getInt("Speed",SPEED_MODE_QUICK);
    // warn_light_onoff = pref.getInt("WarnMode",true);
    warn_angle = pref.getFloat("WarnAngle",0);
    warn_flat = pref.getFloat("WarnFlat",0);
    sleep_time = pref.getInt("Sleep",15);
    pref.end();
    home_size = (meter_type > 10) ? 4 : 2;
    home_mode = home_mode % home_size;

    page = PAGE_CALI_FLAT;
  }

float roundToZeroOrFive(float value,int bits) {
    float decimalPart = value - floor(value);  // 获取小数部分
    int bits_value = static_cast<int>(decimalPart * pow(10, bits) ) % 10;  // 获取小数点第二位数字
    if (bits_value > 3 && bits_value < 8) {
        return floor(value * 10) / 10 + 5 / pow(10, bits);
    } else if (bits_value <= 3) {
        return floor(value * 10) / 10;
    } else if(bits_value >= 8){
        return floor(value * 10) / 10 + 10 / pow(10, bits);
    }
    else{
      ESP_LOGE("","modifyDecimal ERROR!!!");
      return value;
    }
}

float ConvertToSlope(float angle) {
  float standard = slope_standard;
  bool sign = (angle > 0);
  float slope = 0;
  if(fabs(angle) <= 45.0f){
    slope = round(tan(fabs(angle)  * 0.01745f) * standard * 10.0f) / 10.0f;
  }
  else if(fabs(angle)  > 45.0f){
    slope = round(tan((90 - fabs(angle)) * 0.01745f) * standard * 10.0f) / 10.0f;
  }
  return sign ? slope : -slope;
}

  void set_angle_live(float angle,byte arrow){
    if (arrow == 1) {angle = fabs(angle);} 
    else if(arrow == 2){angle = -fabs(angle);}
    else{angle = fabs(angle);}
    if(home_mode == HOME_ANGLE){
      clino.angle_live = roundToZeroOrFive(angle,2);
    }
    else {
      clino.angle_live = angle;
    }
    clino.slope_live = ConvertToSlope(angle);
    clino.arrow_live = arrow;
  }

  void hold_clino(float angle,byte arrow){
    if (arrow == 1) {angle = fabs(angle);} 
    else if(arrow == 2){angle = -fabs(angle);}
    else{angle = fabs(angle);}
    if(home_mode == HOME_ANGLE){
    clino.angle_hold = roundToZeroOrFive(angle,2);
    }else{
      clino.angle_hold = angle;
    }
    clino.slope_hold = ConvertToSlope(angle);
    clino.arrow_hold = arrow;
  }

  void set_angle_progress(int value){
    if(value < 0 || value > 100){
      return;}
    clino.measure.progress = value;
  }

  void start_clino_measure(){
    if(clino.measure.state == M_IDLE 
    || clino.measure.state == M_MEASURE_DONE 
    || clino.measure.state == M_UPLOAD_DONE){
      clino.measure.progress = 0;
      clino.measure.state = M_UNSTABLE;
    }
  }

  void set_flat_live(float value,int arrow){
    if(value >= flat_th){value = 99.9f;}
    if(value <= 0.5)value = 0;
    flat.arrow_live = arrow;
    flat.flat_live  = value;
  }

  void set_flat_progress(int value){
    if(value < 0 || value > 100){return;}
    flat.progress = value;
  }

  void hold_flatness(float value,int arrow){
    flat.arrow_hold = arrow;
    flat.flat_hold  = value;
  }

  void start_flatness_measure(){
    if(flat.measure.state == M_IDLE 
    || flat.measure.state == M_MEASURE_DONE 
    || flat.measure.state == M_UPLOAD_DONE){
      flat.progress = 0;
      flat.measure.state = M_UNSTABLE;
    }
  }

  void updateSystem(){
    if(home_mode == HOME_FLATNESS){
      measure.state    = flat.state;
      measure.progress = flat.measure.progress;
    }
    else if(home_mode == HOME_SLOPE_FLATNESS && auto_mode_select == HOME_AUTO_FLATNESS){
      measure.state    = flat.state;
      measure.progress = flat.measure.progress;
    }
    else{
      measure.state    = clino.measure.state;
      measure.progress = clino.measure.progress;
    }
  }

  void putMeterType(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("Type", meter_type);
    pref.end();
  }

  void putMeterHome(){
    has_home_change = 1;
    clino.measure.state = M_IDLE;
    clino.measure.progress = 0;
    flat.measure.state = M_IDLE;
    flat.measure.progress = 0;
    while(!pref.begin("Meter",false)){
    }
    pref.putInt("Home",home_mode);
    pref.end();
  }

  void putSleepTime(){
    while(!pref.begin("Meter",false)){
    }
    pref.putInt("Sleep",sleep_time);
    pref.end();
  }

  void resetMeasure(){
    measure.state = M_IDLE;
    clino.measure.state = M_IDLE;
    flat.measure.state = M_IDLE;
  }

template<typename T>
String buildFireWaterInfo(const char* prefix, T* array, int length, int decimalPlaces){
  String str = String(prefix) + ":";
  for (int i = 0; i < length; i++) {
    if (i > 0) str += ",";
    str += String(array[i], decimalPlaces);
  }
  return str;
}

void SendToApp(String info){
  to_app_str = info + "\r\n";
}

void put_speed_mode(){
  while(!pref.begin("Meter",false)){}
  pref.putInt("Speed", speed_mode);
  pref.end();
}

void put_warrning_light(){
  while(!pref.begin("Meter",false)){}
  pref.putInt("WarnMode", warn_light_onoff);
  pref.end();
}

void put_warn_angle(){
  while(!pref.begin("Meter",false)){}
  pref.putFloat("WarnAngle", warn_angle);
  pref.end();
}

void put_warn_flat(){
  while(!pref.begin("Meter",false)){}
  pref.putFloat("WarnFlat", warn_flat);
  pref.end();
}

void ControlWarningLight(byte mode){
  static byte last_mode = 0;
  if(mode == last_mode)return;
  switch (mode)
  {
  case 0:// OFF
#ifdef HARDWARE_2_0
    led.Set(0, 0, 0, 4);
    led.Set(0, 0, 0, 4);
#else
    digitalWrite(IO_LIGHT_GREEN,HIGH);
    digitalWrite(IO_LIGHT_RED,HIGH);
#endif
    break;
  case 1:// GREEN
#ifdef HARDWARE_2_0
      led.Set(0, led.G, 1, 4);
      led.Set(1, led.G, 1, 4);
#else
    digitalWrite(IO_LIGHT_GREEN,LOW);
    digitalWrite(IO_LIGHT_RED,HIGH);
#endif
    break; 
  case 2:// RED
#ifdef HARDWARE_2_0
      led.Set(0, led.R, 1, 4);
      led.Set(1, led.R, 1, 4);
#else
    digitalWrite(IO_LIGHT_GREEN,HIGH);
    digitalWrite(IO_LIGHT_RED,LOW);
#endif
    break;
  default:
    ESP_LOGE("mode","%d",mode);
    break;
  }
  last_mode = mode;
}

void WarningLightFSM(){
  if (warn_light_onoff != true) {
    ControlWarningLight(0);
    return;
  }

  if (measure.state == M_MEASURE_DONE || measure.state == M_UPLOAD_DONE) {
    if(home_mode == HOME_ANGLE || home_mode == HOME_SLOPE){
      if (fabs(clino.slope_hold) < ConvertToSlope(warn_angle)) {
        ControlWarningLight(1);
        return;
      }
    }
    else if(home_mode == HOME_FLATNESS){
      if (fabs(flat.flat_hold) < warn_flat) {
        ControlWarningLight(1);
        return;
      }
    }
    else if(home_mode == HOME_SLOPE_FLATNESS){
      if(auto_mode_select == HOME_AUTO_SLOPE){
        if (fabs(clino.slope_hold) < ConvertToSlope(warn_angle)) {
          ControlWarningLight(1);
          return;
        }
      }
      else if(auto_mode_select == HOME_AUTO_FLATNESS){
        if (fabs(flat.flat_hold) < warn_flat) {
          ControlWarningLight(1);
          return;
        }
      }
    }
    ControlWarningLight(2);
  }
  else {
    ControlWarningLight(0);
  }
}
};
#endif 


