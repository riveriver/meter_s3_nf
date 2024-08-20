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
#include <Preferences.h>
#include "MeterDefine.h"
#include "SLED.h"

class Meter{
private:
public:
  byte debug_mode = 0;
  /* measure */
  Measure measure;
  /* angel */
  ClinoMeter  clino;
  float auto_angle = 0.0f;
  float slope_standard = 1000.0f;
  byte speed_mode  = SPEED_MODE_QUICK;
  /* flatness */
  FlatnessMeter flat;
  byte  max_sensor_num = 0;
  float max_filt_peak = 0;
  bool flat_abs = true;
  int8_t  flat_height_level = -1;
  byte flat_debug = 0;
  /* warning */
  byte warn_light_onoff  = true;
  float warn_slope = 0;
  float warn_flat  = 0;
  /* led */
  SLED led;
  /* ui */
  String ui_block_info = "";
  byte page = 0;
  byte cursor = 0;
  byte home_size = 2;
  byte home_mode = 0;
  byte pre_home_mode = 0;
  byte auto_mode_select = 0;
  byte reset_state = 0; // 0:normal, 1:reset, 2:reset_done
  int  block_time;
  bool ui_yes_no = false;
  // HACK
  byte ui_progress = 0;
  bool has_home_change = false;
  bool has_ble_switch = false;
  /* system */
#ifdef TYPE_500
byte meter_type = TYPE_0_5;
#elif defined(TYPE_1000)
byte meter_type = TYPE_1_0;
#elif defined(TYPE_2000)
byte meter_type = TYPE_2_0;
#endif // TYPE
  Preferences   pref;
  int battery = 0;
  int  sleep_time = 15; 
  /* msg */
  String  angle_msg = "";
  String  flatness_msg = "";
  String  ack_msg = "";

#ifdef HARDWARE_1_0
  int version_hardware = 100;
  int version_software = 600;
  int version_imu = 0;
#elif defined(HARDWARE_2_0)
  int version_hardware = 205;
  int version_software = 600;
  int version_imu = 0;
#elif defined(HARDWARE_3_0)
  int version_hardware = 306;
  int version_software = 600;
  int version_imu = 0;
#else
  int version_hardware = 0;
  int version_software = 0;
  int version_imu = 0;
#endif

  void initMeter(){
    // get params from flash
    pref.begin("Meter",false);
    meter_type = pref.getInt("Type",TYPE_2_0);
    home_mode  = pref.getInt("Home",HOME_ANGLE);
    speed_mode = pref.getInt("Speed",SPEED_MODE_QUICK);
    warn_light_onoff = pref.getInt("WarnMode",true);
    warn_slope = pref.getFloat("WarnSlope",0);
    warn_flat = pref.getFloat("WarnFlat",0);
    sleep_time = pref.getInt("Sleep",15);
    pref.end();
    // go to home
    home_size = (meter_type > 10) ? 4 : 2;
    home_mode = home_mode % home_size;
    page = PAGE_HOME;
  }

  void updateMeasure(){
    if(home_mode == HOME_FLATNESS){
      measure.state    = flat.measure.state;
      measure.progress = flat.measure.progress;
    }
    else if(home_mode == HOME_AUTO && auto_mode_select == HOME_AUTO_FLATNESS){
      measure.state    = flat.measure.state;
      measure.progress = flat.measure.progress;
    }
    else{
      measure.state    = clino.measure.state;
      measure.progress = clino.measure.progress;
    }
  }

  void resetMeasure(){
    measure.state = M_IDLE;
    measure.progress = 0;
    clino.measure.state = M_IDLE;
    clino.measure.progress = 0;
    flat.measure.state = M_IDLE;
    flat.measure.progress = 0;
    
  }

/* clino measure function */
  void start_clino_measure(){
    if(clino.measure.state == M_IDLE 
    || clino.measure.state == M_MEASURE_DONE 
    || clino.measure.state == M_UPLOAD_DONE){
      clino.measure.progress = 0;
      clino.measure.state = M_UNSTABLE;
    }
  }

  void set_clino_live(float angle,byte arrow){
    if (arrow == 1) {angle = fabs(angle);} 
    else if(arrow == 2){angle = -fabs(angle);}
    else{angle = fabs(angle);}
    clino.angle_live = roundToZeroOrFive(angle,2);
    clino.slope_live = ConvertToSlope(angle);
    clino.arrow_live = arrow;
  }

  void set_clino_hold(float angle,byte arrow){
    if (arrow == 1) {angle = fabs(angle);} 
    else if(arrow == 2){angle = -fabs(angle);}
    else{angle = fabs(angle);}
    clino.angle_hold = roundToZeroOrFive(angle,2);
    clino.slope_hold = ConvertToSlope(angle);
    clino.arrow_hold = arrow;
  }

  void set_clino_progress(int value){
    if(value < 0 || value > 100)return;
    clino.measure.progress = value;
  }

/* flatness measure function */
  void start_flatness_measure(){
    if(flat.measure.state == M_IDLE 
    || flat.measure.state == M_MEASURE_DONE 
    || flat.measure.state == M_UPLOAD_DONE){
      flat.progress = 0;
      flat.measure.state = M_UNSTABLE;
    }
  }

  void set_flat_live(float value,int arrow){
    if(value >= 50){value = 99.9f;}
    if(value <= 0.5)value = 0;
    flat.arrow_live = arrow;
    flat.flat_live  = value;
  }

  void set_flat_hold(float value,int arrow){
    flat.arrow_hold = arrow;
    flat.flat_hold  = value;
  }

  void set_flat_progress(int value){
    if(value < 0 || value > 100){return;}
    flat.measure.progress = value;
    Serial.printf("pro:%d\n\r",flat.measure.progress);
  }

  void put_meter_type(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("Type", meter_type);
    pref.end();
  }

  void put_meter_home(){
    has_home_change = 1;
    clino.measure.state = M_IDLE;
    clino.measure.progress = 0;
    flat.measure.state = M_IDLE;
    flat.measure.progress = 0;
    while(!pref.begin("Meter",false)){}
    pref.putInt("Home",home_mode);
    pref.end();
  }

  void put_speed_mode(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("Speed", speed_mode);
    pref.end();
  }

  void put_sleep_time(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("Sleep",sleep_time);
    pref.end();
  }

  void put_warn_light(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("WarnMode", warn_light_onoff);
    pref.end();
  }

  void put_warn_slope(){
    while(!pref.begin("Meter",false)){}
    pref.putFloat("WarnSlope", warn_slope);
    pref.end();
  }

  void put_warn_flat(){
    while(!pref.begin("Meter",false)){}
    pref.putFloat("WarnFlat", warn_flat);
    pref.end();
  }

void AckToApp(String info){
  ack_msg = info + "\r\n";
}

void ControlWarningLight(byte mode){
  static byte last_mode = 0;
  if(mode == last_mode)return;
  switch (mode)
  {
  case 0:// OFF
    digitalWrite(IO_LIGHT_GREEN,HIGH);
    digitalWrite(IO_LIGHT_RED,HIGH);
    break;
  case 1:// GREEN
    digitalWrite(IO_LIGHT_GREEN,LOW);
    digitalWrite(IO_LIGHT_RED,HIGH);
    break; 
  case 2:// RED
    digitalWrite(IO_LIGHT_GREEN,HIGH);
    digitalWrite(IO_LIGHT_RED,LOW);
    break;
  default:
    ESP_LOGE("mode","%d",mode);
    break;
  }
  last_mode = mode;
}

void WarningLightFSM(){
  // Warning Light disable,off and return
  if (warn_light_onoff != true) {
    ControlWarningLight(0);
    return;
  }
  // measure done
  if (measure.state == M_MEASURE_DONE || measure.state == M_UPLOAD_DONE) {
    if(home_mode == HOME_ANGLE || home_mode == HOME_SLOPE){
      if (fabs(clino.slope_hold) < warn_slope) {
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
    else if(home_mode == HOME_AUTO){
      if(auto_mode_select == HOME_AUTO_SLOPE){
        if (fabs(clino.slope_hold) < warn_slope) {
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

template<typename T>
String buildFireWaterInfo(const char* prefix, T* array, int length, int decimalPlaces){
  String str = String(prefix) + ":";
  for (int i = 0; i < length; i++) {
    if (i > 0) str += ",";
    str += String(array[i], decimalPlaces);
  }
  return str;
}

};
#endif // METER_H


