/**
 * @file MeterManage.h
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

enum BIE_COM_DEFINE{

  BATTERY_BASE = 0,//电量，小于1000的值就是电量

  METER_TYPE_500 = 2000,//靠尺类型
  METER_TYPE_600 = 2001,
  METER_TYPE_1000 = 2005,
  METER_TYPE_1200 = 2006,

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
  ANGLE,
  SLOPE,
  FLATNESS,
  SLOPE_FLATNESS
};

enum MEASURE_STATE{
  IDLE,
  UNSTABLE,
  MEASURING,
  MEASURE_DONE,
  UPLOAD_DONE,
};

class Meter{
private:
struct Calibration{
  byte step = 0;
  byte status = 0;
};
struct Measure{
  byte state = 0;
  byte progress = 0;
};
struct ClinoMeter{
  float angle_live = 0.0f;
  float angle_hold = 0.0f;
  byte  arrow_live = 0;
  byte  arrow_hold = 0;
  Measure measure;
};
struct FlatnessMeter{
  float flat_live = 0.0f;
  float flat_hold = 0.0f;
  byte  arrow_live = 0;
  byte  arrow_hold = 0;
  Measure measure;
};
Preferences   pref;
public:
  enum MeasureStatus {
    IDLE,
    UNSTABLE,
    MEASURING,
    MEASURE_DONE,
    UPLOAD_DONE
  };
  Measure measure;
  ClinoMeter  clino;
  FlatnessMeter flatness;
  int software_version = 701;
  int hardware_version = 201;
  int imu_version = 0;
  byte meter_type = 1;
  byte cursor = 0;
  byte page = 0;
  byte pre_page = 0;
  byte minor_page = 0;
  byte pre_minor_page = 0;
  byte home_size = 2;
  byte home_mode = 0;
  byte pre_home_mode = 0;
  Calibration imu_cali;
  Calibration dist_cali;
  int     block_time;
  byte    speed_mode  = 1;
  byte    warrning_mode = 2;
  float   warrning_angle = 1.0f;
  float   warrning_flat  = 3.0;
  float   slope_standard = 1000.0f;
  int     battery;
  bool    has_update_dist = false;
  bool    has_home_change = false;
  bool    has_imu_forward = false;
  bool    has_flat_forward = false;
  String  cali_forward_str = "";

  void set_angle_live(float value,int arrow){
    clino.arrow_live = arrow;
    if (arrow == 1) {clino.angle_live = -value;} 
    else if(arrow == 2){clino.angle_live = value;}
    else{ESP_LOGE("","arrow:%d",arrow);}
  }

  void set_angle_progress(int value){
    if(value < 0 || value > 100){
      return;}
    clino.measure.progress = value;
  }

  void start_clino_measure(){
    if(clino.measure.state == IDLE 
    || clino.measure.state == MEASURE_DONE 
    || clino.measure.state == UPLOAD_DONE){
      clino.measure.state = UNSTABLE;
    }
  }

  void hold_clino(float angle,int arrow){
    clino.angle_hold = angle;
    clino.arrow_hold = arrow;
  }

  void set_flat_live(float value,int arrow){
    if(value >= 100){value == 99.9f;}
    flatness.arrow_live = arrow;
    flatness.flat_live  = value;
  }

  void set_flat_progress(int value){
    if(value < 0 || value > 100){return;}
    flatness.measure.progress = value;
  }

  void hold_flatness(float flat,int arrow){
    flatness.arrow_hold = arrow;
    flatness.flat_hold  = flat;
  }

  void start_flatness_measure(){
    if(flatness.measure.state == IDLE 
    || flatness.measure.state == MEASURE_DONE 
    || flatness.measure.state == UPLOAD_DONE){
      flatness.measure.state = UNSTABLE;
    }
  }

  void initMeter(){
    while(!pref.begin("Meter",false)){
        Serial.println(F("[initMeter]getMeter Fail"));
    }
    meter_type = pref.getInt("Type",11);
    home_mode  = pref.getInt("Home",0);
    pref.end();
  // init home_size
    if(meter_type > 10){
      home_size = 4;
    }
    else {home_size = 2;}
    home_mode = home_mode % home_size;
    pinMode(12,OUTPUT); 
    digitalWrite(12,HIGH);
  }

  void updateSystem(){
    if(home_mode == ANGLE 
     ||home_mode == SLOPE
     ||home_mode == SLOPE_FLATNESS ){
      measure.state = clino.measure.state;
      measure.progress = clino.measure.progress;
    }
    else{
      measure.state = flatness.measure.state;
      measure.progress = flatness.measure.progress;
    }
  }

  void putMeterType(){
    while(!pref.begin("Meter",false)){}
    pref.putInt("Type", meter_type);
    pref.end();
  }

  void putMeterHome(){
    has_home_change = 1;
    clino.measure.state = IDLE;
    clino.measure.progress = 0;
    flatness.measure.state = IDLE;
    flatness.measure.progress = 0;
    while(!pref.begin("Meter",false)){
    }
    pref.putInt("Home",home_mode);
    pref.end();
  }

};

#endif 

