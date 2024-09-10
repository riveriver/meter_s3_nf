/**
 * @file Flatness.cpp
 * @author  Vicky Hung
 * @brief
 * @version 0.1
 * @date 2023-12-11
 *
 * Change Log:
 *
 * version 1.0 from Vicky Hung to River
 * version 1.1 calibration
 * version 1.2 simple measurement process because imu measure speed
 * version 1.3 nullptr checking、 while loops add timeout checking、bounds
 * checking、return error code mechanism
 * 
 * @copyright Copyright (c) 2023
 *
 */
#include <Arduino.h>
#include "Flatness.h" 
#define M1_NUM 0
#define M2_NUM 1
TwoWire wire_host = TwoWire(0);
TwoWire wire_sub = TwoWire(1);

void Flatness::init() {
#ifdef SENSOR_1_1
  wire_host.begin(8, 9, 400000U);
  delay(10);
  wire_sub.begin(4, 5, 400000U);
  delay(10);
  TwoWire *p_wire;
  String str = "[ads_init]";
  for(int i = 0;i < 8; i++){
    p_wire = i < 4 ? &wire_host : &wire_sub;
    int addr = 0x48 + i % 4;
    p_wire->beginTransmission(addr);
    if (HandleError_Wire(p_wire->endTransmission(), addr)) {
      ads_init[i] = ads1115[i].begin(addr, p_wire);
      str += String(ads_init[i]) + ",";
    }
    delay(10);
  }
  // Serial.println(str);

#elif defined(SENSOR_1_2)
  /*
   wire_host-->1#:0x49C0; 2#:0x49C1; 
   wire_sub -->3#:0x49C0; 4#:0x49C1;
  */
  wire_host.begin(8, 9, 400000U);
  delay(10);
  wire_sub.begin(4, 5, 400000U);
  delay(10);
  TwoWire *p_wire;
  String str = "[ads_init]";
  for(int i = 0;i < ADS_NUM; i++){
    p_wire = i == 0 ? &wire_host : &wire_sub;
    int addr = 0x49;
    p_wire->beginTransmission(addr);
    if (HandleError_Wire(p_wire->endTransmission(), addr) == true) {
      ads_init[i] = ads1115[i].begin(addr, p_wire);
      str += String(ads_init[i]) + ",";
    }
    delay(10);
  }
  // Serial.println(str);

#elif defined(SENSOR_1_4)
  wire_host.begin(8, 9, 400000U);
  delay(10);
  wire_sub.begin(4, 5, 400000U);
  delay(10);
  TwoWire *p_wire;
  String str = "[ads_init]";
  for(int i = 0;i < ADS_NUM; i++){
    p_wire = i < 1 ? &wire_host : &wire_sub;
    int addr = 0x48;
    p_wire->beginTransmission(addr);
    if (HandleError_Wire(p_wire->endTransmission(), addr)) {
      adc_online[i] = true;
      ads_init[i] = ads1115[i].begin(addr, p_wire);
      manage.flat.adc_online[i] = true;
      str += String(ads_init[i]) + ",";
    }
    delay(10);
  }
  // Serial.println(str);
#endif
  getFitParams();
  for(int i = 0; i < SENSOR_NUM; i++){
    raw_vec[i].resize(30);
    filt_vec[i].resize(5);
  }
}

int Flatness::ProcessMeasureFSM() {
  uint8_t state = manage.flat.measure.state;
  if(state == M_IDLE){return state;}
  float measure_source = manage.flat.flat_live;
  if (state == M_MEASURE_DONE || state == M_UPLOAD_DONE) {
    if(fabs(measure_source - hold_ref) > 2.0f){
      hold_ref = 0;
      manage.set_flat_progress(0);
      return state = M_IDLE;
    }
    return state;
  }
  if (refer_peak > 30) {
    measure_count = 0;
    return state = M_UNSTABLE;
  }else {
    measure_count++;
  }
  if(measure_count == 10){
    measure_count = 0;
    manage.flat.flat_hold = measure_source;
    manage.max_sensor_num = max_dist_num;
    hold_ref = measure_source;
    manage.set_flat_progress(100);
    return state = M_MEASURE_DONE;
  }
  int pro = measure_count * 100.0 / 10.0;
  manage.set_flat_progress(pro);
  return state = M_MEASURE_ING;
}

void Flatness::UpdateAllInOne() {
/*----- data sample ----*/
#ifdef SENSOR_1_1

#elif defined(SENSOR_1_2)
  /*
   wire_host-->1#:0x49C0; 2#:0x49C1; 
   wire_sub -->3#:0x49C0; 4#:0x49C1;
  */
  TwoWire *p_wire;
  byte channel_num = 2;
  for (int i = 0; i < ADS_NUM; i++) {
    p_wire = (i * channel_num < 4) ? &wire_host : &wire_sub;
    byte addr = 0x49;
    // check ads online
    p_wire->beginTransmission(addr);
    // if ads offline, set adc_online[i] = false,continue
    if (p_wire->endTransmission() != 0){
      adc_online[i] = false;
      for (int j = 0; j < channel_num; j++) {
        byte id = i * channel_num + j;
        raw[id] = 0;
        filt[id] = 0;
        dist[id] = -1;
      }
      continue;
    }
    // ads first online
    if(ads_init[i] == false) {
      ads_init[i] = ads1115[i].begin(addr, p_wire);
    }
    // ads online
    if(adc_online[i] == false){
      adc_online[i] = true;
    }
    // read 
    for (int j = 0; j < channel_num; j++) {
      byte id = i * channel_num + j;
      // id check
      if(id >= SENSOR_NUM)continue;
      raw[id] = ads1115[i].readADC_SingleEnded(j);
      // 通过调用erase函数删除vector的第一个元素,再使用push_back函数将raw[id]添加到vector的末尾
      if (raw_vec[id].size() == raw_vec[id].capacity()) {
        raw_vec[id].erase(raw_vec[id].begin());
      }
      raw_vec[id].push_back(raw[id]);
      // HACK calculate raw peak
      float  raw_max = *std::max_element(raw_vec[id].begin(), raw_vec[id].end());
      float  raw_min = *std::min_element(raw_vec[id].begin(), raw_vec[id].end());
      raw_peak[id] = raw_max - raw_min; 
      // filt
      filt[id] = median.calculateMedian(raw_vec[id]);
      if (filt_vec[id].size() == filt_vec[id].capacity()) {
        filt_vec[id].erase(filt_vec[id].begin());
      }
      // HACK calculate filt peak
      filt_vec[id].push_back(filt[id]);
      float  filt_max = *std::max_element(filt_vec[id].begin(), filt_vec[id].end());
      float  filt_min = *std::min_element(filt_vec[id].begin(), filt_vec[id].end());
      filt_peak[id] = filt_max - filt_min; 
      // map distance
      mapDist(id,filt[id]);
    }
  }

#elif defined(SENSOR_1_4)
  TwoWire *p_wire;
  byte channel_num = 4;
  byte wire_adc_num = 1;
  int  adc_addr = 0x48;
  // Read ADC
  for(int i = 0;i < ADS_NUM; i++){
    p_wire = i < wire_adc_num ? &wire_host : &wire_sub;
    p_wire->beginTransmission(adc_addr);
    // ADC offline
    if (p_wire->endTransmission() != 0){
      // Sensor offline
      for (int j = 0; j < channel_num; j++) {
        byte id = i * 4 + j;
        raw[id]  = 0;
        filt[id] = 0;
        dist[id] = -1;
        raw_peak[id] = 0;
        filt_peak[id] = 0;
        sensor_valid[id] = false;
      }
      if(adc_online[i] == true){
        adc_online[i] = false;
        manage.flat.adc_online[i] = false;
        switch (i)
        {
        case 0:
          manage.ui_block_info = "[E]No_ADC Offline";
          break;
        case 1:
          manage.ui_block_info = "1m_Mode";
          break;
        default:
          manage.ui_block_info = "[E]Unkown ADC Offline:" + String(i);
          break;
        }
      }
      continue;
    }
    // ADC online
    if(adc_online[i] == false){
      adc_online[i] = true;
      manage.flat.adc_online[i] = true;
      switch (i)
      {
      case 1:
        manage.ui_block_info = "2m_Mode";
        break;
      default:
        manage.ui_block_info = "[E]Unkown_ADC Online:" + String(i);
        break;
      }
    }
    if(ads_init[i] == false){
      ads_init[i] = ads1115[i].begin(0x48, p_wire);
    }
    // Read Sensor
    for (int j = 0; j < channel_num; j++) {
      byte id = i * 4 + j;
      if(id >= SENSOR_NUM)continue;
      raw[id] = ads1115[i].readADC_SingleEnded(j);
      // raw over th,return
      if(raw[id] < 5000){
        filt[id] = 0;
        filt_peak[id] = 0;
        dist[id] = -2;
        sensor_valid[id] = false;
        continue;
      }

      // get flag
      sensor_valid[id] = true;

      // get raw vec
      if (raw_vec[id].size() == raw_vec[id].capacity()) {
        raw_vec[id].erase(raw_vec[id].begin());
      }
      raw_vec[id].push_back(raw[id]);
      std::vector<int> raw_temp = raw_vec[id];

      // debug raw_peak
      if(manage.debug_flat_mode == 21){
        float  raw_max = *std::max_element(raw_vec[id].begin(), raw_vec[id].end());
        float  raw_min = *std::min_element(raw_vec[id].begin(), raw_vec[id].end());
        raw_peak[id] = raw_max - raw_min;
      }

      // get_filt
      filt[id] = median.calculateMedian(raw_temp);
      if (filt_vec[id].size() == filt_vec[id].capacity()) {
        filt_vec[id].erase(filt_vec[id].begin());
      }
      
      // get filt_peak
      filt_vec[id].push_back(filt[id]);
      float  filt_max = *std::max_element(filt_vec[id].begin(), filt_vec[id].end());
      float  filt_min = *std::min_element(filt_vec[id].begin(), filt_vec[id].end());
      filt_peak[id] = filt_max - filt_min; 

      // map distance
      mapDist(id,filt[id]);
      
    }
  }
#endif
  PrintDebugInfo(manage.debug_flat_mode);
}

void Flatness::mapDist(int id, float x) {
    // param check:id
    if(id >= SENSOR_NUM)return;
    // 
    int start = 0;
    int end = MAP_NUM;
    // 小于1mm默认为0mm
    if(x > map_x[id][0]){
      dist[id] = dist_map[id] = dist_linear[id] = 0;
      return;
    }
    // 大于10mm，线性插值计算距离
    if(x < map_x[id][MAP_NUM - 1]){
      dist[id] = dist_map[id] = dist_linear[id] =  map_y[MAP_NUM - 1] +  (map_x[id][MAP_NUM - 1] - x)/200;
      return;
    }
    // 二分法查找区间：直到 start > end，此时搜索结束，表明目标值不在列表中，但我们已经找到了它所在的两个相邻x值区间
    while (start <= end) {
        int mid = start + (end - start) / 2;
        if (map_x[id][mid] == x){
          dist_map[id] = map_y[mid];
          dist_linear[id] = map_y[mid];
          dist[id] = dist_linear[id];
          return;
        }
        if (x < map_x[id][mid])start = mid + 1;else end = mid - 1;
    }
    dist_map[id] = map_y[end];
    dist_linear[id] = map_y[end] + (map_y[end + 1] - map_y[end]) * (map_x[id][end] - x) / (map_x[id][end] - map_x[id][end + 1]);
    dist[id] = dist_linear[id];
    return;
}

void Flatness::getFlatness() {

  byte valid_size = 0;
  float flat = dist_th;
  float max = -dist_th;
  float min = dist_th;

  for (size_t i = 0; i < SENSOR_NUM; ++i) {

    if(sensor_valid[i] && 0 <= dist[i] && dist[i] <= 20){
      valid_size ++;
      if (dist[i] > max) {
          max = dist[i];
          max_dist_num = i + 1;
      }
      if (dist[i] < min) {
          min = dist[i];
      }
    }
  }
  if(manage.flat_abs){
    flat = valid_size < 1 ? 99.0f : max;
  }else{
    flat = valid_size < 2 ? 99.0f : max - min;
  }
  manage.set_flat_live(modifyDecimal(flat), 0);
}

void Flatness::setZeros() {
  // set zeros
  for(int i = 0;i < SENSOR_NUM;i++){
    zeros[i] = filt[i];
  }
  cali_progress = 100;
}

void Flatness::doRobotArmCali() {
  // status checks
  if (manage.flat.state != FLAT_ROBOT_ARM_CALI) {return;}
  byte step = manage.flat.cali.step;
  
  if(step == CALI_STEP::SAVE){
    putFitParams();
    String str = "";
      for(int i = 0; i < SENSOR_NUM; ++i) {
          str = "flat.cali.param ";
          str += String(i) + ",";
          for(int j = 0; j < MAP_NUM; ++j) {
              str += String(map_x[i][j],0);
              if (j < MAP_NUM - 1) str += ",";
          }
          Serial.println(str);
    }
    manage.flat.state = FLAT_COMMON;
    manage.flat.cali.step = -1;
    return;
  }

  if(step == CALI_STEP::ECHO){
    String str = "";
      for(int i = 0; i < SENSOR_NUM; ++i) {
          str = String(i) + "flat.cali.data ";
          str += String(i) + ",";
          for(int j = 0; j < MAP_NUM; ++j) {
              str += String(map_x[i][j],0);
              if (j < MAP_NUM - 1) str += ",";
          }
          Serial.println(str);
    }
    Serial.println(str);
    manage.flat.progress = 0;
    manage.flat.cali.record_num = 0.0;
    manage.flat.state = FLAT_COMMON;
    manage.flat.cali.step = -1;
    return;
  }

  if(step == CALI_STEP::RECORD){
    String str ="flat.cali.qcqa " + String(manage.flat.cali.record_num,1) + ",";
      for(int i = 0; i < SENSOR_NUM; i++) {
              str += String(dist[i],1);
              if (i < SENSOR_NUM - 1) str += ",";
    }
    Serial.println(str);
    manage.flat.progress = 0;
    manage.flat.cali.record_num = 0.0;
    manage.flat.state = FLAT_COMMON;
    manage.flat.cali.step = -1;
    return;
  }

  // reset data
  if(manage.flat.progress == 0){
    for (int i = 0; i < SENSOR_NUM; i++)fit_x[i] = 0;
  }

  // check stable
  float max = 0;
  for (int i = 0; i < SENSOR_NUM; i++) {
      if (filt_peak[i] > max) {
        max = filt_peak[i];
      }
  }
  if (max > 20) {
      stable_count = 0;
      return;
  }
  // progress
  stable_count++;
  if(stable_count > 10){
    for(int i = 0; i < SENSOR_NUM; i++){fit_x[i] += filt[i];}
    manage.flat.progress += 5;
  }
  
  // check finish
  if(manage.flat.progress == 100){
    for (int i = 0; i < SENSOR_NUM; i++) {
      map_x[i][step] = fit_x[i] / 20;
    }
    if(manage.flat.cali.step == 1){
      for (int i = 0; i < SENSOR_NUM; i++) {
        map_x[i][0] = map_x[i][1] + 300;
      }
    }
    stable_count = 0;
    String str = "flat.cali.cpct 100";
    Serial.println(str);
    manage.flat.progress = 0;
    manage.flat.cali.step = -1;
    manage.flat.state = FLAT_COMMON;
  }
}

float Flatness::modifyDecimal(float value) {
  // 获取小数部分
  float decimalPart = fmod(value, 1.0f);

  // 将小数部分转换为0到9之间的整数
  int firstDecimalDigit = (int)(decimalPart * 10);  // 注意：可能会有精度损失

  // 根据条件修改第一位小数
  if (firstDecimalDigit <= 3) {
    firstDecimalDigit = 0;
  } else if (firstDecimalDigit > 3 && firstDecimalDigit < 8) {
    firstDecimalDigit = 5;
  } else {                  // 假设这里指的是大于7
    firstDecimalDigit = 0;  // 进1后本位变成0
    // 注意：实际进位需要考虑整体数值加1的问题
    value += 1.0f - (float)firstDecimalDigit / 10.0f;
  }

  // 将修改后的第一位小数重新组合回原浮点数
  float modifiedValue = value - decimalPart + (float)firstDecimalDigit / 10.0f;

  return modifiedValue;
}

void Flatness::putFitParams() {
  stores.begin("FIT", false);
  String key = "";
  for(int i = 0; i < SENSOR_NUM; i++){
    for(int j = 0; j < MAP_NUM; j++){
      key = String(i) + String(j);
      stores.putFloat(key.c_str(),map_x[i][j]);
    }
  }
  stores.end();
}

void Flatness::getFitParams() {
  stores.begin("FIT", true);
  String key = "";
  for (int i = 0; i < SENSOR_NUM; i++) {
    for(int j = 0; j < MAP_NUM; j++){
      key = String(i) + String(j);
      map_x[i][j] = stores.getFloat(key.c_str(),0);
    }
  }
  stores.end();
}

bool  Flatness::HandleError_Wire(byte error, byte addr) {
  switch (error) {
    case 0:
      ESP_LOGE("wire", "0x%X:success", addr);
      return true;
    case 1:
      ESP_LOGE("wire", "0x%X:data too long to fit in transmit buffer", addr);
      return false;
    case 2:
      ESP_LOGE("wire", "0x%X:received NACK on transmit of i", addr);
      return false;
    case 3:
      ESP_LOGE("wire", "0x%X:received NACK on transmit of data", addr);
      return false;
    case 4:
      ESP_LOGE("wire", "other error");
      return false;
    case 5:
      ESP_LOGE("wire", "timeout");
      return false;
    default:
      ESP_LOGE("wire", "error:%d\n");
      return false;
  }
}

void Flatness::PrintDebugInfo(byte debug_mode) {
  // TODO Not elegant, needs to be optimized
  // Serial_Print_Raw_Data  = debug & 0b00000001;
  // Serial_Print_Filt_Data = debug & 0b00000010;
  // Serial_Print_Dist_Data = debug & 0b00000100;
  if (!manage.debug_flat_mode) return;
  String str = "";
  switch (manage.debug_flat_mode) {
    case 1:
      // TODO Not elegant: use template<typename T>
      str = "raw:";
      for (int i = 0; i < SENSOR_NUM; i++) {
        if (i > 0) str += ",";
        str += String(raw[i]);
      }
      break;
    case 2:
      str = manage.buildFireWaterInfo("filt", filt, SENSOR_NUM, 0);
      break;
    case 3:
      str = manage.buildFireWaterInfo("dist", dist, SENSOR_NUM, 1);
      break;
    case 4:
      str = "compare:";
      for (int i = 0; i < SENSOR_NUM; i++) {
        if (i > 0) str += ",";
        str += String(raw[i]);
        str += ",";
        str += String(filt[i], 0);
      }
      break;
    default:
      ESP_LOGE("", "set_debug:%d", manage.debug_flat_mode);
      break;
  }
  if (str != NULL) Serial.println(str);
}

