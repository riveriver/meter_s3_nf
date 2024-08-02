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
 * 24/6/2024 平整度标定：拟合曲线由二元一次函数改为二次多项式，同时不再使用ADC采样值而是使用电压值作为因变量，修改距离计算方式
 * 24/6/2024 平整度标定采样逻辑:现在会等peak稳定之后再采样
 * 
 * @copyright Copyright (c) 2023
 *
 */
#include <Arduino.h>
#include "Flatness.h" 

TwoWire wire_host = TwoWire(0);
TwoWire wire_sub = TwoWire(1);

void Flatness::init() {
#ifdef HARDWARE_0_0
  /*
   wire_host-->1#:0x48C0; 2#:0x48C1; 
   wire_host-->3#:0x49C0; 4#:0x49C1; 
   wire_sub -->1#:0x4AC0; 2#:0x4AC1; 
   wire_sub -->3#:0x4BC0; 4#:0x4BC1; 
  */
  // wire process
  wire_host.begin(8, 9, 400000U);
  delay(10);
  wire_sub.begin(4, 5, 400000U);
  delay(10);
  TwoWire *p_wire;
  String str = "[ads_init]";
  for(int i = 0;i < ADS_NUM; i++){
    p_wire = i < 2 ? &wire_host : &wire_sub;
    int addr = 0x48 + i;
    p_wire->beginTransmission(addr);
    if (HandleError_Wire(p_wire->endTransmission(), addr) == true) {
      ads_init[i] = ads1115[i].begin(addr, p_wire);
      str += String(ads_init[i]) + ",";
    }
    delay(10);
  }
  // Serial.println(str);

#elif defined(SENSOR_1_1)
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
      ads_init[i] = ads1115[i].begin(addr, p_wire);
      str += String(ads_init[i]) + ",";
    }
    delay(10);
  }
  // Serial.println(str);
#endif
  getFitParams();
  for(int i = 0; i < SENSOR_NUM; i++){
    raw_vec[i].resize(20);
    filt_vec[i].resize(5);
  }
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
    // if ads offline, set ads_online[i] = false,continue
    if (p_wire->endTransmission() != 0){
      ads_online[i] = false;
      for (int j = 0; j < channel_num; j++) {
        byte id = i * channel_num + j;
        raw[id] = 0;
        filt[id] = 0;
        dist[id] = 0;
      }
      continue;
    }
    // ads first online
    if(ads_init[i] == false) {
      ads_init[i] = ads1115[i].begin(addr, p_wire);
    }
    // ads online
    if(ads_online[i] == false){
      ads_online[i] = true;
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
  String str = "[ads_init]";
  for(int i = 0;i < ADS_NUM; i++){
    p_wire = i < 1 ? &wire_host : &wire_sub;
    int addr = 0x48;
    p_wire->beginTransmission(addr);
    // ADC offline
    if (p_wire->endTransmission() != 0){
      for (int j = 0; j < channel_num; j++) {
        byte id = i * 4 + j;
        raw[id]  = 0;
        filt[id] = 0;
        dist[id] = 0;
        raw_peak[id] = 0;
        filt_peak[id] = 0;
        sensor_online[id] = false;
      }
      continue;
    }
    for (int j = 0; j < channel_num; j++) {
      byte id = i * 4 + j;
      // id check
      if(id >= SENSOR_NUM)continue;
      // read
      raw[id] = ads1115[i].readADC_SingleEnded(j);
      // 通过调用erase函数删除vector的第一个元素,再使用push_back函数将raw[id]添加到vector的末尾
      if (raw_vec[id].size() == raw_vec[id].capacity()) {
        raw_vec[id].erase(raw_vec[id].begin());
      }
      raw_vec[id].push_back(raw[id]);
      std::vector<int> raw_temp = raw_vec[id];
#ifdef FACTORY_TEST
      // HACK calculate raw peak
      float  raw_max = *std::max_element(raw_vec[id].begin(), raw_vec[id].end());
      float  raw_min = *std::min_element(raw_vec[id].begin(), raw_vec[id].end());
      raw_peak[id] = raw_max - raw_min; 
#endif
      // filt
      filt[id] = median.calculateMedian(raw_temp);
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
#endif
  PrintDebugInfo(manage.flat_debug);
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
  if(manage.flat.progress < 100){
    manage.flat.progress += 10;
    return state = M_MEASURING;
  }
  manage.flat.flat_hold = measure_source;
  hold_ref = measure_source;
  manage.set_flat_progress(100);
  return state = M_MEASURE_DONE;
}

void Flatness::mapDist(int id, float x) {
    // param check:id
    if(id >= SENSOR_NUM)return;

    int start = 0;
    int end = MAP_NUM;
    // 小于0cm默认为0cm
    if(x > map_x[id][0]){
      dist[id] = dist_map[id] = dist_linear[id] = 0;
      return;
    }
    // 大于门洞阈值
    if(x < 6000){
      dist[id] = dist_map[id] = dist_linear[id] = 99.9;
      return;
    }
    // 大于10cm，线性插值计算距离
    if(x < map_x[id][MAP_NUM - 1]){
      dist[id] = dist_map[id] = dist_linear[id] =  map_y[MAP_NUM - 1] +  (map_x[id][MAP_NUM - 1] - x)/100;
      return;
    }
    // 二分法查找区间：直到 start > end，此时搜索结束，表明目标值不在列表中，但我们已经找到了它所在的两个相邻x值区间
    while (start <= end) {
        int mid = start + (end - start) / 2;
        if (map_x[id][mid] == x){
          dist_map[id] = map_y[mid];
          dist_linear[id] = map_y[mid];
          return;
        }
        if (x < map_x[id][mid])start = mid + 1;else end = mid - 1;
    }
    dist_map[id] = map_y[end];
    dist_linear[id] = map_y[end] + (map_y[end + 1] - map_y[end]) * (map_x[id][end] - x) / (map_x[id][end] - map_x[id][end + 1]);
    if(filt_peak[id] < 30){dist[id] = dist_linear[id];}else{dist[id] = dist_map[id];}
    return;
}

void Flatness::calculateFlatness() {
  // block flag
  if(ads_online[2] && ads_online[3] && manage.flat.sub_online == false){
    manage.flat.sub_online = true;
    manage.flat.online_block = 2;
  }
  else if((!ads_online[2] || !ads_online[3])&& manage.flat.sub_online == true){
    manage.flat.sub_online = false;
    manage.flat.online_block = 1;
  }
  // ready flag
for(int i = 0;i < SENSOR_NUM;i++){
  if(filt_peak[i] > 30)manage.flat.ready[i] = 0;
  else manage.flat.ready[i] = 1;
}
byte valid_size = 0;
float flat = dist_th;
float max = -dist_th;
float min = dist_th;
for (size_t i = 0; i < SENSOR_NUM; ++i) {
  if(dist[i] != 0 && dist[i] < dist_th){
    valid_size ++;
    if (dist[i] > max) {
        max = dist[i];
        manage.dash_num = i + 1;
    }
    if (dist[i] < min) {
        min = dist[i];
    }
  }
}
if(manage.flat_abs){
  flat = valid_size < 1 ? 99.0f : max;
  manage.set_flat_live(modifyDecimal(flat), 0);
}else{
  flat = valid_size < 2 ? 99.0f : max - min;
  manage.set_flat_live(modifyDecimal(flat), 0);
}

}

int  Flatness::ifStable() {
  // check
  for(int i = 0;i < SENSOR_NUM;i++){
    if(filt_peak[i] > 30)return i + 1;
  }
  return 0;
}

void Flatness::setZeros() {
  // check
  if(ifStable())return;
  // set zeros
  for(int i = 0;i < SENSOR_NUM;i++){
    zeros[i] = filt[i];
  }
  cali_progress = 100;
}

int stable_count  = 0;
void Flatness::doRobotArmCali() {
  // status checks
  if (manage.flat.state != FLAT_ROBOT_ARM_CALI) {return;}
  byte step = manage.flat.cali.step;
  if(step == 11){
    manage.flat.state = FLAT_COMMON;
    manage.flat.cali.step = 0;
    putFitParams();
    Serial.println("[D] SaveCalibrationParams");
    return;
  }

  if(manage.flat.progress == 0){
    for (int i = 0; i < SENSOR_NUM; i++)fit_x[i] = 0;
  }

  String str = "[D] " + String(step) + "mm:";

  float max = 0;
  for (int i = 0; i < SENSOR_NUM; i++) {
      if (filt_peak[i] > max) {
        max = filt_peak[i];
      }
  }
  
  manage.max_filt_peak = max;
  if (max > 100) {
      stable_count = 0;
      return;
  } else {
      stable_count++;
  }

  if(stable_count == 20){
  for (int i = 0; i < SENSOR_NUM; i++){fit_x[i] += filt[i];}
    manage.flat.progress += 50;
    stable_count = 0;
  }
  // wait for stable
  
  if(manage.flat.progress == 100){
    for (int i = 0; i < SENSOR_NUM; i++) {
      map_x[i][step] = fit_x[i] / 2;
      str += String(map_x[i][step],0);
      if (i != SENSOR_NUM - 1) {
        str += ",";
      }
    }
    Serial.println(str);
    manage.flat.progress = 0;
    manage.flat.cali.step ++;
    String str_cmd = "robot.cali.flat,"+ String(manage.flat.cali.step);
    Serial.println(str_cmd);
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
    for(int j = 0; j < 10; j++){
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
  String str;
    for(int i = 0; i < 8; ++i) {
        for(int j = 0; j < MAP_NUM; ++j) {
            str += String(map_x[i][j]);
            if (j < MAP_NUM - 1) str += ", ";
        }
        str += "\n"; 
    }
    Serial.println(str); 
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

void Flatness::PrintDebugInfo(byte debug) {
  // TODO Not elegant, needs to be optimized
  // Serial_Print_Raw_Data  = debug & 0b00000001;
  // Serial_Print_Filt_Data = debug & 0b00000010;
  // Serial_Print_Dist_Data = debug & 0b00000100;

  if (debug == 0) return;
  // debug = 1;
  String str = "";
  switch (debug) {
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
      ESP_LOGE("", "set_debug:%d", debug);
      break;
  }
  if (str != NULL) Serial.println(str);
}

// void Flatness::doAppCali() {
//   // status checks
//   if (manage.flat.state != FLAT_APP_CALI) {return;}
//   if (manage.flat_height_level == -1) {return;}
//   // get param
//   byte step = manage.flat_height_level;
//   // if collected all already,execute algorithm
//   if (step == CALI_STEP::ECHO) {
//     String str = "";
//     for(int i = 0; i < SENSOR_NUM; ++i) {
//         for(int j = 0; j < MAP_NUM; ++j) {
//             str += String(map_x[i][j]);
//             if (j < MAP_NUM - 1) str += ", ";
//         }
//         str += "\r\n"; 
//     }
//     // Serial.println(str); 
//     manage.flat_cali_str += str;
//     manage.has_flat_forward = true;
//   }
//   else if (step == CALI_STEP::SAVE) {
//     for(int i = 0; i < SENSOR_NUM; ++i) {
//         for(int j = 0; j < MAP_NUM; ++j) {
//             map_x[i][j] += fit_x[i][j];
//         }
//     }
//     putFitParams();
//     String str = "OK:Fit";
//     str += "\r\n";
//     manage.flat_cali_str = str;
//     manage.has_flat_forward = true;
//   } else {
//     // print sample data
//     if(step == 0){
//           String str = "warning:step = 0";
//           str += "\r\n";
//           manage.flat_cali_str = str;
//           manage.has_flat_forward = true;
//           manage.flat_height_level = -1;
//           manage.flat.state = FLAT_COMMON;
//           return;      
//     }
//     String str = String(step) + "mm:";
//     for (int i = 0; i < SENSOR_NUM; i++) {
//         if(filt_peak[i] > 30){
//           str = "filt_peak:" + String(filt_peak[i],0);
//           str += "\r\n";
//           manage.flat_cali_str = str;
//           manage.has_flat_forward = true;
//           return;
//         }
//         fit_x[i][step - 1] = filt[i];
//         str += String(fit_x[i][step - 1],0);
//         if (i != SENSOR_NUM - 1) {
//           str += ",";
//         }
//     }
//     str += "\r\n";
//     manage.flat_cali_str = str;
//     manage.has_flat_forward = true;
//   }
//   manage.flat_height_level = -1;
//   manage.flat.state = FLAT_COMMON;
// }
