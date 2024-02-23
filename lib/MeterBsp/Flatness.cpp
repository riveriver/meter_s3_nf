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
#include "Flatness.h"

#include <Arduino.h>

TwoWire Wire_1 = TwoWire(0);
TwoWire Wire_2 = TwoWire(1);

void Flatness::init() {
  getDistanceZero(zeros, 8);
  getDistanceScale(slopes, 8);

  if (Serial_Print_Param) {
    String str = "";
    str = "[DS_Slopes]";
    for (int i = 0; i < 8; i++) {
      str += String(slopes[i]) + ",";
    }
    Serial.println(str);
    str = "[DS_Zeros]";
    for (int i = 0; i < 8; i++) {
      str += String(zeros[i]) + ",";
    }
    Serial.println(str);
  }

  // Begin Two Wire
  // 400000U is the highest frequency of the ESP32 I2C Bus
  byte error = 0;
  Wire_1.begin(IIC_1_SDA, IIC_1_SCL, 400000U);
  Wire_1.beginTransmission(ADS_1_ADDR);
  error = Wire_1.endTransmission();
  if (error == 0) {
    ESP_LOGE("", "Wire_1 transmission success");
    ads_connect[0] = ads1115[0].begin(ADS_1_ADDR, &Wire_1);
  } else {
    ESP_LOGE("", "Wire_1 transmission returned error %d", error);
  }

  Wire_2.begin(IIC_2_SDA, IIC_2_SCL, 400000U);
  Wire_2.beginTransmission(ADS_2_ADDR);
  error = Wire_2.endTransmission();
  if (error == 0) {
    ESP_LOGE("", "Wire_2 transmission success");
    ads_connect[1] = ads1115[1].begin(ADS_2_ADDR, &Wire_2);
  } else {
    ESP_LOGE("", "Wire_2 transmission returned error %d", error);
  }
}

/**
 * @brief Update Wire(wire) sensor reading
 * @param [in] wire - 0 to update Wire(0) sensor, 1 to update Wire(1) sensor.
 * @param [out] MaxMin - Max value, min value and difference of the sensors.
 */
void Flatness::update(byte wire) {
  if (isUpdate[0] && isUpdate[1]) {
    memset(isUpdate, 0, sizeof(isUpdate));
    memmove(&ADCread[1][0], &ADCread[0][0], sizeof(ADCread[0][0]) * 8 * 1);
    memmove(&ADCSurgeOut[1][0], &ADCSurgeOut[0][0],
            sizeof(ADCSurgeOut[0][0]) * 8 * 1);
  }
  // Update all distancesensor wire is I2C num 0/1
  if (!isUpdate[wire]) {
    for (int i = wire * 4; i < wire * 4 + 4; i++) {
      updateSingleChannel(i);
    }
    isUpdate[wire] = true;
  }
  if (Serial_Print_Raw_Data) {
    String S = "";
    for (int i = 0; i < 8; i++) {
      S += String(ADCread[0][i]) + ",";
    }
    Serial.println(S);
  }
  // Calculate
  if (isUpdate[0] && isUpdate[1]) {
    // Print distance data.
    if (Serial_Print_Distance) {
      String S = "[DS_Distance]";
      for (int i = 0; i < 8; i++) {
        S += String(distance[i]) + ",";
      }
      Serial.println(S);
    }

    // If recieve set zero command.
    if (has_set_zero) {
      caliZero();
    } else if (manage.dist_cali.status) {
      doCalibration();
    } else {
      // Calculate maximum displacement.
      Mm.Clear();
      Mm.Add(&distance[0], 8);
      // if (pReceive->isSenseConnect)  // If connect to sub ruler, consider the
      // sub
      //                                // ruler value.
      //   Mm.Add(&pReceive->distance[0], 8);
      if (Mm.MaxVal >= Cut_Off_Distance) Mm.Diff = Cut_Off_Distance;
      manage.flatness.flat_live = Mm.Diff;
      manage.has_update_dist = true;
    }
  }
}

void Flatness::updateSingleChannel(int id) {
  // id process
  // if(id == 2 || id == 3){
  //   ADCread[0][id] = ADCfilt[id] = distance[id] = 0;
  //   return;
  // }
  if (id < 4 && ads_connect[0] == true) {
    ADCread[0][id] = ads1115[0].readADC_SingleEnded(id);
  } else if (id >= 4 && id <= 7 && ads_connect[1] == true) {
    ADCread[0][id] = ads1115[1].readADC_SingleEnded(id - 4);
  }
  // check cut_off:too far,data does not make sense
  // if (ADCread[0][id] < 2000) {
  //   ADCread[0][id] = 2000;
  //   ADCfilt[id]    = 2000;
  //   Distance[id]   = 2000;
  //   return;
  // }
  // check amplitude:too much amplitude,data does not make sense
  bool TH1 = abs(ADCread[0][id] - ADCread[1][id]) < 50;
  bool TH2 = abs(ADCread[0][id] - ADCread[2][id]) < 50;
  ADCSurgeOut[0][id] = (!TH1 && !TH2) ? ADCSurgeOut[1][id] : ADCread[0][id];
  if (!TH1 && !TH2) return;
  // Apply Butterworth filter.
  ADCfilt[id] = ADCfilt[id] * BW_C[0] + ADCread[0][id] * BW_C[1] +
                ADCread[1][id] * BW_C[2];
  if (abs(ADCfilt[id] - ADCread[0][id]) > BW_Reset_TH) {
    if (TH1)
      ADCfilt[id] = ADCread[0][id] * 0.5 + ADCread[1][id] * 0.5;
    else
      ADCfilt[id] = ADCread[0][id] * 0.5 + ADCread[2][id] * 0.5;
  }
  // switch slope
  if (Enable_Cali_Slope && Read_Slope_First) {
    getDistanceScale(slopes, 8);
    Read_Slope_First = false;
  } else if (!Enable_Cali_Slope && Read_Slope_First) {
    for (int i = 0; i < 8; i++) {
      slopes[i] = 330000;
    }
    Read_Slope_First = false;
  }
  // calculate distance
  if (ADCfilt[id] != 0 && zeros[id] != 0) {
    distance[id] = slopes[id] * (1 / ADCfilt[id] - 1 / zeros[id]);
  } else {
    distance[id] = 0;
  }
}

/**
 * @brief Command to calibrate the sensor's zero position.
 *
 * @param [in] OnOff True to start calibrating the zero position. False to stop
 * the calibration.
 * @param [in] isForced True if do the reset zero position anyway.
 *                      False if the command is from the system auto calibration
 * detected and need to consider the \b Enable_Auto_Reset setting.
 */
void Flatness::reset(bool onoff, bool force) {
  if (!onoff) {
    has_set_zero = false;
    sample_count = 0;
    SetZeroProgress = 1;
  } else if (force || Enable_Auto_Reset) {
    has_set_zero = true;
    sample_count = 0;
    SetZeroProgress = 0;
  }
}

byte Flatness::getDistanceScale(float *data, int data_size) {
  // nullptr checking
  if (data == nullptr) {
    return ERROR_NULL_POINTER;
  }
  //
  unsigned long start = millis();
  while (!stores.begin("Distance_Scale", true)) {
    if (millis() - start >= action_timeout) {
      return ERROR_BEGIN_FAIL;
    }
  }
  int size = min(data_size, 8);
  for (int i = 0; i < size; i++) {
    String key = "S" + String(i);
    data[i] = stores.getFloat(key.c_str(), 330000);
  }
  stores.end();
  return ERROR_SUCCESS;
}

byte Flatness::getDistanceZero(float *data, int data_size) {
  // nullptr checking
  if (data == nullptr) {
    return ERROR_NULL_POINTER;
  }
  //
  unsigned long start = millis();
  while (!stores.begin("Distance_Zero", true)) {
    if (millis() - start >= action_timeout) {
      return ERROR_BEGIN_FAIL;
    }
  }
  int size = min(data_size, 8);
  for (int i = 0; i < size; i++) {
    String key = "Z" + String(i);
    data[i] = stores.getFloat(key.c_str(), 10000);
  }
  stores.end();
  return ERROR_SUCCESS;
}

byte Flatness::putDistanceScale(float *data, int data_size) {
  // nullptr checking
  if (data == nullptr) {
    return ERROR_NULL_POINTER;
  }
  // while loops add timeout checking
  unsigned long start = millis();
  while (!stores.begin("Distance_Slope", false)) {
    if (millis() - start >= action_timeout) {
      return ERROR_BEGIN_FAIL;
    }
  }
  // bounds checking
  int size = min(data_size, 8);
  // put
  for (int i = 0; i < size; i++) {
    String key = "S" + String(i);
    stores.putFloat(key.c_str(), data[i]);
  }
  stores.end();
  return ERROR_SUCCESS;
}

byte Flatness::putDistanceZero(float *data, int data_size) {
  // nullptr checking
  if (data == nullptr) {
    return ERROR_NULL_POINTER;
  }
  // while loops add timeout checking
  unsigned long start = millis();
  while (!stores.begin("Distance_Zero", false)) {
    if (millis() - start >= action_timeout) {
      return ERROR_BEGIN_FAIL;
    }
  }
  // bounds checking
  int size = min(data_size, 8);
  // put
  for (int i = 0; i < size; i++) {
    String key = "Z" + String(i);
    stores.putFloat(key.c_str(), data[i]);
  }
  stores.end();
  return ERROR_SUCCESS;
}

/**
 * @brief Check if data within threshold and save data into buffer.
 */
void Flatness::collectCaliSample() {
  // Initialize Collection
  if (sample_count == 0) {
    memcpy(&sample_start[0], &ADCread[0][0], sizeof(sample_start[0]) * 8);
    memset(&sample_sum[0], 0, sizeof(sample_sum[0]) * 8);
  }
  for (int i = 0; i < 8; i++) {
    // if (isConnect[i]) {
    // Check the stability
    if (abs(ADCread[0][i] - sample_start[i]) > sample_threshold ||
        ADCread[0][i] == Cut_Off_Voltage) {
      sample_count = 0;
      return;
    } else {
      sample_sum[i] += ADCread[0][i];
    }
    // }
  }
  sample_count++;
}

void Flatness::doCalibration() {
  uint8_t step = manage.dist_cali.step;
  if (step == cali_size) {
    for (int i = 0; i < 8; i++) {
      // cali_sample[i][0] = zeros[i];
      slopes[i] = calcIterativeCoefficientsB0(i, cali_sample[i], cali_target,
      cali_size);
    }
    putDistanceScale(slopes,8);
    String S = "[DS_Cali]CalibrationComplete: ";
    for (int i = 0; i < 8; i++) {
      S += String(slopes[i]) + " ,";
    }
    Serial.println(S);
  } else {
    String step_str = "[step" + String(step) + "mm]";
    for (int i = 0; i < sensor_size; i++) {
      cali_sample[sensor_size][step] = ADCfilt[i];
      step_str += String(cali_sample[sensor_size][step]) + ",";
    }
    Serial.println(step_str);
  }
  manage.dist_cali.status = 0;
  // sample_count = 0;
}

// void Flatness::doCalibration() {
//   Mm.Diff = 0;
//   uint8_t step = manage.dist_cali.step;
//   if (step == 11) {
//     // cali_fix_1[0] = zeros[0];
//     // cali_fix_2[0] = zeros[1];
//     // cali_fix_3[0] = zeros[2];
//     // cali_fix_4[0] = zeros[3];
//     // cali_fix_5[0] = zeros[4];
//     // cali_fix_6[0] = zeros[5];
//     // cali_fix_7[0] = zeros[6];
//     // slopes[0] = calcIterativeCoefficientsB0(0, cali_fix_1, cali_fix_y, cali_num);
//     // slopes[1] = calcIterativeCoefficientsB0(1, cali_fix_2, cali_fix_y, cali_num);
//     // slopes[2] = calcIterativeCoefficientsB0(2, cali_fix_3, cali_fix_y, cali_num);
//     // slopes[3] = calcIterativeCoefficientsB0(3, cali_fix_4, cali_fix_y, cali_num);
//     // slopes[4] = calcIterativeCoefficientsB0(4, cali_fix_5, cali_fix_y, cali_num);
//     // slopes[5] = calcIterativeCoefficientsB0(5, cali_fix_6, cali_fix_y, cali_num);
//     // slopes[6] = calcIterativeCoefficientsB0(6, cali_fix_7, cali_fix_y, cali_num);
//     for (int i = 0; i < 8; i++) {
//       cali_sample[i][0] = zeros[i];
//       slopes[i] = calcIterativeCoefficientsB0(i, cali_sample[i], cali_target,
//       cali_size);
//     }
//     putDistanceScale(slopes,8);
//     String S = "[DS_Cali]CalibrationComplete: ";
//     for (int i = 0; i < 8; i++) {
//       S += String(slopes[i], 3) + " ,";
//       Serial.println(S);
//     }
//   } else {
//     // cali_fix_1[step] = ADCfilt[0];
//     // cali_fix_2[step] = ADCfilt[1];
//     // cali_fix_3[step] = ADCfilt[2];
//     // cali_fix_4[step] = ADCfilt[3];
//     // cali_fix_5[step] = ADCfilt[4];
//     // cali_fix_6[step] = ADCfilt[5];
//     // cali_fix_7[step] = ADCfilt[6];
//     String step_str = "[step" + String(step) + "mm]";
//     for (int i = 0; i < sensor_size; i++) {
//       cali_sample[sensor_size][step] = ADCfilt[i];
//       step_str += String(cali_sample[sensor_size][step]) + " ,";
//     }
//     Serial.println(step_str);
//   }
//   manage.dist_cali.status = 0;
//   sample_count = 0;
// }

/**
 * @brief Collect \b sample_size of ADC data and set the average value as zeros.
 */
void Flatness::caliZero() {
  // Step 1. Set output to default while calibrating
  // ============================================================
  Mm.Diff = 0;
  // Step 2. Collect
  // data.=======================================================================================
  collectCaliSample();
  // Step 3. Calculate data collection progress.
  // ===============================================================
  if (sample_count < sample_size)
    SetZeroProgress =
        (sample_count <= 1) ? 0 : (sample_count - 1) / sample_size;
  // Step 4. If data collection is complete
  // ====================================================================
  else {
    for (int i = 0; i < 8; i++) {
      // if (isConnect[i]) {
      // Step 4.1. Calculate sensor's average reading and set it as zeros
      // ================================
      zeros[i] = sample_sum[i] / sample_size;
      // Step 4.2. Set current value to zeros
      // ============================================================
      ADCfilt[i] = zeros[i];
      // }
    }
    putDistanceZero(zeros, 8);
    // Step 4.3. Print Result
    // ===================================================================================
    String S = "[Dist]SetZeroComplete: ";
    for (int i = 0; i < 8; i++) {
      // if (isConnect[i])
      S += String(zeros[i], 3) + ",";
      // else
      //   S += "0,";
    }
    Serial.println(S);
    // Step 4.4. Calibration done. Reset all buffer.
    // ============================================================
    SetZeroProgress = 1;
    has_set_zero = false;
    sample_count = 0;
  }
}

float Flatness::calcIterativeCoefficientsB0(int id, float *sample, float *target, uint8_t size) {
  float k_final = 330000;
  float error = 0.0;
  float max_error = 0.0;
  float min_max_error = 1000.0;
  float dist;
  float learning_rate = 500;
  float decay_rate = 0.95;

  /* OLS start---------------------------------------------*/
  float sum_dist = 0.0, sum_target = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
  for (int i = 0; i < size; i++) {
    // slope dynamic in Iterative loop, so calculate here
    dist = (1 / sample[i] - 1 / zeros[id]);
    sum_dist += dist;
    sum_target += target[i];
    sum_xy += dist * target[i];
    sum_x2 += dist * dist;
  }
  float k_temp = (size * sum_xy - sum_dist * sum_target) /
                 (size * sum_x2 - sum_dist * sum_dist);
  float b_temp = (sum_target - k_temp * sum_dist) / size;
  // b must be 0 per calculate mode
  if (b_temp != 0) {
    k_temp = k_temp - (b_temp / sum_dist);
    b_temp = 0;
  }
  Serial.printf("k_temp:%f\n\r",k_temp);
  /* OLS end---------------------------------------------*/

  /* Manual Correction------------------------------------*/
  for (int iteration = 0; iteration < Iteration_size; iteration++) {
    // calculate error using k_temp
    error = 0.0;
    for (int i = 0; i < size; i++) {
      float predicted = (1 / sample[i] - 1 / zeros[id]) * k_temp;
      error += pow(predicted - target[i], 2);
      // only determined by max_error
      // if(max_error < error){max_error = error;max_index = i;}
    }
    error /= size;
    if (error > 0.1) {
      // determine adjust direction
      float predicted = (1 / sample[size - 1] - 1 / zeros[id]) * k_temp;
      if (predicted > target[size - 1]) {
        k_temp = k_temp - learning_rate * error;
      } else {
        k_temp = k_temp + learning_rate * error;
      }
      learning_rate *= decay_rate;
      if (error < min_max_error) {
        min_max_error = error;
        k_final = k_temp;
      }
      Serial.printf("[iteration]k_final:%f,min_max_error:%f\n",k_final,min_max_error);
    } else {
      min_max_error = error;
      k_final = k_temp;
      Serial.printf("[OK]k_final:%f,min_max_error:%f\n",k_final,min_max_error);
      break;
    }
  }
  return k_final;
}

void Flatness::sendToHost(float *data, int size) {
  if (data == nullptr) {
    return;
  }
  String tx_str = "<D";
  for (int i = 0; i < size; i++) {
    tx_str.concat(String(data[i], 2));
    if (i < size - 1) {
      tx_str += ",";
    }
  }
  tx_str += ">";
  Serial.print(tx_str);
}

void Flatness::sendToSub() {
  String tx_str = "<C";
  tx_str += ">";
  Serial.print(tx_str);
}

void Flatness::unpackFromHost(unsigned char info) {
  switch (unpack_step) {
    case STEP_FRAME_HEAD:
      if (info == '<') {
        unpack_step = STEP_PRASE_CMD;
      }
      break;
    case STEP_PRASE_CMD:
      if (info == 'C') {
        if (has_flat_cali == false) {
          has_flat_cali = true;
        }
        ESP_LOGE("MCOMM", "[Cali]STEP_PRASE_CMD:0x%X", info);
        unpack_step = STEP_FRAME_HEAD;
        // } else if (info == 'D') {
        // unpack_step = STEP_PRASE_DATA;
      } else {
        ESP_LOGE("MCOMM", "[ERROR]STEP_PRASE_CMD:0x%X", info);
        unpack_step = STEP_FRAME_HEAD;
      }
      break;
    default:
      ESP_LOGE("MCOMM", "[ERROR]unpack_step:%d", unpack_step);
      break;
  }
}

void Flatness::unpackFromSub(unsigned char info) {
  switch (unpack_step) {
    case STEP_FRAME_HEAD:
      if (info == '<') {
        unpack_step = STEP_PRASE_CMD;
      }
      break;
    case STEP_PRASE_CMD:
      if (info == 'D') {
        unpack_step = STEP_PRASE_DATA;
      } else {
        ESP_LOGE("MCOMM", "[ERROR]STEP_PRASE_CMD:0x%X", info);
        unpack_step = STEP_FRAME_HEAD;
      }
      break;
    case STEP_PRASE_DATA:
      if (info == '>') {
        String data_str;
        rx_sub_buffer[rx_sub_index] = '\0';
        rx_sub_index = 0;
        parseFromSub();
        unpack_step = STEP_FRAME_HEAD;
      } else {
        rx_sub_buffer[rx_sub_index] = info;
        rx_sub_index++;
        if (rx_sub_index > 9) {
          ESP_LOGE("MCOMM", "[ERROR]rx_sub_index:%d", rx_sub_index);
          rx_sub_index = 8;
        }
      }
      break;
    default:
      ESP_LOGE("MCOMM", "[ERROR]unpack_step:%d", unpack_step);
      break;
  }
}

void Flatness::parseFromSub() {
  char *index;
  index = strtok(rx_sub_buffer, ",");
  for (int i = 0; i < 8; i++) {
    if (index != NULL) {
      sub_distance[i] = atof(index);
    }
    index = strtok(NULL, ",");
  }
}

int Flatness::processMeasureFSM() {
  byte state = manage.flatness.measure.state;
  if (state == IDLE) {
    return state;
  }
  if (state == MEASURE_DONE || state == UPLOAD_DONE) {
    if (fabs(manage.flatness.flat_hold - manage.flatness.flat_live) > 20.0f) {
      return state = IDLE;
    }
    return state;
  }
  if (state == UNSTABLE) {
    manage.hold_flatness(manage.flatness.flat_live, 0);
    return state = MEASURE_DONE;
  }
  return state;
}