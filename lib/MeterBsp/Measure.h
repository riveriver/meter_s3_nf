#ifndef Measure_H
#define Measure_H
#include <Arduino.h>
#define ANGLE_AXIS 1
#define DIST_DIFF 3
#include <MaxMin.h>
#include "MeterManage.h"
#define Sensor_ID 4
extern Meter manage;
class Measure {
 public:
  const byte Sleep = 0;
  const byte Not_Stable = 1;
  const byte Measuring = 2;
  const byte Measure_Done = 3;
  const byte Not_Update = 4;
  const byte Send_Done = 6;
  byte State = Sleep;
  byte MeasurePercent = 0;   /** @brief Measurement data collect percentage.*/
  bool hasUpdate[Sensor_ID]; /** @brief Bool array to indicate if the data had
                                been update.*/
  float Result[Sensor_ID] = {0}; /** @brief Measurement result output*/
  void Switch(bool OnOff) {
    // 开始测量
    if (OnOff &&
        (State == Sleep || State == Measure_Done || State == Send_Done)) {
      State = Not_Stable;
    }
    // 停止测量
    if (!OnOff) {
      for (int i = 0; i < Sensor_ID; i++) {
        MeasureCount[i] = 0;
        Measure[i] = 0;
        StableCount[i] = 0;
        StableStart[i] = (Stable[i]) ? *Stable[i] : 0;
        // HACK 关灯
      }
      State = Sleep;
    }
  }
  byte AccurateMeasure() {
    if (manage.speed_mode != 2 && State == Sleep) {
      return State;
    }
    // flatness mode
    if (manage.home_mode == 2 && manage.has_update_dist == true) {
      manage.has_update_dist == false;
      return State = AccurateMeasureFlat();
    };
    // angle mode all in here
    if ((manage.home_mode == 0 || manage.home_mode == 1 ||
         manage.home_mode == 3) &&
         hasUpdate[ANGLE_AXIS] == true) {
      hasUpdate[ANGLE_AXIS] == false;
      State = StableMeasure(ANGLE_AXIS);
    }
    return State;
  }
  /**
   * @brief Indicete the data update situcation.
   *
   * @param Start Poisition of the float in the array that had been update
   * successifully.
   * @param Length Number of float that had been update successiflly
   */
  void DataIsUpdte(int Start, int Length) {
    for (int i = 0; i < Length; i++) {
      hasUpdate[Start + i] = true;
    }
  }
  void SetInput(int id, float *pSource) {
    Input[id] = pSource;
  }
  void SetStable(int id, float *pSource) {
    Stable[id] = pSource;
  }

 private:
  const bool Print_Measure_Result =
      false; /** @brief True to print result for debug.*/
  const bool Print_Stable_Situation =
      false; /** @brief True to print stabilization situation for debug.*/
  const byte StableCountNum =
      5; /** @brief Require data length for stabilization identification .*/
  const float Stable_Default_Value =
      1000; /** @brief Default stabilization identification reference
               position.*/
  float *Input[Sensor_ID]; /** @brief Array of pointer to the measure data.*/
  byte StableCount[Sensor_ID] = {
      0}; /** @brief Stabilization identification Data collection counter.*/
  float StableStart[Sensor_ID] = {
      0}; /** @brief Sensor stabilization identification reference position.*/
  float MeasureStart[Sensor_ID] = {0};
  float Stable_TH[Sensor_ID] = {
      0}; /** @brief Sensor stabilization identification threshold.*/
  float *Stable[Sensor_ID]; /** @brief Array of pointer to the data for identify
                               the stabalization of the sensors.*/                          
  byte MeasureCount[Sensor_ID] = {
      0}; /** @brief Measure Data collection counter.*/
  byte MeasureCountNum[Sensor_ID] = {
      15}; /** @brief Require data length for measurement.*/
  float Measure_TH[Sensor_ID] = {
      0}; /** @brief Sensor moving identification threshold.*/
  float Measure[Sensor_ID] = {0}; /** @brief Measure buffer.*/

  void SetParam(int num, uint8_t mode) {
    if (num < 3) {
      switch (mode) {
        case 0:
          MeasureCountNum[num] = 5;
          Stable_TH[num] = 5.0f;
          Measure_TH[num] = 0.05f;
          break;
        case 1:
          MeasureCountNum[num] = 5;
          Stable_TH[num] = 5.0f;
          Measure_TH[num] = 0.1f;
          break;
        case 2:
          MeasureCountNum[num] = 5;
          Stable_TH[num] = 5.0f;
          Measure_TH[num] = 0.2f;
          break;
        default:
          ESP_LOGE("USER", "[SetParam]ERROR!!!");
          break;
      }
    } else {
      ESP_LOGE("USER", "SetParam:num %d\n", num);
    }
  }

  bool CheckQuitMeasure(byte i) {
    // check if exceed stable threshold
    if (fabs(*Input[i] - MeasureStart[i]) > Measure_TH[i]) {
      MeasureStart[i] = (Input[i]) ? *Input[i] : 0;
      return true;
    }
    return false;
  }

  bool CheckExitDone(byte i) {
    // check if exceed stable threshold
    if (fabs(*Stable[i] - StableStart[i]) > Stable_TH[i]) {
      StableStart[i] = (Stable[i]) ? *Stable[i] : 0;
      return true;
    }
    return false;
  }

  /* 0:OK 1:ERROR*/
  uint8_t StableMeasure(byte i) {
    // set measure param
    SetParam(i, manage.speed_mode);
    // measure done,show result.if check large motion,show live angle
    if (State == Measure_Done || State == Send_Done) {
      if (CheckExitDone(i)) {
        MeasureCount[i] = 0;
        Measure[i] = 0;
        StableCount[i] = 0;
        StableStart[i] = (Stable[i]) ? *Stable[i] : 0;
        State = Sleep;
      }
      return State;
    }
    // measure
    if (!CheckQuitMeasure(i)) {
      // Measure once
      Measure[i] += (Input[i]) ? *Input[i] : 0;
      MeasureCount[i]++;
      MeasurePercent = (MeasureCount[i] * 100 / MeasureCountNum[i]);
      // If measure complete
      if (MeasureCount[i] == MeasureCountNum[i]) {
        manage.hold_clino(Measure[i] / MeasureCountNum[i],manage.clino.arrow_live);
        // HACK 开灯
        MeasureCount[i] = 0;
        Measure[i] = 0.0f;
        StableCount[i] = 0;
        StableStart[i] = (Stable[i]) ? *Stable[i] : 0;
        return Measure_Done;
      }
      return Measuring;
    } else {
      MeasureCount[i] = 0;
      Measure[i] = 0.0f;
      StableCount[i] = 0;
      StableStart[i] = (Stable[i]) ? *Stable[i] : 0;
      return Not_Stable;
    }
  }

  int AccurateMeasureFlat() {
    // measure done,show result.if check large motion,show live angle
    if (State == Measure_Done || State == Send_Done) {
      if (fabs(manage.flatness.flat_hold - manage.flatness.flat_live) > 20.0f) {
        return State = Sleep;
      }
      return State;
    }
    if(State == Not_Stable){
      manage.hold_flatness(manage.flatness.flat_live,0);
      return State = Measure_Done;
    }
    return State;
  }
};

#endif