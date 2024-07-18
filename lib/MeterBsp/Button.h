/**
 * @file Button.h
 * @author Vicky Hung (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-01
 *
 * @copyright Wonder Construct (c) 2023
 *
 * @note
 */
#ifndef Button_H
#define Button_H

#include "Flatness.h"
#include "IMU42688.h"
#include "MeterUI.h"
#include "OnOff.h"
#include "MeterManage.h"
extern Meter manage;
#define BUTTON_DEAD_TIME 50
class Button
{
private:
  uint8_t dead_times = 0;
  int LastPress = 0;
public:
  uint8_t Cursor = 0;
  int brightness_select = 4;
  Flatness *pDS;
  IMU42688 *pIMU;
  uint8_t *pBLEState;
  OnOff *pSleepTime;
  MeterUI *p_ui;
  bool Press[6];
  void Update();
  bool CanMeasure();
  
};
#endif
