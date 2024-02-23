/**
 * @file Button.cpp
 * @author Vicky Hung (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-01
 *
 * @copyright Wonder Construct (c) 2023
 *
 * @note
 * @todo
 * 1.change button logic from if/if/if to if/else if/else
 * 2.change back logic from back main to back upper level
 * 3.change " case 81:" from if to %
 * 4.think "Cursor < 2 + *(pBLEState + 9)
 *
 * Main OLED and Button distribution :
 *                 ________________
 *   SLED 0 -> o  |                | O  <- Button 1
 *   SLED 1 -> o  |    SSD 1306    |
 * Button 0 -> O  |________________| O  <- Button 2
 *
 * Side OLED and Button distribution :
 *  _________    ___
 * | CH 1115 |  /   \ <- Long Button ( Button 3 ) with Buttton LED
 * |_________|  \___/
 */

#include "Button.h"

void Button::Update() {
  // check
  if (!Press[0] && !Press[1] && !Press[2] && !Press[3]) {
    return;
  }

  if (millis() - LastPress < BUTTON_DEAD_TIME) {
    memset(Press, false, sizeof(Press));
    return;
  }
  if (millis() < manage.block_time) {
    memset(Press, false, sizeof(Press));
    return;
  }
  // process
  LastPress = millis();
  if (Press[3] && CanMeasure()) {
    if(manage.home_mode == 0 || manage.home_mode == 1){
      manage.start_clino_measure();
    }else if(manage.home_mode == 2){
      manage.start_flatness_measure();
    }
    else if(manage.home_mode == 3){
      manage.start_clino_measure();
      manage.start_flatness_measure();
    }
    *(pBLEState + 8) = *(pBLEState + 6);
  }
  switch (manage.page) {
    case 0:
        if (Press[0]) {
          pMeasure->Switch(false);
          *(pBLEState + 8) = false;
          manage.page = 1;
          manage.cursor = 0;
        } else if (Press[1]) {
          pMeasure->Switch(false);
          manage.page = 3;
          manage.cursor = 0;
        } else if (Press[2]) {
          pMeasure->Switch(false);
          manage.home_mode = manage.home_mode + 1;
          manage.home_mode = manage.home_mode % manage.home_size;
          manage.cursor = 0;
          manage.putMeterHome();
        }
        break;
    case 1:  // menu
      if (Press[0]) {
        manage.page = (manage.cursor == 0) ? (manage.page = 0) : manage.cursor + 1;
        manage.cursor = 0;
      } else if (Press[2])
        manage.cursor++;
      else if (Press[1])
        manage.cursor += 7;
      manage.cursor %= 8;
      break;
    // case 1:
    //   // angle calibration
    //   if (pIMU->CalibrateCheck == -1 && pIMU->Cursor == 0 && Press[0])
    //     manage.page = 0;
    //   else {
    //     if (Press[0]) pIMU->CalibrateSelect(0);
    //     if (Press[1]) pIMU->CalibrateSelect(1);
    //     if (Press[2]) pIMU->CalibrateSelect(2);
    //     if (pIMU->CalibrateCheck == 0 && Press[1]) pIMU->CalibrateSelect(3);
    //     if (pIMU->CalibrateCheck == 0 && Press[2]) pIMU->CalibrateSelect(4);
    //   }
    //   break;
        case 2:  // Flatness
          if (Press[0]) {
            switch (manage.cursor) {
              case 1:
                pDS->reset(true, true);
                manage.page = 0;
                manage.cursor = 0;
                break;
              case 2:
                pDS->Enable_Auto_Reset = !pDS->Enable_Auto_Reset;
                break;
              case 3:
                manage.page = 41;
                manage.cursor = 0;
                break;
              default:
                manage.page = 1;
                manage.cursor = 0;
                break;
            }
          }
          if (Press[2] && manage.cursor < 3) manage.cursor++;
          if (Press[1] && manage.cursor > 0) manage.cursor--;
          break;
        case 3:  // BLE
          if (Press[2]) *(pBLEState + 7) = false;
          if (Press[1]) *(pBLEState + 7) = true;
          if (Press[0]) {
            manage.page = 0;
            manage.cursor = 0;
          }
          break;
        case 4:  // angle calibration
          // angle calibration
          if (pIMU->CalibrateCheck == -1 && pIMU->Cursor == 0 && Press[0])
            manage.page = 0;
          else {
            if (Press[0]) pIMU->CalibrateSelect(0);
            if (Press[1]) pIMU->CalibrateSelect(1);
            if (Press[2]) pIMU->CalibrateSelect(2);
            if (pIMU->CalibrateCheck == 0 && Press[1]) pIMU->CalibrateSelect(3);
            if (pIMU->CalibrateCheck == 0 && Press[2]) pIMU->CalibrateSelect(4);
          }
          break;
        case 5:  // imu_calibration
          if (Press[0]) {
            if (manage.imu_cali.step % 8 == 0) {
              manage.page = 1;
              manage.imu_cali.status = 0;
            } else if (manage.imu_cali.step % 8 == 7) {
              manage.imu_cali.step = 0;
              manage.imu_cali.status = 0;
            } else if (manage.imu_cali.status == 0) {
              manage.imu_cali.status = 1;
            }
          } else if (Press[1])
            manage.imu_cali.step++;
          else if (Press[2])
            manage.imu_cali.step += 7;
          manage.imu_cali.step %= 8;
          break;
        case 6:  // detail
          if (Press[0]) {
            manage.page = 1;
            manage.cursor = 0;
          } else if (Press[2]) {
            manage.page = 63;
            manage.cursor = 0;
          } else if (Press[1]) {
            manage.page = 61;
            manage.cursor = 0;
          }
          break;
        case 7:
          if (Press[0]) {
            manage.page = 1;
            manage.cursor = 0;
          } else if (Press[2]) {
            manage.page = 1;
            manage.cursor = 0;
          } else if (Press[1]) {
            manage.page = 1;
            manage.cursor = 0;
          }
          break;
        case 8:
          if (Press[0]) {
            switch (manage.cursor) {
              case 0:
                manage.meter_type = 1;
                manage.home_mode = 0;
                manage.home_size = 2;
                manage.page = 0;
                break;
              case 1:
                manage.meter_type = 11;
                manage.home_mode = 2;
                manage.home_size = 4;
                manage.page = 0;
                break;
              case 2:
                manage.meter_type = 12;
                manage.home_mode = 3;
                manage.home_size = 4;
                manage.page = 0;
                break;
              // case 3:
              //   manage.meter_type = 12;
              //   manage.home_mode =  3;
              //   manage.home_size = 2;
              //   manage.page = 0;
              //   break;
              default:
                manage.cursor = 0;
                manage.page = 1;
                break;
            }
            manage.putMeterType();
          } else if (Press[1])
            manage.cursor--;
          else if (Press[2])
            manage.cursor++;
          manage.cursor %= 3;
          break;
        case 61:
          if (Press[0]) {
            manage.page = 1;
            manage.cursor = 0;
          } else if (Press[1])
            manage.page = 62;
          else if (Press[2])
            manage.page = 6;
          break;
        case 62:
          if (Press[0]) {
            manage.page = 1;
            manage.cursor = 0;
          } else if (Press[1])
            manage.page = 63;
          else if (Press[2])
            manage.page = 61;
          break;
        case 63:
          if (Press[0]) {
            manage.page = 1;
            manage.cursor = 0;
          } else if (Press[1])
            manage.page = 6;
          else if (Press[2])
            manage.page = 62;
          break;
        /*-----flatness------*/
        case 41:
          if (Press[0]) {
            switch (manage.cursor) {
              case 1:
                manage.page = 42;
                break;
              case 2:
                pDS->Enable_Cali_Slope = !pDS->Enable_Cali_Slope;
                pDS->Read_Slope_First = true;
                break;
              default:
                manage.page = 1;
                manage.cursor = 0;
                break;
            }
          }
          if (Press[1] && manage.cursor > 0) manage.cursor--;
          if (Press[2] && manage.cursor < 2) manage.cursor++;
          break;
        case 42:  // flatness calibration
          if (Press[0]) {
            if (manage.dist_cali.step % 12 == 0) {
              manage.page = 1;
              manage.dist_cali.status = 0;
            } else if (manage.dist_cali.status == 0) {
              manage.dist_cali.status = 1;
            }
          } else if (Press[1])
            manage.dist_cali.step++;
          else if (Press[2])
            manage.dist_cali.step += 11;
          manage.dist_cali.step %= 12;
          break;
        default :
          if (Press[0] || Press[1] || Press[2]) {
            manage.page = 1;
            manage.cursor = 0;
          }
          break;
  }
  // clear button action
  memset(Press, false, sizeof(Press));
}

bool Button::CanMeasure() {
  if (pIMU->fWarmUp != 100)
    return false;
  else if (pMeasure->State == pMeasure->Not_Stable)
    return false;
  else if (pMeasure->State == pMeasure->Measuring)
    return false;
  else if (pIMU->CalibrateCheck != -1)
    return false;
  return true;
}

