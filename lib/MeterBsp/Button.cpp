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
 */
#include "Button.h"

void Button::Update() {
  // check
  // HACK 经常导致两次才能达成目的
  // if (!Press[0] && !Press_Up && !Press_Down && !Press_Back && !Press_Enter && !Press[5]) {
  //   return;
  // }

  if (millis() < manage.block_time) {
    memset(Press, false, sizeof(Press));
    return;
  }
  bool Press_Back  = Press[3];
  bool Press_Enter = Press[4];
  bool Press_Up    = Press[1];
  bool Press_Down  = Press[2];
  // process
  if (Press[5] && CanMeasure()) {
    if(manage.home_mode == HOME_ANGLE || manage.home_mode == HOME_SLOPE){
      manage.start_clino_measure();
    }else if(manage.home_mode == HOME_FLATNESS){
      manage.start_flatness_measure();
    }
    else if(manage.home_mode == HOME_SLOPE_FLATNESS){
      if(manage.auto_mode_select == HOME_AUTO_SLOPE){
      manage.start_clino_measure();
      }else if(manage.auto_mode_select == HOME_AUTO_FLATNESS){
      manage.start_flatness_measure();
      }else{
      manage.start_clino_measure();
      manage.start_flatness_measure();
      }
    }
    *(pBLEState + 8) = *(pBLEState + 6);
  }
switch (manage.page) {
/* HOME */
 case PAGE_HOME:
    // 开关预警模式
    if (Press_Up) {
#ifdef HARDWARE_2_0
    manage.resetMeasure();
    manage.page = PAGE_BLE;
#else
    manage.page = PAGE_LIGHT_SWITCH;
#endif

    }
    // 切换测量模式
    else if (Press_Down) {
      manage.resetMeasure();
      manage.home_mode = (manage.home_mode + 1) & (manage.home_size - 1);
      manage.putMeterHome();
    }
    // 跳转蓝牙页面
    else if (Press_Back) {
      manage.resetMeasure();
      manage.page = PAGE_BLE;
    }
    // 跳转调零页面
    else if (Press_Enter) {
      manage.resetMeasure();
      manage.cursor = 0;
      manage.page = PAGE_ZERO_MENU;
    }
    else if(Press[0]) {
      manage.resetMeasure();
      manage.cursor = 0;
#ifdef HARDWARE_2_0
  manage.page = PAGE_ZERO_MENU;
#endif
#ifdef FACTORY_TEST
  manage.page = PAGE_INFO;
#endif
#ifdef HARDWARE_1_0
  manage.page = PAGE_INFO;
#endif
    }
  break;
/* BLE */
  case PAGE_BLE:
    if (Press_Up) {*(pBLEState + 7) = true;manage.if_ble_switch = true;}
    else if (Press_Down) {*(pBLEState + 7) = false;manage.if_ble_switch = true;}
    else if (Press_Back) {manage.page = PAGE_HOME;}
    else if (Press[0]) {manage.page = PAGE_HOME;}    
  break;
/* ZERO_MENU */
  case PAGE_ZERO_MENU:
    if (Press_Up)      {manage.cursor = (manage.cursor - 1 + 4) % 4;}
    else if (Press_Down) {manage.cursor = (manage.cursor + 1) % 4;}
#ifdef HARDWARE_2_0 
    else if (Press[0]) {
      if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_ANGLE){
        pIMU->cali_state = IMU_COMMON;
        manage.page = PAGE_ZERO_MENU + manage.cursor + 1;
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_FLAT){
        manage.flat.state = FLAT_COMMON; 
        manage.page = PAGE_ZERO_MENU + manage.cursor + 1;    
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_RESET){
        manage.page = PAGE_ZERO_RESET;manage.cursor = 0;
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_HOME){
        manage.page = PAGE_HOME;manage.cursor = 0;
      }else{
        ESP_LOGE("cursor + 1","%d",manage.cursor);
      }
    }
#else
    else if (Press_Back) {manage.page = PAGE_HOME;manage.cursor = 0;}
    else if (Press_Enter) {
      if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_ANGLE){
        pIMU->cali_state = IMU_COMMON;
        manage.page = PAGE_ZERO_MENU + manage.cursor + 1;
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_FLAT){
        manage.flat.state = FLAT_COMMON; 
        manage.page = PAGE_ZERO_MENU + manage.cursor + 1;    
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_RESET){
        manage.page = PAGE_ZERO_RESET;manage.cursor = 0;
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_HOME){
        manage.page = PAGE_HOME;manage.cursor = 0;
      }else{
        ESP_LOGE("cursor + 1","%d",manage.cursor);
      }
    }
#endif
  break;
/* PAGE_ZERO_ANGLE */
  case PAGE_ZERO_ANGLE:
    switch (pIMU->cali_state) {
    case IMU_COMMON:
        if (Press_Up) pIMU->yes_no = false;
        else if(Press_Down) pIMU->yes_no = true;
#ifdef HARDWARE_2_0 
        else if(Press[0] && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press[0] && pIMU->yes_no)pIMU->cali_state = IMU_CALI_ZERO;
#else
        else if(Press_Back ){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press_Enter && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press_Enter && pIMU->yes_no)pIMU->cali_state = IMU_CALI_ZERO;
#endif
    break;
    case 1:
        if(Press_Back){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press[0]){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
    break;
    default:
        pIMU->cali_state = IMU_COMMON;
        manage.page = PAGE_ZERO_MENU;
      break;
    }
  break;
/* PAGE_ZERO_ANGLE */
  case PAGE_IMU_FACTORY_ZERO:
    switch (pIMU->cali_state) {
      case IMU_COMMON:
        if (Press_Up) pIMU->yes_no = false;
        else if(Press_Down) pIMU->yes_no = true;
#ifdef HARDWARE_2_0 
        else if(Press[0] && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_HOME;}
        else if(Press[0] && pIMU->yes_no)pIMU->cali_state = IMU_FACTORY_ZERO;
#else
        else if(Press_Back ){pIMU->StopCali();manage.page = PAGE_HOME;}
        else if(Press_Enter && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_HOME;}
        else if(Press_Enter && pIMU->yes_no)pIMU->cali_state = IMU_FACTORY_ZERO;
#endif
      break;
      case IMU_FACTORY_ZERO:
#ifdef HARDWARE_2_0 
      if(Press_Back){pIMU->StopCali();manage.page = PAGE_HOME;}
#else
      if(Press[0]){pIMU->StopCali();manage.page = PAGE_HOME;}
#endif
      break;
      default:
        pIMU->cali_state = IMU_COMMON;
        manage.page = PAGE_HOME;
      break;
    }
  break;
  case PAGE_ZERO_RESET:
switch (manage.reset_state) {
    case 0:
      if (Press_Up) manage.reset_yesno = false;
      else if(Press_Down) manage.reset_yesno = true;
#ifdef HARDWARE_2_0 
      else if(Press[0] && !manage.reset_yesno){manage.page = PAGE_HOME;}
      else if(Press[0] && manage.reset_yesno)manage.reset_state = 1;
#else
      else if(Press_Back ){manage.page = PAGE_HOME;}
      else if(Press_Enter && !manage.reset_yesno){manage.page = PAGE_HOME;}
      else if(Press_Enter && manage.reset_yesno)manage.reset_state = 1;
#endif
    break;
    case 1:
    if(manage.reset_progress == 0){
    pIMU->ResetFactoryZero();
// #ifndef TYPE_500
//     pDS->ResetFactoryZero();
// #endif
    }
    manage.reset_progress = manage.reset_progress + 20;
    if(manage.reset_progress > 90){manage.reset_progress  = 100;manage.reset_state = 2;}
    break;
    case 2:
      pUI->Block("Reset FactoryZero",1000);
      manage.reset_state = 0;
      manage.reset_progress = 0;
      manage.page = PAGE_HOME;
      manage.cursor = 0;
      break;
    default:
      pUI->Block("E: ResetFactoryZero",1000);
      manage.reset_state = 0;
      manage.page = PAGE_HOME;
    break;
}
  break;
  case PAGE_INFO:
    if (Press_Up)
      manage.cursor = (manage.cursor - 1 + 7) % 7;
    else if(Press_Down)
      manage.cursor = (manage.cursor + 1) % 7;
    else if(Press_Back){
      manage.page = PAGE_HOME;
      manage.cursor = 0;
    }else if(Press[0]){
      manage.page = PAGE_HOME;
      manage.cursor = 0;
    }
    break;
  case PAGE_CALI_FLAT:
    if(Press_Back){
      manage.page = PAGE_HOME;
      manage.cursor = 0;
    } 
    else if(Press_Enter) {
      if (manage.flat.state == FLAT_COMMON) {
        manage.flat.state = FLAT_ROBOT_ARM_CALI;
      }
    }
    else if (Press_Up){
      manage.flat.cali.step++;
      manage.flat.cali.step %= 11;
    }
    else if (Press_Down){
      manage.flat.cali.step += 10;
      manage.flat.cali.step %= 11;
    }
  break;
  case PAGE_CALI_ANGLE:
    if(Press_Back){
      manage.page = PAGE_HOME;
      manage.cursor = 0;
    } 
  break;
  default:
  if(Press_Back){manage.page = PAGE_HOME;manage.cursor = 0;}
  break;
}
  // clear button action
  memset(Press, false, sizeof(Press));
}

bool Button::CanMeasure() {
  if (manage.clino.measure.state == M_UNSTABLE)
    return false;
  else if (manage.clino.measure.state == M_MEASURE_ING)
    return false;
  else if (manage.flat.measure.state == M_UNSTABLE)
    return false;
  else if (manage.flat.measure.state == M_MEASURE_ING)
    return false;
  return true;
}

