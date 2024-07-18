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
  // if (!Press[0] && !Press[1] && !Press[2] && !Press[3] && !Press[4] && !Press[5]) {
  //   return;
  // }

  if (millis() < manage.block_time) {
    memset(Press, false, sizeof(Press));
    return;
  }

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
    if (Press[1]) {
#ifdef HARDWARE_2_0
    manage.resetMeasure();
    manage.page = PAGE_BLE;
#else
    manage.page = PAGE_LIGHT_SWITCH;
#endif

    }
    // 切换测量模式
    else if (Press[2]) {
      manage.resetMeasure();
      manage.home_mode = (manage.home_mode + 1) & (manage.home_size - 1);
      manage.putMeterHome();
    }
    // 跳转蓝牙页面
    else if (Press[3]) {
      manage.resetMeasure();
      manage.page = PAGE_BLE;
    }
    // 跳转调零页面
    else if (Press[4]) {
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
    if (Press[1]) {*(pBLEState + 7) = true;manage.if_ble_switch = true;}
    else if (Press[2]) {*(pBLEState + 7) = false;manage.if_ble_switch = true;}
    else if (Press[3]) {manage.page = PAGE_HOME;}
    else if (Press[0]) {manage.page = PAGE_HOME;}    
  break;
/* ZERO_MENU */
  case PAGE_ZERO_MENU:
    if (Press[1])      {manage.cursor = (manage.cursor - 1 + 4) % 4;}
    else if (Press[2]) {manage.cursor = (manage.cursor + 1) % 4;}
#ifdef HARDWARE_2_0 
    else if (Press[0]) {
      if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_ANGLE){
        pIMU->cali_state = IMU_COMMON;
        manage.page = PAGE_ZERO_MENU + manage.cursor + 1;
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_FLAT){
        manage.flat_state = FLAT_COMMON; 
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
    else if (Press[3]) {manage.page = PAGE_HOME;manage.cursor = 0;}
    else if (Press[4]) {
      if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_ANGLE){
        pIMU->cali_state = IMU_COMMON;
        manage.page = PAGE_ZERO_MENU + manage.cursor + 1;
      }else if(PAGE_ZERO_MENU + manage.cursor + 1 == PAGE_ZERO_FLAT){
        manage.flat_state = FLAT_COMMON; 
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
        if (Press[1]) pIMU->yes_no = false;
        else if(Press[2]) pIMU->yes_no = true;
#ifdef HARDWARE_2_0 
        else if(Press[0] && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press[0] && pIMU->yes_no)pIMU->cali_state = IMU_CALI_ZERO;
#else
        else if(Press[3] ){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press[4] && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
        else if(Press[4] && pIMU->yes_no)pIMU->cali_state = IMU_CALI_ZERO;
#endif
    break;
    case 1:
        if(Press[3]){pIMU->StopCali();manage.page = PAGE_ZERO_MENU;}
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
        if (Press[1]) pIMU->yes_no = false;
        else if(Press[2]) pIMU->yes_no = true;
#ifdef HARDWARE_2_0 
        else if(Press[0] && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_HOME;}
        else if(Press[0] && pIMU->yes_no)pIMU->cali_state = IMU_FACTORY_ZERO;
#else
        else if(Press[3] ){pIMU->StopCali();manage.page = PAGE_HOME;}
        else if(Press[4] && !pIMU->yes_no){pIMU->StopCali();manage.page = PAGE_HOME;}
        else if(Press[4] && pIMU->yes_no)pIMU->cali_state = IMU_FACTORY_ZERO;
#endif
      break;
      case IMU_FACTORY_ZERO:
#ifdef HARDWARE_2_0 
      if(Press[3]){pIMU->StopCali();manage.page = PAGE_HOME;}
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
/* PAGE_ZERO_ANGLE */
//   case PAGE_ZERO_FLAT:
// #ifdef TYPE_500
//     p_ui->Block("[ERROR] TYPE_500",1000);
//     manage.page = PAGE_HOME;
//     manage.cursor = 0;
// #else
//   switch (manage.flat_state) {
//     case FLAT_COMMON:
//       if (Press[1]) pDS->yes_no = false;
//       else if(Press[2]) pDS->yes_no = true;
// #ifdef HARDWARE_3_0 
//       else if(Press[3] ){pDS->StopCali();manage.page = PAGE_ZERO_MENU;}
//       else if(Press[4] && !pDS->yes_no){pDS->StopCali();manage.page = PAGE_ZERO_MENU;}
//       else if(Press[4] && pDS->yes_no)manage.flat_state = FLAT_CALI_ZERO;
// #else
//       else if(Press[0] && !pDS->yes_no){pDS->StopCali();manage.page = PAGE_ZERO_MENU;}
//       else if(Press[0] && pDS->yes_no)manage.flat_state = FLAT_CALI_ZERO;
// #endif
//     break;
//     case 1:
//       if(Press[3]){pDS->StopCali();manage.page = PAGE_ZERO_MENU;}
//       else if(Press[0]){pDS->StopCali();manage.page = PAGE_ZERO_MENU;}
//     break;
//     default:
//       manage.flat_state = FLAT_COMMON;
//       manage.page = PAGE_ZERO_MENU;
//       manage.cursor = 0;
//     break;
//   }
// #endif
// break;
  case PAGE_ZERO_RESET:
switch (manage.reset_state) {
    case 0:
      if (Press[1]) manage.reset_yesno = false;
      else if(Press[2]) manage.reset_yesno = true;
#ifdef HARDWARE_2_0 
      else if(Press[0] && !manage.reset_yesno){manage.page = PAGE_HOME;}
      else if(Press[0] && manage.reset_yesno)manage.reset_state = 1;
#else
      else if(Press[3] ){manage.page = PAGE_HOME;}
      else if(Press[4] && !manage.reset_yesno){manage.page = PAGE_HOME;}
      else if(Press[4] && manage.reset_yesno)manage.reset_state = 1;
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
      p_ui->Block("Reset FactoryZero",1000);
      manage.reset_state = 0;
      manage.reset_progress = 0;
      manage.page = PAGE_HOME;
      manage.cursor = 0;
      break;
    default:
      p_ui->Block("E: ResetFactoryZero",1000);
      manage.reset_state = 0;
      manage.page = PAGE_HOME;
    break;
}
  break;
  case PAGE_INFO:
    if (Press[1])
      manage.cursor = (manage.cursor - 1 + 7) % 7;
    else if(Press[2])
      manage.cursor = (manage.cursor + 1) % 7;
    else if(Press[3]){
      manage.page = PAGE_HOME;
      manage.cursor = 0;
    }else if(Press[0]){
      manage.page = PAGE_HOME;
      manage.cursor = 0;
    }
    break;
  case PAGE_CALI_FLAT:
#ifdef HARDWARE_2_0
    if (Press[0]) {
      if (manage.dist_cali.step % 12 == 0) {
        manage.page = PAGE_HOME;
        manage.flat_state = FLAT_COMMON;
      } else if (manage.flat_state = FLAT_COMMON) {
        manage.cali_count++;
        manage.flat_state = FLAT_FIT_10;
      }
    }
#else  
    if (Press[4]) {
      if (manage.dist_cali.step % 12 == 0) {
        manage.page = PAGE_HOME;
        manage.flat_state = FLAT_COMMON;
      } else if (manage.flat_state== FLAT_COMMON) {
        manage.flat_state = FLAT_FIT_10;
      }
    }
#endif
    else if (Press[1]){
      manage.dist_cali.step++;
      manage.dist_cali.step %= 12;
    }
    else if (Press[2]){
      manage.dist_cali.step += 11;
      manage.dist_cali.step %= 12;
    }
  break;
// case PAGE_FLAT_FACTORY_ZERO:
// #ifdef TYPE_500
//     p_ui->Block("[ERROR] TYPE_500",1000);
//     manage.page = PAGE_HOME;
//     manage.cursor = 0;
// #else
//   switch (manage.flat_state) {
//     case FLAT_COMMON:
//       if (Press[1]) pDS->yes_no = false;
//       else if(Press[2]) pDS->yes_no = true;
// #ifdef HARDWARE_2_0 
//       else if(Press[0] && !pDS->yes_no){pDS->StopCali();manage.page = PAGE_HOME;}
//       else if(Press[0] && pDS->yes_no)manage.flat_state = FLAT_FACTORY_ZERO;
// #else
//       else if(Press[3] ){pDS->StopCali();manage.page = PAGE_HOME;}
//       else if(Press[4] && !pDS->yes_no){pDS->StopCali();manage.page = PAGE_HOME;}
//       else if(Press[4] && pDS->yes_no)manage.flat_state = FLAT_FACTORY_ZERO;
// #endif
//     break;
//     case FLAT_FACTORY_ZERO:
//       if(Press[3]){pDS->StopCali();manage.page = PAGE_HOME;}
//       else if(Press[0]){pDS->StopCali();manage.page = PAGE_HOME;}
//     break;
//     default:
//       manage.flat_state = FLAT_COMMON;
//       manage.page = PAGE_HOME;
//       manage.cursor = 0;
//     break;
//   }
//   #endif
//   break;
  default:
  if(Press[3]){manage.page = PAGE_HOME;manage.cursor = 0;}
  break;
}
  // clear button action
  memset(Press, false, sizeof(Press));
}

bool Button::CanMeasure() {
  if (manage.clino.measure.state == M_UNSTABLE)
    return false;
  else if (manage.clino.measure.state == M_MEASURING)
    return false;
  else if (manage.flatness.measure.state == M_UNSTABLE)
    return false;
  else if (manage.flatness.measure.state == M_MEASURING)
    return false;
  return true;
}

