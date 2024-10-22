#include <Arduino.h>
#include <Ticker.h>
#include "BLE.h"
#include "Battery.h"
#include "Button.h"
#include "Flatness.h"
#include "IMU42688.h"
#include "MeterUI.h"
#include "OnOff.h"
#include "SLED.h"
#include "MeterManage.h"
#include "CmdParser.h"
#include "BLE.h"
#include "CommProtocol.h"
extern BLE ble;
StringCmdParser cmd_parser(" ");
Meter manage;
Flatness flat;
IMU42688 imu;
OnOff on_off;
Button But;
MeterUI ui;
Battery Bat;
Ticker timer_button;

TaskHandle_t *ui_handle;
TaskHandle_t *flat_handle;
TaskHandle_t *angle_handle;
TaskHandle_t *comm_handle;
TaskHandle_t *firmware_handle;

void setup() {
  on_off.On(IO_Button0, IO_EN, manage.led, ui);
  ui.TurnOn();
  imu.init(IO_IMU_RX, IO_IMU_TX);
  flat.init();
  Bat.SetPin(IO_Battery);
  Bat.Update_BW();
  ui.pDS = &flat;
  ui.pIMU = &imu;
  ui.pBattry = &manage.battery;
  ui.pBLEState = &ble.state.addr[0];
  But.pDS = &flat;
  But.pIMU = &imu;
  But.pBLEState = &ble.state.addr[0];
  But.pSleepTime = &on_off;
  But.pUI = &ui;
  manage.initMeter();
  ui.Update();
  pinMode(IO_Button_LED, OUTPUT);
  digitalWrite(IO_Button_LED, LOW);
  while (digitalRead(IO_Button0)) {
  }

  xTaskCreatePinnedToCore(ui_task, "ui_task", 16384, NULL, 5, ui_handle,1);
  xTaskCreatePinnedToCore(flat_measure_task, "flat_measure_task", 16384, NULL, 4, flat_handle, 0);
  xTaskCreatePinnedToCore(angle_measure_task, "angle_measure_task", 16384, NULL, 4, angle_handle, 0);
  xTaskCreatePinnedToCore(comm_task, "comm_task", 8192, NULL, 3, comm_handle, 0);
  xTaskCreatePinnedToCore(firmware_task, "firmware_task", 8192, NULL, 2, firmware_handle, 0);
  attachInterrupt(digitalPinToInterrupt(IO_Button0), ButtonPress0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IO_Button1), ButtonPress1, FALLING);
  attachInterrupt(digitalPinToInterrupt(IO_Button2), ButtonPress2, FALLING);
#ifdef HARDWARE_3_0
  attachInterrupt(digitalPinToInterrupt(IO_Button3), ButtonPress3, FALLING);
  attachInterrupt(digitalPinToInterrupt(IO_Button4), ButtonPress4, FALLING);
#endif
  attachInterrupt(digitalPinToInterrupt(IO_Long_Button), ButtonPress5, RISING);
}

void loop() {}

static void comm_task(void *pvParameter) {
  ble.Init();
  // record
  cmd_parser.register_cmd(KEY_METER_CALI_FLAT,cmd_meter_flat_cali);
  cmd_parser.register_cmd(KEY_METER_FLAT_RECORD,cmd_meter_flat_record);
  cmd_parser.register_cmd(KEY_METER_CALI_ANGLE,cmd_meter_angle_cali);
  cmd_parser.register_cmd(KEY_METER_ANGLE_RECORD,cmd_meter_angle_record);
  // cmd
  cmd_parser.register_cmd(KEY_UI_PAGE,cmd_ui_page);
  cmd_parser.register_cmd(KEY_SYSTEM_TYPE,cmd_system_type);
  cmd_parser.register_cmd(KEY_SYSTEM_MODE,cmd_system_mode);
  // show
  cmd_parser.register_cmd(KEY_SYS_SHOW,cmd_meter_sys_show);
  cmd_parser.register_cmd(KEY_FLAT_SHOW,cmd_meter_flat_show);
  
  bool last_state = false;
  unsigned long slow_sync = millis();
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {

    /* ble */
    ble.DoSwitch();
    if (last_state != ble.state.is_connect) {
      last_state = ble.state.is_connect;
      ui.Block(last_state ? "Bluetooth Connect" : "Bluetooth Disconnect",3000);
    } 

    if(ble.state.is_connect == true && manage.measure.state == M_MEASURE_DONE){
      // app mode process
      byte app_home = manage.home_mode;
      if(app_home == HOME_AUTO){
        if(manage.auto_mode_select == HOME_AUTO_SLOPE){
          app_home = HOME_SLOPE;
        }else if(manage.auto_mode_select == HOME_AUTO_FLATNESS){
          app_home = HOME_FLATNESS;
        }
      }
      ble.SendHome(&app_home);
      ble.SendAngle(manage.clino.angle_hold);
      ble.SendSlope(manage.clino.slope_hold);
      char temp_show[4];
      dtostrf(manage.flat.flat_hold, 4, 1, temp_show);
      ble.SendFlatness(strtof(temp_show, NULL));
      manage.measure.state = M_UPLOAD_DONE;
      manage.clino.measure.state = M_UPLOAD_DONE;
      manage.flat.measure.state = M_UPLOAD_DONE;
      on_off.last_active = millis();
    }
    
    ble.QuickNotifyEvent();

    if(millis() - slow_sync > 60000){
      slow_sync = millis();
      Bat.Update_BW();
      ble.SlowNotifyEvent();
    }

    /* cli */
    while (Serial.available())
    {
      String rx_str = "";
      rx_str =  Serial.readStringUntil('\n');
      if(cmd_parser.parse(rx_str.c_str()) != 0){
        Serial.println("[E]" + rx_str);
      }
      on_off.last_active = millis();
    }
    
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 300);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("comm_task","Time Out!!!");
    }
  }
}

static void angle_measure_task(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) {
    /* imu update */
    if (imu.Update() == true) {
      switch (imu.cali_state)
      {
      case IMU_CALI_ZERO:
        imu.QuickCalibrate();
        break;
      case IMU_COMPLETE:
        imu.StopCali();
        ui.Block("Calibrate Complete", 2000);
        // HACK
        manage.page = PAGE_INFO;
        manage.cursor = 0;
        break;
      case IMU_FACTORY_ZERO:
        imu.CaliFactoryZero();
        break;     
      default:
        manage.clino.measure.state = imu.ProcessMeasureFSM();
        manage.flat.measure.state = flat.ProcessMeasureFSM();
        if(manage.home_mode == HOME_AUTO){
          if((manage.auto_angle > -95 && manage.auto_angle < -85) 
          || (manage.auto_angle > 85 && manage.auto_angle < 95)){
            manage.auto_mode_select = HOME_AUTO_SLOPE;
          }
          else{manage.auto_mode_select = HOME_AUTO_FLATNESS;}
        }
        manage.updateMeasure();
        break;
      }
    }

    manage.WarningLightFSM();
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 200);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("angle_measure_task","Time Out!!!");
    }
  }
}

static void flat_measure_task(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    flat.UpdateAllInOne();
    switch (manage.flat.state)
    {
      case FLAT_CALI_ZERO:
        // flat.CaliZero();
        break;
      case FLAT_CALI_COMPLETE:
        manage.flat.state = FLAT_COMMON;
        ui.Block("Calibrate Complete", 2000);
        manage.page = PAGE_HOME;
        manage.cursor = 0;
        break;
      case FLAT_FACTORY_ZERO:
        // flat.CaliFactoryZero();
        break;
      case FLAT_APP_CALI:
        // flat.doAppCali();
        break;
      case FLAT_ROBOT_ARM_CALI:
        flat.doRobotArmCali();
        break;
      default:
        flat.getFlatness();
        break;
    }
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 100);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("flat_measure_task","Time Out!!!");
    }  
  } 
}

static void firmware_task(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 300);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("firmware_task","Time Out!!!");
    }
    But.Update();
    digitalWrite(IO_Button_LED, But.CanMeasure());
    on_off.Off_Clock_Check();
  }
}

static void ui_task(void *pvParameter) {

  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 250);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("ui_task","Time Out!!!");
    }
    ui.Update();
  }
}

#define button_delay 3
void Press0_TimerCallback(){
  if (!digitalRead(IO_Button0)) {
    But.Press[0] = true;
    on_off.Off_Clock_Stop();
    on_off.last_active = millis();
  }else{
    on_off.Off_Clock_Start();
    on_off.last_active = millis();
  }
}
void Press1_TimerCallback(){
  if (digitalRead(IO_Button1)) {
    But.Press[1] = true;
    on_off.last_active = millis();
  }
}
void Press2_TimerCallback(){
  if (digitalRead(IO_Button2)) {
    But.Press[2] = true;
    on_off.last_active = millis();
  }
}
void ButtonPress0() {
  timer_button.once_ms(button_delay,Press0_TimerCallback);
}
void ButtonPress1() {
  timer_button.once_ms(button_delay,Press1_TimerCallback);
}
void ButtonPress2() {
  timer_button.once_ms(button_delay,Press2_TimerCallback);
}
void ButtonPress5() {
  But.Press[5] = true;
  on_off.last_active = millis();
}
#ifdef HARDWARE_3_0
void Press3_TimerCallback(){
  if (digitalRead(IO_Button3)) {
    But.Press[3] = true;
    on_off.last_active = millis();
  }
}
void Press4_TimerCallback(){
  if (digitalRead(IO_Button4)) {
    But.Press[4] = true;
    on_off.last_active = millis();
  }
}
void ButtonPress3() {
  timer_button.once_ms(button_delay,Press3_TimerCallback);
}
void ButtonPress4() {
  timer_button.once_ms(button_delay,Press4_TimerCallback);
}
#endif
