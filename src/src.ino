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
StringCmdParser cmd_parser(",");
Meter manage;
Flatness flat;
IMU42688 imu;
OnOff on_off;
Button But;
MeterUI ui;
Battery Bat;
// TODO 6-->1
Ticker timer_button[6];


/*****************************************************************************************************/
/**
 @brief FreeRTOS task code

 Core 0 : UI
  SPI : OLED (SSD1306, CH1115)
  Calculation : \b Button::Update()
 Core 1 :
  I2C : Wire 0 : 0x48 ~ 0x4b ads1115 (Distance sensor)
        Wire 1 : 0x48 ~ 0x4a ads1115 (Distance sensor) + 0x68 DS3231 (Real Time
 Clock) imu_task : Serial 1 : Esp32-c3 communication (IMU) Serial 2 : Esp32-s3
 communication (POGO Pin Communication) BLE : Server. adc : Battery input. Other
 : LED (LLC2842) Calculation : \b Measure::Update()
*/
/*****************************************************************************************************/

TaskHandle_t *T_OLED;
TaskHandle_t *T_I2C0;
TaskHandle_t *T_I2C1;
TaskHandle_t *T_UART;
TaskHandle_t *T_SEND;
TaskHandle_t *T_LOOP;

int IMU_Period  = 300;
int Flat_Period = 200;
int SEND_Period = 300;
int OLED_Period = 250;
int LOOP_Period = 300;

static void comm_task(void *pvParameter) {
#ifdef COMM_CLI
  cmd_parser.register_cmd(KEY_METER_CALI_FLAT,cmd_robot_cali_flat);
  cmd_parser.register_cmd(KEY_METER_CALI_ANGLE,cmd_meter_cali_angle);
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 300);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_SEND","Time Out!!!");
    }
    // read a row of data
    String rx_str = "";
    while (Serial.available())
    {
      rx_str =  Serial.readStringUntil('\n');
    }
    if (rx_str.length() > 0){
      cmd_parser.parse(rx_str.c_str()); // call command parser
    }
  }
#else
  ble.Init();
  bool last_state = false;
  unsigned long slow_sync = millis();
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    ble.DoSwich();
    if (last_state != ble.state.is_connect) {
      last_state = ble.state.is_connect;
      ui.Block(last_state ? "Bluetooth Connect" : "Bluetooth Disconnect",5000);
    } 

    if(ble.state.is_connect == true && manage.measure.state == M_MEASURE_DONE){
      byte app_home = manage.home_mode;
      if(app_home == HOME_SLOPE_FLATNESS){
        if(manage.auto_mode_select == HOME_AUTO_SLOPE){
          app_home = HOME_SLOPE;
        }else if(manage.auto_mode_select == HOME_AUTO_FLATNESS){
          app_home = HOME_FLATNESS;
        }
      }
      ble.SendHome(&app_home);
      ble.SendAngle(manage.clino.angle_hold);
      ble.SendFlatness(manage.flat.flat_hold);
      manage.measure.state = M_UPLOAD_DONE;
      manage.clino.measure.state = M_UPLOAD_DONE;
      manage.flat.measure.state = M_UPLOAD_DONE;
      on_off.last_active = millis();
    }
    
    if(ble.QuickNotifyEvent()){
      on_off.last_active = millis();
    }
    if(millis() - slow_sync > 60000){
      slow_sync = millis();
      Bat.Update_BW();
      ble.SlowNotifyEvent();
    }

    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, SEND_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_SEND","Time Out!!!");
    }
  }
#endif 
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
      flat.calculateFlatness();
      break;
    }
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, Flat_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("","Task flat_measure_task Time Out.");
    }  
  } 
}

void setup() {
  on_off.On(IO_Button0, IO_EN, manage.led, ui);
  ui.TurnOn();
  imu.Initialize(IO_IMU_RX, IO_IMU_TX);
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
  But.p_ui = &ui;
  ble.pIMU = &imu;
  ble.pFlat = &flat;
  pinMode(IO_Button_LED, OUTPUT);
  digitalWrite(IO_Button_LED, LOW);
  while (digitalRead(IO_Button0)) {
  }
  manage.initMeter();
  Bat.Update_BW();
  ui.Update();
  
  xTaskCreatePinnedToCore(flat_measure_task, "Core 1 flat_measure_task", 16384, NULL, 4, T_I2C0, 0);
  xTaskCreatePinnedToCore(comm_task, "Core 1 comm_task", 8192, NULL, 3, T_SEND, 0);
#ifdef UI_ON
  xTaskCreatePinnedToCore(User_Interface, "Core 0 UI", 16384, NULL, 5, T_OLED,1);
#endif
  xTaskCreatePinnedToCore(imu_task, "Core 1 imu_task", 16384, NULL, 4, T_UART, 0);
  xTaskCreatePinnedToCore(LOOP, "Core 1 LOOP", 8192, NULL, 2, T_LOOP, 0);
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

// TODO AllCallback in one function
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
  timer_button[0].once_ms(button_delay,Press0_TimerCallback);
}
void ButtonPress1() {
  timer_button[1].once_ms(button_delay,Press1_TimerCallback);
}
void ButtonPress2() {
  timer_button[2].once_ms(button_delay,Press2_TimerCallback);
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
  timer_button[3].once_ms(button_delay,Press3_TimerCallback);
}
void ButtonPress4() {
  timer_button[4].once_ms(button_delay,Press4_TimerCallback);
}
#endif
/**
 * @brief
 * @param [in] pvParameter
 */

static void LOOP(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, LOOP_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_LOOP","Time Out!!!");
    }
    But.Update();
    digitalWrite(IO_Button_LED, But.CanMeasure());
    on_off.Off_Clock_Check();
  }
}

static void User_Interface(void *pvParameter) {

  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, OLED_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_UI","Time Out!!!");
    }
    ui.Update();
  }
}

static void imu_task(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
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
        manage.page = PAGE_HOME;
        manage.cursor = 0;
        break;
      case IMU_FACTORY_ZERO:
        imu.CaliFactoryZero();
        break;     
      default:
        manage.clino.measure.state = imu.ProcessMeasureFSM();
        break;
      }
    }
    manage.flat.measure.state = flat.ProcessMeasureFSM();
    if(manage.home_mode == HOME_SLOPE_FLATNESS){
      if((manage.auto_angle > -95 && manage.auto_angle < -85) 
      || (manage.auto_angle > 85 && manage.auto_angle < 95)){
        manage.auto_mode_select = HOME_AUTO_SLOPE;
      }
      else{manage.auto_mode_select = HOME_AUTO_FLATNESS;}
    }
    manage.updateSystem();
    // manage.WarningLightFSM();
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, IMU_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_UART","Time Out!!!");
    }
  }
}
