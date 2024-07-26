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

Meter manage;
Flatness flat;
IMU42688 imu;
OnOff on_off;
Button But;
MeterUI ui;
Battery Bat;
extern BLE ble;
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
 Clock) UART : Serial 1 : Esp32-c3 communication (IMU) Serial 2 : Esp32-s3
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
  
  #ifndef TYPE_500
    xTaskCreatePinnedToCore(I2C0, "Core 1 I2C0", 16384, NULL, 4, T_I2C0, 1);
  #endif
  xTaskCreatePinnedToCore(SEND, "Core 1 SEND", 8192, NULL, 3, T_SEND, 1);
  // xTaskCreatePinnedToCore(commArmTask, "Core 1 SEND", 8192, NULL, 3, T_SEND, 1);
  xTaskCreatePinnedToCore(User_Interface, "Core 0 Loop", 16384, NULL, 5, T_OLED,0);
  xTaskCreatePinnedToCore(UART, "Core 1 UART", 16384, NULL, 4, T_UART, 1);
  xTaskCreatePinnedToCore(LOOP, "Core 1 LOOP", 8192, NULL, 2, T_LOOP, 1);
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

#ifdef BLE
static void SEND(void *pvParameter) {

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
      ble.SendFlatness(manage.flatness.flat_hold);
      manage.measure.state = M_UPLOAD_DONE;
      manage.clino.measure.state = M_UPLOAD_DONE;
      manage.flatness.measure.state = M_UPLOAD_DONE;
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
}
#else
uint8_t auto_cali_data[4] = {0xA5,0x01,0x03,0xF5};
static void SEND(void *pvParameter) {

  bool last_state = false;
  unsigned long slow_sync = millis();
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    Serial.write(auto_cali_data, sizeof(auto_cali_data));
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 1000);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("TASK_SEND","Time Out!!!");
    }
  }
}
#endif
static void I2C0(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    flat.UpdateAllInOne();
    switch (manage.flat_state)
    {
    case FLAT_CALI_ZERO:
      // flat.CaliZero();
      break;
    case FLAT_CALI_COMPLETE:
      manage.flat_state = FLAT_COMMON;
      ui.Block("Calibrate Complete", 2000);
      manage.page = PAGE_HOME;
      manage.cursor = 0;
      break;
    case FLAT_FACTORY_ZERO:
      // flat.CaliFactoryZero();
      break;
    case FLAT_APP_CALI:
      flat.doAppCali();
      break;
    default:
      flat.CalculateFlatness();
      break;
    }
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, Flat_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("","Task I2C0 Time Out.");
    }  
  } 
}

static void UART(void *pvParameter) {
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
    manage.flatness.measure.state = flat.ProcessMeasureFSM();
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

// /*
//   Modbus-Arduino Example - Master Modbus IP Client (ESP8266/ESP32)
//   Read Holding Register from Server device

//   (c)2018 Alexander Emelianov (a.m.emelianov@gmail.com)
//   https://github.com/emelianov/modbus-esp8266
// */

// #include <WiFi.h>
// #include <ModbusIP_ESP8266.h>

// const int REG = 102;               // Modbus Hreg Offset
// IPAddress remote(192, 168, 57, 2);  // Address of Modbus Slave device
// const int LOOP_COUNT = 10;
// uint16_t res = 5;
// uint8_t show = LOOP_COUNT;
// ModbusIP mb;  //ModbusIP object

// static void commArmTask(void *pvParameter) {
  
//   WiFi.begin("Wifi-7628-15B0", "12345678");
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     ESP_LOGE("", "Connecting to WiFi...");
//   }
//   ESP_LOGE("", "WIFI_Connecting:%c",WiFi.localIP());
//   mb.client();
//   BaseType_t xWasDelayed;
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   for (;;) {
//     if (mb.isConnected(remote)) {   // Check if connection to Modbus Slave is established
//       mb.readHreg(remote, REG, &res);  // Initiate Read Coil from Modbus Slave
//     } else {
//       mb.connect(remote);           // Try to connect if no connection
//     }
//     mb.task();                      // Common local Modbus task
//     delay(100);                     // Pulling interval
//     if (!show--) {                   // Display Slave register value one time per second (with default settings)
//       ESP_LOGE("","res:%d",res);
//       show = LOOP_COUNT;
//     }
//     xLastWakeTime = xTaskGetTickCount();
//     xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 10);
//     if (!xWasDelayed && millis() > 10000){
//       ESP_LOGE("commArmTask","Time Out!!!");
//     }
//   }
// }


/* esp32Modbus

Copyright 2018 Bert Melis

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/*

The modbus server (= SMA Sunny Boy) is defined as
ModbusTCP sunnyboy(3, {192, 168, 123, 123}, 502);
where:
- 3 = device ID
- {192, 168, 123, 13} = device IP address
- 502 = port number

All defined registers are holding registers, 2 word size (4 bytes)

*/


#include <Arduino.h>
#include <WiFi.h>
#include <esp32ModbusTCP.h>

const char* ssid = "Wifi-7628-15B0";
const char* pass = "12345678";
bool WiFiConnected = false;

esp32ModbusTCP sunnyboy(3, {192, 168, 57, 2}, 502);
enum smaType {
  ENUM,   // enumeration
  UFIX0,  // unsigned, no decimals
  SFIX0,  // signed, no decimals
};
struct smaData {
  const char* name;
  uint16_t address;
  uint16_t length;
  smaType type;
  uint16_t packetId;
};
smaData smaRegisters[] = {
  "status", 30102, 2, ENUM, 0,
};
uint8_t numberSmaRegisters = sizeof(smaRegisters) / sizeof(smaRegisters[0]);
uint8_t currentSmaRegister = 0;


// void setup() {
//     WiFi.disconnect(true);  // delete old config
//     delay(1000);
//     WiFi.begin(ssid, pass);
//     Serial.println();
//     Serial.println("Connecting to WiFi... ");
// }

// void loop() {
//   static uint32_t lastMillis = 0;
//   if ((millis() - lastMillis > 30000 && WiFiConnected)) {
//     lastMillis = millis();
//     Serial.print("reading registers\n");
//     for (uint8_t i = 0; i < numberSmaRegisters; ++i) {
//       uint16_t packetId = sunnyboy.readHoldingRegisters(smaRegisters[i].address, smaRegisters[i].length);
//       if (packetId > 0) {
//         smaRegisters[i].packetId = packetId;
//       } else {
//         Serial.print("reading error\n");
//       }
//     }
//   }
// }

static void commArmTask(void *pvParameter) {

  WiFi.disconnect(true);  // delete old config
  delay(1000);
  WiFi.begin("Wifi-7628-15B0", "12345678");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    ESP_LOGE("", "Connecting to WiFi...");
  }
  ESP_LOGE("", "WIFI_Connecting:%c",WiFi.localIP());
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
  static uint32_t lastMillis = 0;
  if ((millis() - lastMillis > 30000 && WiFiConnected)) {
    lastMillis = millis();
    Serial.print("reading registers\n");
    for (uint8_t i = 0; i < numberSmaRegisters; ++i) {
      uint16_t packetId = sunnyboy.readHoldingRegisters(smaRegisters[i].address, smaRegisters[i].length);
      if (packetId > 0) {
        smaRegisters[i].packetId = packetId;
      } else {
        Serial.print("reading error\n");
      }
    }
  }
  }
}
