#include <Arduino.h>
#include "BLE.h"
#include "Battery.h"
#include "Button.h"
#include "Flatness.h"
#include "IMU42688.h"
#include "Measure.h"
#include "OLED.h"
#include "OnOff.h"
#include "SLED.h"
#include "config.h"
#include "MeterManage.h"


const byte IO_Button2 = 0;
const byte IO_Battery = 1;
const byte IO_Long_Button = 2;
const byte IO_Button0 = 6;
const byte IO_Button1 = 7;
const byte IO_SCL2 = 4;
const byte IO_SDA2 = 5;
const byte IO_SDA1 = 8;
const byte IO_SCL1 = 9;
const byte IO_POGO_S_TX = 11;
const byte IO_POGO_S_RX = 12;
const byte IO_POGO_P_TX = 13;
const byte IO_POGO_P_RX = 14;
const byte IO_SPI2_DC = 15;  // MISO
const byte IO_RFID_RST = 16;
const byte IO_SD_CTRL = 17;
const byte IO_EN = 18;
const byte IO_SPI_MOSI = 19;
const byte IO_SPI_MISO = 20;
const byte IO_Button_LED = 21;
const byte IO_SPI2_SDA = 35;  // MOSI
const byte IO_SPI2_CLK = 36;
const byte IO_11V = 37;
const byte IO_SPI_CLK = 47;
const byte IO_SD_CS = 48;
const byte IO_IMU_RX = 38;
const byte IO_IMU_TX = 39;
#ifndef IO_LED
#define IO_LED 40;
#endif
#ifndef IO_OLED_RST
#define IO_OLED_RST 41
#endif
#ifndef CS1
#define CS1 10;
#endif
#ifndef CS2
#define CS2 42;
#endif

Meter manage;
Flatness flatness;
IMU42688 imu;
Measure measure;
SLED led;
OnOff Swich;
Button But;
OLED oled;
Battery Bat;
extern BLE ble;


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
int Flat_Period = 100;
int SEND_Period = 300;
int OLED_Period = 250;
int LOOP_Period = 300;

void setup() {
  Swich.On(IO_Button0, IO_EN, led, oled);
  oled.TurnOn(IO_11V);
  imu.fWarmUpTime = &Swich.LastEdit;
  imu.ExpertMode = &ble.State.ExpertMode;
  imu.Initialize(IO_IMU_RX, IO_IMU_TX);
  flatness.init();
  for(int i = 0;i < 3;i++){
    measure.SetInput(i,&(imu.AngleUser[i]));
    measure.SetStable(i,&(imu.AngleUser[1]));
  }
  measure.SetInput(3,&(flatness.Mm.Diff));
  measure.SetStable(3,&(flatness.Mm.Diff));
  Bat.SetPin(IO_Battery);
  Bat.Update();
  pinMode(IO_Button_LED, OUTPUT);
  digitalWrite(IO_Button_LED, LOW);
  oled.pDS = &flatness;
  oled.pMeasure = &measure;
  oled.pIMU = &imu;
  oled.pBatt = &manage.battery;
  oled.pBLEState = &ble.State.Address[0];
  oled.pLED = &led;
  But.pDS = &flatness;
  But.pMeasure = &measure;
  But.pIMU = &imu;
  But.pBLEState = &ble.State.Address[0];
  But.pSleepTime = &Swich;
  ble.pIMU = &imu;
  pinMode(IO_Button_LED, OUTPUT);
  digitalWrite(IO_Button_LED, LOW);
  while (digitalRead(IO_Button0)) {
  }
  led.Set(0, 0, 0, 4);
  led.Set(1, 0, 0, 4);
  led.Update();
  manage.initMeter();
  oled.Update();
  xTaskCreatePinnedToCore(I2C0, "Core 1 I2C0", 16384, NULL, 4, T_I2C0, 1);
  // xTaskCreatePinnedToCore(I2C1, "Core 1 I2C1", 16384, NULL, 4, T_I2C1, 1);
  xTaskCreatePinnedToCore(SEND, "Core 1 SEND", 8192, NULL, 3, T_SEND, 1);
  xTaskCreatePinnedToCore(User_Interface, "Core 0 Loop", 16384, NULL, 5, T_OLED,0);
  xTaskCreatePinnedToCore(UART, "Core 1 UART", 16384, NULL, 4, T_UART, 1);
  xTaskCreatePinnedToCore(LOOP, "Core 1 LOOP", 8192, NULL, 2, T_LOOP, 1);
  attachInterrupt(digitalPinToInterrupt(IO_Button0), ButtonPress0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IO_Button1), ButtonPress1, FALLING);
  attachInterrupt(digitalPinToInterrupt(IO_Button2), ButtonPress2, FALLING);
  attachInterrupt(digitalPinToInterrupt(IO_Long_Button), ButtonPress3, RISING);
}

void loop() {}

void ButtonPress0() {
  if (!digitalRead(IO_Button0)) {
    But.Press[0] = true;
    Swich.Off_Clock_Stop();
    Swich.LastEdit = millis();
    
  } else {
    Swich.Off_Clock_Start();
    Swich.LastEdit = millis();
  }
}

void ButtonPress1() {
  But.Press[1] = true;
  Swich.LastEdit = millis();
}

void ButtonPress2() {
  But.Press[2] = true;
  Swich.LastEdit = millis();
}

void ButtonPress3() {
  But.Press[3] = true;
  Swich.LastEdit = millis();
}

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
      ESP_LOGE("","[Warning] Task LOOP Time Out.");
    }
    But.Update();
    digitalWrite(IO_Button_LED, But.CanMeasure());
    Swich.Off_Clock_Check();
  }
}

static void User_Interface(void *pvParameter) {
  bool BLE_Status_Pre = false;
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, OLED_Period);
    if (BLE_Status_Pre != ble.State.isConnect) {
      BLE_Status_Pre = ble.State.isConnect;
      oled.Block(BLE_Status_Pre ? "Bluetooth Connect" : "Bluetooth Disconnect",5000);
    } else if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("","[Warning] Task OLED Time Out.");
    }
    oled.Update();
  }
}

static void SEND(void *pvParameter) {

  ble.Initialize(Swich.LastEdit, &manage.meter_type);

  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {

    Bat.Update_BW();
    ble.DoSwich();
    if(ble.State.isConnect == 1){
      // if(manage.clino.measure.state == MEASURE_DONE){
      //   ble.SendAngle(manage.clino.angle_hold);
      //   ble.SendFlatness(0);
      //   manage.clino.measure.state = UPLOAD_DONE;
      // }
      // if(manage.flatness.measure.state == MEASURE_DONE){
      //   ble.SendAngle(0);
      //   ble.SendFlatness(manage.flatness.flat_hold);
      //   manage.flatness.measure.state = UPLOAD_DONE;
      // }
      if(manage.measure.state == MEASURE_DONE){
        ble.SendAngle(manage.clino.angle_hold);
        ble.SendFlatness(manage.flatness.flat_hold);
        manage.measure.state = UPLOAD_DONE;
        manage.clino.measure.state = UPLOAD_DONE;
        manage.flatness.measure.state = UPLOAD_DONE;
      }
      ble.NotifyEvent();
    }
    
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, SEND_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("","[Warning] Task SEND Time Out.");
    }
  }
}


static void I2C0(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    flatness.update(0);
    xLastWakeTime = xTaskGetTickCount();
    xTaskDelayUntil(&xLastWakeTime, 100);
    flatness.update(1);
    xLastWakeTime = xTaskGetTickCount();
    xTaskDelayUntil(&xLastWakeTime, 100);
  }
}

// static void I2C1(void *pvParameter) {
//   BaseType_t xWasDelayed;
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   for (;;) {
//     xLastWakeTime = xTaskGetTickCount();
//     xWasDelayed = xTaskDelayUntil(&xLastWakeTime, 200);
//     if (!xWasDelayed && millis() > 10000){
//       ESP_LOGE("","[Warning] Task I2C1 Time Out.");
//     }
//     flatness.update(1);
//   }
// }

static void UART(void *pvParameter) {
  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xLastWakeTime = xTaskGetTickCount();
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, IMU_Period);
    if (!xWasDelayed && millis() > 10000){
      ESP_LOGE("","[Warning] Task UART Time Out.");
    }
    /* imu update */
    if (imu.Update() == imu.IMU_Update_Success) {
      measure.DataIsUpdte(0,3);
      // imu.send_to_salve.imu_cali_step   = manage.imu_cali.step;
      // imu.send_to_salve.imu_cali_status = manage.imu_cali.step;
      // imu.SendTOSlave(&imu.send_to_salve);
      if (imu.CalibrateCheck == 1){
        imu.Calibrate();
      }
      if (imu.CalibrateCheck == 2)
      {
        oled.Block((imu.Cursor == 2) ? "Calibration Data Clear" : "Calibrate Complete", 3000);
        imu.CalStop();
      }
    }    
    manage.clino.measure.state = imu.processMeasureFSM();
    manage.flatness.measure.state = flatness.processMeasureFSM();
    manage.updateSystem();
  }
}
