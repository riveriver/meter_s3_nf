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
  // xTaskCreatePinnedToCore(SEND, "Core 1 SEND", 8192, NULL, 3, T_SEND, 1);''
  // xTaskCreatePinnedToCore(task_wifi, "Core 1 SEND", 8192, NULL, 3, T_SEND, 1);
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
    case FLAT_FIT_10:
      flat.doCalibration();
      break;
    case FLAT_FIT_5:
      // flat.CollectSample_Average();
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

#include <WiFi.h>
WiFiClient client;
// 设置了设备的本地IP地址为192.168.4.61
// 设备在网络中的唯一标识符，其他设备可以通过这个地址与之通信。
IPAddress loIP(192, 168, 4, 61);
// 子网掩码用于确定IP地址中哪些部分是网络地址，哪些部分是主机地址。
// 这里设置的子网掩码255.255.255.0意味着前三个八位组（192.168.4）是网络地址，
// 最后一个八位组（.61）是主机地址。这意味着同一子网内的所有设备的IP地址前三个八位组都相同。
IPAddress snIP(255, 255, 255, 0);
// 网关地址，用于确定网络中的其他设备可以通过哪个IP地址与网关通信。
// 设备用来向其他网络发送数据包的下一跳地址。
// 此处192.168.4.1通常是路由器的地址，所有的出站流量都会被路由到这个地址。
IPAddress gwIP(192, 168, 4, 1);
// Modbus TCP通信的服务器地址
IPAddress mbTCP(192, 168, 4, 51);
// 创建了一个监听端口6600的WiFi服务器。这意味着设备可以接收来自网络上其他设备的连接请求。
//一旦有客户端连接，服务器就可以处理这些连接，进行数据传输
WiFiServer server(6600); 

/*********************************************************************
*                          Modbus-TCP报文帧格式
* |-----------------MBAP报头------------------------|-----PDU-----|
* 读取报文：事务处理标识符[2]+协议符[2]+协议长度[2]+设备地址[1]+功能码[1]+起始地址[2]+读取数量[2] 
* 响应报文：事务处理标识符[2]+协议符[2]+协议长度[2]+设备地址[1]+功能码[1]+数据段长度[1]+数据[n] 
* eg. read msgs: xx xx xx xx xx 05 03 00 00 00 0A
**********************************************************************/
void modbusAckFsm(uint8_t* buff, int16_t buff_len)
{
  int pack_len;
  int start_addr;
  int read_num;
  // check device address
  if (buff[6] != 0x05)return;
  // check cmd
  switch (buff[7])
  {
    case 3:// read hold register
      start_addr = (buff[8] << 8) + buff[9];    // register address
      read_num = (buff[10] << 8) + buff[11];
      // 协议数据单元（PDU）长度是指从功能码开始到数据结束的长度
      // CMD[1] + ByteNum[2] + Data[n]
      // buff[5] = read_num * 2 + 3;
      uint16_t protocol_len = read_num * 2 + 3;
      buff[5] = protocol_len & 0xFF; // 获取低8位
      buff[4] = (protocol_len >> 8) & 0xFF; // 获取高8位
      //HACK 数据段长度,因为Modbus/TCP软件问题，n要乘以2，正确的报文不需要乘以2   
      buff[8] = read_num * 2;
      // 通信包长度：MBAP报头[7] + 功能码[1] + 数据段长度[1] + 数据段[n]
      pack_len = 9 + buff[8];
      // 填充数据
      if ( start_addr == 0)
      {
        for (int i = 0; i < buff[8]/2; i++)
        {
          buff[9 + (i * 2)] = 0x00;
          buff[10 + (i * 2)] = 0xA5;
        }
      }
      client.write(buff, pack_len);
      break;  
  }
}

uint8_t tcp_buff[64];
static void task_wifi(void *pvParameters) {
    // 应用网络配置
	  if (!WiFi.config(loIP, gwIP, snIP))
	  {
	    Serial.println("Satation配置不成功");
	    delay(3000);
	  }
	  WiFi.mode(WIFI_STA);
    String ap_ssid = "Meter_" + String(ESP.getEfuseMac(), HEX);
    // 启动WIFI连接
	  WiFi.begin(ap_ssid, "1234567890");
    // 启动服务器
    server.begin();	
  while(1){
    // 检查服务器是否有可用的客户端连接
    if (server.hasClient()) 
    {
      client = server.available();
      Serial.println("client connected");
    }	
    // 读取并处理客户端数据
    if ( client && client.connected())
    {
      int i = 0;
      while (client.available())
      {
        char c = client.read();
        tcp_buff[i] = c;
        i++;
      }
      if ( i > 0 )
      {
        modbusAckFsm(tcp_buff, i);
      }
    }
    ESP_LOGI("TASK_WIFI","WiFi.status() = %d",WiFi.status());
    vTaskDelay(1000/portTICK_PERIOD_MS);
  }
}

