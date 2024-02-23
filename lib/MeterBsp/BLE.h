#ifndef BLE_H
#define BLE_H
#include <Arduino.h>
#include "IMU42688.h"
#include "Measure.h"
#include "NimBLEDevice.h"
// #include "RealTimeClock.h"
#include "SLED.h"
#include "MeterManage.h"

extern Meter manage;

struct BLEState {
  uint8_t Address[6];        /** @brief BLE advertising address.*/
  uint8_t isConnect = false; /** @brief Indicate BLE connection status.*/
  uint8_t OnOff = true; /** @brief BLE connsction or advertising status control.*/
  uint8_t Send_Info = false; /** @brief Waiting to send info.*/
  uint8_t ExpertMode = true; /** @brief Default false. True if BLE recieve
                                correct expert mode key.*/
};

class MyServerCallbacks : public BLEServerCallbacks {
public:
  BLEState *State;
  int *LastEdit;
  void onConnect(BLEServer *pServer);
  void onDisconnect(BLEServer *pServer);

};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *pCharacteristic,
                    ble_gap_conn_desc *desc, uint16_t subValue);
  void onWrite(NimBLECharacteristic *pCharacteristic);
};

class DeveloperCallbacks : public BLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic);
};

class BLE {
 public:
  BLEState State;
  String AddrStr = "";
  IMU42688 *pIMU;
  Measure *pMeasure;
  int rx_info;
  bool has_sync_rx;
  int *LastEdit;
  void Initialize(int &LastEdit, uint8_t *type);
  void Send(float *SendFloat);
  void DoSwich();
  void SendStatus(byte *Send);
  void SendAngle(float SendFloat);
  void SendFlatness(float SendFloat);
  void SendDistance(float *SendFloat);
  void SendHome(byte *Send);
  void sendSyncInfo();
  void parseSyncInfo();
  void parseDeveloperInfo(int info);
  void NotifyEvent();
  void ForwardToC3(int info);

 private:
  bool DEBUG = 0;
  BLEServer *pServer;
  BLECharacteristic *DeveloperChar;
  BLECharacteristic *HomeChar;
  BLECharacteristic *StatusChar;
  BLECharacteristic *AngleXChar;
  BLECharacteristic *AngleYChar;
  BLECharacteristic *AngleZChar;
  BLECharacteristic *DisChar[15];
  // BLECharacteristic *SetClkChar;
  // BLECharacteristic *SetUniChar;
  // BLECharacteristic *SetKeyChar;
  BLECharacteristic *ControlChar;
  MyServerCallbacks ServerCB;
  ControlCallbacks ControlCallback;
  DeveloperCallbacks DeveloperCallback;
  uint8_t Pre_OnOff = true;
  unsigned long slow_sync = 0;
  unsigned long medium_sync = 0;

};


#endif