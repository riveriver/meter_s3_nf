#ifndef BLE_H
#define BLE_H
#include <Arduino.h>
#include "IMU42688.h"
#include "NimBLEDevice.h"
#include "MeterManage.h"
#include "Flatness.h"

extern Meter manage;

struct BLEState {
  uint8_t addr[6];        /** @brief BLE advertising address.*/
  uint8_t is_connect = false; /** @brief Indicate BLE connection status.*/
  uint8_t is_advertising = true; /** @brief BLE connsction or advertising status control.*/
};

class MyServerCallbacks : public BLEServerCallbacks {
public:
  BLEState *p_state;
  void onConnect(BLEServer *pServer, ble_gap_conn_desc* desc);
  void onDisconnect(BLEServer *pServer);

};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *pCharacteristic);
  void onWrite(NimBLECharacteristic *pCharacteristic);
};

class DeveloperCallbacks : public BLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic);
};

class BLE {
 public:
  BLEState state;
  void Init();
  void DoSwitch();
  void SendStatus(byte *Send);
  void SendAngle(float SendFloat);
  void SendSlope(float SendFloat);
  void SendFlatness(float SendFloat);
  void SendHome(byte *Send);
  void sendSyncInfo();
  void parseSyncInfo(int info);
  void parseDeveloperInfo(int info);
  bool QuickNotifyEvent();
  void ParseAngleCaliCmd(int info);
  void ParseFlatnessCali(int info);
  void ParseDebugMode(byte part,byte data);
  void ParseFlatCaliCmd(int info);
  void SlowNotifyEvent();

 private:
  bool DEBUG = 0;
  uint8_t pre_ble_state = true;
  unsigned long slow_sync = 0;
  BLEServer *pServer;
  BLECharacteristic *SlopeChar;
  BLECharacteristic *AngleChar;
  BLECharacteristic *HomeChar;
  BLECharacteristic *StatusChar;
  BLECharacteristic *ControlChar;
  BLECharacteristic *DeveloperChar;
  BLECharacteristic *FlatChar;
  MyServerCallbacks ServerCB;
  ControlCallbacks ControlCallback;
  DeveloperCallbacks DeveloperCallback;
};
#endif