#include "BLE.h"
BLE ble;
void BLE::Init() {
  // Start BLE Deviec ----------------------------------------
  BLEDevice::init("Ensightful");
  std::string name_char;
  if (manage.meter_type == 1) {
    name_char = "Ensightful_500_";
  } else if (manage.meter_type == 11) {
    name_char = "Ensightful_1000_";
  } else if (manage.meter_type == 12) {
    name_char = "Ensightful_2000_";
  } else {
    name_char = "Ensightful_";
  }
  std::string macAddress =
      BLEDevice::getAddress().toString();
  name_char =
      name_char + macAddress.substr(macAddress.length() - 5);;
  BLEDevice::setDeviceName(name_char.c_str()); 
  memcpy(state.addr, BLEDevice::getAddress().getNative(), sizeof(state.addr));

  // Create Server --------------------------------------------
  BLEDevice::setMTU(512);
  pServer = BLEDevice::createServer();
  // Create Address -------------------------------------------
  BLEUUID ServiceUUID("0000abcd-00f1-0123-4567-0123456789ab");
  BLEUUID SlopeUUID("0000a002-00f1-0123-4567-0123456789ab");
  BLEUUID AngleUUID("0000a003-00f1-0123-4567-0123456789ab");
  BLEUUID StatusUUID("0000a004-00f1-0123-4567-0123456789ab");
  BLEUUID HomeUUID("0000a005-00f1-0123-4567-0123456789ab");
  BLEUUID ControlUUID("0000a006-00f1-0123-4567-0123456789ab");
  BLEUUID DeveloperUUID("0000a007-00f1-0123-4567-0123456789ab");
  BLEUUID DisUUID("0000d00e-00f1-0123-4567-0123456789ab");

  // Create Service -------------------------------------------
  BLEService *pService = pServer->createService(ServiceUUID);
  // Create Characteristic ------------------------------------
  SlopeChar = pService->createCharacteristic(
      SlopeUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  AngleChar = pService->createCharacteristic(
      AngleUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  StatusChar = pService->createCharacteristic(
      StatusUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  HomeChar = pService->createCharacteristic(
      HomeUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  ControlChar = pService->createCharacteristic(
      ControlUUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
      128);
  DeveloperChar = pService->createCharacteristic(
      DeveloperUUID,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,
      128);
  FlatChar = pService->createCharacteristic(
      DisUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Add Descriptor (Avoid using UUID 2902 (Already used by NimBLE))
  // Add 2901 Descriptor
  SlopeChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Measure:Slope");
  AngleChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Measure:Angle");
  HomeChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("AppHome");
  StatusChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Status");
  ControlChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Control");
  DeveloperChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Developer");
  FlatChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 32)
      ->setValue("Measure:Flatness");

  // Set input type and unit ------------------------------------------
  BLE2904 *p2904Slope = new BLE2904();
  BLE2904 *p2904Angle = new BLE2904();
  BLE2904 *p2904Status = new BLE2904();
  BLE2904 *p2904Home = new BLE2904();
  BLE2904 *p2904Control = new BLE2904();
  BLE2904 *p2904Dist = new BLE2904();

  p2904Slope->setFormat(BLE2904::FORMAT_FLOAT32);
  p2904Angle->setFormat(BLE2904::FORMAT_FLOAT32);
  p2904Status->setFormat(BLE2904::FORMAT_UINT8);
  p2904Home->setFormat(BLE2904::FORMAT_UINT8);
  p2904Control->setFormat(BLE2904::FORMAT_UINT32);
  p2904Dist->setFormat(BLE2904::FORMAT_FLOAT32);
  
  SlopeChar->addDescriptor(p2904Slope);
  AngleChar->addDescriptor(p2904Angle);
  StatusChar->addDescriptor(p2904Status);
  HomeChar->addDescriptor(p2904Home);
  ControlChar->addDescriptor(p2904Control);
  FlatChar->addDescriptor(p2904Dist);

  // Set Characteristic Callback ------------------------------
  ServerCB.p_state = &state;
  ControlChar->setCallbacks(&ControlCallback);
  DeveloperChar->setCallbacks(&DeveloperCallback);
  pServer->setCallbacks(&ServerCB);

  // Start the Service ---------------------------------------------
  pServer->advertiseOnDisconnect(false);
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(ServiceUUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  // start sync count
  slow_sync = millis();
}

void BLE::SendAngle(float SendFloat) {
  AngleChar->setValue(SendFloat);
  AngleChar->notify(true);
}

void BLE::SendSlope(float SendFloat) {
  SlopeChar->setValue(SendFloat);
  SlopeChar->notify(true);
}

void BLE::SendFlatness(float SendFloat) {
  FlatChar->setValue(SendFloat);
  FlatChar->notify(true);
}

void BLE::SendStatus(byte *Send) {
  StatusChar->setValue(*Send);
  StatusChar->notify(true);
}

void BLE::SendHome(byte *Send) {
  ControlChar->setValue((*Send) + HOME_MODE_BASE * 1000);
  ControlChar->notify(true);
}

/* ServerCallbacks */ 
void MyServerCallbacks::onConnect(BLEServer *pServer, ble_gap_conn_desc *desc) {
  manage.resetMeasure();
  p_state->is_connect = true;
}

void BLE::DoSwitch() {
  // Do nothing if is_advertising status remain the same.
  if (pre_ble_state == state.is_advertising) return;
  // If swich from off to on;
  if (state.is_advertising == true) {
    if (!BLEDevice::getAdvertising()->isAdvertising())
      BLEDevice::startAdvertising();
  }
  // If swich from on to off;
  else {
    if (ble.state.is_connect) {
      pServer->disconnect(1);
      ble.state.is_connect = false;
    }
    if (BLEDevice::getAdvertising()->isAdvertising())
      BLEDevice::stopAdvertising();
  }
  pre_ble_state = ble.state.is_advertising;
};

void BLE::sendSyncInfo() {
  // version_info
  ControlChar->setValue(manage.ver_software + VERSION_SOFTWARE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue(manage.ver_hardware + VERSION_HARDWARE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue(manage.battery);
  ControlChar->notify(true);
  ControlChar->setValue(manage.meter_type + METER_TYPE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue((manage.home_mode) + HOME_MODE_BASE * 1000);
  ControlChar->notify(true);
}

void BLE::parseDeveloperInfo(int info) {
  uint8_t cmd = info / 1000;
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  uint8_t data = tens * 10 + ones;
  switch (cmd) {
    case 0:
      manage.page = data;
      break;
    case 1:
      ParseDebugMode(huns, data);
      break;
    case 7:
      ParseAngleCali(info);
      break;
    case 8:
      ParseFlatCaliCmd(info);
      break;
    default:
      ESP_LOGE("AppDeveloper", "%d", info);
      break;
  }
}

bool BLE::QuickNotifyEvent() {
  bool if_active = false;
  SendStatus(&manage.measure.state);
  if (manage.has_home_change == true) {
    ControlChar->setValue((manage.home_mode) + HOME_MODE_BASE * 1000);
    ControlChar->notify(true);
    manage.has_home_change = 0;
    if_active = true;
  }

  if (manage.to_app_str != "") {
    DeveloperChar->setValue(manage.to_app_str);
    DeveloperChar->notify(true);
    manage.to_app_str = "";
    if_active = true;
  }

  if (manage.cali_forward_str != "") {
    DeveloperChar->setValue(manage.cali_forward_str);
    DeveloperChar->notify(true);
    manage.cali_forward_str = "";
    if_active = true;
  }

  if (manage.flat_cali_str != "") {
    DeveloperChar->setValue(manage.flat_cali_str);
    DeveloperChar->notify(true);
    manage.flat_cali_str = "";
    if_active = true;
  }

  if (manage.angle_info != "") {
    DeveloperChar->setValue(manage.angle_info);
    DeveloperChar->notify(true);
    manage.angle_info = "";
    if_active = true;
  }

  return if_active;
}

void BLE::SlowNotifyEvent() {
  if (millis() - slow_sync > 60000) {
    ControlChar->setValue(manage.battery);
    ControlChar->notify(true);
    slow_sync = millis();
  }
}

void BLE::ParseAngleCali(int info) {
  manage.page = PAGE_CALI_ANGLE;
  String dataString;
  dataString = "";
  dataString += "<";
  dataString += String(info);
  dataString += ",";
  dataString += ">";
  Serial1.print(dataString);
}

void BLE::ParseFlatCaliCmd(int info) {
  manage.page = PAGE_CALI_FLAT;
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  uint8_t data = tens * 10 + ones;
  if (huns == 0) {
    if (data == CALI_STEP::RESET) {
      manage.page = PAGE_INFO;
      manage.flat_height_level = -1;
      manage.SendToApp("cmd[0]:reset success");
      return;
    }
    if (manage.flat_height_level == -1) {
      manage.flat_height_level = data;
      manage.flat.state = FLAT_APP_CALI;
    } else {
      manage.SendToApp("cmd[0]:please reset or wait!");
    }
  } else if (huns == 9) {
    manage.adjust_num = ones;
    manage.flat_height_level = 14;
    manage.flat.state = FLAT_APP_CALI;
    manage.SendToApp("single cali:" + String(ones));
  } else if (huns > 8) {
    manage.SendToApp("[error]cmd:" + String(huns));
  }
}

void BLE::ParseDebugMode(byte part, byte data) {
  switch (part) {
    case 2:
      manage.flat_debug = data;
      ESP_LOGE("", "part:%d;data:%d", part, data);
      break;
    default:
      ESP_LOGE("", "part:%d", part);
      break;
  }
}

void BLE::parseSyncInfo(int info) {
  uint8_t cmd = info / 1000;
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  switch (cmd) {
    case HOME_MODE_BASE:
      manage.home_mode = ones;
      break;
    case SLOPE_STD_BASE:
      if (huns == 0) {
        if (ones == 0) {
          manage.slope_standard = 1000.0f;
        } else if (ones == 1) {
          manage.slope_standard = 1200.0f;
        } else if (ones == 2) {
          manage.slope_standard = 2000.0f;
        }
      } else if (huns == 1) {
        manage.sleep_time = tens * 10 + ones;
        manage.putSleepTime();
      }
      break;
    case ANGLE_SEPPD_BASE:
      manage.speed_mode = ones;
      manage.put_speed_mode();
      break;
    case WARN_BASE:
      if (huns == 0) {
        if (ones == 2) {
          manage.warn_light_onoff = true;
        } else {
          manage.warn_light_onoff = false;
        }
        manage.put_warrning_light();
      } else if (huns == 1) {
        manage.warn_slope = (float)(tens * 10 + ones) / 10.0;
        manage.put_warn_slope();
      } else if (huns == 2) {
        manage.warn_flat = (float)(tens * 10 + ones) / 10.0;
        manage.put_warn_flat();
      } else {
      }
      break;
    default:
      ESP_LOGE("AppSync", "%d", info);
      break;
  }
}

/* ServerCallbacks */
void MyServerCallbacks::onDisconnect(BLEServer *pServer) {
  manage.resetMeasure();
  p_state->is_connect = false;
  if (p_state->is_advertising == true) {
    BLEDevice::startAdvertising();
  }
}

void ControlCallbacks::onSubscribe(BLECharacteristic *pCharacteristic) {
  ble.sendSyncInfo();
}

void ControlCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  String value = pCharacteristic->getValue();
  ble.parseSyncInfo(value.toInt());
}

void DeveloperCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  String value_s = pCharacteristic->getValue();
  ble.parseDeveloperInfo(value_s.toInt());
}
