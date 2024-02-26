#include "BLE.h"
BLE ble;
void BLE::Initialize(int &LastEdit, uint8_t *type) {
  // Start BLE Deviec ----------------------------------------
  BLEDevice::init("Flatness");
  std::string name_char;
  if (*type < 10) {
    name_char = "Flatness500_";
  } else if (*type > 10) {
    name_char = "Flatness1000_";
  }
  std::string macAddress =
      BLEDevice::getAddress().toString();  // 获取完整的MAC地址字符串
  std::string lastFourDigits =
      macAddress.substr(macAddress.length() - 5);  // 获取MAC地址的后5位
  name_char =
      name_char + lastFourDigits;  // 将BLE名称和后4位拼接起来作为新的名称
  BLEDevice::setDeviceName(name_char.c_str());  // 设置BLE设备的新名称
  memcpy(State.Address, BLEDevice::getAddress().getNative(),
         sizeof(State.Address));
  AddrStr = BLEDevice::getAddress().toString().c_str();

  // Create Server --------------------------------------------
  pServer = BLEDevice::createServer();
  // Create Address -------------------------------------------
  BLEUUID ServiceUUID("0000abcd-00f1-0123-4567-0123456789ab");
  BLEUUID AngleXUUID("0000a001-00f1-0123-4567-0123456789ab");
  BLEUUID AngleYUUID("0000a002-00f1-0123-4567-0123456789ab");
  BLEUUID AngleZUUID("0000a003-00f1-0123-4567-0123456789ab");
  BLEUUID StatusUUID("0000a004-00f1-0123-4567-0123456789ab");
  BLEUUID HomeUUID("0000a005-00f1-0123-4567-0123456789ab");
  BLEUUID ControlUUID("0000a006-00f1-0123-4567-0123456789ab");
  BLEUUID DeveloperUUID("0000a007-00f1-0123-4567-0123456789ab");

  // Create Service -------------------------------------------
  BLEService *pService = pServer->createService(ServiceUUID);
  // Create Characteristic ------------------------------------
  DeveloperChar =  pService->createCharacteristic(
      DeveloperUUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  ControlChar = pService->createCharacteristic(
      ControlUUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  HomeChar = pService->createCharacteristic(
      HomeUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  StatusChar = pService->createCharacteristic(
      StatusUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  AngleXChar = pService->createCharacteristic(
      AngleXUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  AngleYChar = pService->createCharacteristic(
      AngleYUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  AngleZChar = pService->createCharacteristic(
      AngleZUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Add Descriptor (Avoid using UUID 2902 (Already used by NimBLE))
  // Add 2901 Descriptor
  DeveloperChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("DeveloperMode");
  ControlChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Control:Upper_to_Lower");
  HomeChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("HomeMode");
  StatusChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Status");
  AngleXChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Angle X");
  AngleYChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Angle Y");
  AngleZChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 8)
      ->setValue("Angle Z");

  // Set input type and unit ------------------------------------------
  BLE2904 *p2904Control = new BLE2904();
  BLE2904 *p2904Home = new BLE2904();
  BLE2904 *p2904Status = new BLE2904();
  BLE2904 *p2904Ang = new BLE2904();
  BLE2904 *p2904Dis = new BLE2904();

  p2904Control->setFormat(BLE2904::FORMAT_SINT32);
  p2904Home->setFormat(BLE2904::FORMAT_UINT8);
  p2904Status->setFormat(BLE2904::FORMAT_UINT8);  // Format int
  p2904Ang->setFormat(BLE2904::FORMAT_FLOAT32);   // Format float
  p2904Dis->setFormat(BLE2904::FORMAT_FLOAT32);   // Format float

  p2904Control->setUnit(0);
  p2904Home->setUnit(0);
  p2904Status->setUnit(0);
  p2904Ang->setUnit(0x2763);  // plane angle (degree)
  p2904Dis->setUnit(0x2701);  // length (meter)
  p2904Ang->setExponent(0);
  p2904Dis->setExponent(-3);

  ControlChar->addDescriptor(p2904Control);
  HomeChar->addDescriptor(p2904Home);
  StatusChar->addDescriptor(p2904Status);
  AngleXChar->addDescriptor(p2904Ang);
  AngleYChar->addDescriptor(p2904Ang);
  AngleZChar->addDescriptor(p2904Ang);

  // Set Initial Value
  HomeChar->setValue(0);
  StatusChar->setValue(0);
  AngleXChar->setValue(0.0F);
  AngleYChar->setValue(0.0F);
  AngleZChar->setValue(0.0F);
  
  // Add Displacement relative characteristic
  String DisplacementUUID = "0000d000-00f1-0123-4567-0123456789ab";
  for (int i = 0; i < 15; i++) {
    DisplacementUUID.setCharAt(7, "0123456789abcdef"[i]);
    BLEUUID DisUUID(DisplacementUUID.c_str());
    DisChar[i] = pService->createCharacteristic(
        DisUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    if (i == 0)
      DisChar[i]
          ->createDescriptor("2901", NIMBLE_PROPERTY::READ, 32)
          ->setValue("Maximum Displacement");
    else
      DisChar[i]
          ->createDescriptor("2901", NIMBLE_PROPERTY::READ, 16)
          ->setValue("Distance " + String(i));
    DisChar[i]->addDescriptor(p2904Dis);
    DisChar[i]->setValue(0.0F);
  }

  // Set Characteristic Callback ------------------------------
  ServerCB.State = &State;
  ServerCB.LastEdit = &LastEdit;
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
  slow_sync = millis();
  medium_sync = millis();
}

void BLE::Send(float *SendFloat) {
  AngleZChar->setValue(*SendFloat);
  AngleZChar->notify(true);
  for (int i = 0; i < 14; i++) {
    DisChar[i]->setValue(*(SendFloat + 3 + i));
    DisChar[i]->notify(true);
  }
  DisChar[14]->setValue(*(SendFloat + 17));
  DisChar[14]->notify(true);
}

void BLE::SendFlatness(float SendFloat) {
  DisChar[14]->setValue(SendFloat);
  DisChar[14]->notify(true);
}

void BLE::SendAngle(float SendFloat) {
  AngleZChar->setValue(SendFloat);
  AngleZChar->notify(true);
}

void BLE::SendDistance(float *SendFloat) {
  for (int i = 0; i < 14; i++) {
    DisChar[i]->setValue(*(SendFloat + 3 + i));
    DisChar[i]->notify(true);
  }
}


void BLE::DoSwich() {
  // Do nothing if OnOff status remain the same.
  if (Pre_OnOff == State.OnOff) return;
  // If swich from off to on;
  if (State.OnOff == true) {
    if (!BLEDevice::getAdvertising()->isAdvertising())
      BLEDevice::startAdvertising();
  }
  // If swich from on to off;
  else {
    if (ble.State.isConnect) {
      pServer->disconnect(1);
      ble.State.isConnect = false;
    }
    if (BLEDevice::getAdvertising()->isAdvertising())
      BLEDevice::stopAdvertising();
  }
  Pre_OnOff = ble.State.OnOff;
};

void BLE::sendSyncInfo(){
  // version_info
  ControlChar->setValue(manage.software_version + VERSION_SOFTWARE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue(manage.hardware_version + VERSION_HARDWARE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue(manage.battery);
  ControlChar->notify(true);
  ControlChar->setValue(manage.meter_type + 2000);
  ControlChar->notify(true);
  ControlChar->setValue((manage.home_mode) + HOME_MODE_BASE*1000);
  ControlChar->notify(true);
}

void BLE::ForwardToC3(int info) {
  String dataString;
  dataString = "";
  dataString += "<";
  dataString += String(info);
  dataString += ",";
  dataString += ">";
  Serial1.print(dataString);
}

void BLE::parseSyncInfo(){
  Serial.printf("rx_info:%d\n",rx_info);
  uint8_t cmd  = rx_info / 1000;
  uint8_t huns = (rx_info / 100) % 10;
  uint8_t tens = (rx_info / 10) % 10;
  uint8_t ones = (rx_info) % 10;
  switch (cmd)
  {
    case HOME_MODE_BASE:
      manage.home_mode = ones;
      break;
    case SLOPE_STD_BASE:
      if(ones == 0){manage.slope_standard = 1000.0f;}
      else if(ones == 1){manage.slope_standard = 1200.0f;}
      else if(ones == 2){manage.slope_standard = 2000.0f;}
      else{}
      break;
    case ANGLE_SEPPD_BASE:
      manage.speed_mode = ones;
      break;
    case WARN_BASE:
      if(huns == 0){
        manage.warrning_mode = ones;
      }else if(huns == 1){
        manage.warrning_angle =(float)(tens * 10 + ones)/10.0;
      }else if(huns == 2){
        manage.warrning_flat = (float)(tens * 10 + ones)/10.0;
      }else{}
      break;
  default:
    ESP_LOGE("","%d",rx_info);
    break;
  }
}


void BLE::parseDeveloperInfo(int info){
  uint8_t cmd  = info / 1000;
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  switch (cmd)
  {
    case 0:
      manage.page = ones;
      break;
    case 7:
      ForwardToC3(info);
      break;
    default:
      ESP_LOGE("","rx_info:%d",rx_info);
      break;
  }
}

void BLE::NotifyEvent() {

  SendHome(&manage.home_mode);
  SendStatus(&manage.measure.state);
  if(manage.has_home_change == true){
    ControlChar->setValue((manage.home_mode) + HOME_MODE_BASE*1000);
    ControlChar->notify(true);
    manage.has_home_change = 0;
  }
  if(manage.has_imu_forward == true){
    DeveloperChar->setValue(manage.cali_forward_str);
    DeveloperChar->notify();
    manage.has_imu_forward = false;
  }
  if(millis() - slow_sync > 60000){
    ControlChar->setValue(manage.battery);
    ControlChar->notify(true);
    slow_sync = millis();
  }
}

void BLE::SendStatus(byte *Send) {
  StatusChar->setValue(*Send);
  StatusChar->notify(true);
}

void BLE::SendHome(byte *Send) {
  HomeChar->setValue(*Send);
  HomeChar->notify(true);
}

void MyServerCallbacks::onConnect(BLEServer *pServer){
  State->isConnect = true;
  *LastEdit = millis();
  ESP_LOGE("USER","Connect");
}

void MyServerCallbacks::onDisconnect(BLEServer *pServer){
  State->isConnect = false;
  *(LastEdit) = millis();
  if (State->OnOff == true) {
    BLEDevice::startAdvertising();
  }
  ESP_LOGE("USER","Disconnect");
}

void ControlCallbacks::onSubscribe(BLECharacteristic *pCharacteristic,ble_gap_conn_desc *desc, uint16_t subValue){
  ESP_LOGE("USER","Control onSubscribe");
  ble.sendSyncInfo();
}

void ControlCallbacks::onWrite(BLECharacteristic *pCharacteristic){
  String value = pCharacteristic->getValue();
  ble.rx_info = value.toInt();
  ble.parseSyncInfo();
}

void DeveloperCallbacks::onWrite(BLECharacteristic *pCharacteristic){
  String value_s = pCharacteristic->getValue();
  int value_d = value_s.toInt();
  ESP_LOGE("Developer","%d",value_d);
  ble.parseDeveloperInfo(value_d);
}
