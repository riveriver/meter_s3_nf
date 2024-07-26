#include "BLE.h"
BLE ble;
void BLE::Init() {
  // Start BLE Deviec ----------------------------------------
  BLEDevice::init("Ensightful");
  std::string name_char;
  if (manage.meter_type == 1) {
    name_char = "Ensightful_500_";
  } 
  else if (manage.meter_type == 11) {
    name_char = "Ensightful_1000_";
  }
  else if (manage.meter_type == 12) {
    name_char = "Ensightful_2000_";
  }
  else{
    name_char = "Ensightful_";
  }
  std::string macAddress =
      BLEDevice::getAddress().toString();  // 获取完整的MAC地址字符串
  std::string lastFourDigits =
      macAddress.substr(macAddress.length() - 5);  // 获取MAC地址的后5位
  name_char =
      name_char + lastFourDigits;  // 将BLE名称和后4位拼接起来作为新的名称
  BLEDevice::setDeviceName(name_char.c_str());  // 设置BLE设备的新名称
  memcpy(state.addr, BLEDevice::getAddress().getNative(),
         sizeof(state.addr));
  AddrStr = BLEDevice::getAddress().toString().c_str();

  // Create Server --------------------------------------------
  BLEDevice::setMTU(512);
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
      DeveloperUUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,128);
  ControlChar = pService->createCharacteristic(
      ControlUUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY,128);
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
  p2904Dis->setUnit(0x2701);
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
  BLEUUID DisUUID("0000d00e-00f1-0123-4567-0123456789ab");
  FlatChar = pService->createCharacteristic(
    DisUUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  FlatChar->createDescriptor("2901", NIMBLE_PROPERTY::READ, 32)->setValue("Maximum Displacement");
  FlatChar->addDescriptor(p2904Dis);
  FlatChar->setValue(0.0F);

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
  slow_sync = millis();
  medium_sync = millis();
}

void BLE::SendFlatness(float SendFloat) {
  FlatChar->setValue(SendFloat);
  FlatChar->notify(true);
}

void BLE::SendAngle(float SendFloat) {
  AngleZChar->setValue(SendFloat);
  AngleZChar->notify(true);
}

void BLE::DoSwich() {
  // Do nothing if is_advertising status remain the same.
  if (Pre_OnOff == state.is_advertising) return;
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
  Pre_OnOff = ble.state.is_advertising;
};

void BLE::sendSyncInfo(){
  // version_info
  ControlChar->setValue(manage.ver_software + VERSION_SOFTWARE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue(manage.ver_hardware + VERSION_HARDWARE_BASE * 1000);
  ControlChar->notify(true);
  ControlChar->setValue(manage.battery);
  ControlChar->notify(true);
  ControlChar->setValue(manage.meter_type + METER_TYPE_BASE * 1000);
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

void BLE::ParseFlatnessCali(int info) {
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  uint8_t data = tens * 10 + ones;
  if(huns == 0){
    ESP_LOGE("APP","cmd:%d",data);
    if(data > 13){return;}
    manage.dist_cali.step = data;
    manage.flat_state = FLAT_APP_CALI;
  }else if(huns > 8){
    ESP_LOGE("APP","huns:%d > 8",huns);
  }else{
    ESP_LOGE("APP","sensor:%d,AorD:%d,value:%d",huns,tens,ones);
  }
}

void BLE::ParseFlatCaliCmd(int info) {
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  uint8_t data = tens * 10 + ones;
  if(huns == 0){
    if(data == CALI_STEP::RESET){
      manage.page = PAGE_INFO;
      manage.flat_height_level = -1;
      manage.SendToApp("cmd[0]:reset success");
      return;
    }
    if(manage.flat_height_level == -1){
      manage.flat_height_level = data;
      manage.flat_state = FLAT_APP_CALI;
    }else{
      manage.SendToApp("cmd[0]:please reset or wait!");
    }
  }
  else if(huns == 9){
    manage.adjust_num = ones;      
    manage.flat_height_level = 14;
    manage.flat_state = FLAT_APP_CALI;
    manage.SendToApp("single cali:" + String(ones));
  }else if(huns > 8){
    manage.SendToApp("[error]cmd:" + String(huns));
  }
  // else{
  //   float value = ones/10.0f;
  //   if(tens == 0){
  //     pFlat->bias[huns - 1] -= value;
  //     pFlat->putDistanceScale();
  //     String str = "OK!sensor:" + String(huns) + "value:-" + String(value);
  //     manage.SendToApp(str);
  //   }else if(tens == 1){
  //     pFlat->bias[huns - 1] += value;
  //     pFlat->putDistanceScale();
  //     String str = "OK!sensor:" + String(huns) + "value:+" + String(value);
  //     manage.SendToApp(str);
  //   }else{
  //     manage.SendToApp("[error]cmd: " + String(huns));
  //   }
  // }

}

void BLE::ParseDebugMode(byte part,byte data) {
  switch (part)
  {
    case 2:
      manage.flat_debug = data;
      ESP_LOGI("","part:%d;data:%d",part,data);
      break;
    default:
      ESP_LOGE("","part:%d",part);
      break;
  }
  
}


void BLE::parseSyncInfo(int info){

  uint8_t cmd  = info / 1000;
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  switch (cmd)
  {
    case HOME_MODE_BASE:
      manage.home_mode = ones;
      break;
    case SLOPE_STD_BASE:
      if(huns == 0){
      if(ones == 0){manage.slope_standard = 1000.0f;}
      else if(ones == 1){manage.slope_standard = 1200.0f;}
      else if(ones == 2){manage.slope_standard = 2000.0f;}
      }
      else if(huns == 1){
        manage.sleep_time = tens * 10 + ones;
        manage.putSleepTime();
      }
      break;
    case ANGLE_SEPPD_BASE:
      manage.speed_mode = ones;
      manage.put_speed_mode();
      break;
    case WARN_BASE:
      if(huns == 0){
      if(ones == 2){manage.warn_light_onoff = true;}
      else{manage.warn_light_onoff = false;}
      manage.put_warrning_light();
      }else if(huns == 1){
        manage.warn_angle =(float)(tens * 10 + ones)/10.0;
        manage.put_warn_angle();
      }else if(huns == 2){
        manage.warn_flat = (float)(tens * 10 + ones)/10.0;
        manage.put_warn_flat();
      }else{}
      break;
  default:
    ESP_LOGE("AppSync","%d",info);
    break;
  }
}


void BLE::parseDeveloperInfo(int info){
  uint8_t cmd  = info / 1000;
  uint8_t huns = (info / 100) % 10;
  uint8_t tens = (info / 10) % 10;
  uint8_t ones = (info) % 10;
  uint8_t data = tens * 10 + ones;
  switch (cmd)
  {
    case 0:
      manage.page = data;
      break;
    case 1:
      ParseDebugMode(huns,data);
      break;
    case 3:
      ForwardToC3(info);
      break;
    case 8:
      ParseFlatCaliCmd(info);
      break;
    default:
      ESP_LOGE("AppDeveloper","%d",info);
      break;
  }
}

bool BLE::QuickNotifyEvent() {
  bool if_active = false;
  SendStatus(&manage.measure.state);
  if(manage.to_app_str != ""){
    DeveloperChar->setValue(manage.to_app_str);
    DeveloperChar->notify(true);
    manage.to_app_str = "";
    if_active = true;
  }

  if(manage.has_home_change == true){
    ControlChar->setValue((manage.home_mode) + HOME_MODE_BASE*1000);
    ControlChar->notify(true);
    manage.has_home_change = 0;
    if_active = true;
  }
  if(manage.has_imu_forward == true){
    DeveloperChar->setValue(manage.cali_forward_str);
    DeveloperChar->notify(true);
    manage.has_imu_forward = false;
    if_active = true;
  }
  if(manage.has_flat_forward == true){
    DeveloperChar->setValue(manage.flat_cali_str);
    DeveloperChar->notify(true);
    manage.has_flat_forward = false;
    if_active = true;
  }
  if(manage.has_angle_forward == true){
    DeveloperChar->setValue(manage.angle_info);
    DeveloperChar->notify(true);
    manage.has_angle_forward = false;
    if_active = true;
  }
  return if_active;
}

void BLE::SlowNotifyEvent() {
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

void BLE:: SendHome(byte *Send) {
  ControlChar->setValue((*Send)+ HOME_MODE_BASE*1000);
  ControlChar->notify(true);
}

void MyServerCallbacks::onConnect(BLEServer *pServer, ble_gap_conn_desc* desc){
  manage.resetMeasure();
  p_state->is_connect = true;
}

void MyServerCallbacks::onDisconnect(BLEServer *pServer){
  manage.resetMeasure();
  p_state->is_connect = false;
  if (p_state->is_advertising == true) {
    BLEDevice::startAdvertising();
  }
  ESP_LOGE("BLE","Disconnect");
}

void ControlCallbacks::onSubscribe(BLECharacteristic *pCharacteristic,ble_gap_conn_desc *desc, uint16_t subValue){
  ble.sendSyncInfo();
}

void ControlCallbacks::onWrite(BLECharacteristic *pCharacteristic){
  String value = pCharacteristic->getValue();
  ble.parseSyncInfo(value.toInt());
}

void DeveloperCallbacks::onWrite(BLECharacteristic *pCharacteristic){
  String value_s = pCharacteristic->getValue();
  int value_d = value_s.toInt();
  ble.parseDeveloperInfo(value_d);
}
