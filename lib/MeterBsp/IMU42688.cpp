/**
 * @file IMU42688.cpp
 * @author Vicky Hung
 * @brief Read and calibrate the WC_IMU-X-Y-Z
 * @version final
 * @date 2023-07-21
 *
 * @copyright Wonder Construct
 *
 */
#include "IMU42688.h"
#include <Preferences.h>
Preferences pref;

/**
 * @brief Initialize Serial1 and read calibrated data from memory
 * @param [in] Rx : The GPIO for Serial1 RX. default -1.
 * @param [in] Tx : The GPIO for Serial1 TX. default -1.
 */
void IMU42688::init(uint8_t Rx, uint8_t Tx) {
  Serial1.setRxBufferSize(256);
  Serial1.begin(921600, SERIAL_8N1, Rx, Tx);

  if(pref.begin("Angle_Cal", false)){
    // e[0] = pref.getFloat("Ex", 0.0);
    e[1] = pref.getFloat("Ey", 90.0);
    // e[2] = pref.getFloat("Ez", 0.0);
    // s[0] = pref.getFloat("Sx", 1.0);
    // s[1] = pref.getFloat("Sy", 1.0);
    // s[2] = pref.getFloat("Sz", 1.0);
    // b[0] = pref.getFloat("Bx", 0.0);
    b[1] = pref.getFloat("By", 90.0);
    // b[2] = pref.getFloat("Bz", 0.0);
    pref.end();
  }

} 

void IMU42688::unpackFromC3(unsigned char info){
    switch(unpack_step){
      case STEP_FRAME_HEAD:
        if(info == '<'){
          unpack_step = STEP_PRASE_CMD;
        }
        break;
      case STEP_PRASE_CMD:
        if(info == 'A'){
          unpack_step = STEP_PRASE_DATA;
        }
        else if(info == 'B'){
          unpack_step = STEP_PRASE_CALI;
        }
        else{
          unpack_step = STEP_FRAME_HEAD;
        }
        break;
      case STEP_PRASE_DATA:
        if(info == '>'){
            String data_str;
            data_rx_buffer[data_rx_index] = '\0';
            data_rx_index = 0;
            has_imu_data = true;
            unpack_step = STEP_FRAME_HEAD;
        }else{
          data_rx_buffer[data_rx_index] = info;
          data_rx_index++;
          if (data_rx_index >= 64) {data_rx_index = 63;}
        }
        break;
      case STEP_PRASE_CALI:
          if(info == '>'){
              cali_rx_buffer[cali_rx_index] = '\0';
              manage.angle_msg = "";
              for(int i = 0;i < cali_rx_index;i++){
                manage.angle_msg += cali_rx_buffer[i];
              }
              cali_rx_index = 0;
              Serial.printf(manage.angle_msg.c_str());
              unpack_step = STEP_FRAME_HEAD;
          }else{
            cali_rx_buffer[cali_rx_index] = info;
            cali_rx_index++;
            if (cali_rx_index >= 64) {cali_rx_index = 64 - 1;}
        }
        break;
      default:
        ESP_LOGE("","unpack_step:%d",unpack_step);
        break;
  }
}


void IMU42688::parseImuData() {      // split the data into its parts
    if(has_imu_data){
        char * Index; // this is used by strtok() as an index
        Index = strtok(data_rx_buffer,",");
        if(Index != NULL) info_parsed[0] = atoi(Index); //cmd
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[1] = atof(Index); //version 
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[2] = atof(Index); //euler angle 0 
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[3] = atof(Index); //euler angle 1
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[4] = atof(Index); //euler angle 2
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[5] = atof(Index); //ui angle 0
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[6] = atof(Index); //ui angle 1
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[7] = atof(Index); //ui angle 2
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[8] = atoi(Index); // gravity
        Index = strtok(NULL, ",");
        if(Index != NULL) info_parsed[9] = atof(Index); // temperature
        has_imu_data = false;
    }
}

/**
 * @brief Update IMU data and check the IMU warm-up situation
 * @param [out] \b angle_cali : Calibrated real-time angle degree {X, Y, Z}
 * @param [out] \b angle_cali_show : Calibrated special filted angle degree for
 * interface display  {X, Y, Z}
 * @param [out] \b SensorTemperature : Filted sensor temperature (C).
 * @param [out] \b _gravity : Current _gravity Direction.
 * @param [out] \b _last_gravity : _gravity Direction for interface rotation.
 * @param [out] \b fWarmUp : IMU warm-up % (from 0 to 100).
 * @retval Return uint8_t to indicate if update successfully.
 * @retval 0 - Update success.
 * @retval 1 - IMU not warm up.
 * @retval 2 - Recieve data formatting error.
 * @retval 3 - Recieve angle value located outside the threshold.
 * @retval >4 - Times that IMU coped failed. Usually because Serial1 not
 * available.
 * @note If keep returning uint8_t > 2, the TX RX order in Initialize().
 */
uint8_t IMU42688::Update() {
  // Action 1: Read Data *********************************************
  // Read multiple time in case of wrong coping
  bool error_code = false;
  uint8_t arrow = 0;
  for (int j = 0; j < 3 * 2; j++) {
    // Step 1: Read Data from Serial1 --------------------------
    int start = millis();
    // wait serial read all:1.serial free 2.time out 3.have new data
    while (Serial1.available() && millis() - start < 1000 && !has_imu_data) {
      unpackFromC3(Serial1.read());
    }

    float AngleCope[6] = {0};
    int8_t Gravity_cope = 0;

    // Step 2: Check if Got New Data -------------------------------------
    if (has_imu_data) {
      parseImuData();
    } else {
      goto NextLoop;
    }

    // Step 3: Read angle_raw and do basic check -----------------------------
    // get imu version
    manage.version_imu = info_parsed[0];
    if (Gravity_cope >= 0 && Gravity_cope <= 5) {
      Gravity_cope = info_parsed[7];
    }else{
      ESP_LOGE("","Gravity_cope[0,5]:%d",info_parsed[7]);
    }
    _gravity = Gravity_cope;
    // 3. Check angle data
    for (size_t i = 1; i <= 6; i++) {
      AngleCope[i - 1] = info_parsed[i];
    }
    // 3. Update angle data
    memmove(&angle_raw[0], &AngleCope[0], sizeof(angle_raw));
    memmove(&angle_raw_show[0], &AngleCope[3], sizeof(angle_raw_show));

    angle_user[0] = angle_std[0] = angle_cali[0] = angle_raw[0];
    angle_user_show[0] = angle_std_show[0] = angle_cali_show[0] = angle_raw_show[0];
    angle_user[2] = angle_std[2] = angle_cali[2] = angle_raw[2];
    angle_user_show[2] = angle_std_show[2] = angle_cali_show[2] = angle_raw_show[2];
    /* Calibration */
    angle_cali[1] = angle_raw[1] + e[1];
    if(angle_cali[1] > 180.0f){
      angle_cali[1] = -360.0f + angle_cali[1];
    }
    else if(angle_cali[1] < -180.0f){
      angle_cali[1] = 360.0f + angle_cali[1];
    }
    angle_cali_show[1] =  angle_raw_show[1] + e[1];
    if(angle_cali_show[1] > 180.0f){
      angle_cali_show[1] = -360.0f + angle_cali_show[1];
    }
    else if(angle_cali_show[1] < -180.0f){
      angle_cali_show[1] = 360.0f + angle_cali_show[1];
    }
    angle_std[1] = angle_cali[1];
    angle_std_show[1] = angle_cali_show[1];

    // get arrow
    if((angle_std_show[1] >=    0.0f && angle_std_show[1] <   45.0f)
    || (angle_std_show[1] >=  90.0f && angle_std_show[1] <  135.0f)
    || (angle_std_show[1] >= -180.0f && angle_std_show[1] < -135.0f)
    || (angle_std_show[1] >= - 90.0f && angle_std_show[1] < - 45.0f)){
      arrow = 2;
    }
    else if( (angle_std_show[1] >=   45.0f && angle_std_show[1] <    90.0f)
          || (angle_std_show[1] >=  135.0f  && angle_std_show[1] <  180.0f)
          || (angle_std_show[1] >= -135.0f  && angle_std_show[1] < - 90.0f)
          || (angle_std_show[1] >= - 45.0f  && angle_std_show[1] < -  0.0f)){
      arrow = 1;
    }
    else{arrow = 0;}

    /* [-180,+180] --> [-90,+90]*/
    if(angle_std[1] >  90.0f){angle_std[1] = angle_std[1] - 180.0f;}
    if(angle_std[1] < -90.0f){angle_std[1] = angle_std[1] + 180.0f;}
    // angle_user[1] = fabs(angle_std[1]);
    angle_user[1] = angle_std[1];

    if(angle_std_show[1] >  90.0f){angle_std_show[1] = angle_std_show[1] - 180.0f;}
    if(angle_std_show[1] < -90.0f){angle_std_show[1] = angle_std_show[1] + 180.0f;}
    // angle_user_show[1] = fabs(angle_std_show[1]);
    angle_user_show[1] = angle_std_show[1];
    manage.set_clino_live(angle_user_show[1],arrow);
#ifdef HARDWARE_2_0
  manage.auto_angle = angle_raw[2];
#else
  manage.auto_angle = angle_raw[0];
#endif
    error_code = true;
    has_new_data = true;

    // Step 5: Print data to serial
    // ------------------------------------- Write ALL result onto ONE string
    // and print them together to save the Serial,print time consume
    break;//读取成功，用于强行结束循环并退出循环，不执行循环中剩余的语句
    NextLoop:
    //  ESP_LOGE("USER","IMU_ERROR:NextLoop");
    break;   
  }
  return error_code;
}  // end Update()


/**
 * @brief Collect the angle data if IMU stay stable.
 * @note Will cancel the calibration if the IMU's attitude change severe such
 * that the gravity direction change.
 */
void IMU42688::CollectCalData()
{
    // Check if the angle data already been load.
    if (!has_new_data)
        return;
    else
        has_new_data = false;

    // Initialize the collection
    if (avg_count == 0)
    {
        _start_g = _gravity;
        memmove(&_start_angle[0], &angle_raw[0], sizeof(_start_angle));
    }

    // If the IMU's attitude change severe such that the gravity direction change, cancel the calibration.
    else if (_gravity != _start_g)
    {
        StopCali();
        return;
    }

    // Check the stability.
    for (int i = 0; i < 2; i++)
    {
        // If the IMU's reading outside the threshold, restart the collection.
        if(fabs(angle_raw[i] - _start_angle[i] > set_zero_th))
        {
            avg_count = 1;
            memmove(&_start_angle[0], &angle_raw[0], sizeof(_start_angle));
            memmove(&_sum_angle[0], &angle_raw[0], sizeof(_sum_angle));
            return;
        }
        // else, add the value to the buffer.
        _sum_angle[i] += angle_raw[i];
    }
    avg_count++;
}

void IMU42688::CaliFactoryZero() {
  // Collect angle data.
  CollectCalData();

 if (avg_count < avg_total){
    cali_progress = (avg_count <= 1) ? 0 : (avg_count - 1) * 100.0 / avg_total;
  }
  else {
    pref.begin("Angle_Cal", false);
    b[1] = 0 - _sum_angle[1] / avg_total;
    pref.putFloat("By", b[1]);
    e[1] = b[1];
    pref.putFloat("Ey", e[1]);
    pref.end();
    avg_count = 0;
    memset(&_sum_angle, 0, sizeof(_sum_angle));
    cali_state = IMU_COMPLETE;
    manage.angle_msg = "imu_factory_zero:";
    manage.angle_msg += String(e[1], 2) + ",";
  }
}

void IMU42688::ResetFactoryZero() {
  e[1] = b[1];
  if(pref.begin("Angle_Cal", false)){
    pref.putFloat("Ey", e[1]);
    pref.end();
  }
}

void IMU42688::QuickCalibrate() {
  // Collect angle data.
  CollectCalData();

 if (avg_count < avg_total){
    cali_progress = (avg_count <= 1) ? 0 : (avg_count - 1) * 100.0 / avg_total;
  }
  else {
    pref.begin("Angle_Cal", false);
    e[1] = 0 - _sum_angle[1] / avg_total;
    pref.putFloat("Ey", e[1]);
    pref.end();
    cali_progress = 100;
    avg_count = 0;
    memset(&_sum_angle, 0, sizeof(_sum_angle));
    cali_state = IMU_COMPLETE;
    
    String S = "[Angle]SetZeroComplete: ";
    S += String(e[1], 2) + ",";
    // Serial.println(S);
  }
}

/**
 * @brief Set all parameters related to the calibration procedure to default.
 */
void IMU42688::StopCali() {
  cali_state = IMU_COMMON;
  yes_no = false;
  cali_progress = 0;
  avg_count = 0;
  memset(&_sum_angle, 0, sizeof(_sum_angle));
}

void IMU42688::onMeasureReset() {
  measure_count = 0;
  measure_sum = 0.0f;
  stable_count  = 0;
}

void IMU42688::setParam(uint8_t mode) {
  // HACK measure_total 过小会不会导致未稳定下来就测量
  switch (mode) {
    case SPEED_MODE_STANDARD:
      measure_total = 10;
      un_hold_th = 2.0f;
      un_stable_th = 0.1f;
      break;
    case SPEED_MODE_QUICK:
      measure_total = 5;
      un_hold_th = 2.0f;
      un_stable_th = 0.2f;
      break;
    case SPEED_MODE_AUTO:
      measure_total = 5;
      un_hold_th = 2.0f;
      un_stable_th = 0.3f;
      break;
    default:
      ESP_LOGE("USER", "[SetParam]ERROR!!!");
      break;
  }
}

int IMU42688::ProcessMeasureFSM() {
  uint8_t state = manage.clino.measure.state;
  if(state == M_IDLE && manage.speed_mode != SPEED_MODE_AUTO){return state;}
  measure_source = angle_user[1];
  setParam(manage.speed_mode);
  if (state == M_MEASURE_DONE || state == M_UPLOAD_DONE) {
    if(fabs(measure_source - hold_ref) > un_hold_th){
      onMeasureReset();
      manage.set_clino_progress(0);
      return state = M_IDLE;
    }
    return state;
  }

  // if(state == M_MEASURE_ING){
    if(fabs(measure_source - stable_ref) > un_stable_th){
      onMeasureReset();
      stable_ref = measure_source;
      manage.set_clino_progress(0);
      return state = M_UNSTABLE;
    }
    measure_sum += measure_source;
    measure_count++;
    manage.set_clino_progress(measure_count * 100 / measure_total);
    if(measure_count == measure_total){
      manage.set_clino_hold(measure_sum / measure_total,manage.clino.arrow_live);
      onMeasureReset();
      hold_ref  = measure_source;
      return state = M_MEASURE_DONE;
    }
    return state = M_MEASURE_ING;
  // }

  // if(state == M_UNSTABLE){
  //   if(fabs(measure_source - stable_ref) > un_stable_th){
  //     onMeasureReset();
  //   }
  //   if(stable_count == stable_total){
  //     onMeasureReset();
  //     return state == M_MEASURE_ING;
  //   }
  //   stable_count++;
  // }
  // return state;
}
