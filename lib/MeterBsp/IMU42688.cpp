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

float roundToZeroOrFive(float value,int bits) {
    float decimalPart = value - floor(value);  // 获取小数部分
    int bits_value = static_cast<int>(decimalPart * pow(10, bits) ) % 10;  // 获取小数点第二位数字
    if (bits_value > 3 && bits_value < 7) {
        return floor(value * 10) / 10 + 5 / pow(10, bits);
    } else if (bits_value <= 3) {
        return floor(value * 10) / 10;
    } else if(bits_value >= 7){
        return floor(value * 10) / 10 + 10 / pow(10, bits);
    }
    else{
      ESP_LOGE("","modifyDecimal ERROR!!!");
      return value;
    }
}

/**
 * @brief Initialize Serial1 and read calibrated data from memory
 * @param [in] Rx : The GPIO for Serial1 RX. default -1.
 * @param [in] Tx : The GPIO for Serial1 TX. default -1.
 */
void IMU42688::Initialize(byte Rx, byte Tx) {
  Serial1.setRxBufferSize(256);
  Serial1.begin(921600, SERIAL_8N1, Rx, Tx);

  while (!pref.begin("Angle_Cal", false)) {
    ESP_LOGE("[Warning]","getAngle_Cal Fail");
  }
  // e[0] = pref.getFloat("Ex", 0.0);
  e[1] = pref.getFloat("Ey", 0.0);
  // e[2] = pref.getFloat("Ez", 0.0);
  // s[0] = pref.getFloat("Sx", 1.0);
  // s[1] = pref.getFloat("Sy", 1.0);
  // s[2] = pref.getFloat("Sz", 1.0);
  // b[0] = pref.getFloat("Bx", 0.0);
  b[1] = pref.getFloat("By", 0.0);
  // b[2] = pref.getFloat("Bz", 0.0);
  pref.end();

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
          // ESP_LOGE("COMM","STEP_PRASE_CMD:0x%X",info);
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
          if (data_rx_index >= RX_BUFFER_SIZE) {data_rx_index = RX_BUFFER_SIZE - 1;}
        }
        break;
      case STEP_PRASE_CALI:
        // if(manage.has_imu_forward != true){
          if(info == '>'){
              cali_rx_buffer[cali_rx_index] = '\0';
              manage.cali_forward_str = "";
              for(int i = 0;i < cali_rx_index;i++){
                manage.cali_forward_str += cali_rx_buffer[i];
              }
              // HACK
              Serial.println(manage.cali_forward_str);
              
              cali_rx_index = 0;
              manage.has_imu_forward = true;
              unpack_step = STEP_FRAME_HEAD;
          }else{
            cali_rx_buffer[cali_rx_index] = info;
            cali_rx_index++;
            if (cali_rx_index >= RX_BUFFER_SIZE) {cali_rx_index = RX_BUFFER_SIZE - 1;}
          // }
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
        if(Index != NULL) imu_raw[0] = atoi(Index); //cmd
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[1] = atof(Index); //version 
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[2] = atof(Index); //euler angle 0 
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[3] = atof(Index); //euler angle 1
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[4] = atof(Index); //euler angle 2
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[5] = atof(Index); //ui angle 0
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[6] = atof(Index); //ui angle 1
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[7] = atof(Index); //ui angle 2
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[8] = atoi(Index); // gravity
        Index = strtok(NULL, ",");
        if(Index != NULL) imu_raw[9] = atof(Index); // temperature
        has_imu_data = false;
    }
}

/**
 * @brief Update IMU data and check the IMU warm-up situation
 * @param [out] \b AngleCal : Calibrated real-time angle degree {X, Y, Z}
 * @param [out] \b AngleCalShow : Calibrated special filted angle degree for
 * interface display  {X, Y, Z}
 * @param [out] \b SensorTemperature : Filted sensor temperature (C).
 * @param [out] \b Gravity : Current Gravity Direction.
 * @param [out] \b GravityPrevious : Gravity Direction for interface rotation.
 * @param [out] \b fWarmUp : IMU warm-up % (from 0 to 100).
 * @retval Return byte to indicate if update successfully.
 * @retval 0 - Update success.
 * @retval 1 - IMU not warm up.
 * @retval 2 - Recieve data formatting error.
 * @retval 3 - Recieve angle value located outside the threshold.
 * @retval >4 - Times that IMU coped failed. Usually because Serial1 not
 * available.
 * @note If keep returning byte > 2, the TX RX order in Initialize().
 */
byte IMU42688::Update() {
  // Action 1: Read Data *********************************************
  // Read multiple time in case of wrong coping
  for (int j = 0; j < IMU_C * 2; j++) {
    // Step 1: Read Data from Serial1 --------------------------
    CopeFailed++;
    int StartTime = millis();
    // wait serial read all:1.serial free 2.time out 3.have new data
    while (Serial1.available() && millis() - StartTime < 1000 && !has_imu_data) {
      unpackFromC3(Serial1.read());
    }

    float AngleCope[6] = {0};
    int8_t Gravity_cope = 0;

    // Step 2: Check if Got New Data -------------------------------------
    if (has_imu_data) {
      CopeFailed = 0;
      parseImuData();
    } else {
      goto NextLoop;
    }

    // Step 3: Read AngleRaw and do basic check -----------------------------
    // get imu version
    manage.imu_version = imu_raw[0];
    if (Gravity_cope >= 0 && Gravity_cope <= 5) {
      Gravity_cope = imu_raw[7];
    }else{
      ESP_LOGE("","Gravity_cope[0,5]:%d",imu_raw[7]);
    }
    Gravity = Gravity_cope;
    // 3. Check angle data
    for (size_t i = 1; i <= 6; i++) {
      AngleCope[i - 1] = imu_raw[i];
    }
    // 3. Update angle data
    memmove(&AngleRaw[0], &AngleCope[0], sizeof(AngleRaw));
    memmove(&AngleRawShow[0], &AngleCope[3], sizeof(AngleRawShow));

    AngleUser[0] = AngleStd[0] = AngleCal[0] = AngleRaw[0];
    AngleUserShow[0] = AngleStdShow[0] = AngleCalShow[0] = AngleRawShow[0];
    AngleUser[2] = AngleStd[2] = AngleCal[2] = AngleRaw[2];
    AngleUserShow[2] = AngleStdShow[2] = AngleCalShow[2] = AngleRawShow[2];
    /* Calibration */
    AngleCal[1] = AngleRaw[1] + e[1];
    if(AngleCal[1] > 180.0f){
      AngleCal[1] = -360.0f + AngleCal[1];
    }
    else if(AngleCal[1] < -180.0f){
      AngleCal[1] = 360.0f + AngleCal[1];
    }
    AngleCalShow[1] =  AngleRawShow[1] + e[1];
    if(AngleCalShow[1] > 180.0f){
      AngleCalShow[1] = -360.0f + AngleCalShow[1];
    }
    else if(AngleCalShow[1] < -180.0f){
      AngleCalShow[1] = 360.0f + AngleCalShow[1];
    }
    AngleStd[1] = AngleCal[1];
    AngleStdShow[1] = AngleCalShow[1];
    // byte arrow = 0;
    /* get arrow */
    // if((AngleStdShow[1] >    0.05f && AngleStdShow[1] <   44.95f)
    // || (AngleStdShow[1] >   90.05f && AngleStdShow[1] <  134.95f)
    // || (AngleStdShow[1] > -179.95f && AngleStdShow[1] < -135.05f)
    // || (AngleStdShow[1] > - 89.95f && AngleStdShow[1] < - 45.05f)){
    //   manage.set_live_arrow_angle(0);
    // }
    // else if( (AngleStdShow[1] >   45.05f && AngleStdShow[1] <  89.95f)
    //       || (AngleStdShow[1] >  135.05f  && AngleStdShow[1] <  179.95f)
    //       || (AngleStdShow[1] > -134.95f  && AngleStdShow[1] < - 90.05f)
    //       || (AngleStdShow[1] > - 44.95f  && AngleStdShow[1] <    0.05f)){
    //   manage.set_live_arrow_angle(1);
    // }
    // else{manage.set_live_arrow_angle(2);}
    if((AngleStdShow[1] >    0.0f && AngleStdShow[1] <   45.0f)
    || (AngleStdShow[1] >   90.0f && AngleStdShow[1] <  135.0f)
    || (AngleStdShow[1] > -180.0f && AngleStdShow[1] < -135.0f)
    || (AngleStdShow[1] > - 90.0f && AngleStdShow[1] < - 45.0f)){
      // arrow = 1;
    }
    else if( (AngleStdShow[1] >   45.0f && AngleStdShow[1] <    90.0f)
          || (AngleStdShow[1] >  135.0f  && AngleStdShow[1] <  180.0f)
          || (AngleStdShow[1] > -135.0f  && AngleStdShow[1] < - 90.0f)
          || (AngleStdShow[1] > - 45.0f  && AngleStdShow[1] < -  0.0f)){
      // arrow = 2;
    }
    // else{arrow = 0;}
    /* [-180,+180] --> [-90,+90]*/
    if(AngleStd[1] >  90.0f){AngleStd[1] = AngleStd[1] - 180.0f;}
    if(AngleStd[1] < -90.0f){AngleStd[1] = AngleStd[1] + 180.0f;}
    AngleUser[1] = fabs(AngleStd[1]);

    if(AngleStdShow[1] >  90.0f){AngleStdShow[1] = AngleStdShow[1] - 180.0f;}
    if(AngleStdShow[1] < -90.0f){AngleStdShow[1] = AngleStdShow[1] + 180.0f;}
    AngleUserShow[1] = fabs(AngleStdShow[1]);
    // manage.set_angle_live(roundToZeroOrFive(AngleUserShow[1],2),1);
    manage.set_angle_live(AngleUserShow[1],1);
    ErrorCode = IMU_Update_Success;
    Have_New_Data_for_Collect = true;

    // Step 5: Print data to serial
    // ------------------------------------- Write ALL result onto ONE string
    // and print them together to save the Serial,print time consume
    break;//读取成功，用于强行结束循环并退出循环，不执行循环中剩余的语句
    NextLoop:
    //  ESP_LOGE("USER","IMU_ERROR:NextLoop");
    break;   
  }
  // Action 2: Initialize sensor temperature relative value
  // ****************************************
  if (fWarmUp != 100 && millis() < 10000) {
    ErrorCode = Err_IMU_Not_Warm_Up;
    fWarmUp = (millis()) * 100 / 10000;
  } else if (fWarmUp != 100) {
    fWarmUp = 100;
  }
  return ErrorCode;
}  // end Update()

/**
 * @brief Convert float degree into String.
 *
 * @param [in] degree AngleRaw for convert (degree).
 * @return String - AngleRaw in degree with length = 6, precision = 2.
 */
String IMU42688::String_degree(float degree) {
  char A[16];
  dtostrf(degree, 7, 2, A);
  return A;
}

/**
 * @brief Convert float degree into radian String.
 *
 * @param [in] degree AngleRaw for convert (degree).
 * @return String -AngleRaw in radian with length = 6, precision = 4.
 */
String IMU42688::String_rad(float degree) {
  char A[16]; 
  dtostrf(degree * PI / 180.0, 7, 4, A);
  return A;
}

/**
 * @brief Convert float degree into mm/m String.
 *
 * @param [in] degree AngleRaw for convert (degree).
 * @return String - AngleRaw in tangent*1000 (mm/m) with length = 6, precision = 1.
 * @note This function return the tangent value to the closet horizontal or
 * vertical reference (-90, 0, 90, or 180 degree)
 */
String IMU42688::String_mm(float degree) {
  char A[16];
  if (fabs(degree) < 45.0f || fabs(degree) > 135.0f)
  {
      float mm = round(tan(degree * 0.01745f) * manage.slope_standard * 10.0f) / 10.0f;
      dtostrf(mm, 7, 1, A);
  }
  else if (fabs(degree - 90.0f) < 45.0f)
  {
      float mm = round(tan((degree - 90.0f) * 0.01745f) * manage.slope_standard * 10.0f) / 10.0f;
      dtostrf(mm, 7, 1, A);
  }
  else if (fabs(degree + 90) < 45)
  {
      float mm = round(tan((degree + 90.0f) * 0.01745f) * manage.slope_standard * 10.0f) / 10.0f;
      dtostrf(mm, 7, 1, A);
  }
  else{
    ESP_LOGE("USER","String_mm: %f",degree);
    float mm = round(tan(degree * 0.01745f) * manage.slope_standard * 10.0f) / 10.0f;
    dtostrf(mm, 7, 1, A);
  }
  return A;
}

String IMU42688::String_now_unit(float degree, uint8_t unit) {
  switch (unit) {
    case 0:
      return String_degree(degree);
    case 1:
      return String_mm(degree);
    case 2:
      return String_rad(degree);
  }
  char A[16];
  dtostrf(degree, 7, 2, A);
  return A;
}

/**
 * @brief Collect the angle data if IMU stay stable.
 * @note Will cancel the calibration if the IMU's attitude change severe such
 * that the gravity direction change.
 */
void IMU42688::CollectCalData()
{
    // Check if the angle data already been load.
    if (!Have_New_Data_for_Collect)
        return;
    else
        Have_New_Data_for_Collect = false;

    // Initialize the collection
    if (CalibrateCount == 0)
    {
        StartCalG = Gravity;
        memmove(&StartCalA[0], &AngleRaw[0], sizeof(StartCalA));
    }

    // If the IMU's attitude change severe such that the gravity direction change, cancel the calibration.
    else if (Gravity != StartCalG)
    {
        CalStop();
        return;
    }

    // Check the stability.
    for (int i = 0; i < 2; i++)
    {
        // If the IMU's reading outside the threshold, restart the collection.
        if(fabs(AngleRaw[i] - StartCalA[i] > Cal_TH))
        {
            CalibrateCount = 1;
            memmove(&StartCalA[0], &AngleRaw[0], sizeof(StartCalA));
            memmove(&SumCalA[0], &AngleRaw[0], sizeof(SumCalA));
            return;
        }
        // else, add the value to the buffer.
        SumCalA[i] += AngleRaw[i];
    }
    CalibrateCount++;
}
/**
 * @brief Calculate the average of the array excluding the angle located outside
 * 2 angle array StDev.
 *
 * @param [in] AngleRaw : Pointer to the float angle array
 * @param [in] Count : Pointer to the bool array for saving the angle value
 * distinguish result.
 * @param [in] Countlength : Lenght of the angle array.
 * @return float : Average value of the input array.
 */
float IMU42688::Avg_in_2StDev(float *AngleRaw, bool *Count, int Countlength) {
  bool ChangeCount = true;
  double Avg;
  // Calculate until all angel stay inside 2 StDev.
  while (ChangeCount) {
    // 1. Calculate the mean value of the angle array.
    float Sum = 0;
    int SumCount = 0;
    for (int i = 1; i < Countlength; i++) {
      if (*(Count + i)) {
        Sum += *(AngleRaw + i);
        SumCount++;
      }
    }
    Avg = Sum / SumCount;
    // 2. Calculate the standard diviation
    Sum = 0;
    for (int i = 1; i < Countlength; i++) {
      if (*(Count + i)) Sum += pow(*(AngleRaw + i) - Avg, 2);
    }
    float Stdev = pow(Sum / SumCount, 0.5);
    // 3. Check if any angle located outside 2 StDev
    ChangeCount = false;
    for (int i = 1; i < Countlength; i++) {
      if (*(Count + i)) {
        if (fabs(*(AngleRaw + i) - Avg) > 2 * Stdev) {
          *(Count + i) = false;
          ChangeCount = true;
        }
      }
    }
  }
  return Avg;
}

/**
 * @brief Set the current attitude as ideal attitude.
 *
 */
void IMU42688::QuickCalibrate() {
  // Collect angle data.
  CollectCalData();

  // If collection complete, calculate the angle difference between the measure
  // result and ideal value.
  // 1. Identify the axis to calibration in the gravity direction.
  // 2. Identify the ideal value of the axis to calibration.
  // 3. Compare the ideal value and measure value and get the error.
  // 4. Save the error to the flash memory.
  if (CalibrateCount == CalAvgNum) {
    pref.begin("Angle_Cal", false);
    switch (StartCalG) {
      // case 0:
      //   e[1] = -90 - SumCalA[1] / CalAvgNum;
      //   pref.putFloat("Ey", e[1]);
      //   break;
      // case 1:
      //   if(SumCalA[1] >= 0){
      //     e[1] = 180 - SumCalA[1] / CalAvgNum;
      //   }
      //   else if(SumCalA[1] < 0){
      //     e[1] = -180 - SumCalA[1] / CalAvgNum;
      //   }
      //   pref.putFloat("Ey", e[1]);
      //   break;
      // case 2:
      //   e[0] = 0 - SumCalA[0] / CalAvgNum;
      //   pref.putFloat("Ex", e[0]);
      //   break;
      case 3:
        e[1] = 0 - SumCalA[1] / CalAvgNum;
        pref.putFloat("Ey", e[1]);
        break;
      // case 4:
      //   e[1] = 0 - SumCalA[1] / CalAvgNum;
      //   pref.putFloat("Ey", e[1]);
      //   break;
      // case 5:
      //   e[0] = 0 - SumCalA[0] / CalAvgNum;
      //   pref.putFloat("Ex", e[0]);
      //   break;
      default:
        ESP_LOGE("USER","StartCalG: %d",StartCalG);
      break;
    }
    pref.end();
    CalibrateCheck = 2;
  }
}

/**
 * @brief Calibrate the zero position and the scale for all three axis.
 * @warning Didn't eliminate the human operation error. Not recommand to use
 * this function to calibrate.
 *
 */
void IMU42688::FullCalibrate() {
  // Check if the require the current gravity direction infromation.
  if (Gravity > 2 || FullCalComplete[Gravity]) {
    return;
  }

  CollectCalData();

  // Calculate the angle value when measurement complete
  if (CalibrateCount == CalAvgNum) {
    // Get the Average
    FullCalAngle[4 - StartCalG * 2] = SumCalA[(StartCalG + 1) % 3] / CalAvgNum;
    FullCalAngle[5 - StartCalG * 2] = SumCalA[(StartCalG + 2) % 3] / CalAvgNum;

    // Reset data collect buffer.
    CalibrateCount = 0;
    memset(&SumCalA, 0, sizeof(SumCalA));

    // Measure on #n direction complete ( n = Gravity )
    // If calibration procedure isn't finished, back to confirm page.
    FullCalComplete[Gravity] = true;
    CalibrateCheck = 0;
  }

  // If all measurment complete, calculate and save the parameter to flash
  // memory.
  if (FullCalComplete[0] && FullCalComplete[1] && FullCalComplete[2]) {
    pref.begin("Angle_Cal", false);
    char prefkey[9][3] = {"Sx", "Sy", "Sz", "Bx", "By", "Bz", "Ex", "Ey", "Ez"};
    for (int i = 0; i < 3; i++) {
      // Calculate the result ------------------------------------------------
      // 1. Calculate Scale factor
      //  - Ideal angle reding between two result is 90.
      //  - Used the scale facter to eliminate the scale error of the IMU
      s[i] = 90.0 * pow(-1, i) / (FullCalAngle[i + 3] - FullCalAngle[i]);
      // 2. Calculate zero position bias
      //  - Compare the angle difference between ideal 0 and measure result.
      //  - Used bias to eliminate this error.
      b[i] = -FullCalAngle[i];
      // 3. Set zero position error into bias.
      e[i] = b[i];

      // Save the Result to flash ---------------------------------------------
      pref.putFloat(&prefkey[i][0], s[i]);
      pref.putFloat(&prefkey[i + 3][0], b[i]);
      pref.putFloat(&prefkey[i + 6][0], e[i]);

    }
    pref.end();

    // Calibration Complete
    CalibrateCheck = 2;
  }
}

/**
 * @brief Make a full calibration on Z Axis.
 * @warning This function should only be available to manufactorer.
 * @warning This calibration should be done in lab.
 * @note Calibration Procedure :
 * @note 1. Measure the axis Z reading when axis x point to world up direction
 * for 10 times.
 * @note 2. Measure the axis Z reading when axis x point to world down direction
 * for 10 times.
 * @note 3. Calculate the result and finish.
 */
void IMU42688::FullCalibrate_Z() {
  // Step 1 : Check if the IMU is placed along the required diection.
  // =============
  if (Gravity % 3 != 0 || FullCalComplete[Gravity]) {
    return;
  }

  // Step 2 : Collect angle data
  // ==================================================
  CollectCalData();

  // Step 3 : Save result to buffer
  // ===============================================
  if (CalibrateCount == CalAvgNum)  // When complete collection,
  {
    // Save result to buffer.
    CalibrateCollect[Gravity][CalibrateCollectCount[Gravity]] = SumCalA[2] / CalAvgNum;
    CalibrateCollectCount[Gravity]++;

    // Reset data collect buffer.
    CalibrateCount = 0;
    memset(&SumCalA, 0, sizeof(SumCalA));

    // Measure No. n complete. (n = CalibrateCollectCount[Gravity])
    // If calibration procedure isn't finished, back to confirm page.
    CalibrateCheck = 0;
  }

  // Step 4 : Averaging the result
  // =================================================
  if (CalibrateCollectCount[Gravity] == 10)  // If complete 10 times collections on curent directions,
  {
    // Average the reading excluding the data outside 2 standard diviation.
    bool CalDevCount[10]; /** @brief bool array for \b Avg_in_2StDev
                             calaulation.*/
    memset(&CalDevCount, true, sizeof(CalDevCount));
    FullCalAngle[(int)Gravity / 3] =
        Avg_in_2StDev(&CalibrateCollect[Gravity][0], &CalDevCount[0], 10);
    // Rrecord complete.
    FullCalComplete[Gravity] = true;
  }

  // Step 5 : Calaulate rand save the result
  // ==========================================
  if (FullCalComplete[0] && FullCalComplete[3]) {
    // Calculate the result ------------------------------------------------
    // 1. Calculate Scale factor
    //  - Ideal angle reding between two result is 180.
    //  - Used the scale facter to eliminate the scale error of the IMU
    s[2] = 180 / (FullCalAngle[0] - FullCalAngle[1]);
    // 2. Calculate zero position bias
    //  - The ideal 0 position located at the middle of two reading.
    //  - Used bias to eliminate this error.
    b[2] = -(FullCalAngle[0] + FullCalAngle[1]) / 2.0;
    // 3. Set zero position error into bias.
    e[2] = b[2];

    // Save  result to flash memory --------------------------------------
    pref.begin("Angle_Cal", false);
    pref.putFloat("Sz", s[2]);
    pref.putFloat("Bz", b[2]);
    pref.putFloat("Ez", e[2]);
    pref.end();
    // Step 6 : Calibration Complete
    // ================================================
    CalibrateCheck = 2;
  }
}

void IMU42688::FullCalibrate_Y() {
  // Step 1 : Check if the IMU is placed along the required diection.
  // =============
  if (FullCalComplete[Gravity]) {
    ESP_LOGE("USER","[FullCalibrate_Y]ERROR:Gravity: %d,FullCalCompleteFlag: %d",Gravity,FullCalComplete[Gravity]);
    return;
  }

  // Step 2 : Collect angle data
  // ==================================================
  CollectCalData();

  // Step 3 : Save result to buffer
  // ===============================================
  if (CalibrateCount == CalAvgNum)  // When complete collection,
  {
    // Save result to buffer.
    CalibrateCollect[Gravity][CalibrateCollectCount[Gravity]] = SumCalA[1] / CalAvgNum;
    ESP_LOGE("USER","[FullCalibrate_Y]times: %d  ResultRaw: %f\n",
    CalibrateCollectCount[Gravity],CalibrateCollect[Gravity][CalibrateCollectCount[Gravity]]);
    CalibrateCollectCount[Gravity]++;
    ESP_LOGE("USER","[FullCalibrate_Y]Count: %d  Result: %f\n",
    CalibrateCollectCount[Gravity],
    CalibrateCollect[Gravity][CalibrateCollectCount[Gravity]]);
    // Reset data collect buffer.
    CalibrateCount = 0;
    memset(&SumCalA, 0, sizeof(SumCalA));

    // Measure No. n complete. (n = CalibrateCollectCount[Gravity])
    // If calibration procedure isn't finished, back to confirm page.
    CalibrateCheck = 0;
  }

  // Step 4 : Averaging the result
  // =================================================
  if (CalibrateCollectCount[Gravity] == 3)  // If complete 10 times collections on curent directions,
  {
    // Average the reading excluding the data outside 2 standard diviation.
    bool CalDevCount[10]; /** @brief bool array for \b Avg_in_2StDev
                             calaulation.*/
    memset(&CalDevCount, true, sizeof(CalDevCount));
    if(Gravity == 3){
      FullCalAngle[0] = Avg_in_2StDev(&CalibrateCollect[Gravity][0], &CalDevCount[0], 3);
      ESP_LOGE("USER","[FullCalibrate_Y]Gravity: %d  Result0: %f\n",Gravity,FullCalAngle[0]);
    }
    else if(Gravity == 1){
      FullCalAngle[1] = Avg_in_2StDev(&CalibrateCollect[Gravity][0], &CalDevCount[0], 3);
      ESP_LOGE("USER","[FullCalibrate_Y]Gravity: %d  Result1: %f\n",Gravity,FullCalAngle[1]);
    }
    else{
      ESP_LOGE("USER","[FullCalibrate_Y]ERROR,Gravity: %d",Gravity);
    }
    // FullCalAngle[(int)Gravity / 3] =
    //     Avg_in_2StDev(&CalibrateCollect[Gravity][0], &CalDevCount[0], 10);
    // Rrecord complete.
    FullCalComplete[Gravity] = true;
  }

  // Step 5 : Calaulate rand save the result
  // ==========================================
  // ESP_LOGE("USER","FullCalComplete[3]: %d,FullCalComplete[4]: %d",FullCalComplete[3],FullCalComplete[4]);
  if (FullCalComplete[3] && FullCalComplete[1]) {
    // ESP_LOGE("USER","FullCalComplete[3]: %d,FullCalComplete[4]: %d",FullCalComplete[3],FullCalComplete[4]);
    // Calculate the result ------------------------------------------------
    // 1. Calculate Scale factor
    //  - Ideal angle reding between two result is 180.
    //  - Used the scale facter to eliminate the scale error of the IMU
    s[1] = 90.0f / (FullCalAngle[0] - FullCalAngle[1]);
    // 2. Calculate zero position bias
    //  - The ideal 0 position located at the middle of two reading.
    //  - Used bias to eliminate this error.
    b[1] = 0;
    // 3. Set zero position error into bias.
    e[1] = b[1];

    // Save  result to flash memory --------------------------------------
    ESP_LOGE("USER","Angle_Cal");
    while(!pref.begin("Angle_Cal",false)){
      ESP_LOGE("[Angle_Cal]","put Fail");
    }
    pref.putFloat("Sy", s[1]);
    pref.putFloat("By", b[1]);
    pref.putFloat("Ey", e[1]);
    pref.end();
    // Step 6 : Calibration Complete
    CalibrateCheck = 2;
  }
}

/**
 * @brief Reset the IMU zero position errors back to origion setting.
 * @note Reset the IMU zero position errors estimate from \b QuickCalibration
 * back to the biases estimate from \b FullCalibrate() or \b FullCalibrate_Z() ,
 * and save the result in flash memory.
 */
void IMU42688::ClearZeros() {
  s[0] = 1.0f;
  s[1] = 1.0f;
  s[2] = 1.0f;
  e[0] = 0.0f;
  e[1] = 0.0f;
  e[2] = 0.0f;
  pref.begin("Angle_Cal", false);
  pref.putFloat("Sx", s[0]);
  pref.putFloat("Sy", s[1]);
  pref.putFloat("Sz", s[2]);
  pref.putFloat("Ex", e[0]);
  pref.putFloat("Ey", e[1]);
  pref.putFloat("Ez", e[2]);
  pref.end();
  CalibrateCheck = 2;
    // UI relative parameter.
  Cursor = 0;
  CursorStart = 0;
  YesNo = false;
  // Calibration data collected relative parameter.
  CalibrateCount = 0;
  FullCalStep = 0;
  memset(&SumCalA, 0, sizeof(SumCalA));
  memset(&FullCalComplete, false, sizeof(FullCalComplete));
  memset(&CalibrateCollectCount, 0, sizeof(CalibrateCollectCount));
  memset(&CalibrateCollect, 0, sizeof(CalibrateCollect));
}

/**
 * @brief Calibrate the IMU with selected method.
 * @attention Only execute when \b CalibrateCheck == 1.
 */
void IMU42688::Calibrate() {
  if (CalibrateCheck != 1) return;
  if (Cursor == 1) {
    QuickCalibrate();
  } else if (Cursor == 2) {
    ClearZeros();
  } else if (Cursor == 3) {
    FullCalibrate();
  } else if (Cursor == 4) {
    FullCalibrate_Y();
  }
}

/**
 * @brief Set all parameters related to the calibration procedure to default.
 */
void IMU42688:: CalStop() {
  // UI relative parameter.
  Cursor = 0;
  CursorStart = 0;
  CalibrateCheck = -1;
  YesNo = false;

  // Calibration data collected relative parameter.
  CalibrateCount = 0;
  FullCalStep = 0;
  memset(&SumCalA, 0, sizeof(SumCalA));
  // memset(&FullCalComplete, false, sizeof(FullCalComplete));
  memset(&CalibrateCollectCount, 0, sizeof(CalibrateCollectCount));
  memset(&CalibrateCollect, 0, sizeof(CalibrateCollect));
}

/**
 * @brief Calibration UI action.
 * @attention Not including the "Back to page ---" action.
 * @exception \b CalibrateSelect() doesn't include the function "Back to page
 * ---" ( i.e. action taken when user press select in calibration select page
 * when \b Cursor = 0). You will need to write the "Back to page ---" function
 * in UI code individually.
 * @param [in] Do : Button press action.  0 : Select, 1:Up, 2: Down, 3 :Min, 4 :
 * Add
 * @param [out] \b Cursor : Calibration method selection cursor.
 * @param [out] \b CursorStart : Method selection list start displaying flag
 * @param [out] \b YesNo : Yes No Cursor on action confirm page.
 * @param [out] \b CalibrationCheck : Indicate the user get into and pass
 * through the confirm page.
 * @note Can rewrite or delete this function and relative parameter if change
 * the user interface logic.
 */
void IMU42688::CalibrateSelect(byte Do) {
  switch (CalibrateCheck) {
    case -1:  // Calibration method selection

      if (Do == 1 &&
          Cursor >
              0)  // If user press up and still exist space for cursor to go up,
        Cursor--;  // cursor move up.
      if (Do == 2 &&
          Cursor <
              ((ExpertMode) ? 4 : 2))  // If user press down and still exist
                                       // space for cursor to go down,
        Cursor++;                      // cursor move down.
      if (Do == 0 && Cursor != 0)  // If user press select and cursor doesn't
                                   // point to "back to page ---" selection,
        CalibrateCheck = 0;        // go into the calibration confirm page.
      CursorStart = max(min((int)Cursor, (int)CursorStart),
                        (int)Cursor - Calibrate_Display_Length +
                            1);  // Calculate the list start displaying flag.
      YesNo =
          (Cursor > 2);  // Set Default Yes No Cursor for action confirm page.
      break;
    case 0:  // Action Confirm

      /* In full calibration, stop calibration WHEN USER PRESS SELECT when
       1. Gravity > 2, since the full calibration only need the data from
       Gravity = 0, 1, 2.
       2. Current gracity direction's data had been collected already.
      */
      if (Cursor == 3 && (Gravity > 2 || FullCalComplete[Gravity])) {
        if (Do == 0) CalStop();
      }
      /* In Z direction full calibration, stop calibration WHEN USER PRESS
       SELECT when
       1. Gravity = 1, 2, 4, or 5, since the z direction full calibration only
       need the data from Gravity = 0 and 3.
       2. Current gracity direction's data had been collected already.
      */
      else if (Cursor == 4 && (FullCalComplete[Gravity])) {
        if (Do == 0) CalStop();
      } else {
        if (Do == 3)            // Press Minus
          YesNo = false;        // Yes No Cursor point to NO
        if (Do == 4)            // Press Add
          YesNo = true;         // Yes No Cursor Point to YES
        if (Do == 0 && !YesNo)  // If Select No
          CalStop();            // Stop the calibration.
        if (Do == 0 && YesNo)   // If Select Yes
        {
          CalibrateCheck = 1;  // Finish the confirmation and start calibration data collect.
        }
      }
      break;
    /* If have any action when collecing data,Stop calibration */
    case 1:
      if (Do >= 0 && Do < 5)  // If have any action when collecing data.
      {
        CalStop();     // Stop calibration.
      }
      break;
  }
}  // end CalibrateSelect(byte Do)

void IMU42688::SendTOSlave(to_salve_t *to_salve) {
  String dataString;
  dataString = "";
  dataString += "<";
  dataString += String(to_salve->imu_cali_step);
  dataString += ",";
  dataString += String(to_salve->imu_cali_step);
  dataString += ">";
  Serial1.print(dataString);
}

void IMU42688::onMeasureReset() {
  measure_count = 0;
  measure_sum = 0.0f;
  stable_count  = 0;
  hold_ref  = measure_source;
}

void IMU42688::setParam(byte mode) {
  switch (mode) {
    case 0:
      measure_total = 10;
      un_hold_th = 10.0f;
      un_stable_th = 0.08f;
      break;
    case 1:
      measure_total = 5;
      un_hold_th = 10.0f;
      un_stable_th = 0.1f;
      break;
    case 2:
      measure_total = 5;
      un_hold_th = 10.0f;
      un_stable_th = 0.2f;
      break;
    default:
      ESP_LOGE("USER", "[SetParam]ERROR!!!");
      break;
  }
}

int IMU42688::processMeasureFSM() {
  byte state = manage.clino.measure.state;
  if(manage.speed_mode != 2){
    if(state == IDLE){return state;}
  }
  measure_source = AngleUser[1];
  setParam(manage.speed_mode);
  if (state == MEASURE_DONE || state == UPLOAD_DONE) {
    if(fabs(measure_source - hold_ref) > un_hold_th){
      onMeasureReset();
      manage.set_angle_progress(0);
      return state = IDLE;
    }
    return state;
  }

  // if(state == MEASURING){
    if(fabs(measure_source - stable_ref) > un_stable_th){
      onMeasureReset();
      stable_ref = measure_source;
      return state == UNSTABLE;
    }
    measure_sum += measure_source;
    measure_count++;
    manage.set_angle_progress(measure_count * 100 / measure_total);
    if(measure_count == measure_total){
      // manage.hold_clino(roundToZeroOrFive(measure_sum / measure_total,2),manage.clino.arrow_live);
      manage.hold_clino(measure_sum / measure_total,manage.clino.arrow_live);
      onMeasureReset();
      return state = MEASURE_DONE;
    }
    return state = MEASURING;
  // }

  // if(state == UNSTABLE){
  //   if(fabs(measure_source - stable_ref) > un_stable_th){
  //     onMeasureReset();
  //   }
  //   if(stable_count == stable_total){
  //     onMeasureReset();
  //     return state == MEASURING;
  //   }
  //   stable_count++;
  // }
  // return state;
}
