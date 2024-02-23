/**
 * @file IMU42688.h
 * @author Vicky Hung
 * @brief Cope, filted, and calibrate the data from WC_IMU
 * @version 0.1
 * @date 2023-07-31
 *
 * @copyright Wonder Construct (c) 2023
 *
 */

#ifndef IMU42688_h
#define IMU42688_h
#include <Arduino.h>
#include "SLED.h"
#include "MeterManage.h"
extern Meter manage;
#ifndef IMU_C
#define IMU_C 3
#endif

struct to_salve_t {
  uint8_t imu_cali_status;
  uint8_t imu_cali_step;
};

class IMU42688 {
 private:
  /******************************************************************************************************
   * Parameter setting.
   *******************************************************************************************************/
  const bool Debug_Mode = 1;
  const bool Skip_Warm_Up = true; /** @brief True if want to run through the warm up procedure in 10
               seconds @warning For debug only, never print in final version*/
  const bool Serial_Print_Update_Data = false; /** @brief Set true to print recieved IMU data. @warning For debug
                only, never print in final version*/
  const bool Serial_Print_Update_Acc_Gyro = false; 
  const bool Serial_Print_Warm_Up_Progress = false; /** @brief True to print the warm-up pregress.*/

    enum MeasureStatus {
        IDLE,
        UNSTABLE,
        MEASURING,
        MEASURE_DONE,
        UPLOAD_DONE
    };
    enum CommStatus{
        STEP_FRAME_HEAD,
        STEP_PRASE_CMD,
        STEP_PRASE_DATA,
        STEP_PRASE_CALI
    };
    #define RX_BUFFER_SIZE 128
    MeasureStatus measure_state;
    byte unpack_step = STEP_FRAME_HEAD; 
    char data_rx_buffer[RX_BUFFER_SIZE];
    byte data_rx_index;
    char cali_rx_buffer[RX_BUFFER_SIZE];
    byte cali_rx_index;
    bool has_imu_data = false;
    float imu_raw[12];
    float measure_source;
    float un_stable_th  = 10.0f;
    float un_hold_th    = 10.0f;
    int   measure_count;
    int   measure_total = 10;
    float measure_sum;
    int   stable_count;
    int   stable_total = 10;
    float hold_ref;
    float stable_ref;
    byte  measuring_percent;
  // Warm-up Setting
  /**
   * @brief Temperature stability determination period ( seconds ).
   * @note Sensor temperature stability is used as the secondary reference for
   * warm-up situation determination. Therefore, peroid = 90 (1.5 minutes) is a
   * good choice.
   */
  const int SteadyTempCalPeriod = 90;
  const float SteadyTempTH = 0.5; /** @brief Temperature stability threshold ( C ).*/
  const float WarmUpTemperature = 40; /** @brief Default IMU warm-up complete temperature ( C ).*/
  // For calibration
  const float Cal_TH = 0.1; /** @brief Threshold value for identify the stability of IMU in
               calibration data collection.*/
  // For calibration UI
  const uint8_t Calibrate_Display_Length = 3; /** @brief Number of rows display in calibration method select UI.*/
  const uint8_t Cal_LED_Pri = 6; /** @brief LED hint display priority for calibration ( 0 to 6 ) @note
            Make sure the code turn off the correct LED.*/

  /*****************************************************************************************************
   * IMU informations.
   ******************************************************************************************************/

  float AngleRawShow[3] = {0, 0, 0}; /** @brief Raw special filted angle in degree.
                                     < angle X , angle Y , angle Z > */
  float SensorTemperatureCollect[2] = {0}; /** @brief Raw IMU temperature. < T ( t ) , T ( t - dt ) >*/
  float MaxTemp = 0; /** @brief Detected maximum temperature within the
                        temperature stability determination period.*/
  float minTemp = WarmUpTemperature;      /** @brief Detected minimum temperature within the
                                 temperature stability determination period.*/
  float StartTemperature;     /** @brief Sensor intiail temperature*/
  int SteadyTempCalStart = 0; /** @brief Timestamp when start the temperature
                                 stability determination period.*/
  int CopeFailed = 0;         /** @brief Coped failed counter*/

  String String_rad(float degree);
  String String_mm(float degree);

  /*****************************************************************************************************
   * IMU parmeter load from flash memory.
   ******************************************************************************************************/

  // float e[3] = {0.0}; /** @brief IMU error between user-set zero positions
  // and raw data. < error X , error Y , error Z >*/
  float b[3] = {0.0}; /** @brief IMU bias between lab calibrated zero positions
                         and raw data. < bias X , bias Y , bias Z >*/

  /********************************************************************************************************
   * Calibration Information.
   *********************************************************************************************************/
  bool Have_New_Data_for_Collect =
      false;     /** @brief True if imu already update and the value havn't been
                    collected by calibration.*/
  int StartCalG; /** @brief Gravity direction when the angle collection start.
                    Used to identify the severe change on IMU's attitude.*/
  float StartCalA[3]; /** @brief Angle value when the angle collection start.
                         Used to identify IMU's stability.*/
  float CalibrateCollect[6][10] = {
      0.0}; /** @brief \b FullCalibrate_Z() angle measurement result buffer.*/
  float SumCalA[3]; /** @brief \b CollectCalData() collected angle summation.*/
  float FullCalAngle[6] = {
      0.0}; /** @brief Result angle in corresponding gravity direction.*/

  float Avg_in_2StDev(float *Angle, bool *Count, int length);
  void CollectCalData();
  void QuickCalibrate();
  void FullCalibrate();
  void FullCalibrate_Z();
  void FullCalibrate_Y();
  void ClearZeros();
  
  void onMeasureReset();
  void unpackFromC3(unsigned char info);
  void parseImuData(); 
  void setParam(byte mode);

 public:
 int  processMeasureFSM(); 
 to_salve_t send_to_salve;
 void SendTOSlave(to_salve_t *to_salve);
 float acc_raw[3];
 float gyro_raw[3];
  /********************************************************************************************************
   * Remember to set these values in setup()
   * Chance to cause the panic / crash / ... if not setting these pointer.
   ********************************************************************************************************/
  int *fWarmUpTime;    /** @brief Pointer to the time-off auto-sleep clock*/
  uint8_t *ExpertMode; /** @brief True if allow getting into full calibration
                          function.*/

  /********************************************************************************************************
   * IMU Information
   ********************************************************************************************************/
  /**
   * @brief Indicate current IMU update situations.
   * @note 0 - Update success.
   * @note 1 - IMU not warm up.
   * @note 2 - Recieve data formatting error.
   * @note 3 - Recieve angle value located outside the threshold.
   * @note >4 - Times that IMU coped failed. Usually because Serial1 not
   * available.
   * */
  uint8_t ErrorCode = Err_IMU_Not_Warm_Up;
  const uint8_t IMU_Update_Success =
      0; /** Current imu Update situation indicator.*/
  const uint8_t Err_IMU_Not_Warm_Up =
      1; /** Current imu Update situation indicator.*/
  const uint8_t Err_IMU_Receive_Data_Error =
      2; /** Current imu Update situation indicator.*/
  const uint8_t Err_IMU_Data_StDev_Outside_TrustHold =
      3; /** Current imu Update situation indicator.*/
  const uint8_t Err_IMU_Cope_Failed =
      4; /** Current imu Update situation indicator.*/

  /**
   * @brief Current Gravity direction
   * @note 0 : +X direction @note 1 : +Y direction @note 2 : +Z direction
   * @note 3 : -X direction @note 4 : -Y direction @note 5 : -Z direction
   */
  uint8_t Gravity = 3;

  /**
   * @brief Gravity Direction for interface rotation
   * @note @note 1 : +Y direction @note 4 : -Y direction
   */
  uint8_t GravityPrevious = 3;

  float AngleRaw[3] = {0, 0, 0}; /** @brief Raw real-time angle in degree. < angle
                                 X , angle Y , angle Z >*/
  float AngleCal[3] = {0, 0, 0}; /** @brief Calibrated real-time angle in
                                    degree. < angle X , angle Y , angle Z >*/
  float AngleStd[3] = {0, 0, 0};
  float AngleUser[3] = {0, 0, 0};
  float AngleCal_ExG[3] = {
      0, 0, 0}; /** @brief Calibrated real-time angle in degree. Set the G
                   direction angle to 200. < angle X , angle Y , angle Z >*/
  float AngleStdShow[3] = {0, 0, 0};
  float AngleCalShow[3] = {
      0, 0, 0}; /** @brief Calibrated special filted angle in degree (for
                   display only). < angle X , angle Y , angle Z >*/
  float AngleUserShow[3] = {0, 0, 0};
//   float SensorTemperature; /** @brief Filted sensor temperature ( C ).*/
  uint8_t unit = 0; /** @brief Current interface levelness display unit. @note 0
                       : degree @note  1 : mm/m @note 2 : radian */
  uint8_t fWarmUp;  /** @brief IMU warm-up % (0 to 100).*/
  String Cal_Info_From_Flash =
      ""; /** @brief IMU calibrated parameter load from flash memory.*/

  /**********************************************************************************************************
   * IMU Regular Function
   **********************************************************************************************************/

  void Initialize(byte Rx /*(-1)*/, byte Tx /*(-1)*/);
  uint8_t Update();
  String String_now_unit(float degree, uint8_t unit);
  String String_degree(float degree);
  // String String_now_unit(float degree);
  String String_Slope_mm(float degree);
  void SetUnit(String Info);
//   void SendTOSlave(to_salve_t *to_salve);

  /**********************************************************************************************************
   * Calibrate function
   **********************************************************************************************************/

  /*********************************************************************************************************
   * Calibration UI ( user interface ) relative,
   * Put these parameters and function here in order to make the UI code look
   *clear. If the complex code is acceptable in the UI code, than it is better
   *to rewrite these part of code in the UI code file.
   **********************************************************************************************************/

  /**
   * @brief Indicate the user calibration status.
   * @note \b CalibrateCheck value indicate that the user is currently in the
   * state listed below :
   * @note -1 : Not in the calibration procedure or in the calibration select
   * menu.
   * @note 0 : In the confirm page.
   * @note 1 : Pass the confirm page and start collecting the data.
   * @note 2 : Calibration done.
   * @note --------------------------------------------------------------------
   * @note The calibration procedure may be different in different calibration
   * method selections. For example, if the calibration require 3 times measure
   * result, the procedure may appear like
   * @note -1 -> 0 -> 1 -> 0 -> 1 -> 0 -> 1 -> 2
   * @note If user select "Back" or "No" in the confirm page, the procedure will
   * turn back to -1.
   */
  int CalibrateCheck = -1;

  /**
   * @brief Calibration method selection cursor.
   * @note Calibration method select menu arrange as below :
   * @note 0 : Back to page ---.
   * @note 1 : Quick calibration.
   * @note 2 : Reset user-set zero position error.
   * @note 3 : Full calibrate.
   * @note 4 : Full calibration for Z axis
   */
  uint8_t Cursor = 0;
  uint8_t CursorStart = 0; /** @brief Method selection list start displaying
                              flag. @see \b Cursor */
  bool YesNo = false;      /** @brief Yes No Cursor on action confirm page.*/

  const uint8_t CalAvgNum = 10; /** @brief The required number of raw angle data
                                   for generating one angle result.*/
  uint8_t CalibrateCount = 0; /** @brief The number of collected raw angle data
                                 for generating the angle result.*/
  uint8_t CalibrateCollectCount[6] = {
      0}; /** @brief The number of angle measurement result on corresponding
             gravity direction.*/
  bool FullCalComplete[6] = {
      false}; /** @brief True if number of measurement result is enough in the
                 corresponding direction.*/

  void CalibrateSelect(byte Do);
  void CalStop();
  void Calibrate();

  int FullCalStep = 0;
  float e[3] = {0.0};
  float s[3] = {0.0}; /** @brief IMU data scale error. < scale X , scale Y , scale Z >*/
  /***********************************************************
   * Function not used currently.
   ***********************************************************/
};

#endif