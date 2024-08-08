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
#include "MeterManage.h"
extern Meter manage;

class IMU42688 {
 private:
enum MeasureStatus {
    M_IDLE,
    M_UNSTABLE,
    M_MEASURE_ING,
    M_MEASURE_DONE,
    M_UPLOAD_DONE
};

enum CommStatus{
    STEP_FRAME_HEAD,
    STEP_PRASE_CMD,
    STEP_PRASE_DATA,
    STEP_PRASE_CALI
};
public:
    void init(uint8_t Rx, uint8_t Tx);
    uint8_t unpack_step = STEP_FRAME_HEAD; 
    char data_rx_buffer[64];
    uint8_t data_rx_index;
    char cali_rx_buffer[64];
    uint8_t cali_rx_index;
    float info_parsed[10];
    bool has_new_data = false;
    bool has_imu_data = false;
    uint8_t Update();
    void unpackFromC3(unsigned char info);
    void parseImuData(); 

    uint8_t _gravity = 4;
    uint8_t _last_gravity = 3;
    float angle_raw[3] = {0, 0, 0}; 
    float angle_cali[3] = {0, 0, 0}; 
    float angle_std[3] = {0, 0, 0};
    float angle_user[3] = {0, 0, 0};
    float angle_raw_show[3] = {0, 0, 0}; /** @brief Raw special filted angle in degree. */
    float angle_std_show[3] = {0, 0, 0};
    float angle_cali_show[3] = {0, 0, 0}; 
    float angle_user_show[3] = {0, 0, 0};
    uint8_t stable_count;
    uint8_t stable_total = 10;
    float un_stable_th  = 0.3f;
    float stable_ref;
    float   measure_source;
    uint8_t measure_total = 10;
    uint8_t measure_count;
    float measure_sum;
    float un_hold_th    = 1.5f;
    float hold_ref;
    float set_zero_th   = 0.5f;
    int  ProcessMeasureFSM(); 
    void setParam(uint8_t mode);
    void onMeasureReset();
    float b[3] = {90.0f,90.0f,90.0f};
    float e[3] = {90.0f,90.0f,90.0f};
    uint8_t cali_state = IMU_COMMON;
    bool yes_no = false;
    const uint8_t avg_total = 20; 
    uint8_t avg_count = 0; 
    uint8_t _start_g; /** @brief _gravity direction when the angle collection start.Used to identify the severe change on IMU's attitude.*/
    float _start_angle[3]; /** @brief Angle value when the angle collection start.Used to identify IMU's stability.*/
    float _sum_angle[3]; /** @brief \b CollectCalData() collected angle summation.*/
    float cali_progress = 0;
    void CollectCalData();
    void ClearZeros();
    void QuickCalibrate();
    void StopCali();
    void CaliFactoryZero();
    void ResetFactoryZero();

};

#endif