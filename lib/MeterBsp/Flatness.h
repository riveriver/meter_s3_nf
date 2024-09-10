/**
 * @file Flatness.h
 * @author Vicky Hung
 * @brief Scan, read, filt and calibrate the distance information from IR sensor and ads1115.
 * @version 0.1
 * @date 2023-07-31
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef Flatness_H
#define Flatness_H
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include "MeterManage.h"
#include <Preferences.h>
#include <vector>
#include "MedianFilter.h"

extern Meter manage;
class Flatness
{
private: 
#ifdef SENSOR_1_2
    #define ADS_NUM 4
    #define SENSOR_NUM 8
#elif defined(SENSOR_1_4)
#ifdef TYPE_1000
    #define ADS_NUM 1
    #define SENSOR_NUM 4
#else
    #define ADS_NUM 2
    #define SENSOR_NUM 8
#endif
#else
    #define ADS_NUM 8
    #define SENSOR_NUM 8
#endif
    Adafruit_ADS1115 ads1115[8];
    Preferences stores;
public:
    Flatness(){}
    uint16_t  raw[8]   = {0}; /** @brief ADS1115 ADC raw value buffer.*/
    float raw_peak[8]   = {0};
    float filt_peak[8]  = {0};
    float refer_peak = 0;
    float filt[8]   = {0}; /** @brief Filted ADS1115 ADC reading.*/
    float dist[8]   = {0}; /** @brief Distance value from ADCfilt.*/
    float dist_map[8]   = {0};
    float dist_linear[8]   = {0};
    float zeros[8]  = {0};
    float ft_zeros[8]  = {0};  
    float hold_ref = 0;
    float stable_ref = 0;
    float measure_sum = 0;
    float max_dist_num = 0;
    int stable_count  = 0;
void  init();
    /*****************************************************************************************************
     * Measure
     ******************************************************************************************************/
    bool ads_init[4] = {false};
    bool adc_online[4] = {false};
    bool sensor_valid[8] = {false};
    byte max_sensor = 0;
    const float dist_th = 50.0f;
    MedianFilter median;
    std::vector<float> filt_vec[8];
    std::vector<int> raw_vec[8];
    float modifyDecimal(float value);
    int  ProcessMeasureFSM();
    void UpdateAllInOne();
    void getFlatness();
    void PrintDebugInfo(byte debug_mode);
    bool HandleError_Wire(byte error, byte addr);
    void mapDist(int id, float x);
    int ifStable();
    void setZeros();
    void doAppCali();

    /*****************************************************************************************************
     * Calibration
     ******************************************************************************************************/
    /* calibration */
// 
#define MAP_NUM 12
    float map_x[8][MAP_NUM] = {
9262, 9075, 8770, 8573, 8475, 8260, 8055, 7855, 7655, 7554, 7357, 7157,
9090, 8990, 8695, 8493, 8396, 8201, 8004, 7805, 7594, 7492, 7294, 7094,
8894, 8798, 8499, 8300, 8105, 7907, 7711, 7613, 7397, 7190, 7089, 6889,
9170, 8974, 8780, 8583, 8390, 8195, 8000, 7804, 7585, 7483, 7285, 7085,
9045, 8951, 8795, 8502, 8303, 8112, 7913, 7756, 7614, 7416, 7190, 6990,
9187, 9086, 8899, 8694, 8476, 8282, 8085, 7888, 7692, 7494, 7396, 7196,
9077, 8982, 8750, 8550, 8352, 8165, 7961, 7763, 7664, 7466, 7344, 7144,
9141, 9041, 8746, 8548, 8350, 8153, 7953, 7758, 7659, 7445, 7248, 7048};
    float map_y[MAP_NUM] = {
0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,  10.0, 11.0};
    float fit_x[8] = {0};
    bool  yes_no = true;
    float progress = 0;
    float cali_progress = 0; 
    float max_peak = 0; 
    float measure_count = 0.0;
    // void doAppCali();  
    void doRobotArmCali();
    void getFitParams();
    void putFitParams();
    void set_robot_cali_ready();
};
#endif