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
#ifdef HARDWARE_0_0
    #define ADS_NUM 8
    #define SENSOR_NUM 4
#elif defined(HARDWARE_1_0)
    #define ADS_NUM 1
    #define SENSOR_NUM 2
#elif defined(HARDWARE_2_0)
    #define ADS_NUM 2
    #define SENSOR_NUM 7
#elif defined(HARDWARE_3_0)
// #ifdef TYPE_1000
//     #define ADS_NUM 2
//     #define SENSOR_NUM 4
// #elif defined(TYPE_2000)
//     #define ADS_NUM 4
//     #define SENSOR_NUM 8
// #else
//     #define ADS_NUM 4
//     #define SENSOR_NUM 8
// #endif
    #define ADS_NUM 1
    #define SENSOR_NUM 4
#else
    #define ADS_NUM 8
    #define SENSOR_NUM 8
#endif
    Adafruit_ADS1115 ads1115[ADS_NUM];
    Preferences stores;
public:
    Flatness(){}
    
    bool ads_block = false;
    uint16_t  raw[8]   = {0}; /** @brief ADS1115 ADC raw value buffer.*/
    float raw_peak[8]   = {0};
    float filt_peak[8]  = {0};
    float filt[8]   = {0}; /** @brief Filted ADS1115 ADC reading.*/
    float dist[8]   = {0}; /** @brief Distance value CalculateFlatness from ADCfilt.*/
    float dist_map[8]   = {0};
    float dist_linear[8]   = {0};
    float zeros[8]  = {0};
    float ft_zeros[8]  = {0};  
    float hold_ref = 0;
    float stable_ref = 0;
    byte measure_count = 0;
    float measure_sum = 0;
void  init();
    /*****************************************************************************************************
     * Measure
     ******************************************************************************************************/
    bool ads_init[4] = {false};
    bool ads_online[4] = {false};
    bool sensor_online[8] = {false};
    byte max_sensor = 0;
    const float dist_th = 50.0f;
    MedianFilter median;
    std::vector<int> filt_vec[8];
    std::vector<int> raw_vec[8];
    float modifyDecimal(float value);
    int  ProcessMeasureFSM();
    void UpdateAllInOne();
    void CalculateFlatness();
    void PrintDebugInfo(byte debug);
    bool HandleError_Wire(byte error, byte addr);
    void mapDist(int id, float x);
    int ifStable();
    void setZeros();

    /*****************************************************************************************************
     * Calibration
     ******************************************************************************************************/
    /* calibration */
// 
#define MAP_NUM 11
    float map_x[8][MAP_NUM] = {
9262, 9075, 8770, 8573, 8475, 8260, 8055, 7855, 7655, 7554, 7357,
9090, 8990, 8695, 8493, 8396, 8201, 8004, 7805, 7594, 7492, 7294,
8894, 8798, 8499, 8300, 8105, 7907, 7711, 7613, 7397, 7190, 7089,
9170, 8974, 8780, 8583, 8390, 8195, 8000, 7804, 7585, 7483, 7285,
9045, 8951, 8795, 8502, 8303, 8112, 7913, 7756, 7614, 7416, 7190,
9187, 9086, 8899, 8694, 8476, 8282, 8085, 7888, 7692, 7494, 7396,
9077, 8982, 8750, 8550, 8352, 8165, 7961, 7763, 7664, 7466, 7344,
9141, 9041, 8746, 8548, 8350, 8153, 7953, 7758, 7659, 7445, 7248};
    float map_y[MAP_NUM] = {
0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0,  9.0,  10.0};
    float fit_x[8][MAP_NUM] = {0};
    bool  yes_no = false;
    float progress = 0;
    float cali_progress = 0; 
    void  doCalibration();
    void getFitParams();
    void putFitParams();
};
#endif