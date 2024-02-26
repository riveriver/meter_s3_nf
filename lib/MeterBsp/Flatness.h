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
#include "MaxMin.h"
#include "SLED.h"
#include "MeterManage.h"
#include <Preferences.h>

extern Meter manage;

class Flatness
{
private: 
    const bool debug_mode = 1;
    const bool Serial_Print_Param       = true;  /** @brief True if require printing the I2C scanning result in Setup.*/
    const bool Serial_Print_Raw_Data    = true;   /** @brief True if require printing the Raw Data while Update.*/
    const bool Serial_Print_Distance    = false;   /** @brief True if require printing the Distance while Update.*/
    const int   Cut_Off_Voltage = 2000;/** @brief Cut-off voltage*/
    const float Cut_Off_Distance = 99.9f;/** @brief Cut-off Distance*/
    const byte IIC_1_SCL = 9;   
    const byte IIC_1_SDA = 8;
    const byte IIC_2_SCL = 4;
    const byte IIC_2_SDA = 5;
    const int  ADS_1_ADDR = 0x49;
    const int  ADS_2_ADDR = 0x49;
    int offline_time = millis();
    
    // stores
    enum StoresError {
      ERROR_SUCCESS = 0,
      ERROR_NULL_POINTER = 1,
      ERROR_BEGIN_FAIL = 2,
    };
    enum CommStatus {
    STEP_FRAME_HEAD,
    STEP_PRASE_CMD,
    STEP_PRASE_DATA,
  };
  // HACK
    bool has_flat_cali = false;
    byte unpack_step = STEP_FRAME_HEAD;
    char rx_sub_buffer[256];
    char rx_sub_index;
    Preferences stores;
    unsigned long action_timeout = 100;

    // calibration
    #define cali_size 11 
    #define sensor_size 8
    float slopes[8] = {0};
    float zeros[8] = {0}; /** @brief Voltage reading when distance = 0.*/
    const int sample_size  = 20; /** @brief Number of data require for calibrating zeros.*/
    const int sample_threshold = 110; /** @brief Stabilization identification threshold when doing calibration.*/
    int sample_count = 0;
    uint16_t sample_start[8] = {0};
    unsigned int sample_sum[8] = {0};
    const int  Iteration_size = 120;
    float cali_target[cali_size] = {0.01f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f};
    // float cali_sample[sensor_size][cali_size] = {0};
    // Testing
    float cali_sample[sensor_size][cali_size] = {
    {9284.74, 9089.80, 8794.04, 8598.54, 8384.79, 8178.11, 8002.90, 7864.10, 7680.26, 7484.30, 7363.76},
    {9455.39, 9166.42, 8965.61, 8737.30, 8534.51, 8277.72, 8134.43, 7865.87, 7749.15, 7537.39, 7339.20},
    {9663.16, 9383.07, 9166.26, 8968.87, 8672.78, 8477.28, 8318.95, 8081.02, 7944.45, 7759.41, 7605.08},
    {9620.26, 9321.40, 9057.17, 8831.13, 8598.86, 8385.34, 8284.14, 8011.59, 7815.78, 7708.15, 7515.58},
    {9255.09, 9063.94, 8772.17, 8635.35, 8415.24, 8233.27, 8043.61, 7846.69, 7648.52, 7462.35, 7348.82},
    {9156.36, 8898.76, 8669.68, 8474.58, 8227.06, 8028.68, 7854.94, 7635.39, 7524.48, 7338.47, 7171.33},
    {9547.12, 9256.83, 9063.58, 8846.53, 8649.42, 8449.86, 8254.50, 8055.44, 7857.88, 7758.30, 7561.44},
    {9016.54, 8812.82, 8607.98, 8310.62, 8111.38, 7912.58, 7712.95, 7513.79, 7315.36, 7192.48, 6988.30}};
    // #define cali_num 11 
    // float cali_fix_y[cali_num] = {0.01,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0}; 
    // float cali_fix_1[cali_num] = {0}; 
    // float cali_fix_2[cali_num] = {0};
    // float cali_fix_3[cali_num] = {0};
    // float cali_fix_4[cali_num] = {0};
    // float cali_fix_5[cali_num] = {0};
    // float cali_fix_6[cali_num] = {0};
    // float cali_fix_7[cali_num] = {0};
    Adafruit_ADS1115 ads1115[2];       /** @brief Sensor driver for the Adafruit ADS1115 ADC breakout.*/
    uint16_t ADCread[2][8] = {0}; /** @brief ADS1115 ADC raw value buffer.*/
    bool isUpdate[2] = {0};        /** @brief isUpdate[i] = True if sensor connect to Wire(i) is update.*/
    // Filter Setting
    const int   surges_TH = 50;              /** @brief Surges filter threshold @deprecated &Delta;Distance &asymp; surges_TH * 0.005 mm (@ ADCread &asymp; 8000).*/
    const int   BW_Reset_TH = 500;           /** @brief Reset the Butterworth filter if the raw data exceed the BW_Reset_TH.*/
    const float BW_C[3] = {0.8, 0.1, 0.1}; /** @brief Buttorworth filter coefficient @note c0 + c1 + c2 = 1.0 @note c1 = c2*/
    /*****************************************************************************************************
     * Calibration buffer.
     ******************************************************************************************************/
    void  updateSingleChannel(int id);
    void  collectCaliSample();
    void  caliZero();
    void  doCalibration();
    float calcIterativeCoefficientsB0(int id, float *sample, float *target,uint8_t size);
    byte getDistanceZero(float *data, int data_size);
    byte putDistanceZero(float *data, int data_size);
    byte getDistanceScale(float *data, int data_size);
    byte putDistanceScale(float *data, int data_size);

public:
    Flatness(){
      for (int i = 0; i < 8; i++) {
          slopes[i] = 330000;
          zeros[i] = 10000;
      }
    }
    byte ads_connect[2] = {false};

    /*****************************************************************************************************
     * Basic.
     ******************************************************************************************************/
    
     /** @brief Default distance when ads reading under the cut-off voltage.*/
    uint16_t ADCSurgeOut[20][8] = {0}; /** @brief ADS1115 ADC reading with surge filter.*/
    float ADCfilt[8]   = {0}; /** @brief Filted ADS1115 ADC reading.*/
    float distance[8]  = {0}; /** @brief Distance value calculate from ADCfilt.*/
    float sub_distance[8] = {0}; /** @brief Distance value calculate from ADCfilt.*/
    MaxMin Mm;

    /*****************************************************************************************************
     * CalibrationZero
     ******************************************************************************************************/
    bool Enable_Auto_Reset = false;            /** @brief True if enable auto zero position calibration.*/
    bool Enable_Cali_Slope = false;
    bool Read_Slope_First  = false;
    bool has_set_zero     = Enable_Auto_Reset; /** @brief True when doing zero position calibration.*/
    float SetZeroProgress = Enable_Auto_Reset; /** @brief Zero position calibration's data collection progerss.*/
    /*****************************************************************************************************
     * Calibration
     ******************************************************************************************************/
    void  init();
    void  update(byte wire);
    void  reset(bool onoff, bool force);
    float get_Mm_Diff(){return Mm.Diff;};
    
    void  sendToHost(float *data,int size); // send sub distance
    void  sendToSub(); // send set zero cmd
    void  unpackFromHost(unsigned char info); //unpack set zero cmd
    void  unpackFromSub(unsigned char info); // unpack sub distance
    void  parseFromSub(); // prase sub distance
    void  Test_receiveFromSub();
    int   processMeasureFSM();
};
#endif