#ifndef OLED_H
#define OLED_H
#include <U8g2lib.h>
#include "Flatness.h"
#include "Measure.h"
#include "IMU42688.h"
// #include "RealTimeClock.h"
// #include "Serial_Communication_Pogo.h"
#include "SLED.h"
#include <Preferences.h>
#include "MeterManage.h"
extern Meter manage;
#ifndef CS1
#define CS1 10
#endif
#ifndef CS2
#define CS2 42
#endif
#ifndef IO_OLED_RST
#define IO_OLED_RST 41
#endif

class OLED
{
private:
    int     Timer         = 0;
    bool    direction     = 0;
    bool    isU8g2Begin   = false;
    byte IO_VCC        = 0;
    byte R_now         = 0;
    byte measure_state;
    byte measure_progress;
    String  BlockInfo;
    void DoBlock();
    void DrawNum_10x16(int x, int y, String f);
    void DrawNum_16x24(int x, int y, String f);
    void Minor_DrawNum_10x16(int x, int y, String f);
    void Minor_DrawNum_16x24(int x, int y, String f);
    void ProcessAngleShow(float *ui_show);
    void Warning(float angle,float dist);
    /* ----- Menu -----*/
    void Menu();
    void Bluetooth();
    void TypeSelect();
    void AngleXYZ();
    void ShowDistData();
    void Show_Sensor_Address();
    void One_Sensor_Info();
    void Show_System_Info();
    void Cal_Menu();
    void Cal_Check();
    void YesNo(bool IsH, bool Select);
    void ImuCalibration();
    void FlatnessSetZero();
    void CaliFlatness();
    void AutoCaliFlatness();
    
    /* ----- Home ----- */
    bool ifSwitchHome();
    void DrawPrimaryCommon();
    void DrawMinorCommon();
    void DrawPrimaryArrow(); 
    void DrawMinorArrow(); 
    void DrawPrimaryAngle();
    void DrawMinorAngle();
    void DrawPrimarySlope();
    void DrawMinorSlope();
    void DrawPrimaryFlat();
    void DrawMinorFlat();
    void DrawPrimaryFlatSlope();
    void DrawMinorFlatSlope();
public:    
    SLED *pLED;
    int *pBatt;
    uint8_t *pBLEState;
    IMU42688 *pIMU;
    Flatness *pDS;
    Measure *pMeasure;
    void TurnOn(byte VCC);
    void TurnOff();
    void DoRST();
    void Update();
    void Block(String Info, int time);
};

#endif