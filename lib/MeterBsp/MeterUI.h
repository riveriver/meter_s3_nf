#ifndef DISPLAY_H
#define DISPLAY_H
#include <U8g2lib.h>
#include "Flatness.h"
#include "IMU42688.h"
#include <Preferences.h>
#include "MeterManage.h"
extern Meter manage;

class MeterUI
{
private:
    struct INFO_KEY_VALUE {
        const char* key;
        float value;
    };
    // measure
    byte  measure_state = 0;
    byte  measure_bar = 0;
    float angle_show = 0;
    float slope_show = 0;
    float flat_show = 0;
    byte  auto_mode_select  = 5;
    byte  dash_num = 0;
    char  str_show[6] = "-----";
    char  str_slope[7] = "-----";
    /* screen */
    bool is_begin = false;
    byte rotation = 0;
    byte g_this = 0;
    byte g_last = 0;
    uint8_t bar_timer = 0;
    String  block_info = "";
    void Flip();
    void FlipFirst();
    void DoBlock();
    void drawNum_10x16(int x,int y,String str,int size);
    void drawNum_16x24(int x,int y,String str,int size);
    void Minor_drawNum_10x16(int x, int y,String str,int size);
    void Minor_drawNum_16x24(int x, int y,String str,int size);
    /* ----- Menu -----*/
    void pageCaliMenu();
    void pageSwitchBLE();
    void pageSwitchLight();
    void pageAngleCaliInfo();
    void pageImuFactoryZero();
    void pageOption_YesNo(bool option);
    void pageImuInfo();
    void pageInfo(int selection);
    void pageCalAngleCheck();
    void pageCaliFlatCheck();
    void pageFlatFactoryZero();
    void pageAutoCaliFlatness();
    void pageResetFactoryZero();
    void pageRobotCali();
    void pageRobotCaliFlatness();
    void pageRobotCaliAngle();
    /* ----- Home ----- */
    bool ifSwitchHome();
    void Primary_DrawCommon();
    void Primary_DrawArrow(); 
    void Primary_DrawAngle();
    void Primary_DrawSlope();
    void Primary_DrawFlat();
    void Primary_DrawFlatSlope();
    void Primary_DrawHome();
    void Minor_DrawCommon();
    void Minor_DrawArrow(); 
    void Minor_DrawAngle();
    void Minor_DrawSlope();
    void Minor_DrawFlat();
    void Minor_DrawFlatSlope();
public:  
    int *pBattry;
    uint8_t *pBLEState;
    IMU42688 *pIMU;
    Flatness *pDS;
    void TurnOn();
    void TurnOff();
    void Update();
    void Block(String Info, int time);
};

#endif