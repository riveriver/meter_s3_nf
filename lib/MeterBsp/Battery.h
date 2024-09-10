#ifndef Battery_H
#define Battery_H

#include <Arduino.h>
#include "MeterManage.h"
extern Meter manage;

class Battery
{
public:
    int Percent = 0;
private:
    int stage_table[6] = {2360,2300,2180,2060,1840,1630};
    // 根据电池剩余电量查找对应的寿命
    int LookupBatteryPercent(int adc_raw) {
        if (adc_raw > stage_table[1]) {
            return 100;
        }
        else if(adc_raw > stage_table[2]){
            return 75;
        }
        else if(adc_raw > stage_table[3]){
            return 50;
        }
        else if(adc_raw > stage_table[4]){
            return 25;
        }else{
            return 0;
        }
    }
    byte p;
    int V[2] = {0};
    float V_BW = 0;
public:
    void SetPin(byte Pin)
    {
        p = 1;
        pinMode(p, INPUT);
        V[0] = analogRead(p);
        V_BW = analogRead(p);
    }

    void Update_BW()
    {
        V[1] = V[0];
        V[0] = analogRead(p);
        V_BW = V_BW * 0.96 + V[0] * 0.02 + V[1] * 0.02;

#ifdef HARDWARE_2_0
    // 4.2 -> 2370, 3.6-> 2120
    Percent = ((V_BW - 2120.0) * 100.0 / 250.0 + 0.5);
#else
    Percent = LookupBatteryPercent(V_BW);
#endif
    manage.battery = min(max(Percent, 0), 100);
}

};

#endif