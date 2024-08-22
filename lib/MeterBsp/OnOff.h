#ifndef OnOff_H
#define OnOff_H
#include <Arduino.h>
#include <MeterUI.h>
#include <SLED.h>
#include <Preferences.h>

class OnOff
{
private:
    const int on_time = 1000;             /** @brief Time to trigger Long Press power off.*/
    const int off_time = 1000;             /** @brief Time period for showing the power off image.*/
    int off_count = 0;
    MeterUI *pUI;
    SLED *p_led;
    byte OnOffPin;
    byte EN_Pin;

public:
    int last_active = 0;

    /**
     * @brief System power on procedure.
     *
     * @param IO_OnOff Button to detect wake up.
     * @param IO_EN Power controll GPIO. Set high when system power on.
     * @param LED Pointer to SLED.
     * @param Bat Pointer to battery.
     * @param ui Pointer to ui.
     */
    void On(byte IO_OnOff, byte IO_EN, SLED &LED, MeterUI &ui)
    {
        // Enable VCC
        pinMode(IO_OnOff, INPUT);
        pinMode(IO_EN, OUTPUT);
        digitalWrite(IO_EN, HIGH);
        pinMode(IO_LIGHT_GREEN,OUTPUT); 
        digitalWrite(IO_LIGHT_GREEN,HIGH);
        pinMode(IO_LIGHT_RED,OUTPUT); 
        digitalWrite(IO_LIGHT_RED,HIGH);
        Serial.begin(115200);
        while (millis() < on_time && digitalRead(IO_OnOff))
        {
            delay(100);
        }
        // If is not long press
        if (millis() < on_time)
        {
            ESP_LOGE("","If is not long press");
            digitalWrite(IO_EN, LOW);
            while (true){}
        }
        
        pUI = &ui;
        p_led = &LED;
        OnOffPin = IO_OnOff;
        EN_Pin = IO_EN;
    }

    /**
     * @brief Start the long press detection clock
     */
    void Off_Clock_Start()
    {
        off_count = millis();
    }

    /**
     * @brief Stop the long press detection clock
     */
    void Off_Clock_Stop()
    {
        off_count = 0;
    }

    /**
     * @brief Check if system should power off
     *
     */
    void Off_Clock_Check()
    {
        bool CommandSleep = (off_count == 0) ? false : (millis() - off_count) > on_time;
        bool AutoSleep = ((millis() - last_active > manage.sleep_time * 60 * 1000) && millis() > manage.sleep_time * 60 * 1000);
        // bool AutoSleep = 0;
        if (!CommandSleep && !AutoSleep)return;
        if (CommandSleep)
        {
            pUI->Block("Shutting Down", off_time);
            ESP_LOGE("","Command Sleep");
        }
        else if (AutoSleep)
        {
            pUI->Block("Auto Sleep", 5000);
            ESP_LOGE("","Auto Sleep");
        }

        // Power Off
        if (CommandSleep || AutoSleep)
        {
            // Wait for Powe Off Display finish
            int start = millis();
            while (millis() - start < off_time)
            {
                // If user press button when showing time out power off, stop the power off procedure.
                if (AutoSleep)
                {
                    if (millis() - last_active < off_time)
                    {
                        manage.block_time = millis();
                        return;
                    }
                }
            }
            // Wait for button release
            while (digitalRead(OnOffPin)){}
            pUI->TurnOff();
            digitalWrite(EN_Pin, LOW);
            while (true){};
        }
    }
};

#endif