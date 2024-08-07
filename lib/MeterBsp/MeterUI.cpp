/**
 * @file MeterUI.cpp
 * @author YANG
 * @brief 
 * @version 0.1
 * @date 2024-06-24
 * @note 
 * @copyright Copyright (c) 2024
 * 
 */
#include "MeterUI.h"
#include <U8g2lib.h>
#include "Bitmap_Manage.h"
#include "MeterManage.h"

#define NUM_LHX 16
#define NUM_LHY 20
#define NUM_LVX 0
#define NUM_LVY 52
#define LINE_LHX 60
#define LINE_LHY 32
#define LINE_LVX 6
#define LINE_LVY 64

#ifdef HARDWARE_1_0
#define SCREEN_RST 41
#define SCREEN_CS1 10
#define SCREEN_CS2 42
#define SCREEN_VCC 37
#define HORIZONTAL 0
#define VERTICAL 1
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI screen_h(U8G2_R0, /*CLK*/ 36, /*SDA*/ 35,
                                                /*CS*/ 10, /*DC*/ 15,
                                                /*RST*/ 41);
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI screen_s(U8G2_R0, /*CLK*/ 36, /*SDA*/ 35,
                                                /*CS*/ 42, /*DC*/ 15,
                                                /*RST*/ 41);

#elif defined(HARDWARE_2_0)
#define SCREEN_RST 41
#define SCREEN_CS1 10
#define SCREEN_CS2 42
#define SCREEN_VCC 37
#define HORIZONTAL 0
#define VERTICAL 1
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI screen_h(U8G2_R0, /*CLK*/ 36, /*SDA*/ 35,
                                                /*CS*/ 10, /*DC*/ 15,
                                                /*RST*/ 41);
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI screen_s(U8G2_R0, /*CLK*/ 36, /*SDA*/ 35,
                                                /*CS*/ 42, /*DC*/ 15,
                                                /*RST*/ 41);
void MeterUI::TurnOn() {
  pinMode(SCREEN_CS1, OUTPUT);
  digitalWrite(SCREEN_CS1, LOW);
  pinMode(SCREEN_CS2, OUTPUT);
  digitalWrite(SCREEN_CS2, LOW);
  pinMode(SCREEN_RST, OUTPUT);
  digitalWrite(SCREEN_RST, LOW);
  delay(1);
  digitalWrite(SCREEN_RST, HIGH);
  delay(1);
  pinMode(SCREEN_VCC, OUTPUT);
  digitalWrite(SCREEN_VCC, HIGH);
  delay(1);
  if (!screen_h.begin()) {
    ESP_LOGE("", "SSD1306 allocation failed");
    return;
  }
  digitalWrite(SCREEN_CS1, HIGH);
  digitalWrite(SCREEN_CS2, HIGH);
  screen_h.clearBuffer();
  screen_h.setDisplayRotation(U8G2_R2);
  screen_h.drawXBM(0, 0, 128, 64, Open_Logo);
  manage.block_time = millis() + 1000;
  digitalWrite(SCREEN_CS1, LOW);
  screen_h.sendBuffer();
  digitalWrite(SCREEN_CS1, HIGH);
  screen_h.clearBuffer();
  screen_s.clearBuffer();
}

void MeterUI::TurnOff() {
  manage.block_time = millis() + 1000000;
  screen_h.clear();
  digitalWrite(SCREEN_CS1, LOW);
  screen_h.sendBuffer();
  delay(1);
  screen_h.setPowerSave(1);
  delay(1);
  digitalWrite(SCREEN_CS1, HIGH);
  delay(1);
  digitalWrite(SCREEN_CS2, LOW);
  screen_s.sendBuffer();
  delay(1);
  screen_s.setPowerSave(1);
  delay(1);
  digitalWrite(SCREEN_CS2, HIGH);

  if (SCREEN_VCC != 0) {
    digitalWrite(SCREEN_VCC, LOW);
    pinMode(SCREEN_VCC, INPUT);
  }
}
#elif defined(HARDWARE_3_0)
#define SCREEN_RST 41
#define SCREEN_CS1 15
#define SCREEN_CS2 37
#define HORIZONTAL 1
#define VERTICAL 0
U8G2_ST7567_JLX12864_F_4W_SW_SPI screen_h(U8G2_R2, /*CLK*/ 12, /*SDA*/ 11,
                                          /*CS*/ 15, /*DC*/ 10, /*RST*/ 41);
U8G2_ST7539_192X64_F_4W_SW_SPI screen_s(U8G2_R2, /*CLK*/ 12, /*SDA*/ 11,
                                        /*CS*/ 37, /*DC*/ 13, /*RST*/ 14);                
void MeterUI::TurnOn() {
  if (!screen_h.begin()) {
    Serial.println("screen_h failed");
    return;
  }
  screen_h.setContrast(0x87);

  if (!screen_s.begin()) {
    Serial.println("screen_s failed");
    return;
  }
  screen_s.setContrast(0x70);

  // open logo
  screen_h.drawXBM(0, 0, 128, 64, Open_Logo);
  manage.block_time = millis() + 1000;
  digitalWrite(SCREEN_CS1, LOW);
  screen_h.sendBuffer();
  digitalWrite(SCREEN_CS1, HIGH);
  screen_h.clearBuffer();
}

void MeterUI::TurnOff() {
  manage.block_time = millis() + 1000000;
  // screen_h
  delay(1);
  screen_h.clear();
  digitalWrite(SCREEN_CS1, LOW);
  screen_h.sendBuffer();
  delay(1);
  screen_h.setPowerSave(1);
  delay(1);
  digitalWrite(SCREEN_CS1, HIGH);
  delay(1);
  // screen_s
  screen_s.clear();
  digitalWrite(SCREEN_CS2, LOW);
  screen_s.sendBuffer();
  delay(1);
  screen_s.setPowerSave(1);
  delay(1);
  digitalWrite(SCREEN_CS2, HIGH);
  delay(1);
}
#endif

void MeterUI::Primary_DrawCommon() {
  screen_h.drawXBM(18, 2, 17, 9, BITMAP_BATTERY);
  screen_h.drawBox(21, 4, *pBattry * 12 / 100, 5);
  if (*(pBLEState + 6) == true) {
    screen_h.drawXBM(39, 2, 13, 12, bitmap_bluetooth);
  }
  if (manage.flatness.sub_online) {
    screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_2000mm);
  } else screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_1000mm);
  screen_h.drawXBM(18, 54, 92, 6, bitmap_h_loading);
  int px[2] = {19, 16};
  int py[2] = {55, 3};
  bar_timer++;
  int l = 90;
  if (measure_state == M_UNSTABLE) {
    bar_timer %= (l - px[1]) * 2;
    screen_h.drawBox((bar_timer > (l - px[1]))
                         ? px[0] + (l - px[1]) * 2 - bar_timer
                         : px[0] + bar_timer,
                     py[0], px[1], py[1]);
  } else if (measure_state == M_MEASURING) {
    bar_timer = 0;
    screen_h.drawBox(px[0], py[0], measure_bar * l / 100, py[1]);
  } else if (measure_state == M_MEASURE_DONE ||
             measure_state == M_UPLOAD_DONE) {
    bar_timer = 0;
    screen_h.drawBox(px[0], py[0], l, py[1]);
  }
}

void MeterUI::Primary_DrawArrow() {
  uint8_t arrow;
  if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
    arrow = manage.clino.arrow_hold;
  } else {
    arrow = manage.clino.arrow_live;
  }
  if (arrow == 2) {
    screen_h.drawXBM(2, 3, 13, 13, bitmap_up_0);       // left_up
    screen_h.drawXBM(2, 50, 13, 13, bitmap_down_1);    // left_down
    screen_h.drawXBM(113, 3, 13, 13, bitmap_up_1);     // light_up
    screen_h.drawXBM(113, 50, 13, 13, bitmap_down_0);  // light_down
    return;
  }
  if (arrow == 1) {
    screen_h.drawXBM(2, 3, 13, 13, bitmap_up_1);
    screen_h.drawXBM(2, 50, 13, 13, bitmap_down_0);
    screen_h.drawXBM(113, 3, 13, 13, bitmap_up_0);
    screen_h.drawXBM(113, 50, 13, 13, bitmap_down_1);
    return;
  }
  if (arrow == 0) {
    screen_h.drawXBM(2, 3, 13, 13, bitmap_up_1);
    screen_h.drawXBM(2, 50, 13, 13, bitmap_down_1);
    screen_h.drawXBM(113, 3, 13, 13, bitmap_up_1);
    screen_h.drawXBM(113, 50, 13, 13, bitmap_down_1);
    return;
  }
}

void MeterUI::Primary_DrawHome() {
  switch (manage.home_mode) {
    case 0:
      Primary_DrawAngle();
      break;
    case 1:
      Primary_DrawSlope();
      break;
    case 2:
      Primary_DrawFlat();
      break;
    case 3:
      Primary_DrawFlatSlope();
      break;
    default:
      ESP_LOGE("PrimaryHome", "home_mode:%d", manage.home_mode);
      break;
  }
}

void MeterUI::Primary_DrawAngle() {
  screen_h.drawXBM(110, 20, 9, 9, BITMAP_DEGREE);
  if (g_this == 2 || g_this == 5) {
    screen_h.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
  } else {
    dtostrf(angle_show, 6, 2, str_show);
    drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
  }
}

void MeterUI::Primary_DrawSlope() {
  screen_h.drawXBM(110, 30, 17, 15, BITMAP_MMM);
  if (g_this == 2 || g_this == 5) {
    screen_h.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
  } else {
    dtostrf(slope_show, 6, 1, str_slope);
    drawNum_16x24(NUM_LHX, NUM_LHY, str_slope, 6);
  }
}

void MeterUI::Primary_DrawFlat() {
  screen_h.drawXBM(110, 30, 17, 15, BITMAP_UNIT_MM);

  if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
    screen_h.setDrawColor(2);
    screen_h.drawBox(3, 20, 35, 24);
    screen_h.setDrawColor(0);
    screen_h.drawXBM(18, 24, 10, 16, bitmap_hash_s);
    screen_h.drawXBM(6, 24, 10, 16, bitmap_num_10x16[dash_num]);
    screen_h.setDrawColor(1);
  }

  screen_h.setFont(u8g2_font_6x12_mr);
  for(int i = 0; i < 8; i++){
    if(pDS->sensor_online[i]){
      if(manage.flatness.ready[i]){
        screen_h.setDrawColor(0);
      }
      screen_h.drawStr(37 + 7 * i, 10,String(i + 1).c_str());
    }
  }
  screen_h.setDrawColor(1);
  if (flat_show > 50.0f) {
    screen_h.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
  } else {
    dtostrf(flat_show, 6, 1, str_show);
    drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
  }
}

void MeterUI::Primary_DrawFlatSlope() {
  if (manage.auto_mode_select == HOME_AUTO_FLATNESS) {
    // screen_h.drawXBM(5, 25, 14, 14, bitmap_flat_icon);
    screen_h.drawXBM(110, 30, 17, 15, BITMAP_UNIT_MM);
  if (manage.flatness.sub_online) {
    screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_2000mm);
  } else screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_1000mm);
    if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
      screen_h.setDrawColor(2);
      screen_h.drawBox(3, 20, 35, 24);
      screen_h.setDrawColor(0);
      screen_h.drawXBM(18, 24, 10, 16, bitmap_hash_s);
      screen_h.drawXBM(6, 24, 10, 16, bitmap_num_10x16[dash_num]);
      screen_h.setDrawColor(1);
    }
    if (flat_show > 50.0f) {
      screen_h.drawXBM(LINE_LHX, NUM_LHY + 12, 45, 3, Main_Dash_45x3);
    } else {
      dtostrf(flat_show, 6, 1, str_show);
      drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
    }
  } else {
    screen_h.drawXBM(5, 25, 14, 14, bitmap_vertical_icon);
    if (manage.slope_standard == 1000.0f) {
      screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_1000mm);
    } else if (manage.slope_standard == 1200.0f) {
      screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_500mm);
    } else if (manage.slope_standard == 2000.0f) {
      screen_h.drawXBM(82, 2, 28, 9, bitmap_unit_2000mm);
    } else {
      ESP_LOGE("USER", "[ERROR]slope_standard!!!");
    }
    screen_h.drawXBM(108, 31, 17, 15, BITMAP_MMM);
    (slope_show, 6, 1, str_show);
    drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
  }
}

void MeterUI::Minor_DrawCommon() {
  if (rotation == VERTICAL) {
    screen_s.drawXBM(2, 105, 60, 6, bitmap_v_loading);
    int px[2] = {3, 21};
    int py[2] = {106, 3};
    bar_timer++;
    int l = 58;
    if (measure_state == M_UNSTABLE) {
      bar_timer %= (l - px[1]) * 2;
      screen_s.drawBox((bar_timer > (l - px[1]))
                           ? px[0] + (l - px[1]) * 2 - bar_timer
                           : px[0] + bar_timer,
                       py[0], px[1], py[1]);
    } else if (measure_state == M_MEASURING) {
      bar_timer = 0;
      screen_s.drawBox(px[0], py[0], measure_bar * l / 100, py[1]);
    } else if (measure_state == M_MEASURE_DONE ||
               measure_state == M_UPLOAD_DONE) {
      bar_timer = 0;
      screen_s.drawBox(px[0], py[0], l, py[1]);
    }
  } else {
    screen_s.drawXBM(18, 2, 17, 9, BITMAP_BATTERY);
    screen_s.drawBox(21, 4, *pBattry * 12 / 100, 5);
    if (*(pBLEState + 6) == true) {
      screen_s.drawXBM(39, 2, 13, 12, bitmap_bluetooth);
    }
    if (manage.flatness.sub_online) {
      screen_s.drawXBM(82, 2, 28, 9, bitmap_unit_2000mm);
    } else screen_s.drawXBM(82, 2, 28, 9, bitmap_unit_1000mm);
    screen_s.drawXBM(18, 54, 92, 6, bitmap_h_loading);

    int px[2] = {19, 16};
    int py[2] = {55, 3};
    bar_timer++;
    int l = 90;
    if (measure_state == M_UNSTABLE) {
      bar_timer %= (l - px[1]) * 2;
      screen_s.drawBox((bar_timer > (l - px[1]))
                           ? px[0] + (l - px[1]) * 2 - bar_timer
                           : px[0] + bar_timer,
                       py[0], px[1], py[1]);
    } else if (measure_state == M_MEASURING) {
      bar_timer = 0;
      screen_s.drawBox(px[0], py[0], measure_bar * l / 100, py[1]);
    } else if (measure_state == M_MEASURE_DONE ||
               measure_state == M_UPLOAD_DONE) {
      bar_timer = 0;
      screen_s.drawBox(px[0], py[0], l, py[1]);
    }
  }
}

void MeterUI::Minor_DrawAngle() {
  if (rotation == VERTICAL) {
    screen_s.drawXBM(54, 40, 9, 9, BITMAP_DEGREE);
    screen_s.drawXBM(26, 3, 14, 14, bitmap_angle_icon);
    if (g_this == 2 || g_this == 5) {
      screen_s.drawXBM(LINE_LVX, LINE_LVY, 45, 3, Main_Dash_45x3);
    } else {
      dtostrf(angle_show, 6, 2, str_show);
      Minor_drawNum_10x16(NUM_LVX, NUM_LVY, str_show, 6);
    }
  } else {
    screen_s.drawXBM(110, 20, 9, 9, BITMAP_DEGREE);
    if (g_this == 2 || g_this == 5) {
      screen_s.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
    } else {
      dtostrf(angle_show, 6, 2, str_show);
      Minor_drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
    }
  }
}

void MeterUI::Minor_DrawSlope() {
  dtostrf(slope_show, 6, 1, str_slope);
  if (rotation == VERTICAL) {
    screen_s.drawXBM(41, 72, 17, 15, BITMAP_MMM);
    screen_s.drawXBM(26, 3, 14, 14, bitmap_slope_icon);
    if (g_this == 2 || g_this == 5) {
      screen_s.drawXBM(LINE_LVX, LINE_LVY, 45, 3, Main_Dash_45x3);
    } else {
      Minor_drawNum_10x16(NUM_LVX, NUM_LVY, str_slope, 6);
    }
  } else {
    screen_s.drawXBM(110, 30, 17, 15, BITMAP_MMM);
    if (g_this == 2 || g_this == 5) {
      screen_s.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
    } else {
      Minor_drawNum_16x24(NUM_LHX, NUM_LHY, str_slope, 6);
    }
  }
}
void MeterUI::Minor_DrawFlat() {
  if (rotation == VERTICAL) {
    screen_s.drawXBM(26, 3, 14, 14, bitmap_flat_icon);
    screen_s.drawXBM(41, 72, 17, 15, BITMAP_UNIT_MM);
    if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
      screen_s.setDrawColor(2);
      screen_s.drawBox(2, 22, 60, 24);
      screen_s.setDrawColor(0);
      screen_s.drawXBM(20, 26, 10, 16, bitmap_num_10x16[dash_num]);
      screen_s.drawXBM(33, 26, 10, 16, bitmap_hash_s);
      screen_s.setDrawColor(1);
    }
    if (flat_show > 50.0f) {
      screen_s.drawXBM(LINE_LVX, LINE_LVY, 45, 3, Main_Dash_45x3);
    } else {
      dtostrf(flat_show, 6, 1, str_show);
      Minor_drawNum_10x16(NUM_LVX, NUM_LVY, str_show, 6);
    }
  } else {
    screen_s.drawXBM(110, 30, 17, 15, BITMAP_UNIT_MM);
    if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
      screen_s.setDrawColor(2);
      screen_s.drawBox(3, 20, 35, 24);
      screen_s.setDrawColor(0);
      screen_s.drawXBM(6, 24, 10, 16, bitmap_num_10x16[dash_num]);
      screen_s.drawXBM(18, 24, 10, 16, bitmap_hash_s);
      screen_s.setDrawColor(1);
    }

  screen_s.setFont(u8g2_font_6x12_mr);
  for(int i = 0; i < 8; i++){
    if(pDS->sensor_online[i]){
      if(manage.flatness.ready[i]){
        screen_s.setDrawColor(0);
      }
      screen_s.drawStr(37 + 7 * i, 10,String(i + 1).c_str());
    }
  }
  screen_s.setDrawColor(1);
  if (flat_show > 50.0f) {
    screen_s.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
  } else {
      dtostrf(flat_show, 6, 1, str_show);
      Minor_drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
  }
}
}

void MeterUI::Minor_DrawFlatSlope() {
  if (rotation == VERTICAL) {
    if (manage.auto_mode_select == HOME_AUTO_FLATNESS) {
      screen_s.drawXBM(27, 2, 14, 14, bitmap_flat_icon);
      screen_s.drawXBM(41, 72, 17, 15, BITMAP_UNIT_MM);
      if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
        screen_s.setDrawColor(2);
        screen_s.drawBox(2, 22, 60, 24);
        screen_s.setDrawColor(0);
        screen_s.drawXBM(20, 26, 10, 16, bitmap_num_10x16[dash_num]);
        screen_s.drawXBM(33, 26, 10, 16, bitmap_hash_s);
        screen_s.setDrawColor(1);
      }
      if (flat_show > 50.0f) {
        screen_s.drawXBM(LINE_LVX, LINE_LVY, 45, 3, Main_Dash_45x3);
      } else {
        dtostrf(flat_show, 6, 1, str_show);
        Minor_drawNum_10x16(NUM_LVX, NUM_LVY, str_show, 6);
      }
    } else {
      screen_s.drawXBM(27, 2, 14, 14, bitmap_vertical_icon);
      screen_s.drawXBM(41, 72, 17, 15, BITMAP_MMM);
      dtostrf(slope_show, 6, 1, str_show);
      Minor_drawNum_10x16(NUM_LVX, NUM_LVY, str_show, 6);
    }
  } else {
    if (manage.auto_mode_select == HOME_AUTO_FLATNESS) {
      screen_s.drawXBM(5, 25, 14, 14, bitmap_flat_icon);
      screen_s.drawXBM(108, 31, 17, 15, BITMAP_UNIT_MM);
      if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
        screen_s.setDrawColor(2);
        screen_s.drawBox(3, 20, 35, 24);
        screen_s.setDrawColor(0);
        screen_s.drawXBM(6, 24, 10, 16, bitmap_num_10x16[dash_num]);
        screen_s.drawXBM(18, 24, 10, 16, bitmap_hash_s);
        screen_s.setDrawColor(1);
      }
      if (flat_show > 50.0f) {
        screen_s.drawXBM(LINE_LHX, LINE_LHY, 45, 3, Main_Dash_45x3);
      } else {
        dtostrf(flat_show, 6, 1, str_show);
        Minor_drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
      }
    } else {
      screen_s.drawXBM(5, 25, 14, 14, bitmap_vertical_icon);
      screen_s.drawXBM(108, 31, 17, 15, BITMAP_MMM);
      dtostrf(slope_show, 6, 1, str_show);
      Minor_drawNum_16x24(NUM_LHX, NUM_LHY, str_show, 6);
    }
  }
}

void MeterUI::Minor_DrawArrow() {
  // just satify use cases:shkp
  byte x[4] = {2, 49, 3, 112};
  byte y[4] = {4, 112, 3, 50};
  byte s[2] = {13, 13};
  uint8_t arrow;
  if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
    arrow = manage.clino.arrow_hold;
  } else {
    arrow = manage.clino.arrow_live;
  }

  if (rotation == VERTICAL) {
    if (pIMU->_gravity == 3) {
      arrow = (arrow == 1) ? 2 : 1;
    }
    if (arrow == 1) {
      screen_s.drawXBM(x[0], y[0], s[0], s[1], bitmap_up_1);    // left_up
      screen_s.drawXBM(x[1], y[0], s[0], s[1], bitmap_up_1);    // light_up
      screen_s.drawXBM(x[0], y[1], s[0], s[1], bitmap_down_0);  // left_down
      screen_s.drawXBM(x[1], y[1], s[0], s[1], bitmap_down_0);  // light_down
    } else if (arrow == 2) {
      screen_s.drawXBM(x[0], y[0], s[0], s[1], bitmap_up_0);    // left_up
      screen_s.drawXBM(x[1], y[0], s[0], s[1], bitmap_up_0);    // light_up
      screen_s.drawXBM(x[0], y[1], s[0], s[1], bitmap_down_1);  // left_down
      screen_s.drawXBM(x[1], y[1], s[0], s[1], bitmap_down_1);  // light_down
    } else {
      screen_s.drawXBM(x[0], y[0], s[0], s[1], bitmap_up_1);    // left_up
      screen_s.drawXBM(x[1], y[0], s[0], s[1], bitmap_up_1);    // light_up
      screen_s.drawXBM(x[0], y[1], s[0], s[1], bitmap_down_1);  // left_down
      screen_s.drawXBM(x[1], y[1], s[0], s[1], bitmap_down_1);  // light_down
    }
  } else {
    if (arrow == 2) {
      screen_s.drawXBM(x[2], y[2], s[0], s[1], bitmap_up_0_rotate);  // left_up
      screen_s.drawXBM(x[2], y[3], s[0], s[1], bitmap_up_0_rotate);  // light_up
      screen_s.drawXBM(x[3], y[2], s[0], s[1],
                       bitmap_down_1_rotate);  // left_down
      screen_s.drawXBM(x[3], y[3], s[0], s[1],
                       bitmap_down_1_rotate);  // light_down
    } else if (arrow == 1) {
      screen_s.drawXBM(x[2], y[2], s[0], s[1], bitmap_up_1_rotate);  // left_up
      screen_s.drawXBM(x[2], y[3], s[0], s[1], bitmap_up_1_rotate);  // light_up
      screen_s.drawXBM(x[3], y[2], s[0], s[1],
                       bitmap_down_0_rotate);  // left_down
      screen_s.drawXBM(x[3], y[3], s[0], s[1],
                       bitmap_down_0_rotate);  // light_down
    } else {
      screen_s.drawXBM(x[2], y[2], s[0], s[1], bitmap_up_1_rotate);  // left_up
      screen_s.drawXBM(x[2], y[3], s[0], s[1], bitmap_up_1_rotate);  // light_up
      screen_s.drawXBM(x[3], y[2], s[0], s[1],
                       bitmap_down_1_rotate);  // left_down
      screen_s.drawXBM(x[3], y[3], s[0], s[1],
                       bitmap_down_1_rotate);  // light_down
    }
  }
}

void MeterUI::pageAngleCaliInfo() {
  screen_h.setFont(u8g2_font_7x13B_tr);
  screen_h.drawStr(2, 10, "AngleCali:");
  char str[7] = "-----";
  dtostrf(pIMU->angle_raw[1], 7, 2, str);
  drawNum_16x24(NUM_LHX, NUM_LHY, str, 7);
}

void MeterUI::pageSwitchLight() {
  if (manage.warn_light_onoff == true) {
    manage.warn_light_onoff = false;
    // manage.warrning_mode =  WARN_ON_LIGHT_OFF;
    // manage.put_warrning_light();
    // screen_h.drawXBM(0, 0, 128, 64, BITMAP_BLOCK_LIGHT_OFF);
    // manage.block_time = millis() + 1000;
    manage.flat_abs = false;
    Block("Relative_Flatness",1000);
    manage.page = PAGE_HOME;
  } else {
    manage.warn_light_onoff = true;
    // manage.warrning_mode = WARN_ON_LIGHT_ON;
    // manage.put_warrning_light();
    // screen_h.drawXBM(0, 0, 128, 64, BITMAP_BLOCK_LIGHT_ON);
    // manage.block_time = millis() + 1000;
    manage.flat_abs = true;
    Block("Absolute_Flatness",1000);
    manage.page = PAGE_HOME;
  }
}

void MeterUI::pageSwitchBLE() {
  if (manage.if_ble_switch) {
    if (*(pBLEState + 7) == true) {
      manage.if_ble_switch = false;
      screen_h.drawXBM(0, 0, 128, 64, BITMAP_BLOCK_BLE_ON);
      manage.block_time = millis() + 1000;
      manage.page = PAGE_HOME;
    } else {
      manage.if_ble_switch = false;
      screen_h.drawXBM(0, 0, 128, 64, BITMAP_BLOCK_BLE_OFF);
      manage.block_time = millis() + 1000;
      manage.page = PAGE_HOME;
    }
  } else {
    if (*(pBLEState + 7) == true)
      screen_h.drawXBM(0, 0, 128, 64, BITMAP_SETTING_BLE_ON);
    else {
      screen_h.drawXBM(0, 0, 128, 64, BITMAP_SETTING_BLE_OFF);
    }
    for (int i = 4; i < 6; i++) {
    }
    screen_h.drawXBM(40, 24, 10, 16, bitmap_num_10x16[*(pBLEState + 1) / 0x10]);
    screen_h.drawXBM(53, 24, 10, 16, bitmap_num_10x16[*(pBLEState + 1) % 0x10]);
    screen_h.drawBox(66, 37, 3, 3);
    screen_h.drawXBM(72, 24, 10, 16, bitmap_num_10x16[*(pBLEState) / 0x10]);
    screen_h.drawXBM(85, 24, 10, 16, bitmap_num_10x16[*(pBLEState) % 0x10]);
  }
}

void MeterUI::pageCaliMenu() {
  if (manage.cursor > 2) {
    screen_h.drawXBM(0, 0, 128, 64, BITMAP_PAGE_CALI_2);
    screen_h.setDrawColor(2);
    screen_h.drawBox(0, manage.cursor * 21 - 21, 128, 21);
    screen_h.setDrawColor(1);
  } else {
    screen_h.drawXBM(0, 0, 128, 64, BITMAP_PAGE_CALI_1);
    screen_h.setDrawColor(2);
    screen_h.drawBox(0, manage.cursor * 21, 128, 21);
    screen_h.setDrawColor(1);
  }
}

void MeterUI::pageAutoCaliFlatness() {
  screen_h.setFont(u8g2_font_10x20_tr);
  screen_h.drawStr(0, 11, "FlatCali");
  screen_h.drawBox(0, 13, 128, 2);
  char height_char[12][8] = {"BACK", "1mm", "2mm", "3mm", "4mm",  "5mm",
                             "6mm",  "7mm", "8mm", "9mm", "10mm", "SAVE"};
  screen_h.drawStr(48, 36, height_char[manage.dist_cali.step]);
  screen_h.drawStr(5, 54, "Press:");
  screen_h.drawStr(64, 54, String(manage.cali_count).c_str());
}

void MeterUI::pageOption_YesNo(bool option) {
  char S1[4] = "No";
  char S2[4] = "Yes";
  screen_h.setFont(u8g2_font_7x14B_tr);
  screen_h.drawFrame(41, 26, 46, 18);
  screen_h.drawFrame(41, 46, 46, 18);
  screen_h.drawStr(57, 40, S1);
  screen_h.drawStr(54, 60, S2);
  screen_h.setDrawColor(2);
  screen_h.drawBox(43, 28 + 20 * option, 42, 14);
  screen_h.setDrawColor(1);
}

void MeterUI::pageCalAngleCheck() {
  screen_h.setFont(u8g2_font_7x14B_tr);
  // YES_NO确认界面
  if (pIMU->cali_state == IMU_COMMON) {
    screen_h.drawStr(0, 15, "AngleCali Ready?");
    pageOption_YesNo(pIMU->yes_no);
  }
  // 采集数据页面
  else if (pIMU->cali_state == IMU_CALI_ZERO) {
    screen_h.drawStr(0, 15, "AngleCali Going!");
    screen_h.drawFrame(12, 50, 104, 14);
    screen_h.drawBox(14, 52, pIMU->cali_progress, 10);
  }
}

void MeterUI::pageImuFactoryZero() {
  screen_h.setFont(u8g2_font_7x14B_tr);
  // YES_NO确认界面
  if (pIMU->cali_state == IMU_COMMON) {
    screen_h.drawStr(0, 15, "AngleFactoryReady!!!?");
    pageOption_YesNo(pIMU->yes_no);
  }
  // 采集数据页面
  else if (pIMU->cali_state == IMU_FACTORY_ZERO) {
    screen_h.drawStr(0, 15, "AngleFactoryZeroGoing!");
    screen_h.drawFrame(12, 50, 104, 14);
    screen_h.drawBox(14, 52, pIMU->cali_progress, 10);
  }
}

void MeterUI::pageCaliFlatCheck() {
  screen_h.setFont(u8g2_font_7x14B_tr);
  // YES_NO确认界面
  if (manage.flat_state == FLAT_COMMON) {
    screen_h.drawStr(0, 15, "FlatCali Ready?");
    pageOption_YesNo(pDS->yes_no);
  }
  // 采集数据页面
  else if (manage.flat_state == FLAT_CALI_ZERO) {
    screen_h.drawStr(0, 15, "FlatCali Going!");
    screen_h.drawFrame(12, 50, 104, 14);
    screen_h.drawBox(14, 52, pDS->cali_progress, 10);
  }
}

void MeterUI::pageResetFactoryZero() {
  screen_h.setFont(u8g2_font_7x14B_tr);
  if (manage.reset_state == 0) {
    screen_h.drawStr(0, 15, "ResetFactoryReady?");
    pageOption_YesNo(manage.reset_yesno);
  }
  // 采集数据页面
  else if (manage.reset_state == 1) {
    screen_h.drawStr(0, 15, "ResetFactoryGoing!");
    screen_h.drawFrame(12, 50, 104, 14);
    screen_h.drawBox(14, 52, manage.reset_progress, 10);
  }
}

void MeterUI::pageFlatFactoryZero() {
  screen_h.setFont(u8g2_font_7x14B_tr);
  // YES_NO确认界面
  if (manage.flat_state == FLAT_COMMON) {
    screen_h.drawStr(0, 15, "FlatFactory Ready?");
    pageOption_YesNo(pDS->yes_no);
  }
  // 采集数据页面
  else if (manage.flat_state == FLAT_FACTORY_ZERO) {
    screen_h.drawStr(0, 15, "FlatFactory Going!");
    screen_h.drawFrame(12, 50, 104, 14);
    screen_h.drawBox(14, 52, pDS->cali_progress, 10);
  }
}

void MeterUI::pageInfo(int selection) {
  screen_h.setFont(u8g2_font_6x12_mr);
  byte size = 10; 
  byte sensor_num = 0; 
  switch (selection) {
    case 0:
      screen_h.drawStr(0, 8, "Dist_Linear_FPeak");
      for (int i = 0; i < 8; i++) {
        int x_offset = (i < 4) ? 0 : 64;
        int y_offset = 20 + (i < 4 ? i : i - 4) * 14;
        screen_h.drawStr(x_offset, y_offset, String(pDS->dist[i],1).c_str());
        screen_h.drawStr(x_offset + 24, y_offset,String(pDS->dist_linear[i], 1).c_str());
        screen_h.drawStr(x_offset + 48, y_offset,String(pDS->filt_peak[i],0).c_str());
      }
      break;
    case 1:
      screen_h.drawStr(0, 8, "Filt_FPeak");
      for (int i = 0; i < 8; i++) {
        int x_offset = (i < 4) ? 0 : 64;
        int y_offset = 20 + (i < 4 ? i : i - 4) * 14;
        screen_h.drawStr(x_offset, y_offset, String(pDS->filt[i],0).c_str());
        screen_h.drawStr(x_offset + 32, y_offset,String(pDS->filt_peak[i], 0).c_str());
      }
      break;
    case 2:
      screen_h.drawStr(0, 8, "Raw_RPeak");
      for (int i = 0; i < 8; i++) {
        int x_offset = (i < 4) ? 0 : 64;
        int y_offset = 20 + (i < 4 ? i : i - 4) * 14;
        screen_h.drawStr(x_offset, y_offset, String(pDS->raw[i]).c_str());
        screen_h.drawStr(x_offset + 32, y_offset,
                         String(pDS->raw_peak[i], 0).c_str());
      }
      break;
    case 3:
        pageImuInfo();
      break;
    case 4:
      size = 10; 
      sensor_num = 0; 
      screen_h.drawStr(2, 58, "SensorNum:");
      screen_h.drawStr(64,58, String(sensor_num).c_str());
      for(int i = 0; i < size; i++){
        int x_offset = (i < size/2) ? 0 : 64;
        int y_offset = (i < size/2 ? i : i - size/2) * 10;
        screen_h.drawStr(x_offset, y_offset, String(pDS->map_x[0][i],0).c_str());
      }
      break;
    // case 5:
    //   break;
    default:
      screen_h.drawStr(2, 8, "PAGE:");
      screen_h.drawStr(70,8, String(selection).c_str());
      screen_h.drawStr(2, 18, "SOFTWARE:");
      screen_h.drawStr(70,18, String(manage.ver_software).c_str());
      screen_h.drawStr(2, 28, "HARDWARE:");
      screen_h.drawStr(70,28, String(manage.ver_hardware).c_str());
      screen_h.drawStr(2, 38, "IMU:");
      screen_h.drawStr(70,38, String(manage.imu_version).c_str());
      screen_h.drawStr(2, 48, "Battery:");
      screen_h.drawStr(70,48, String(*pBattry).c_str());
      screen_h.drawStr(2, 58, "Battery:");
      screen_h.drawStr(70,58, String(*pBattry).c_str());
      break;
  }
}
void MeterUI::pageImuInfo() {
  screen_h.drawStr(0, 8, "IMU_INFO");
  char U[8] = "-----";
  dtostrf(pIMU->angle_user[1], 7, 2, U);
  screen_h.drawStr(0, 20, "U:");
  screen_h.drawStr(10, 20, U);

  char X[8] = "-----";
  dtostrf(pIMU->angle_raw[0], 7, 2, X);
  screen_h.drawStr(0, 32, "X:");
  screen_h.drawStr(10, 32, X);

  char Y[8] = "-----";
  dtostrf(pIMU->angle_raw[1], 7, 2, Y);
  screen_h.drawStr(0, 48, "Y:");
  screen_h.drawStr(10, 48, Y);

  char Z[8] = "-----";
  dtostrf(pIMU->angle_raw[2], 7, 2, Z);
  screen_h.drawStr(0, 62, "Z:");
  screen_h.drawStr(10, 62, Z);

  char A[8] = "-----";
  dtostrf(manage.auto_angle, 7, 2, A);
  screen_h.drawStr(66, 20, "A:");
  screen_h.drawStr(76, 20, A);

  char G[8] = "-----";
  dtostrf(pIMU->_gravity, 7, 2, G);
  screen_h.drawStr(66, 32, "G:");
  screen_h.drawStr(76, 32, G);

  char B[8] = "-----";
  dtostrf(pIMU->b[1], 7, 2, B);
  screen_h.drawStr(66, 48, "B:");
  screen_h.drawStr(76, 48, B);

  char E[8] = "-----";
  dtostrf(pIMU->e[1], 7, 2, E);
  screen_h.drawStr(66, 62, "E:");
  screen_h.drawStr(76, 62, E);
}

void MeterUI::Flip() {
  g_this = pIMU->_gravity;
  rotation = (g_this % 3 == 0) ? 0 : 1;
#ifdef HARDWARE_2_0
  if (g_last != g_this) {
    switch (g_this) {  // 0321
      case 0:
        screen_s.setDisplayRotation(U8G2_R0);
        break;
      case 1:
        screen_s.setDisplayRotation(U8G2_R3);
        break;
      case 3:
        screen_s.setDisplayRotation(U8G2_R2);
        break;
      case 4:
        screen_s.setDisplayRotation(U8G2_R1);
        break;
      default:
        screen_s.setDisplayRotation(U8G2_R1);
        break;
    }
    g_last = g_this;
  }
#else  // 3210
#ifdef TYPE_2000
  if (g_last != g_this) {
    switch (g_this) {
      case 0:
        screen_s.setDisplayRotation(U8G2_R3);
        break;
      case 1:
        screen_s.setDisplayRotation(U8G2_R2);
        break;
      case 3:
        screen_s.setDisplayRotation(U8G2_R1);
        break;
      case 4:
        screen_s.setDisplayRotation(U8G2_R0);
        break;
      default:
        screen_s.setDisplayRotation(U8G2_R0);
        break;
    }
    g_last = g_this;
  }
#else  // 1032
  if (g_last != g_this) {
    switch (g_this) {
      case 0:
        screen_s.setDisplayRotation(U8G2_R1);
        break;
      case 1:
        screen_s.setDisplayRotation(U8G2_R0);
        break;
      case 3:
        screen_s.setDisplayRotation(U8G2_R3);
        break;
      case 4:
        screen_s.setDisplayRotation(U8G2_R2);
        break;
      default:
        screen_s.setDisplayRotation(U8G2_R2);
        break;
    }
    g_last = g_this;
  }
#endif
#endif
}

bool MeterUI::ifSwitchHome() {
  if (manage.pre_home_mode != manage.home_mode) {
    manage.pre_home_mode = manage.home_mode;
    screen_h.drawXBM(0, 0, 128, 64, bitmap_cover[manage.home_mode]);
    manage.block_time = millis() + 1000;
    return true;
  }
  manage.pre_home_mode = manage.home_mode;
  return false;
}


void MeterUI::Block(String Info, int time) {
  block_info = Info;
  manage.block_time = millis() + time;
}

void MeterUI::DoBlock() {
  if (block_info == "") {
    manage.block_time = millis();
    return;
  }
  screen_h.clearBuffer();
  screen_h.setDisplayRotation(U8G2_R2);
  screen_h.setFont(u8g2_font_helvB10_tr);
  const unsigned int Mx = screen_h.getDisplayWidth();
  const unsigned int My = screen_h.getDisplayHeight();
  const unsigned int a = screen_h.getAscent() - screen_h.getDescent();
  int s[10] = {0};
  int s_now = block_info.indexOf(" ");
  int s_next, w;
  int i = 0;
  if (s_now == -1) {
    s_now = block_info.length();
    i++;
    s[i] = s_now + 1;
  }
  while (s_now < block_info.length()) {
    w = screen_h.getStrWidth(block_info.substring(s[i], s_now).c_str());
    s_next = -1;
    while (w <= Mx) {
      s_now += s_next + 1;
      if (s_now > block_info.length()) break;
      s_next = block_info.substring(s_now + 1).indexOf(" ");
      if (s_next == -1) s_next = block_info.substring(s_now + 1).length();
      w = screen_h.getStrWidth(
          block_info.substring(s[i], s_now + s_next + 1).c_str());
    }
    i++;
    s[i] = s_now + 1;
    s_next = block_info.substring(s[i]).indexOf(" ");
    if (s_next == -1 && s_now < block_info.length()) {
      i++;
      s[i] = block_info.length() + 1;
      break;
    }
    s_now += s_next + 1;
  }
  unsigned int sh = a * 1.25;
  unsigned int h = sh * (i - 1) + a;
  while (h > My) {
    sh--;
    h = sh * (i - 1) + a;
  }
  unsigned int y = (My - h) / 2 + screen_h.getAscent();
  for (int j = 0; j < i; j++) {
    w = screen_h.getStrWidth(block_info.substring(s[j], s[j + 1] - 1).c_str());
    screen_h.drawStr((Mx - w) / 2, y,
                     block_info.substring(s[j], s[j + 1] - 1).c_str());
    y += sh;
  }
  digitalWrite(SCREEN_CS1, LOW);
  screen_h.sendBuffer();
  digitalWrite(SCREEN_CS1, HIGH);
}

void MeterUI::drawNum_10x16(int x, int y, String str, int size) {
  byte detal = 0;
  char a_char;
  for (byte i = 0; i < size; i++) {
    a_char = str.charAt(i);
    if (a_char == '.')
      screen_h.drawBox(x + detal, y + 14, 3, 3);
    else if (a_char == '-')
      screen_h.drawBox(x + detal + 2, y + 8, 6, 3);
    else if (a_char <= '9' && a_char >= '0')
      screen_h.drawXBM(x + detal, y, 10, 16, bitmap_num_10x16[a_char - '0']);
    detal += (a_char == '.') ? 4 : 11;
  }
}

void MeterUI::Minor_drawNum_10x16(int x, int y, String str, int size) {
  byte detal = 0;
  char a_char;
  for (byte i = 0; i < size; i++) {
    a_char = str.charAt(i);
    if (a_char == '.')
      screen_s.drawBox(x + detal, y + 14, 3, 3);
    else if (a_char == '-')
      screen_s.drawBox(x + detal + 2, y + 8, 6, 3);
    else if (a_char <= '9' && a_char >= '0')
      screen_s.drawXBM(x + detal, y, 10, 16, bitmap_num_10x16[a_char - '0']);
    detal += (a_char == '.') ? 4 : 11;
  }
}

void MeterUI::drawNum_16x24(int x, int y, String str, int size) {
  byte detal = 0;
  char a_char;
  for (byte i = 0; i < size; i++) {
    a_char = str.charAt(i);
    if (a_char == '.')
      screen_h.drawBox(x + detal, y + 20, 4, 4);
    else if (a_char == '-')
      screen_h.drawBox(x + detal + 3, y + 12, 10, 3);
    else if (a_char <= '9' && a_char >= '0')
      screen_h.drawXBM(x + detal, y, 16, 24, bitmap_num_16x24[a_char - '0']);
    detal += (a_char == '.') ? 5 : 17;
  }
}

void MeterUI::Minor_drawNum_16x24(int x, int y, String str, int size) {
  byte detal = 0;
  char a_char;
  for (byte i = 0; i < size; i++) {
    a_char = str.charAt(i);
    if (a_char == '.')
      screen_s.drawBox(x + detal, y + 20, 4, 4);
    else if (a_char == '-')
      screen_s.drawBox(x + detal + 3, y + 12, 10, 3);
    else if (a_char <= '9' && a_char >= '0')
      screen_s.drawXBM(x + detal, y, 16, 24, bitmap_num_16x24[a_char - '0']);
    detal += (a_char == '.') ? 5 : 17;
  }
}

void MeterUI::Update() {
  // do block
  switch(manage.flatness.online_block){
    // offline
    case 1:
      Block("SubMeter Disconnect", 2500);
      manage.flatness.online_block = 0;
    break;
    // online
    case 2:
      Block("SubMeter Connect", 1500);
      manage.flatness.online_block = 0;
      break;
    default:
      break;
  }
  if (block_info != "") {
    DoBlock();
    block_info = "";
  }
  if (millis() < manage.block_time) return;
  // get param
  measure_state = manage.measure.state;
  measure_bar = manage.measure.progress;
  if (measure_state == M_MEASURE_DONE || measure_state == M_UPLOAD_DONE) {
    angle_show = manage.clino.angle_hold;
    slope_show = manage.clino.slope_hold;
    flat_show = manage.flatness.flat_hold;
  } else {
    angle_show = manage.clino.angle_live;
    slope_show = manage.clino.slope_live;
    flat_show = manage.flatness.flat_live;
  }
  auto_mode_select = manage.auto_mode_select;
  dash_num = manage.dash_num;

  Flip();
  Minor_DrawCommon();
  Minor_DrawArrow();
  switch (manage.home_mode) {
    case 0:
      Minor_DrawAngle();
      break;
    case 1:
      Minor_DrawSlope();
      break;
    case 2:
      Minor_DrawFlat();
      break;
    case 3:
      Minor_DrawFlatSlope();
      break;
    default:
      ESP_LOGE("MinorHome", "home_mode:%d", manage.home_mode);
      break;
  }
  screen_s.sendBuffer();
  screen_s.clearBuffer();

  screen_h.setDisplayRotation(U8G2_R2);
  if (ifSwitchHome() == false) {
    switch (manage.page) {
      case PAGE_HOME:
        Primary_DrawArrow();
        Primary_DrawCommon();
        Primary_DrawHome();
        break;
      case PAGE_BLE:
        pageSwitchBLE();
        break;
      case PAGE_ZERO_MENU:
        pageCaliMenu();
        break;
      case PAGE_ZERO_ANGLE:
        pageCalAngleCheck();
        break;
      case PAGE_ZERO_FLAT:
        pageCaliFlatCheck();
        break;
      case PAGE_ZERO_RESET:
        pageResetFactoryZero();
        break;
      case PAGE_CALI_FLAT:
        pageAutoCaliFlatness();
        break;
      case PAGE_LIGHT_SWITCH:
        pageSwitchLight();
        break;
      case PAGE_INFO:
        pageInfo(manage.cursor);
        break;
      case PAGE_IMU_CALI_INFO:
        pageAngleCaliInfo();
        break;
      case PAGE_IMU_FACTORY_ZERO:
        pageImuFactoryZero();
        break;
      case PAGE_FLAT_FACTORY_ZERO:
        pageFlatFactoryZero();
        break;
      default:
        ESP_LOGE("UI", "page:%d\r\n", manage.page);
        break;
    }
  }
  screen_h.sendBuffer();
  screen_h.clearBuffer();
}
