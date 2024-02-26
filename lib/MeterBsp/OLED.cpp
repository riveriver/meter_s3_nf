#include "OLED.h"
#include <U8g2lib.h>
#include "Bitmap_Manage.h"


#define HORIZONTAL 0
#define VERTICAL 1
#define BASE_HOME 0
#define BASE_MENU 10
#define BASE_BLE 20
#define BASE_FLAT_CALI 30
// #define OFFSET_VIKCY 0
#define OFFSET_ANGLE 0
#define OFFSET_SLOPE 1
#define OFFSET_FLATNESS 3
#define OFFSET_FLAT_SLOPE 4
#define ShowStableCountNum 10

/* display is primary;display2 is second*/
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI display(U8G2_R0, /*CLK*/ 36, /*SDA*/ 35,
                                               /*CS*/ 3, /*DC*/ 15, /*RST*/ 41);
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI display_s(U8G2_R0, /*CLK*/ 36,
                                                     /*SDA*/ 35,
                                                     /*CS*/ 3, /*DC*/ 15,
                                                     /*RST*/ 41);

void OLED::TurnOn(byte VCC) {
  IO_VCC = VCC;
  if (!isU8g2Begin) {
    pinMode(CS1, OUTPUT);
    pinMode(CS2, OUTPUT);
    digitalWrite(CS1, LOW);
    digitalWrite(CS2, LOW);
    DoRST();
    pinMode(IO_VCC, OUTPUT);
    digitalWrite(IO_VCC, HIGH);
    delay(10);
    if (!display.begin()) {
      ESP_LOGE("USER","SSD1306 allocation failed");
      return;
    }
    isU8g2Begin = true;
  }
  display.clearBuffer();
  display.setDisplayRotation(U8G2_R2);
  display.drawXBM(0, 0, 128, 64, Open_Logo);
  manage.block_time = millis() + 1000;
  digitalWrite(CS1, LOW);
  display.sendBuffer();
  digitalWrite(CS1, HIGH);
}

void OLED::TurnOff() {
  manage.block_time = millis() + 1000000;
  delay(200);
  display.clear();
  digitalWrite(CS1, LOW);
  digitalWrite(CS2, LOW);
  display.sendBuffer();
  // display.initDisplay();
  delay(1);
  display.setPowerSave(1);
  delay(1);
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);
  if (IO_VCC != 0) {
    digitalWrite(IO_VCC, LOW);
    pinMode(IO_VCC, INPUT);
  }
}

void OLED::DoRST() {
  pinMode(IO_OLED_RST, OUTPUT);
  digitalWrite(IO_OLED_RST, LOW);
  delay(1);
  digitalWrite(IO_OLED_RST, HIGH);
  delay(1);
}

void OLED::Block(String Info, int time) {
  BlockInfo = Info;
  manage.block_time = millis() + time;
}

void OLED::DoBlock() {
  if (BlockInfo == "") {
    manage.block_time = millis();
    return;
  } 
    display.clearBuffer();
    display.setDisplayRotation(U8G2_R2);
    display.setFont(u8g2_font_helvB10_tr);
    const unsigned int Mx = display.getDisplayWidth();
    const unsigned int My = display.getDisplayHeight();
    const unsigned int a = display.getAscent() - display.getDescent();
    int s[10] = {0};
    int s_now = BlockInfo.indexOf(" ");
    int s_next, w;
    int i = 0;
    if (s_now == -1) {
      s_now = BlockInfo.length();
      i++;
      s[i] = s_now + 1;
    }
    while (s_now < BlockInfo.length()) {
      w = display.getStrWidth(BlockInfo.substring(s[i], s_now).c_str());
      s_next = -1;
      while (w <= Mx) {
        s_now += s_next + 1;
        if (s_now > BlockInfo.length()) break;
        s_next = BlockInfo.substring(s_now + 1).indexOf(" ");
        if (s_next == -1) s_next = BlockInfo.substring(s_now + 1).length();
        w = display.getStrWidth(
            BlockInfo.substring(s[i], s_now + s_next + 1).c_str());
      }
      i++;
      s[i] = s_now + 1;
      s_next = BlockInfo.substring(s[i]).indexOf(" ");
      if (s_next == -1 && s_now < BlockInfo.length()) {
        i++;
        s[i] = BlockInfo.length() + 1;
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
    unsigned int y = (My - h) / 2 + display.getAscent();
    for (int j = 0; j < i; j++) {
      w = display.getStrWidth(BlockInfo.substring(s[j], s[j + 1] - 1).c_str());
      display.drawStr((Mx - w) / 2, y,
                      BlockInfo.substring(s[j], s[j + 1] - 1).c_str());
      y += sh;
    }
    digitalWrite(CS2, LOW);
    digitalWrite(CS1, LOW);
    display.sendBuffer();
    digitalWrite(CS1, HIGH);
    digitalWrite(CS2, HIGH);
}

void OLED::DrawNum_10x16(int x, int y, String f) {
  if (f.charAt(0) == '-')
  {
    display.drawBox(x, y + 6, 4, 2);
  }
  int p = 5;
  char c;
  for (int i = 2; i < 8; i++) {
    c = f.charAt(i);
    if (c == '.')
      display.drawBox(x + p, y + 14, 3, 3);
    else if (c <= '9' && c >= '0')
      display.drawXBM(x + p, y, 10, 16, bitmap_num_10x16[c - '0']);
    p += (c == '.') ? 5 : 12;
  }
}

void OLED::Minor_DrawNum_10x16(int x, int y, String f) {
  if (f.charAt(0) == '-')
  {
    display.drawBox(x, y + 8, 4, 2);
  }
  int p = 5;
  char c;
  for (int i = 2; i < 8; i++) {
    c = f.charAt(i);
    if (c == '.')
      display_s.drawBox(x + p, y + 14, 3, 3);
    else if (c <= '9' && c >= '0')
      display_s.drawXBM(x + p, y, 10, 16, bitmap_num_10x16[c - '0']);
    p += (c == '.') ? 5 : 12;
  }
}

void OLED::DrawNum_16x24(int x, int y, String f) {
  if (f.charAt(0) == '-')
  {
    display.drawBox(x, y + 12, 6, 3);
  }
  int p = 7;
  char c;
  for (int i = 1; i < 8; i++) {
    c = f.charAt(i);
    if (c == '.')
      display.drawBox(x + p, y + 20, 4, 4);
    else if (c <= '9' && c >= '0')
      display.drawXBM(x + p, y, 16, 24, bitmap_num_16x24[c - '0']);
    p += (c == '.') ? 6 : 18;
  }
}

void OLED::Minor_DrawNum_16x24(int x, int y, String f) {
  if (f.charAt(0) == '-')
  {
    display.drawBox(x, y + 12, 6, 3);
  }
  int p = 7;
  char c;
  for (int i = 1; i < 8; i++) {
    c = f.charAt(i);
    if (c == '.')
      display_s.drawBox(x + p, y + 20, 4, 4);
    else if (c <= '9' && c >= '0')
      display_s.drawXBM(x + p, y, 16, 24, bitmap_num_16x24[c - '0']);
    p += (c == '.') ? 6 : 18;
  }
}

/* ----- Menu -----*/
void OLED::Menu() {
  int dx[5] = {0, 0, 0, 1, 3};
  int dy[5] = {3, 0, 0, 0, 1};
  int x = dx[R_now];
  int y = dy[R_now];
  display.drawXBM(x, y, 128, 64, Menu_H);
  int nx, ny;
  for (int i = 0; i < 8; i++) {
    nx = 31 * i / 2;
    ny = 31 * i % 2;
    switch (i) {
      case 6:  // Battery
        display.drawBox(x + nx + 6, y + ny + 12, *pBatt * 17 / 100, 8);
        break;
    }
  }
  display.setDrawColor(2);
  ny = 2 + 31 * (manage.cursor % 2);
  nx = 2 + 31 * (manage.cursor / 2);
  display.drawXBM(x + nx, y + ny, 28, 28, Menu_Select);
  display.setDrawColor(1);
}

void OLED::FlatnessSetZero() {
  display.drawXBM(0, 0, 128, 64, Open_Page2);
  display.drawXBM(111, 50, 13, 12, Lock_allArray[pDS->Enable_Auto_Reset]);
  display.setDrawColor(2);
  display.drawBox(0, manage.cursor * 16 + 16, 128, 16);
}

void OLED::Bluetooth() {
  for (int i = 0; i < 6; i++) {
    display.drawXBM(16 * i + 9, 33, 5, 9,
                    HEX_5x9_allArray[*(pBLEState + 5 - i) / 0x10]);
    display.drawXBM(16 * i + 9 + 6, 33, 5, 9,
                    HEX_5x9_allArray[*(pBLEState + 5 - i) % 0x10]);
    if (i != 5) display.drawPixel(16 * i + 9 + 13, 41);
  }
  display.drawXBM(111, 0, 17, 64, Switch_allArray[*(pBLEState + 7) + 2]);
  display.drawXBM(2, 44, 46, 16, bitmap_Back);
}

void OLED::Cal_Menu() {
  display.setFont(u8g2_font_7x14B_tr);
  display.drawStr(0, 11, "Calibration");
  display.drawBox(0, 13, 128, 2);
  int w;
  char Cal_Mode[5][16] = {"Back", "Calibrate", "Clear", "Exp Cal",
                          "Z Dir Exp Cal"};
  for (int i = 0; i < 3; i++) {
    w = display.getStrWidth(Cal_Mode[i + pIMU->CursorStart]);
    display.drawStr(64 - w / 2, 29 + i * 16, Cal_Mode[i + pIMU->CursorStart]);
  }
  display.setDrawColor(2);
  display.drawBox(0, 17 + 16 * (pIMU->Cursor - pIMU->CursorStart), 128, 14);
  display.setDrawColor(1);
}

void OLED::CaliFlatness() {
  display.drawXBM(0, 0, 128, 64, Open_Page2);
  display.drawXBM(111, 50, 13, 12, Lock_allArray[pDS->Enable_Cali_Slope]);
  display.setDrawColor(2);
  display.drawBox(0, manage.cursor * 16 + 16, 128, 16);
  display.setFont(u8g2_font_7x14B_tr);
  display.drawStr(90, 11, "!!!");
}

void OLED::AutoCaliFlatness() {
  display.setFont(u8g2_font_7x14B_tr);
  display.drawStr(0, 11, "FlatnessnAutoCali");
  display.drawBox(0, 13, 128, 2);
  char height_char[12][8] = {"Back", "1mm", "2mm", "3mm", "4mm",  "5mm",
                             "6mm",  "7mm", "8mm", "9mm", "10mm", "Save"};
  display.setFont(u8g2_font_10x20_tr);
  display.drawStr(48, 36, height_char[manage.dist_cali.step]);
  if (manage.dist_cali.status == 0)
    display.drawStr(36, 54, "Ready");
  else if (manage.dist_cali.status == 1)
    display.drawStr(36, 54, "Collecting");
}

void OLED::AngleXYZ() {
  display.setFont(u8g2_font_7x13B_tr);

  char G[8] = "-----";
  dtostrf(pIMU->Gravity, 7, 2, G);
  display.drawStr(0, 64, "G:");
  display.drawStr(10,64, G);

  char E[8] = "-----";
  dtostrf(pIMU->e[1], 7, 2, E);
  display.drawStr(68,64, "E:");
  display.drawStr(78,64, E);
  
  /* angle show */
  for (int i = 0; i < 3; i++) {
    display.drawGlyph(0, 24 + 14 * i, i + 88);
    display.drawGlyph(6, 24 + 14 * i, ':');
    char A[8] = "-----";
    dtostrf(pIMU->AngleRaw[i], 7, 2, A);
    display.drawStr(10, 24 + 14 * i, A);
    char U[8] = "-----";
    dtostrf(pIMU->AngleStd[i], 7, 2, U);
    display.drawStr(68, 24 + 14 * i, U);
  }
}

void OLED::TypeSelect() {
  display.setFont(u8g2_font_7x14B_tr);
  display.drawStr(0, 11, "TypeSelect");
  display.drawBox(0, 13, 128, 2);
  display.drawStr(80, 11, "type: ");
  display.drawStr(110, 11, String(manage.meter_type).c_str());
  int w;
  char item_char[3][16] = {"0.5m", "1.0m", "2.0m"};
  for (int i = 0; i < 3; i++) {
    w = display.getStrWidth(item_char[i]);
    display.drawStr(64 - w / 2, 29 + i * 16, item_char[i]);
  }
  display.setDrawColor(2);
  display.drawBox(0, 17 + 16 * (manage.cursor), 128, 14);
  display.setDrawColor(1);
}

// HACK distance ==> sub_distance
void OLED::ShowDistData() {
  display.setFont(u8g2_font_6x13B_tr);
  for (int i = 0; i < 4; i++) {
    display.drawStr(0, 15 + i * 15, String(pDS->ADCfilt[i], 0).c_str());
    display.drawStr(32,15 + i * 15, String(pDS->distance[i],2).c_str());
  }
  for (int i = 4; i < 8; i++) {
    display.drawStr(65, 15 + (i - 4) * 15, String(pDS->ADCfilt[i],0).c_str());
    display.drawStr(97,15 + (i - 4) * 15, String(pDS->distance[i],2).c_str());
  }
}

void OLED::One_Sensor_Info() {
  display.setFont(u8g2_font_7x13B_tr);
  display.drawStr(0, 12, "Wire:");
  display.drawStr(0, 25, "Addr:");
  display.drawStr(0, 38, "Raw:");
  display.drawStr(0, 51, "Val:");
  display.drawStr(0, 64, "Var:");
  for (int i = 0; i < 8; i++) {
      display.drawStr(49, 12, String(i / 4).c_str());
      display.drawStr(49, 25, "0x4");
      display.drawGlyph(70, 25, "89ab"[i % 4]);
      display.drawStr(49, 38, String(pDS->ADCSurgeOut[0][i]).c_str());
      display.drawStr(49, 51, String(pDS->ADCfilt[i]).c_str());
      int MaxVal = pDS->ADCSurgeOut[0][i];
      int MinVal = pDS->ADCSurgeOut[0][i];
      for (int j = 1; j < 20; j++) {
        MaxVal =
            (pDS->ADCSurgeOut[j][i] > MaxVal) ? pDS->ADCSurgeOut[j][i] : MaxVal;
        MinVal =
            (pDS->ADCSurgeOut[j][i] < MinVal) ? pDS->ADCSurgeOut[j][i] : MinVal;
      }
      display.drawStr(49, 64, String(MaxVal - MinVal).c_str());
      break;
  }
}

void OLED::Show_Sensor_Address() {
  // if (pPOGO->Connect == pPOGO->Connect_1)
  //   display.drawXBM(0, 0, 128, 64, Addr_CS);
  // else if (pPOGO->Connect == pPOGO->Connect_2)
  //   display.drawXBM(0, 0, 128, 64, Addr_CP);
  // else
  display.drawXBM(0, 0, 128, 64, Addr_NC);
  // display.setDrawColor(2);
  // for (int i = 0; i < 7; i++) {
  //   if (pDS->isConnect[i]) display.drawBox(102 - 17 * i, 0, 14, 31);
  // }
  // if (pDS->isConnect[8]) display.drawBox(102 - 17 * 8, 0, 14, 31);
}

void OLED::Show_System_Info() {
  display.setFont(u8g2_font_7x13B_tr);
  display.drawStr(2, 10, "WonderMeter(BLE)");
  display.drawLine(2, 12, 125, 12);
  display.drawStr( 2, 30, "Battery:");
  display.drawStr(70, 30, String(*pBatt).c_str());
  display.drawStr( 2, 45, "SOFTWARE:");
  display.drawStr(70, 45, String(manage.software_version).c_str());
  display.drawStr( 2, 60, "IMU:");
  display.drawStr(70, 60, String(manage.imu_version).c_str());
}

void OLED::YesNo(bool IsH, bool Select) {
  char S1[4] = "No";
  char S2[4] = "Yes";
  display.setFont(u8g2_font_7x14B_tr);
  int w;
  display.drawFrame(41, 26, 46, 18);
  display.drawFrame(41, 46, 46, 18);
  display.drawStr(57, 40, S1);
  display.drawStr(54, 60, S2);
  display.setDrawColor(2);
  display.drawBox(43, 28 + 20 * Select, 42, 14);
  display.setDrawColor(1);
}

void OLED::Cal_Check() {
  display.setFont(u8g2_font_7x14B_tr);
  if (pIMU->CalibrateCheck == 0) {
    String Question;
    bool ShowYesNo = true;
    int line3 = 1;
    if (pIMU->Cursor == 1)
      Question = "Ready to Calibrate?";
    else if (pIMU->Cursor == 2)
      Question = "Sure to Clear Data?";
    else if (pIMU->Cursor == 3) {
      line3 = 0;
      Question = "g = " + String(pIMU->Gravity);
      ShowYesNo = false;
      if (pIMU->Gravity < 3) {
        if (pIMU->FullCalComplete[pIMU->Gravity])
          Question += ", Complete.";
        else {
          Question += ", Ready?";
          ShowYesNo = true;
        }
      }
    } else if (pIMU->Cursor == 4) {
      line3 = 0;
      Question = "g = " + String(pIMU->Gravity);
      ShowYesNo = false;
      if (pIMU->FullCalComplete[pIMU->Gravity])
        Question += ", Complete.";
      else {
        Question += ", Ready?";
        Question +=
            "(" + String(pIMU->CalibrateCollectCount[pIMU->Gravity] + 1) + ")";
        line3 = (pIMU->CalibrateCollectCount[pIMU->Gravity] > 8) ? 4 : 3;
        ShowYesNo = true;
      }
    }

    int w = 0;
    int l1 = Question.substring(7).indexOf(" ") + 8;
    display.drawStr(0, 15, Question.substring(0, l1 - 1).c_str());
    for (int i = 0; i < Question.substring(l1).length() - line3; i++) {
      char a = Question.substring(l1).charAt(i);
      w -= (a == 'l') ? 2 : 0;
      display.drawGlyph(w, 35, a);
      w += (a == ' ' || a == 'l') ? 5 : 7;
    }
    if (pIMU->Cursor != 3)
      display.drawStr(0, 55,
                      Question.substring(Question.length() - line3).c_str());
    // }

    if (!ShowYesNo) {
      display.drawStr(
          (display.getDisplayWidth() - display.getStrWidth("Back")) / 2,
          display.getDisplayHeight() - 4, "Back");
      display.drawFrame(display.getDisplayWidth() / 2 - 23,
                        display.getDisplayHeight() - 18, 46, 18);
      display.setDrawColor(2);
      display.drawBox(display.getDisplayWidth() / 2 - 21,
                      display.getDisplayHeight() - 16, 42, 14);
      display.setDrawColor(1);
    } else
      YesNo((R_now % 3 != 0), pIMU->YesNo);
  } else if (pIMU->CalibrateCheck == 1 && pIMU->Cursor == 2) {
    Block("Calibration Data Clear", 3000);
  } else if (pIMU->CalibrateCheck == 1) {
    for (int i = 1; i < 3; i++) {
      display.drawGlyph(0, 17 * i - 7, 120 + ((pIMU->Gravity + i) % 3));
      display.drawGlyph(7, 17 * i - 8, '=');
      char A[8];
      dtostrf(pIMU->AngleCal_ExG[(pIMU->Gravity + i) % 3], 7, 3, A);
      display.drawStr(15, 17 * i - 7, A);
    }
    display.drawFrame(12, 50, 104, 14);
    display.drawBox(14, 52, pIMU->CalibrateCount * 100 / pIMU->CalAvgNum, 10);
  }
}

void OLED::ImuCalibration() {
  display.setFont(u8g2_font_7x14B_tr);
  display.drawStr(0, 11, "ImuCalibration");
  display.drawBox(0, 13, 128, 2);
  char orient_char[12][11] = {"BackHome",  "TopDown",   "BottomDown",
                              "LeftDown",  "RightDown", "BackDown",
                              "FrontDown", "CleanAll"};
  display.setFont(u8g2_font_10x20_tr);
  display.drawStr(10, 36, orient_char[manage.imu_cali.step]);
  if (manage.imu_cali.status == 0)
    display.drawStr(10, 54, "Ready");
  else if (manage.imu_cali.status == 1)
    display.drawStr(10, 54, "Collecting");
}

void OLED::ProcessAngleShow(float *ui_show) {
  float out;
  if (measure_state == MEASURE_DONE ||
      measure_state == UPLOAD_DONE) {
    out = manage.clino.angle_hold;
  }else {
    out = manage.clino.angle_live;
  }
  Warning(out,0.0f);
  *ui_show = out;
  
}

void OLED::DrawMinorArrow() {
  // just satify use cases:shkp
  uint8_t arrow;
  if (measure_state == MEASURE_DONE ||
     measure_state == UPLOAD_DONE) {
    arrow = manage.clino.angle_hold;
  }
  else{
    arrow = manage.clino.angle_live;
  }
  if (pIMU->Gravity == 3 || pIMU->Gravity == 0) {
    if (arrow == 1) {
      display_s.drawXBM(3, 3, 13, 13, bitmap_up_0_rotate);      // left_up
      display_s.drawXBM(3, 50, 13, 13, bitmap_up_0_rotate);     // light_up
      display_s.drawXBM(115, 3, 13, 13, bitmap_down_1_rotate);  // left_down
      display_s.drawXBM(115, 50, 13, 13, bitmap_down_1_rotate);  // light_down
    }else if (arrow == 0) {
      display_s.drawXBM(3, 3, 13, 13, bitmap_up_1_rotate);      // left_up
      display_s.drawXBM(3, 50, 13, 13, bitmap_up_1_rotate);     // light_up
      display_s.drawXBM(115, 3, 13, 13, bitmap_down_0_rotate);  // left_down
      display_s.drawXBM(115, 50, 13, 13,bitmap_down_0_rotate);  // light_down
    }
    else{
      display_s.drawXBM(3, 3, 13, 13, bitmap_up_1_rotate);      // left_up
      display_s.drawXBM(3, 50, 13, 13, bitmap_up_1_rotate);     // light_up
      display_s.drawXBM(115, 3, 13, 13, bitmap_down_1_rotate);  // left_down
      display_s.drawXBM(115, 50, 13, 13, bitmap_down_1_rotate);  // light_down
    }
  } else if (pIMU->Gravity == 4) {
      if (arrow == 0) {
      display_s.drawXBM(2, 4, 13, 13, bitmap_up_1);       // left_up
      display_s.drawXBM(49, 4, 13, 13, bitmap_up_1);      // light_up
      display_s.drawXBM(2, 115, 13, 13, bitmap_down_0);   // left_down
      display_s.drawXBM(49, 115, 13, 13, bitmap_down_0);  // light_down
    }else if (arrow == 1) {
      display_s.drawXBM(2, 4, 13, 13, bitmap_up_0);       // left_up
      display_s.drawXBM(49, 4, 13, 13, bitmap_up_0);      // light_up
      display_s.drawXBM(2, 117, 13, 13, bitmap_down_1);   // left_down
      display_s.drawXBM(49, 117, 13, 13, bitmap_down_1);  // light_down
    }
    else{
      display_s.drawXBM(2, 4, 13, 13, bitmap_up_1);       // left_up
      display_s.drawXBM(49, 4, 13, 13, bitmap_up_1);      // light_up
      display_s.drawXBM(2, 117, 13, 13, bitmap_down_1);   // left_down
      display_s.drawXBM(49, 117, 13, 13, bitmap_down_1);  // light_down
    }
  } else if (pIMU->Gravity == 1) {
    if (arrow == 0) {
      display_s.drawXBM(2, 4, 13, 13, bitmap_up_0);       // left_up
      display_s.drawXBM(49, 4, 13, 13, bitmap_up_0);      // light_up
      display_s.drawXBM(2, 115, 13, 13, bitmap_down_1);   // left_down
      display_s.drawXBM(49, 115, 13, 13, bitmap_down_1);  // light_down
    }else if (arrow == 1) {
      display_s.drawXBM(2, 4, 13, 13, bitmap_up_1);       // left_up
      display_s.drawXBM(49, 4, 13, 13, bitmap_up_1);      // light_up
      display_s.drawXBM(2, 115, 13, 13, bitmap_down_0);   // left_down
      display_s.drawXBM(49, 115, 13, 13, bitmap_down_0);  // light_down
    }
    else{
      display_s.drawXBM(2, 4, 13, 13, bitmap_up_1);       // left_up
      display_s.drawXBM(49, 4, 13, 13, bitmap_up_1);      // light_up
      display_s.drawXBM(2, 115, 13, 13, bitmap_down_1);   // left_down
      display_s.drawXBM(49, 115, 13, 13, bitmap_down_1);  // light_down
    }
  } else{
    display_s.drawXBM(2,   4, 13, 13, bitmap_up_0);    // left_up
    display_s.drawXBM(49,  4, 13, 13, bitmap_up_0);    // light_up
    display_s.drawXBM(2, 115, 13, 13, bitmap_down_0);  // left_down
    display_s.drawXBM(49,115, 13, 13, bitmap_down_0);  // light_down
  }
}

void OLED::DrawPrimaryArrow() {
  uint8_t arrow;
  if (measure_state == MEASURE_DONE ||
     measure_state == UPLOAD_DONE) {
    arrow = manage.clino.arrow_hold;
  }
  else{
    arrow = manage.clino.arrow_live;
  }
  if (arrow == 1) {
    display.drawXBM(  2,  3, 13, 13, bitmap_up_0);       // left_up
    display.drawXBM(  2, 50, 13, 13, bitmap_down_1);    // left_down
    display.drawXBM(113,  3, 13, 13, bitmap_up_1);     // light_up
    display.drawXBM(113, 50, 13, 13, bitmap_down_0);  // light_down
    return;
  }
  if (arrow == 0) {
    display.drawXBM(  2,  3, 13, 13, bitmap_up_1);
    display.drawXBM(  2, 50, 13, 13, bitmap_down_0);
    display.drawXBM(113,  3, 13, 13, bitmap_up_0);
    display.drawXBM(113, 50, 13, 13, bitmap_down_1);
    return;
  }
  if (arrow == 2){
    display.drawXBM(  2,  3, 13, 13, bitmap_up_1);
    display.drawXBM(  2, 50, 13, 13, bitmap_down_1);
    display.drawXBM(113,  3, 13, 13, bitmap_up_1);
    display.drawXBM(113, 50, 13, 13, bitmap_down_1);
    return; 
  }
}

void OLED::DrawMinorCommon() {
  if (direction == HORIZONTAL) {
    display_s.drawXBM(18, 2, 17, 9, bitmap_battery);
    display_s.drawBox(21, 4, *pBatt * 12 / 100, 5);
    if (*(pBLEState + 6) == true) {
      display_s.drawXBM(39, 2, 13, 12, bitmap_bluetooth);
    }
    if (manage.slope_standard == 1000.0f) {
      display_s.drawXBM(82, 2, 28, 9, bitmap_unit_1000mm);
    } else if (manage.slope_standard == 1200.0f) {
      display_s.drawXBM(82, 2, 28, 9, bitmap_unit_500mm);
    } else if (manage.slope_standard == 2000.0f) {
      display_s.drawXBM(82, 2, 28, 9, bitmap_unit_2000mm);
    }
    display_s.drawXBM(18, 54, 92, 5, bitmap_loading);

    int px[2] = {19, 16};
    int py[2] = {55, 3};
    Timer++;
    int l = 90;
    // Draw Icon
    if (pIMU->fWarmUp < 100) {
      Timer = 0;
      display_s.drawBox(px[0], py[0], pIMU->fWarmUp * l / 100, py[1]);
    } else if (measure_state == UNSTABLE) {
      Timer %= (l - px[1]) * 2;
      display_s.drawBox((Timer > (l - px[1])) ? px[0] + (l - px[1]) * 2 - Timer
                            : px[0] + Timer,py[0], px[1], py[1]);
    } else if (measure_state == MEASURING) {
      Timer %= (l - px[1]) * 2;
      display_s.drawBox(px[0], py[0], measure_progress * l /100, py[1]);
    } 
    else if (measure_state == MEASURE_DONE ||
              measure_state == UPLOAD_DONE) {
      Timer = 0;
      display_s.drawBox(px[0], py[0], l, py[1]);
    }
  } else {
    display_s.drawXBM(2, 105, 60, 5, bitmap_v_loading);
    int px[2] = {3, 21};
    int py[2] = {106, 3};
    Timer++;
    int l = 58;
    if (pIMU->fWarmUp < 100) {
      Timer = 0;
      display_s.drawBox(px[0], py[0], pIMU->fWarmUp * l / 100, py[1]);
    } else if (measure_state == UNSTABLE) {
      Timer %= (l - px[1]) * 2;
      display_s.drawBox((Timer > (l - px[1]))? px[0] + (l - px[1]) * 2 - Timer
                            : px[0] + Timer,py[0], px[1], py[1]);
    } else if (measure_state == MEASURING) {
      Timer %= (l - px[1]) * 2;
      display_s.drawBox(px[0], py[0], measure_progress * l / 100, py[1]);
    } 
    else if (measure_state == MEASURE_DONE ||
              measure_state == UPLOAD_DONE) {
      Timer = 0;
      display_s.drawBox(px[0], py[0], l, py[1]);
    }
  }
}

void OLED::DrawPrimaryCommon() {
  display.drawXBM(18, 2, 17, 9, bitmap_battery);
  display.drawBox(21, 4, *pBatt * 12 / 100, 5);
  if (*(pBLEState + 6) == true) {
    display.drawXBM(39, 2, 13, 12, bitmap_bluetooth);
  }
  if (manage.slope_standard == 1000.0f) {
    display.drawXBM(82, 2, 28, 9, bitmap_unit_1000mm);
  } else if (manage.slope_standard == 1200.0f) {
    display.drawXBM(82, 2, 28, 9, bitmap_unit_500mm);
  } else if (manage.slope_standard == 2000.0f) {
    display.drawXBM(82, 2, 28, 9, bitmap_unit_2000mm);
  }else{ESP_LOGE("USER","[ERROR]slope_standard!!!");}
  display.drawXBM(18, 54, 92, 5, bitmap_loading);
  int px[2] = {19, 16};
  int py[2] = {55, 3};
  Timer++;
  int l = 90;
  // Draw Icon
  if (pIMU->fWarmUp < 100) {
    Timer = 0;
    display.drawBox(px[0], py[0], pIMU->fWarmUp * l / 100, py[1]);
  } else if (measure_state == UNSTABLE) {
    Timer %= (l - px[1]) * 2;
    display.drawBox((Timer > (l - px[1])) ? px[0] + (l - px[1]) * 2 - Timer 
                    : px[0] + Timer,py[0], px[1], py[1]);
  } else if (measure_state == MEASURING) {
    Timer %= (l - px[1]) * 2;
    display.drawBox(px[0], py[0], measure_progress * l/ 100, py[1]);
  } else if (measure_state == MEASURE_DONE || measure_state == UPLOAD_DONE) {
    Timer = 0;
    display.drawBox(px[0], py[0], l, py[1]);
  }
}

void OLED::DrawPrimaryAngle() {
  float show = 0;
  ProcessAngleShow(&show);
  DrawNum_16x24(5, 20, pIMU->String_now_unit(show,0));
  display.drawXBM(115, 20, 5, 5, bitmap_unit_degree);
}

void OLED::DrawMinorAngle() {
  float show = 0;
  ProcessAngleShow(&show);
  if (direction == VERTICAL) {
    Minor_DrawNum_10x16(2, 52, pIMU->String_now_unit(show, 0));
    display_s.drawXBM(57, 44, 5, 5, bitmap_unit_degree);
    display_s.drawXBM(26, 3, 14, 14, bitmap_angle_icon);
  }
  else {
    Minor_DrawNum_16x24(5, 20, pIMU->String_now_unit(show,0));
    display_s.drawXBM(115, 20, 5, 5, bitmap_unit_degree);
  }
}

void OLED::DrawPrimarySlope() {
  float show = 0;
  ProcessAngleShow(&show);
  DrawNum_16x24(5, 20, pIMU->String_now_unit(show, 1));
  display.drawXBM(110, 30, 17, 15, bitmap_unit_mmm);
}

void OLED::DrawMinorSlope() {
  float show = 0;
  ProcessAngleShow(&show);
  if (direction == HORIZONTAL) {
    Minor_DrawNum_16x24(5, 20, pIMU->String_now_unit(show, 1));
    display_s.drawXBM(110, 30, 17, 15, bitmap_unit_mmm);
  } else {
    Minor_DrawNum_10x16(2, 52, pIMU->String_now_unit(show, 1));
    display_s.drawXBM(41, 72, 17, 15, bitmap_unit_mmm);
    display_s.drawXBM(26, 3, 14, 14, bitmap_slope_icon);
  }
}

void OLED::DrawPrimaryFlat() {
  char ui_show[8];
  if (measure_state == MEASURE_DONE ||
   measure_state == UPLOAD_DONE) {
    dtostrf(manage.flatness.flat_hold, 7, 1, ui_show);
  } else {
    dtostrf(manage.flatness.flat_live, 7, 1, ui_show);
  }
  // dtostrf(manage.get_live_flat(), 7, 1, ui_show);
  DrawNum_16x24(5, 20, ui_show);
  display.drawXBM(110, 29, 17, 15, bitmap_unit_mm);
  // display.drawXBM(5, 35, 7, 10, HEX_5x9_allArray[pDS->ConnectCount]);
}

void OLED::DrawMinorFlat() {
  char ui_show[8];
  if (measure_state == MEASURE_DONE ||
   measure_state == UPLOAD_DONE) {
    dtostrf(manage.flatness.flat_hold, 7, 1, ui_show);
  } else {
    dtostrf(manage.flatness.flat_live, 7, 1, ui_show);
  }
  // dtostrf(manage.get_live_flat(), 7, 1, ui_show);
  if (direction == HORIZONTAL) {
    Minor_DrawNum_16x24(5, 20, ui_show);
    display_s.drawXBM(110, 29, 17, 15, bitmap_unit_mm);
  } else {
    Minor_DrawNum_10x16(2, 52, ui_show);
    display_s.drawXBM(26, 3, 14, 14, bitmap_flat_icon);
    display_s.drawXBM(41, 72, 17, 15, bitmap_unit_mm);
  }
}

void OLED::DrawPrimaryFlatSlope() {
  float ui_angle = 0;
  ProcessAngleShow(&ui_angle);
  char ui_flat[8];
  if (measure_state == MEASURE_DONE ||
   measure_state == UPLOAD_DONE) {
    dtostrf(manage.flatness.flat_hold, 7, 1, ui_flat);
  } else {
    dtostrf(manage.flatness.flat_live, 7, 1, ui_flat);
  }
  DrawNum_10x16(32, 35, pIMU->String_now_unit(ui_angle, 1));
  DrawNum_10x16(32, 15, ui_flat);
  display.drawXBM(110, 16, 17, 15, bitmap_unit_mm);
  display.drawXBM(110, 38, 17, 15, bitmap_unit_mmm);
  display.drawXBM(18, 15, 14, 14, bitmap_flat_icon);
  display.drawXBM(18, 36, 14, 14, bitmap_slope_icon);
}

void OLED::DrawMinorFlatSlope() {
  float ui_angle = 0;
  ProcessAngleShow(&ui_angle);
  char ui_flat[8];
  if (measure_state == MEASURE_DONE ||
   measure_state == UPLOAD_DONE) {
    dtostrf(manage.flatness.flat_hold, 7, 1, ui_flat);
  } else {
    dtostrf(manage.flatness.flat_live, 7, 1, ui_flat);
  }
  if (direction == HORIZONTAL){
    Minor_DrawNum_10x16(32, 35, pIMU->String_now_unit(ui_angle, 1));
    Minor_DrawNum_10x16(32, 15, ui_flat);
    display_s.drawXBM(18, 15, 14, 14, bitmap_flat_icon);
    display_s.drawXBM(18, 36, 14, 14, bitmap_slope_icon);
    display_s.drawXBM(110, 16, 17, 15, bitmap_unit_mm);
    display_s.drawXBM(110, 38, 17, 15, bitmap_unit_mmm);
  } else {
    Minor_DrawNum_10x16(2, 63, pIMU->String_now_unit(ui_angle, 1));
    Minor_DrawNum_10x16(2, 28, ui_flat);
    display_s.drawXBM(41, 48, 17, 15, bitmap_unit_mm);
    display_s.drawXBM(41, 83, 17, 15, bitmap_unit_mmm);
    display_s.drawXBM(18, 2, 14, 14, bitmap_flat_icon);
    display_s.drawXBM(31, 2, 14, 14, bitmap_slope_icon);
  }
}

void OLED::Update() {
  // do block
  if (BlockInfo != "") {
    DoBlock();
    BlockInfo = "";
  }
  if (millis() < manage.block_time) {
    return;
  }

  measure_state = manage.measure.state;
  measure_progress = manage.measure.progress;

  int R_new;
  if (pIMU->Gravity % 3 == 2)
    R_new = pIMU->GravityPrevious;
  else
    R_new = pIMU->Gravity;
  direction = R_new % 3;
  if (R_now != R_new) {
    switch (R_new) {// 0321
      case 0:
        display_s.setDisplayRotation(U8G2_R0);
        break;
      case 1:
        display_s.setDisplayRotation(U8G2_R3);
        break;
      case 3:
        display_s.setDisplayRotation(U8G2_R2);
        break;
      case 4:
        display_s.setDisplayRotation(U8G2_R1);
        break;
    }
    R_now = R_new;
  }
  DrawMinorCommon();
  DrawMinorArrow();
  switch (manage.home_mode) {
    case 0:
      DrawMinorAngle();
      break;
    case 1:
      DrawMinorSlope();
      break;
    case 2:
      DrawMinorFlat();
      break;
    case 3:
      DrawMinorFlatSlope();
      break;
    default:
      ESP_LOGE("USER","[processMinorHome]ERROR!!!");
      break;
  }
  digitalWrite(CS2, LOW);
  display_s.sendBuffer();
  digitalWrite(CS2, HIGH);
  display_s.clearBuffer();

  display.setDisplayRotation(U8G2_R2); 
  if (ifSwitchHome() == false) {
    switch (manage.page) {
      case 1:
        Menu();
        break;
      // case 1:
      //   if (pIMU->CalibrateCheck == -1)
      //     Cal_Menu();
      //   else
      //     Cal_Check();
      //   break;
      case 2:
        FlatnessSetZero();
        break;
      case 3:
        Bluetooth();
        break;
      case 4:
        if (pIMU->CalibrateCheck == -1)
          Cal_Menu();
        else
          Cal_Check();
        break;
      case 5:
        ImuCalibration();
        break;
      case 6:
        AngleXYZ();
        break;
      case 7:
        Show_System_Info();
        break;
      case 8:
        TypeSelect();
        break;
      case 41:
        CaliFlatness();
        break;
      case 42:
        AutoCaliFlatness();
        break;
      case 61:
        Show_Sensor_Address();
        break;
      case 62:
        One_Sensor_Info();
        break;
      case 63:
        ShowDistData();
        break;
      case 0:
        DrawPrimaryArrow();
        DrawPrimaryCommon();
        switch (manage.home_mode) {
          case 0:
            DrawPrimaryAngle();
            break;
          case 1:
            DrawPrimarySlope();
            break;
          case 2:
            DrawPrimaryFlat();
            break;
          case 3:
            DrawPrimaryFlatSlope();
            break;
          default:
            ESP_LOGE("","[processPrimaryHome]ERROR!!!");
            break;
        }
    }
  }
  digitalWrite(CS1, LOW);
  display.sendBuffer();
  digitalWrite(CS1, HIGH);
  display.clearBuffer();
}

bool OLED::ifSwitchHome() {
  if (manage.pre_home_mode != manage.home_mode) {
    manage.pre_home_mode = manage.home_mode;
    display.drawXBM(0, 0, 128, 64, bitmap_cover[manage.home_mode]);
    manage.block_time = millis() + 1000;
    return true;
  }
  manage.pre_home_mode = manage.home_mode;
  return false;
}

void OLED::Warning(float angle, float dist) {
  // test
  // manage.warrning_mode = 2;
  // manage.warrning_angle = 10.0f;
  uint8_t light_flag = 1;  
  if (manage.warrning_mode != 2) {
    pLED->Set(0, 0, 0, 4);
    pLED->Set(1, 0, 0, 4);
    pLED->Update();
    if(digitalRead(12) == LOW){digitalWrite(12,HIGH);}
    return;
  }
  if (measure_state == MEASURE_DONE || measure_state == UPLOAD_DONE) {
    if (angle > manage.warrning_angle) {
      light_flag = 2;
    }
    if (light_flag == 2) {
      pLED->Set(0, pLED->R, 1, 4);
      pLED->Set(1, pLED->R, 1, 4);
      if(digitalRead(12) == LOW){digitalWrite(12,HIGH);}
    } else if (light_flag == 1) {
      pLED->Set(0, pLED->G, 1, 4);
      pLED->Set(1, pLED->G, 1, 4);
      if(digitalRead(12) == HIGH){digitalWrite(12,LOW);}
    } else {
      ESP_LOGE("USER","Warrning():light_flag = %d", light_flag);
    }
    pLED->Update();
  } else {
    pLED->Set(0, 0, 0, 4);
    pLED->Set(1, 0, 0, 4);
    pLED->Update();
    if(digitalRead(12) == LOW){digitalWrite(12,HIGH);}
  }
}
