#include "Arduino.h"
#include <iostream>
#include <vector>
#include <sstream>
#include "MeterManage.h"
extern Meter manage;
/* example
int cmd_print_test(size_t argc, const std::vector<std::string>& argv) {
    std::string str_output = "[test]";
    for (size_t i = 0; i < argc; i++) {
        if (i > 0) {
            str_output += ", ";
        }
        str_output += argv[i];
    }
    std::cout << str_output << std::endl;
    return 0;
}
StringCmdParser cmd_parser(" ");
cmd_parser.register_cmd("test",cmd_print_test);
cmd_parser.parse(rx_str.c_str()); 
*/

#define KEY_METER_FLAT_RECORD "meter.flat.record"
int cmd_meter_flat_record(size_t argc, const std::vector<std::string>& argv)
{
    if (argc == 0)return 0x03;
    
    std::string str = "";
    try {
        int number = std::stoi(argv[0]);
        if (0 <= number && number <= 10) {
            if (manage.flat.state == FLAT_COMMON) {
                manage.flat.cali.step = CALI_STEP::RECORD;
                manage.flat.cali.record_num = number;
                manage.flat.state = FLAT_ROBOT_ARM_CALI;
                manage.page = PAGE_CALI_FLAT;
                return 0;
            }
        }
    }
    catch (...) {
        // 捕获所有异常
        return 0x04;
    }
    // 如果没有匹配任何条件，则返回错误代码
    return 0x05;
}

#define KEY_METER_CALI_FLAT "meter.flat.cali"
int cmd_meter_flat_cali(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "";
    if (argc == 0)return 0x03;

    if (argv[0] == "save") {
        if (manage.flat.state == FLAT_COMMON) {
            manage.flat.cali.step = CALI_STEP::SAVE;
            manage.flat.state = FLAT_ROBOT_ARM_CALI;
            manage.page = PAGE_CALI_FLAT;
            return 0;
        }
    } 
    else if (argv[0] == "record") {
       if (manage.flat.state == FLAT_COMMON) {
            manage.flat.cali.step = CALI_STEP::RECORD;
            manage.flat.state = FLAT_ROBOT_ARM_CALI;
            manage.page = PAGE_CALI_FLAT;
            return 0;
        }
    } else if (argv[0] == "reset") {
        manage.flat.cali.step = -1;
        manage.flat.state = FLAT_COMMON;
        return 0;
    }
    else if (argv[0] == "echo") {
       if (manage.flat.state == FLAT_COMMON) {
            manage.flat.cali.step = CALI_STEP::ECHO;
            manage.flat.state = FLAT_ROBOT_ARM_CALI;
            manage.page = PAGE_CALI_FLAT;
            return 0;
        }
    }else {
        try {
            int number = std::stoi(argv[0]);
            if (0 <= number && number <= 11) {
                if (manage.flat.state == FLAT_COMMON) {
                    manage.flat.cali.step = number;
                    manage.flat.state = FLAT_ROBOT_ARM_CALI;
                    manage.page = PAGE_CALI_FLAT;
                    return 0;
                }
                return 0x06;
            }
        }
        catch (...) {
            // 捕获所有异常
            return 0x04;
        }
    }
    // 如果没有匹配任何条件，则返回错误代码
    return 0x05;
}

#define KEY_METER_ANGLE_RECORD "meter.angle.record"
int cmd_meter_angle_record(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "";
    // check argc
    if (argc == 0)return 0x03;

    if(argv[0] == "save"){
        manage.page = PAGE_CALI_ANGLE;
        String dataString;
        dataString = "";
        dataString += "<";
        dataString += String(7207);
        dataString += ",";
        dataString += ">";
        Serial1.print(dataString);
        return 0;
    }
    else if(argv[0] == "reset"){
        manage.page = PAGE_CALI_ANGLE;
        String dataString;
        dataString = "";
        dataString += "<";
        dataString += String(7200);
        dataString += ",";
        dataString += ">";
        Serial1.print(dataString);
        return 0;
    }
    else {
        try {
            int number = std::stoi(argv[0]);
            if (0 <= number && number <= 6) {
                manage.page = PAGE_CALI_ANGLE;
                String dataString;
                dataString = "";
                dataString += "<";
                dataString += String(7200 + number);
                dataString += ",";
                dataString += ">";
                Serial1.print(dataString);
                return 0;
            }
        }
        catch (...) {
            // 捕获所有异常
            return 0x04;
        }
    }
    // 如果没有匹配任何条件，则返回错误代码
    return 0x05;
}

#define KEY_METER_CALI_ANGLE "meter.angle.cali"
int cmd_meter_angle_cali(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "";
    // check argc
    if (argc == 0)return 0x03;

    if(argv[0] == "save"){
        manage.page = PAGE_CALI_ANGLE;
        String dataString;
        dataString = "";
        dataString += "<";
        dataString += String(7107);
        dataString += ",";
        dataString += ">";
        Serial1.print(dataString);
        return 0;
    }
    else if(argv[0] == "record"){
        manage.page = PAGE_CALI_ANGLE;
        String dataString;
        dataString = "";
        dataString += "<";
        dataString += String(7108);
        dataString += ",";
        dataString += ">";
        Serial1.print(dataString);
        return 0;
    }
    else if(argv[0] == "reset"){
        manage.page = PAGE_CALI_ANGLE;
        String dataString;
        dataString = "";
        dataString += "<";
        dataString += String(7100);
        dataString += ",";
        dataString += ">";
        Serial1.print(dataString);
        return 0;
    }
    else {
        try {
            int number = std::stoi(argv[0]);
            if (0 <= number && number <= 6) {
                manage.page = PAGE_CALI_ANGLE;
                String dataString;
                dataString = "";
                dataString += "<";
                dataString += String(7100 + number);
                dataString += ",";
                dataString += ">";
                Serial1.print(dataString);

                return 0;
            }
        }
        catch (...) {
            // 捕获所有异常
            return 0x04;
        }
    }
    // 如果没有匹配任何条件，则返回错误代码
    return 0x05;
}

#define KEY_FLAT_SHOW "meter.flat.show"
int cmd_meter_flat_show(size_t argc, const std::vector<std::string>& argv)
{
    if(argv[0] == "off")manage.debug_flat_mode = 0;
    else if(argv[0] == "raw")manage.debug_flat_mode = 1;
    else if(argv[0] == "filt")manage.debug_flat_mode = 2;
    else if(argv[0] == "raw_peak")manage.debug_flat_mode = 21;
    return 0;
}

#define KEY_SYS_SHOW "meter.sys.show"
int cmd_meter_sys_show(size_t argc, const std::vector<std::string>& argv)
{
    if (argc == 0)return 0x03;
    try {
        if(argv[0] == "name"){
            String str = "meter.sys.name " + manage.local_id;
            Serial.println(str.c_str());
        }
        return 0;
    }
    catch (...) {
        // 捕获所有异常
        return 0x04;
    }
    return 0x05;
}

#define KEY_UI_PAGE "meter.ui.page"
int cmd_ui_page(size_t argc, const std::vector<std::string>& argv)
{   
    if (argc == 0)return 0x03;
    try {
        manage.page = std::stoi(argv[0]);
        return 0;
    }
    catch (...) {
        // 捕获所有异常
        return 0x04;
    }
    return 0x05;
}


#define KEY_SYSTEM_TYPE "meter.system.type"
int cmd_system_type(size_t argc, const std::vector<std::string>& argv)
{
    std::string ack_str = "";
    // check argc
    if (argc == 0)return 0x03;
    if(argv[0] == "500"){
        manage.meter_type = METER_TYPE_DEFINE::TYPE_0_5;
        manage.put_meter_type();
        ack_str = "[ACK] MeterType 500";
        Serial.println(ack_str.c_str());
        return 0;
    }
    else if(argv[0] == "1000"){
        manage.meter_type = METER_TYPE_DEFINE::TYPE_1_0;
        manage.put_meter_type();
        ack_str = "[ACK] MeterType 1000";
        Serial.println(ack_str.c_str());
        return 0;
    }
    else if(argv[0] == "2000"){
        manage.meter_type = METER_TYPE_DEFINE::TYPE_2_0;
        manage.put_meter_type();
        ack_str = "[ACK] MeterType 2000";
        Serial.println(ack_str.c_str());
        return 0;
    }
    else if(argv[0] == "!abs"){
        manage.flat_abs = false;
        ack_str = "[ACK] flat_abs false";
        Serial.println(ack_str.c_str());
        return 0;
    }
    else if(argv[0] == "abs"){
        manage.flat_abs = true;
        ack_str = "[ACK] flat_abs true";
        Serial.println(ack_str.c_str());
        return 0;
    }
    return 0x05;
}

#define KEY_SYSTEM_MODE "meter.sys.debug"
int cmd_system_mode(size_t argc, const std::vector<std::string>& argv)
{
    // check argc
    if (argc == 0)return 0x03;
    std::string ack_str = "";
    if(argv[0] == "0"){
        manage.debug_sys_mode = 0;
        ack_str = "[ACK] Mode Normal";
        Serial.println(ack_str.c_str());
        return 0;
    }
    else if(argv[0] == "1"){
        manage.debug_sys_mode = 1;
        ack_str = "[ACK] Mode Debug";
        Serial.println(ack_str.c_str());
        return 0;
    }
    return 0x05;
}
