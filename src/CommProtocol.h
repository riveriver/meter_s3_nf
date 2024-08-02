#include "Arduino.h"
#include <iostream>
#include <vector>
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

int cmd_print_test(size_t argc, const std::vector<std::string>& argv) {
    std::string str_output = "[test] ";
    for (size_t i = 0; i < argc; i++) {
        if (i > 0) {
            str_output += ", ";
        }
        str_output += argv[i];
    }
    Serial.println(str_output.c_str());
    return 0;
}

int cmd_print_version(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "[version] ";
    for (size_t i = 0; i < argc; i++) {
        if (i > 0) {
            str += ", ";
        }
        str += argv[i];
    }
    str += ":HARDWARE_3_0; SENSOR_1_4; SOFTWARE_5_3;";
    Serial.println(str.c_str());
    return 0;
}

int cmd_print_flat(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "[flat] ";
    for (size_t i = 0; i < argc; i++) {
        if (i > 0) {
            str += ", ";
        }
        str += argv[i];
    }
    str += ":9262,9075,8770,8573,8475,8260,8055,7855,7655,7554,7357";
    Serial.println(str.c_str());
    return 0;
}

#define KEY_METER_CALI_FLAT "meter.cali.flat"
int cmd_robot_cali_flat(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "";
    if(argv[0] == "start"){
        if(manage.flat.state == FLAT_COMMON){
            manage.page = PAGE_CALI_FLAT;                           
            manage.flat.state = FLAT_ROBOT_ARM_CALI;
            str += "[D]OK";
        }else{
            str += "[E]FAIL:flat_state = " + std::to_string(manage.flat.state);
        }
    }
    Serial.println(str.c_str());
    return 0;
}

#define KEY_METER_CALI_ANGLE "meter.cali.angle"
int cmd_meter_cali_angle(size_t argc, const std::vector<std::string>& argv)
{
    manage.page = PAGE_CALI_ANGLE;
    std::string str = "";
    uint8_t step = std::stoi(argv[0]);
    if(step < 0 || step > 7){
        str += "[E]FAIL:step = " + std::to_string(step);
    }else{
        String dataString;
        dataString = "";
        dataString += "<";
        dataString += String(7100 + step);
        dataString += ",";
        dataString += ">";
        Serial1.print(dataString);
        str += "[D]OK:step = " + std::to_string(step);
    }
    Serial.println(str.c_str());
    // 一定要用return，否则会导致不可恢复bug
    return 0;
}


