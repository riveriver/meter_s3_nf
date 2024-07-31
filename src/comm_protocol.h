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

int cmd_robot_cali(size_t argc, const std::vector<std::string>& argv)
{
    std::string str = "[cmd_robot_cali] ";
    for (size_t i = 0; i < argc; i++) {
        if (i > 0) {
            str += ", ";
        }
        str += argv[i];
    }
    if(argv[0] == "start"){
        if(manage.flat.state == FLAT_COMMON){
            manage.flat.state = FLAT_ROBOT_ARM_CALI;
        }else{
            Serial.println("robotcali error set_ready_fail");
        }
    }
    Serial.println(str.c_str());
    return 0;
}

