#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <sstream>
#include <functional>

class StringCmdParser {
public:
    using CmdCallback = int(*)(size_t, const std::vector<std::string>&);

    StringCmdParser(const std::string& separator_ = " ") : separator_(separator_) {}

    void register_cmd(const std::string& key, std::function<int(size_t, const std::vector<std::string>&)> cmd_callback) {
        cmd_config[key] = std::move(cmd_callback);
    }

    int parse(const std::string& input_str) {
        std::vector<std::string> tokens;
        split(input_str, tokens);

        if (tokens.empty()) {
            return 0;
        }

        const std::string& command_key = tokens[0];
        auto it = cmd_config.find(command_key);
        if (it != cmd_config.end()) {
            std::vector<std::string> args(tokens.begin() + 1, tokens.end());
            return it->second(args.size(), args);
        } else {
            std::string message = "[Unknown] " + input_str;
            std::cout << message << std::endl;
            return 0;
        }
    }

    void set_separator(const std::string& new_separator) {
        separator_ = new_separator;
    }

private:

    void split(const std::string& input, std::vector<std::string>& tokens) {
        std::string token;
        std::istringstream iss(input);
        while (std::getline(iss, token, separator_[0])) {
            tokens.push_back(token);
        }
    }

    std::unordered_map<std::string, std::function<int(size_t, const std::vector<std::string>&)>> cmd_config;
    std::string separator_;
};

/*
    // example1:read a byte of data
    String rx_str = "";
    char rx_char;
    while (Serial.available())
    {
        rx_char = (char)Serial.read(); 
        if (rx_char == '\n') 
        {
            if (rx_str.length() > 0) 
            {
                cmd_parser.parse(rx_str.c_str()); 
                rx_str = "";
            }
        }
        else rx_str += rx_char; 
    }
    // example2:read a row of data
    String rx_str = "";
    while (Serial.available())
    {
      rx_str =  Serial.readStringUntil('\n');
    }
    if (rx_str.length() > 0){
      cmd_parser.parse(rx_str.c_str()); // call command parser
    }
*/

/* parse version1.0
uint8_t parse(const char *input)
{
    // 将输入的字符串分割成命令关键字和参数
    // 定义一个指针变量，用于存储字符串分割后的首个令牌（token）
    char *token;

    // 使用strtok函数将输入字符串input按空格分割，获取首个令牌 
    // 注意：这里将输入字符串强制转换为char*类型，以适应strtok函数的参数要求 
    token = strtok((char *)input, " ");

    // 定义一个固定大小的字符数组，用于存储提取的命令关键字 
    char command_key[50] = "";

    // 检查strtok函数是否成功获取到令牌 
    if (token != NULL)
    {
        // 将获取到的令牌复制到command_key数组中 
        // 注意：这里使用strncpy函数而非直接赋值，以避免潜在的字符串截断问题 
        strncpy(command_key, token, sizeof(command_key) - 1);
    }

    // 遍历命令表，寻找与输入命令键相匹配的命令
    // 查找匹配的命令
    uint8_t cmd_num = sizeof(meter_cmd_table) / sizeof(meter_cmd_table[0]);
    for (uint8_t i = 0; i < cmd_num; i++)
    {
        // 比较当前命令表项的键与输入命令键是否相同
        if (strcmp(command_key, rcv_cmd_info[i].key) == 0)
        {
            // 对输入命令的参数进行分割，以空格为分隔符
            // 分割参数
            char *args[10];
            int argc = 0;
            while ((token = strtok(NULL, " ")) != NULL && argc < 10)
            {
                args[argc++] = token;
            }

            // 调用匹配命令的回调函数，传入参数数量和参数数组
            // 这里是命令实际执行的地方
            // 调用命令的回调函数
            return rcv_cmd_info[i].callback(argc, args);
        }
    }
    // 如果没有找到匹配的命令
    return 0;
}
*/
