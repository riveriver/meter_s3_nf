# meter_main
# Development History
## Development History
| Date       | Committer | Changes                                                                                     |
|------------|-----------|---------------------------------------------------------------------------------------------|
| 2024-8-20 | rvier | 机械臂标定适配:1.新增串口号和波特率参数选择功能 2.软件打包与使用，在window环境下成功使用 |
| 2024-8-20 | rvier | 机械臂标定适配：1.适配角度和平整度的手动标定指令，可以使用软件作为上位机配合机械臂手动标定设备；2.适配自动标定功能，用户一键即可完成标定或质检|
| 2024-8-21 | rvier | 完善机械臂标定功能：1.新增"Save_File"功能；2.新增标定和质检信息自动保存 |
| 2024-8-21 | rvier | 完善机械臂标定功能：1.新增"Echo_Data"功能；2.新增文本发送功能，可以使用软件作为上位机向设备发送指令 |
| 2024-8-21 | rvier | 完善机械臂标定功能：1.完善日志系统；2.新增标定和质检信息自动保存 |
| 2024-9-10 | rvier | 新增建发客制版本1.0：处于自动模式时，如果垂直放置，同时显示坡度和平整度|
| 2024-9-13 | rvier | 修改警示灯逻辑：通过时亮绿灯，不通过时亮红灯 |
| 2024-9-10 | rvier | 建发客制版本1.1，角度调零改为垂直调零（90°），角度调零时小屏也显示调零信息，新增点按测量键开始调零的功能；修复了一些主页显示BUG|
# Future Plans
| 2024-9-13 | rvier | "Switch_Page","Debug_Mode","Clear","Robot Params","NetIP","Port"|


size_t TwoWire::requestFrom(uint16_t address, size_t size, bool sendStop){
// .......
err = i2cRead(num, address, rxBuffer, size, _timeOutMillis, &rxLength);
if(err){
    // log_e("i2cRead returned Error %d", err);
}
}