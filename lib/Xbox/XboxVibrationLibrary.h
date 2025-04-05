#ifndef XboxVibrationLibrary_h
#define XboxVibrationLibrary_h

#include <XboxSeriesXControllerESP32_asukiaaa.hpp> // 引入 Xbox Series X 控制器库

class XboxVibration {
  public:
    XboxVibration(); // 构造函数
    void begin(); // 初始化函数
    void setVibration(bool center, bool left, bool right, bool shake, uint8_t power, uint8_t timeActive, uint8_t timeSilent, uint8_t countRepeat); // 设置震动参数
    void sendVibration(); // 发送震动指令
    bool isConnected(); // 检查控制器是否连接
    void onLoop(); // 循环函数，用于维持连接

  private:
    XboxSeriesXControllerESP32_asukiaaa::Core _xboxController; // Xbox 控制器对象
    XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase _repo; // HID 报告对象
};

#endif