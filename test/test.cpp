#include "XboxVibrationLibrary.h"

XboxVibration vibration; // 创建震动库对象

void setup() {
  // Serial.begin(115200); // 初始化串口通信
  vibration.begin(); // 初始化震动库
}

void loop() {
  vibration.onLoop(); // 调用循环函数，维持连接

  if (vibration.isConnected()) { // 检查控制器是否连接
    Serial.println("Controller is connected");

    // 设置震动参数
    vibration.setVibration(true, false, false, false, 50, 100, 0, 1); // 中心震动，50% 功率，持续 1 秒
    vibration.sendVibration(); // 发送震动指令
    delay(2000); // 延时 2 秒

    vibration.setVibration(false, true, false, false, 50, 100, 0, 1); // 左侧震动，50% 功率，持续 1 秒
    vibration.sendVibration(); // 发送震动指令
    delay(2000); // 延时 2 秒

    vibration.setVibration(false, false, true, false, 50, 100, 0, 1); // 右侧震动，50% 功率，持续 1 秒
    vibration.sendVibration(); // 发送震动指令
    delay(2000); // 延时 2 秒

    vibration.setVibration(false, false, false, true, 50, 100, 0, 1); // 摇晃震动，50% 功率，持续 1 秒
    vibration.sendVibration(); // 发送震动指令
    delay(2000); // 延时 2 秒
  } else {
    Serial.println("Controller is not connected");
    delay(1000); // 延时 1 秒
  }
}