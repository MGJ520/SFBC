#include "XboxVibrationLibrary.h"

// 构造函数
XboxVibration::XboxVibration() {
  // 初始化控制器对象
}

// 初始化函数
void XboxVibration::begin() {
  _xboxController.begin(); // 初始化 Xbox 控制器
}

// 设置震动参数
void XboxVibration::setVibration(bool center, bool left, bool right, bool shake, uint8_t power, uint8_t timeActive, uint8_t timeSilent, uint8_t countRepeat) {
  _repo.v.select.center = center; // 设置中心震动
  _repo.v.select.left = left; // 设置左侧震动
  _repo.v.select.right = right; // 设置右侧震动
  _repo.v.select.shake = shake; // 设置摇晃震动
  _repo.v.power.center = power; // 设置中心震动功率
  _repo.v.power.left = power; // 设置左侧震动功率
  _repo.v.power.right = power; // 设置右侧震动功率
  _repo.v.power.shake = power; // 设置摇晃震动功率
  _repo.v.timeActive = timeActive; // 设置震动持续时间
  _repo.v.timeSilent = timeSilent; // 设置震动间隔时间
  _repo.v.countRepeat = countRepeat; // 设置重复次数
}

// 发送震动指令
void XboxVibration::sendVibration() {
  _xboxController.writeHIDReport(_repo); // 写入 HID 报告
}

// 检查控制器是否连接
bool XboxVibration::isConnected() {
  return _xboxController.isConnected(); // 返回控制器连接状态
}

// 循环函数，用于维持连接
void XboxVibration::onLoop() {
  _xboxController.onLoop(); // 调用控制器的循环函数
}