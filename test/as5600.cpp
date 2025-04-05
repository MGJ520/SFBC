#include <Wire.h>
#include <Arduino.h>
// AS5600的I2C地址
const uint8_t AS5600_ADDR = 0x36; 

// 定义ESP32-S3的I2C引脚
// const int SDA_PIN = 8;
// const int SCL_PIN = 9;
const int SDA_PIN = 37;
const int SCL_PIN = 36;
void setup() {
  // 初始化I2C通信（指定SDA和SCL引脚）
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  delay(1000); // 等待传感器稳定
}

void loop() {
  // 读取AS5600原始角度值（12位）
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C); // 角度高字节寄存器地址
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2); // 请求2字节数据

  if (Wire.available() >= 2) {
    // 解析高字节和低字节
    uint8_t high_byte = Wire.read();
    uint8_t low_byte = Wire.read();
    uint16_t raw_angle = (high_byte << 8) | low_byte;

    // 转换为实际角度（0~360°）
    float angle_deg = (raw_angle * 360.0) / 4096.0;

    // 输出结果
    Serial.print("原始值: ");
    Serial.print(raw_angle);
    Serial.print(" | 角度: ");
    Serial.print(angle_deg, 1); // 保留1位小数
    Serial.println("°");
  } else {
    Serial.println("AS5600读取失败！");
  }

  delay(100); // 适当延时
}