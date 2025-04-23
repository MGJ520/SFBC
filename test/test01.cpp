// #include <Wire.h>
// #include <Arduino.h>
// // #include "esp_private/usb_console.h"
// // AS5600的I2C地址
// const uint8_t AS5600_ADDR = 0x36;

// // 定义ESP32-S3的I2C引脚(没有问题)
// // const int SDA_PIN = 8;
// // const int SCL_PIN = 9;
// // 定义ESP32-S3的I2C引脚(有问题)
// const int SDA_PIN = 37;
// const int SCL_PIN = 36;
// void setup() {
//   delay(1000); // 等待传感器稳定
//   // 初始化I2C通信（指定SDA和SCL引脚）
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Serial.begin(115200);
//   delay(1000); // 等待传感器稳定
// }

// void loop() {
//   // 读取AS5600原始角度值（12位）
//   Wire.beginTransmission(AS5600_ADDR);
//   Wire.write(0x0C); // 角度高字节寄存器地址
//   Wire.endTransmission(false);
//   Wire.requestFrom(AS5600_ADDR, 2); // 请求2字节数据

//   if (Wire.available() >= 2) {
//     // 解析高字节和低字节
//     uint8_t high_byte = Wire.read();
//     uint8_t low_byte = Wire.read();
//     uint16_t raw_angle = (high_byte << 8) | low_byte;

//     // 转换为实际角度（0~360°）
//     float angle_deg = (raw_angle * 360.0) / 4096.0;

//     // 输出结果
//     Serial.print("原始值: ");
//     Serial.print(raw_angle);
//     Serial.print(" | 角度: ");
//     Serial.print(angle_deg, 1); // 保留1位小数
//     Serial.println("°");
//   } else {
//     Serial.println("AS5600读取失败！");
//   }

//   delay(100); // 适当延时
// }
// #include <Arduino.h>

// // 引脚定义
// const int SDA_PIN = 37;
// const int SCL_PIN = 36;
// const uint8_t AS5600_ADDR = 0x36 << 1;

// // 改进后的GPIO控制函数（避免宏展开问题）
// inline void SDA_HIGH() {
//   pinMode(SDA_PIN, INPUT_PULLUP);
// }

// inline void SDA_LOW() {
//   pinMode(SDA_PIN, OUTPUT);
//   digitalWrite(SDA_PIN, LOW);
// }

// inline void SCL_HIGH() {
//   pinMode(SCL_PIN, INPUT_PULLUP);
// }

// inline void SCL_LOW() {
//   pinMode(SCL_PIN, OUTPUT);
//   digitalWrite(SCL_PIN, LOW);
// }

// #define READ_SDA (digitalRead(SDA_PIN))
// #define I2C_DELAY() delayMicroseconds(3)

// // 函数声明
// void i2c_send_ack();
// void i2c_send_nack();
// uint8_t i2c_wait_ack();
// void i2c_init();
// void i2c_start();
// void i2c_stop();
// uint8_t i2c_write_byte(uint8_t data);
// uint16_t i2c_read_data();

// void i2c_init() {
//   SDA_HIGH();
//   SCL_HIGH();
// }

// void i2c_start() {
//   SDA_HIGH();
//   SCL_HIGH();
//   I2C_DELAY();
//   SDA_LOW();
//   I2C_DELAY();
//   SCL_LOW();
//   I2C_DELAY();
// }

// void i2c_stop() {
//   SCL_LOW();
//   SDA_LOW();
//   I2C_DELAY();
//   SCL_HIGH();
//   I2C_DELAY();
//   SDA_HIGH();
//   I2C_DELAY();
// }

// uint8_t i2c_write_byte(uint8_t data) {
//   for(uint8_t i = 0; i < 8; i++) {
//     // 修改为if-else结构避免宏展开问题
//     if(data & 0x80) {
//       SDA_HIGH();
//     } else {
//       SDA_LOW();
//     }
//     data <<= 1;
//     SCL_HIGH();
//     I2C_DELAY();
//     SCL_LOW();
//     I2C_DELAY();
//   }
//   return i2c_wait_ack();
// }

// uint8_t i2c_wait_ack() {
//   SDA_HIGH();
//   pinMode(SDA_PIN, INPUT_PULLUP);
//   I2C_DELAY();
//   SCL_HIGH();

//   uint8_t retry = 200;
//   while(READ_SDA) {
//     if(--retry == 0) {
//       i2c_stop();
//       return 1;
//     }
//     I2C_DELAY();
//   }

//   SCL_LOW();
//   return 0;
// }

// void i2c_send_ack() {
//   SDA_LOW();
//   SCL_HIGH();
//   I2C_DELAY();
//   SCL_LOW();
//   I2C_DELAY();
//   SDA_HIGH();
// }

// void i2c_send_nack() {
//   SDA_HIGH();
//   SCL_HIGH();
//   I2C_DELAY();
//   SCL_LOW();
//   I2C_DELAY();
// }

// uint16_t i2c_read_data() {
//   uint8_t high_byte = 0, low_byte = 0;

//   // 读取高字节
//   SDA_HIGH();
//   for(uint8_t i=0; i<8; i++){
//     SCL_HIGH();
//     I2C_DELAY();
//     high_byte = (high_byte << 1) | READ_SDA;
//     SCL_LOW();
//     I2C_DELAY();
//   }
//   i2c_send_ack();

//   // 读取低字节
//   for(uint8_t i=0; i<8; i++){
//     SCL_HIGH();
//     I2C_DELAY();
//     low_byte = (low_byte << 1) | READ_SDA;
//     SCL_LOW();
//     I2C_DELAY();
//   }
//   i2c_send_nack();

//   return (high_byte << 8) | low_byte;
// }

// uint16_t read_angle_retry() {
//   for(int i=0; i<3; i++){
//     i2c_start();
//     if(i2c_write_byte(AS5600_ADDR | 0x00)) continue;
//     if(i2c_write_byte(0x0C)) continue;

//     i2c_start();
//     if(i2c_write_byte(AS5600_ADDR | 0x01)) continue;

//     uint16_t data = i2c_read_data();
//     i2c_stop();

//     if(data != 0xFFFF) return data;
//     delay(10);
//   }
//   return 0xFFFF;
// }

// void setup() {
//   Serial.begin(115200);
//   i2c_init();
//   delay(1000);
// }

// void loop() {
//   uint16_t raw_angle = read_angle_retry();

//   if(raw_angle == 0xFFFF) {
//     Serial.println("Read Error");
//     delay(100);
//     return;
//   }

//   float angle_deg = (raw_angle * 360.0) / 4096.0;
//   Serial.printf("Raw: %4d | Angle: %6.1f°\n", raw_angle, angle_deg);

//   delay(50);
// }

// #include "APP.h"
// #include "BatteryAndButton.h"
// #include "SuperCar.h"
// #include "esp_flash.h"
// #include "driver/spi_common.h"

// // 增加注释
// //  pio run -v -t upload

// void setup()
// {
//     PowerAndButton.startTask(); // 开按键与电压检测任务
//     APP.startTask();            // 开手机遥控器APP任务
//     BalanceCar.startTask();     // 平衡车初始化
// }

// void loop()
// {
//     BalanceCar.running();       // 调用平衡车的运行函数
// 播放短促的蜂鸣声，表示FOC自检完成
    // buzzer.play(S_BEEP);
//     // delay(5000);
// }

// #include <Wire.h>
// #include <Arduino.h>
// #include <SimpleFOC.h>

// // MPU6050配置
// const int MPU_ADDR = 0x68;
// const int SDA_PIN = 8;
// const int SCL_PIN = 9;

// // 电机驱动器配置
// BLDCDriver3PWM driver_A = BLDCDriver3PWM(14, 13, 35);
// BLDCDriver3PWM driver_B = BLDCDriver3PWM(11, 10 ,12);

// // 定义电机对象
// BLDCMotor motor_A = BLDCMotor(7); // 极对数需根据电机参数修改
// BLDCMotor motor_B = BLDCMotor(7);

// // PID参数
// float kp = 0.1, ki = 0.01, kd = 0.00;
// float target_angle = 0; // 目标平衡角度
// float integral = 0, last_error = 0;

// // 传感器数据变量
// float accX, accY, accZ, gyroX, gyroY, gyroZ;
// float a1ngle = 0;
// float angle_prev = 0;
// unsigned long last_time = 0;
// float acc1_angle = 0;

// void setupMPU6050()
// {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x6B); // PWR_MGMT_1
//     Wire.write(0x00); // 唤醒
//     Wire.endTransmission();

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1B); // 陀螺仪配置
//     Wire.write(0x00); // ±250dps
//     Wire.endTransmission();

//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x1C); // 加速度配置
//     Wire.write(0x00); // ±2g
//     Wire.endTransmission();
// }

// void setup()
// {
//     Serial.begin(115200);
//     PowerAndButton.startTask(); // 开按键与电压检测任务
//     // 初始化I2C
//     Wire.begin(SDA_PIN, SCL_PIN);
//     setupMPU6050();

//     // 初始化电机驱动器
//     driver_A.voltage_power_supply = 7;
//     driver_A.voltage_limit = 0.5;
//     driver_A.init();
//     driver_B.voltage_power_supply = 7;
//     driver_B.voltage_limit = 0.5;
//     driver_B.init();

//     // 连接电机与驱动器
//     motor_A.linkDriver(&driver_A);
//     motor_B.linkDriver(&driver_B);

//     // 配置电机控制模式
//     motor_A.controller = MotionControlType::velocity_openloop;
//     motor_B.controller = MotionControlType::velocity_openloop;

//     // 初始化电机
//     motor_A.init();
//     motor_B.init();
// }
// void readMPU6050()
// {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x3B);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU_ADDR, 14);

//     // 解析原始数据
//     accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
//     accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
//     Wire.read();
//     Wire.read(); // 跳过accX和温度
//     gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
//     Wire.read();
//     Wire.read(); // 跳过gyroY和gyroZ
// }
// void loop()
// {
//     // 读取传感器数据
//     readMPU6050();

//     // 计算倾斜角度（简单互补滤波）
//     float dt = (millis() - last_time) * 0.001;
//     acc1_angle = atan2(accY, accZ) * (180.0 / PI);
//     a1ngle = (0.98 * (angle + gyroX * dt)) + (0.02 * acc1_angle);

//     // PID计算
//     float error = a1ngle - target_angle;
//     integral += error * dt;
//     float derivative = (error - last_error) / dt;
//     float output = kp * error + ki * integral + kd * derivative;
//     last_error = error;

//     // 电机控制（转换为速度指令）
//     float motor_speed = constrain(output, -2, 2); // 限制最大速度

//     // 根据方向设置电机（可能需要调整正负号）
//     motor_A.move(1);
//     motor_B.move(1); // 假设电机对称安装需反转方向

//     // 记录时间
//     last_time = millis();

//     // 调试输出
//     Serial.print("Angle: ");
//     Serial.print(angle);
//     Serial.print(" Output: ");
//     Serial.println(motor_speed);
// }
