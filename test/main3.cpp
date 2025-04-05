-// #include <Wire.h>
// #include <Arduino.h>

// // MPU6050的I2C地址（AD0引脚接地时为0x68，接高电平为0x69）
// const int MPU_ADDR = 0x68; 

// // 定义ESP32-S3的I2C引脚
// const int SDA_PIN = 8;
// const int SCL_PIN = 9;

// // 加速度和陀螺仪的原始数据变量
// int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;

// void setup() {
//   // 初始化I2C通信（指定SDA和SCL引脚）
//   Wire.begin(SDA_PIN, SCL_PIN);
//   Serial.begin(115200);
  
//   // 唤醒MPU6050
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x6B);      // 电源管理寄存器
//   Wire.write(0x00);      // 退出睡眠模式
//   Wire.endTransmission(true);

//   // 配置加速度计 ±2g
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x1C);      // 加速度配置寄存器
//   Wire.write(0x00);      // ±2g（16384 LSB/g）
//   Wire.endTransmission(true);

//   // 配置陀螺仪 ±250度/秒
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x1B);      // 陀螺仪配置寄存器
//   Wire.write(0x00);      // ±250度/秒（131 LSB/度/秒）
//   Wire.endTransmission(true);
// }

// void loop() {
//   // 请求读取传感器数据
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x3B);      // 起始寄存器地址：加速度X高字节
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU_ADDR, 14, true);

//   // 解析原始数据（注意顺序）
//   accX  = Wire.read() << 8 | Wire.read(); // 加速度X
//   accY  = Wire.read() << 8 | Wire.read(); // 加速度Y
//   accZ  = Wire.read() << 8 | Wire.read(); // 加速度Z
//   int16_t temp = Wire.read() << 8 | Wire.read(); // 温度（未处理）
//   gyroX = Wire.read() << 8 | Wire.read(); // 陀螺仪X
//   gyroY = Wire.read() << 8 | Wire.read(); // 陀螺仪Y
//   gyroZ = Wire.read() << 8 | Wire.read(); // 陀螺仪Z

//   // 转换为实际单位
//   float acc_scale = 16384.0;  // ±2g灵敏度
//   float gyro_scale = 131.0;   // ±250度/秒灵敏度

//   Serial.print("加速度 [g]: X="); 
//   Serial.print(accX / acc_scale, 2);  // 保留两位小数
//   Serial.print(", Y="); 
//   Serial.print(accY / acc_scale, 2); 
//   Serial.print(", Z="); 
//   Serial.print(accZ / acc_scale, 2);

//   Serial.print(" | 陀螺仪 [度/秒]: X="); 
//   Serial.print(gyroX / gyro_scale, 2); 
//   Serial.print(", Y="); 
//   Serial.print(gyroY / gyro_scale, 2); 
//   Serial.print(", Z="); 
//   Serial.println(gyroZ / gyro_scale, 2);

//   delay(100); // 适当降低延时提高数据刷新率
// }