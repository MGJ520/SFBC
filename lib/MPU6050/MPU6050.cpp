// 包含必要的库和头文件
#include "MPU6050.h"
#include "Arduino.h"
#include "UserConfig.h"  // 用户自定义配置（如滤波器系数）

// 构造函数1：使用UserConfig.h中的默认系数初始化
MPU6050::MPU6050(TwoWire &w) {
    wire = &w;  // 保存Wire对象指针用于I2C通信
    // 从UserConfig.h加载滤波器系数
    gyroCoef = GYRO_COEF;       // 陀螺仪主系数（默认0.975）
    gyroCoefX = GYRO_COEFX;     // X轴特殊系数（默认0.85）
    preGyroCoefX = PRE_GYRO_COEFX; // 预滤波系数（需在UserConfig定义）
    accCoef = 1.0f - gyroCoef;  // 加速度计系数（互补滤波器用，默认0.025）
}

// 构造函数2：允许自定义滤波器系数
MPU6050::MPU6050(TwoWire &w, float aC, float gC, float gCX) {
    wire = &w;
    accCoef = aC;    // 自定义加速度计系数
    gyroCoef = gC;   // 自定义陀螺仪主系数
    gyroCoefX = gCX; // 自定义X轴特殊系数
}

// 初始化MPU6050传感器
void MPU6050::begin() {
    // 配置MPU6050寄存器
    writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // 采样率分频器（默认1kHz）
    writeMPU6050(MPU6050_CONFIG, 0x00);       // 禁用低通滤波器
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // 陀螺仪±500dps量程
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // 加速度计±2g量程
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x00);   // 退出睡眠模式

    update();  // 首次读取数据初始化角度计算

    // 初始化状态变量
    angleGyroX = 0.0f;       // 陀螺仪X轴积分角度
    angleGyroY = 0.0f;       // 陀螺仪Y轴积分角度（当前未使用）
    gyroXFV = 0.0f;          // 滤波后的陀螺仪X轴数据
    preGyroX = 0.0f;         // 前一时刻陀螺仪X轴数据（滤波用）
    angleX = getAccAngleX(); // 初始角度取自加速度计
    angleY = getAccAngleY();
    preInterval = millis();  // 记录初始时间戳
}

// 写入MPU6050寄存器
void MPU6050::writeMPU6050(byte reg, byte data) {
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);     // 指定寄存器地址
    wire->write(data);    // 写入配置值
    wire->endTransmission();
}

// 读取MPU6050寄存器
byte MPU6050::readMPU6050(byte reg) {
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(reg);                // 指定寄存器地址
    wire->endTransmission(true);     // 保持连接
    wire->requestFrom(MPU6050_ADDR, 1); // 请求1字节数据
    return wire->read();             // 返回读取值
}

// 设置手动陀螺仪偏移量
void MPU6050::setGyroOffsets(float x, float y, float z) {
    gyroXoffset = x;
    gyroYoffset = y;
    gyroZoffset = z;
}

// 自动计算陀螺仪偏移（需保持传感器静止）
void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter) {
    float calc_time = 20000;
    Preferences preferences;  // 创建Preferences对象
    // 打开Preferences，指定命名空间为"counterStorage"
    preferences.begin("MPU_GyroOffsets", false);

    // 读取计数器
    if (preferences.getInt("new", -1) != 3) {

#ifdef LED_3_GPIO
        pinMode(LED_3_GPIO, OUTPUT);
        digitalWrite(LED_3_GPIO, HIGH);
#endif
        preferences.putInt("new", 3);  // 将新的计数值写入NVS
        delay(delayBefore);  // 校准前等待时间
        float x = 0, y = 0, z = 0;
        int16_t rx, ry, rz;

        // 采集3000次样本求平均
        for (int i = 0; i < (int) calc_time; i++) {
            if (console && i % 1000 == 0) Serial.print(".");
            // 读取陀螺仪原始数据（寄存器0x43开始，6个字节）
            wire->beginTransmission(MPU6050_ADDR);
            wire->write(0x43);
            wire->endTransmission(false);
            wire->requestFrom((int) MPU6050_ADDR, 6);

            // 解析XYZ三轴数据（注意数据顺序）
            rx = wire->read() << 8 | wire->read();
            ry = wire->read() << 8 | wire->read();
            rz = wire->read() << 8 | wire->read();

            // 转换为度/秒并累加（65.5 = 500dps量程的LSB转换系数）
            x += ((float) rx) / 65.5;
            y += ((float) ry) / 65.5;
            z += ((float) rz) / 65.5;
        }

        // 计算平均偏移量（X轴特殊处理-5.0）
        gyroXoffset = x / calc_time - 5.0f;
        gyroYoffset = y / calc_time;
        gyroZoffset = z / calc_time;


        Serial.println("\nDone!");
        Serial.println("X : " + String(gyroXoffset));
        Serial.println("Y : " + String(gyroYoffset));
        Serial.println("Z : " + String(gyroZoffset));
        preferences.putFloat("gyroXoffset", gyroXoffset);  // 将新的计数值写入NVS
        preferences.putFloat("gyroYoffset", gyroYoffset);  // 将新的计数值写入NVS
        preferences.putFloat("gyroZoffset", gyroZoffset);  // 将新的计数值写入NVS
        delay(delayAfter);  // 校准后等待时间
#ifdef LED_3_GPIO
        digitalWrite(LED_3_GPIO, LOW);
#endif
    } else {
        gyroXoffset=preferences.getFloat("gyroXoffset", gyroXoffset);  // 将新的计数值写入NVS
        gyroYoffset=preferences.getFloat("gyroYoffset", gyroYoffset);  // 将新的计数值写入NVS
        gyroZoffset=preferences.getFloat("gyroZoffset", gyroZoffset);  // 将新的计数值写入NVS
    }

    // 关闭Preferences
    preferences.end();

}

// 主更新函数：读取传感器数据并计算角度
void MPU6050::update() {
    // 从寄存器0x3B开始读取14个字节（加速度+温度+陀螺仪）
    wire->beginTransmission(MPU6050_ADDR);
    wire->write(0x3B);
    wire->endTransmission(false);
    wire->requestFrom((int) MPU6050_ADDR, 14);

    // 解析加速度计原始数据（每个轴2个字节）
    rawAccX = wire->read() << 8 | wire->read();
    rawAccY = wire->read() << 8 | wire->read();
    rawAccZ = wire->read() << 8 | wire->read();
    rawTemp = wire->read() << 8 | wire->read();  // 温度数据未使用

    // 解析陀螺仪原始数据
    rawGyroX = wire->read() << 8 | wire->read();
    rawGyroY = wire->read() << 8 | wire->read();
    rawGyroZ = wire->read() << 8 | wire->read();

    // 转换加速度数据到g单位（16384 = ±2g量程的LSB转换系数）
    accX = ((float) rawAccX) / 16384.0;
    accY = ((float) rawAccY) / 16384.0;
    accZ = ((float) rawAccZ) / 16384.0;

    // 通过加速度计计算X轴倾斜角（单位：度）
    // 公式解释：atan2(accY, accZ)计算绕X轴旋转角度
    // 加abs(accX)防止除零，转换为角度值
    angleAccX = atan2(accY, accZ + abs(accX)) * 360 / (2.0 * PI);

    // 转换陀螺仪数据到度/秒（65.5 = ±500dps量程的LSB转换系数）
    gyroX = ((float) rawGyroX) / 65.5;
    gyroZ = ((float) rawGyroZ) / 65.5;

    // 应用陀螺仪偏移校准
    gyroX -= gyroXoffset;
    gyroZ -= gyroZoffset;

    // 计算时间间隔（秒）
    interval = (millis() - preInterval) * 0.001;

    // 通过积分计算陀螺仪角度（角度=角速度×时间）
    angleGyroX += gyroX * interval;
    angleGyroZ += gyroZ * interval;

    // 互补滤波器融合数据（陀螺仪+加速度计）
    // angleX：主要使用陀螺仪，用加速度计修正漂移
    angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
    // angleZ：纯陀螺仪积分（无加速度计参考）
    angleZ = angleGyroZ;

    preInterval = millis();  // 更新时间戳
}