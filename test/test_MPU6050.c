
#include <Arduino.h>
#include <MPU6050.h>
#include <Wire.h>

// 初始化 I2C 和 MPU6050
TwoWire I2C_B = TwoWire(0); // 使用 I2C 总线 0
MPU6050 mpu6050(I2C_B);

void setup()
{
    Serial.begin(115200);

    // 初始化 I2C 总线
    I2C_B.begin(8, 9, 400000UL); // SDA=8, SCL=9, 400kHz

    // 初始化 MPU6050
    mpu6050.begin();

    // 校准陀螺仪（需保持传感器静止）
    Serial.println("正在校准 MPU6050，请保持传感器静止...");
    mpu6050.calcGyroOffsets();
    Serial.println("校准完成！");
}

void loop()
{
    // 更新传感器数据
    mpu6050.update();

    // 读取加速度数据
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();

    // 读取陀螺仪数据
    float gyroX = mpu6050.getGyroX();
    float gyroY = mpu6050.getGyroY();
    float gyroZ = mpu6050.getGyroZ();

    // 打印数据
    Serial.print("Accel (g): X=");
    Serial.print(accelX);
    Serial.print(" Y=");
    Serial.print(accelY);
    Serial.print(" Z=");
    Serial.print(accelZ);

    Serial.print(" | Gyro (deg/s): X=");
    Serial.print(gyroX);
    Serial.print(" Y=");
    Serial.print(gyroY);
    Serial.print(" Z=");
    Serial.print(gyroZ);

    delay(200); // 采样间隔
}