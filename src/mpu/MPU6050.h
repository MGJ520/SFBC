#ifndef MPU6050_H
#define MPU6050_H


#include "Include/precompiled.h"


#define GYRO_COEF 0.975f
#define GYRO_COEFX 0.85f
#define PRE_GYRO_COEFX 0.25f


#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

class MPU6050 {
public:

    bool Is_Connect= false;

    MPU6050(TwoWire &w);

    MPU6050(TwoWire &w, float aC, float gC, float gCX);

    bool begin();

    bool checkMPU6050();

    void setGyroOffsets(float x, float y, float z);

    void writeMPU6050(byte reg, byte data);

    byte readMPU6050(byte reg);

    int16_t getRawAccX() {
        return rawAccX;
    };

    int16_t getRawAccY() {
        return rawAccY;
    };

    int16_t getRawAccZ() {
        return rawAccZ;
    };

    int16_t getRawTemp() {
        return rawTemp;
    };

    int16_t getRawGyroX() {
        return rawGyroX;
    };

    int16_t getRawGyroY() {
        return rawGyroY;
    };

    int16_t getRawGyroZ() {
        return rawGyroZ;
    };

    float getGyroXFV() {
        // 计算陀螺仪 X 轴的滤波值
        // 使用了加权平均滤波算法（也称为一阶低通滤波器）
        // 公式：gyroXFV = α * gyroX + (1 - α) * gyroXFV + β * preGyroX
        // 其中：
        // - gyroX 是当前时刻的陀螺仪 X 轴数据
        // - gyroXFV 是滤波后的陀螺仪 X 轴数据
        // - preGyroX 是上一次的陀螺仪 X 轴数据
        // - gyroCoefX (α) 是当前滤波系数，决定了当前数据的权重
        // - preGyroCoefX (β) 是上一次滤波系数，用于进一步调整滤波效果
        gyroXFV = gyroCoefX * gyroX + (1 - gyroCoefX) * gyroXFV + preGyroCoefX * preGyroX;

        // 更新 preGyroX 为当前的 gyroX，用于下一次计算
        preGyroX = gyroX;

        // 返回滤波后的陀螺仪 X 轴值
        return gyroXFV;
    }

    float getAccX() {
        return accX;
    };

    float getAccY() {
        return accY;
    };

    float getAccZ() {
        return accZ;
    };

    float getGyroX() {
        return gyroX;
    };

    float getGyroY() {
        return gyroY;
    };

    float getGyroZ() {
        return gyroZ;
    };

    void calcGyroOffsets(bool console = false, uint16_t delayBefore = 0, uint16_t delayAfter = 0);

    float getGyroXoffset() {
        return gyroXoffset;
    };

    float getGyroYoffset() {
        return gyroYoffset;
    };

    float getGyroZoffset() {
        return gyroZoffset;
    };

    bool update();

    float getAccAngleX() {
        return angleAccX;
    };

    float getAccAngleY() {
        return angleAccY;
    };

    float getGyroAngleX() {
        return angleGyroX;
    };

    float getGyroAngleY() {
        return angleGyroY;
    };

    float getGyroAngleZ() {
        return angleGyroZ;
    };

    float getAngleX() {
        return angleX-(0.0231f+temperature*0.0421f);
    };

    float getAngleY() {
        return angleY;
    };

    float getAngleZ() {
        return angleZ;
    };


    // 新增变量：用于计算Z轴垂直于地面时的绝对XYZ加速度
    float absAccX, absAccY, absAccZ,temperature; // 绝对加速度（X、Y、Z轴）

    void calculateAbsoluteAcceleration();

private:
    TwoWire *wire; // 指向I2C通信接口的指针，用于与MPU6050传感器通信

    // 原始传感器数据
    int16_t rawAccX, rawAccY, rawAccZ, // 原始加速度计数据（X、Y、Z轴）
    rawTemp,                    // 原始温度数据
    rawGyroX, rawGyroY, rawGyroZ; // 原始陀螺仪数据（X、Y、Z轴）

    // 陀螺仪偏移量
    float gyroXoffset, gyroYoffset, gyroZoffset; // 陀螺仪X、Y、Z轴的偏移量，用于校准

    // 处理后的传感器数据
    float accX, accY, accZ, // 处理后的加速度计数据（X、Y、Z轴）
    gyroX, gyroY, gyroZ, // 处理后的陀螺仪数据（X、Y、Z轴）
    gyroXFV;             // 陀螺仪X轴的滤波值

    // 角度计算相关变量
    float angleGyroX, angleGyroY, angleGyroZ, // 通过陀螺仪计算的角度（X、Y、Z轴）
    angleAccX, angleAccY, angleAccZ;    // 通过加速度计计算的角度（X、Y、Z轴）

    // 最终融合角度
    float angleX, angleY, angleZ; // 通过加速度计和陀螺仪数据融合后的角度（X、Y、Z轴）

    // 时间相关变量
    float interval, preGyroX; // 采样时间间隔、上一次陀螺仪X轴的值
    long preInterval;          // 上一次采样时间

    // 融合算法系数
    float accCoef, gyroCoef, // 加速度计和陀螺仪的融合系数
    gyroCoefX, preGyroCoefX; // 陀螺仪X轴的融合系数和上一次的融合系数


};

#endif
