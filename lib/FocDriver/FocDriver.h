#ifndef _SUPERCAR_H__
#define _SUPERCAR_H__

typedef struct carControl
{
    float MPUangleX = 0.0f;        // MPU6050传感器测量的X轴角度（单位：度）
    float MPUgyroX = 0.0f;         // MPU6050传感器测量的X轴角速度（单位：度/秒，滤波值）

    float MA_Velocity = 0.0f;      // 电机A的实际转轴速度（单位：转/秒）
    float MB_Velocity = 0.0f;      // 电机B的实际转轴速度（单位：转/秒）

    float TargetVelocity = 1.0f;   // 目标速度（单位：转/秒），默认值为1.0
    float SteerVelocity = 0.0f;    // 转向速度（单位：转/秒），用于控制转向

    float VelocityError = 0.0f;    // 当前速度误差（目标速度 - 实际速度）
    float TargetAngle = 0.0f;      // 目标姿态角（单位：度），由速度PID控制器计算得出
    float MotorVelocity = 0.0f;    // 电机速度（单位：转/秒），由姿态PID控制器计算得出

} CarControl_t;  // 定义一个别名 CarControl_t，用于表示车辆控制结构体

class FocDriver
{
  public:
    boolean init();
    void running();
};

extern FocDriver BalanceDriver;

#endif