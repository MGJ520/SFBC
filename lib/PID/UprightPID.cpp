#include "UprightPID.h" // 引入PID控制相关的头文件

// 初始化PID控制器的函数
void UprightPID_Init(UprightPID_t *pidConf, float Kp, float Ki, float Kd, float limit)
{
    pidConf->Kp = Kp;        // 设置比例系数
    pidConf->Ki = Ki;        // 设置积分系数
    pidConf->Kd = Kd;        // 设置微分系数
    pidConf->Ki_Out = 0.0f;  // 初始化积分输出为0
    pidConf->PID_Out = 0.0f; // 初始化PID总输出为0

    // 设置比例输出的最大和最小限制
    pidConf->Kp_Min = -limit;
    pidConf->Kp_Max = limit;

    // 设置积分输出的最大和最小限制（通常是比例限制的一半）
    pidConf->Ki_Min = -limit / 2.0f;
    pidConf->Ki_Max = limit / 2.0f;

    // 设置微分输出的最大和最小限制（通常是比例限制的一半）
    pidConf->Kd_Min = -limit / 2.0f;
    pidConf->Kd_Max = limit / 2.0f;

    // 设置PID总输出的最大和最小限制
    pidConf->outMin = -limit;
    pidConf->outMax = limit;
}

// PID控制算法的实现函数
float UprightPID(UprightPID_t *pidConf, float targetAngleX, float angleX, float GyroX)
{
    float Error_value; // 误差值

    float P_Out; // 比例输出
    float D_Out; // 微分输出

    // 计算误差值，目标角度与当前角度的差值
    Error_value = angleX + targetAngleX;

    // 计算比例输出
    P_Out = Error_value * pidConf->Kp;

    // 累加积分输出
    pidConf->Ki_Out += Error_value * pidConf->Ki;

    // 计算微分输出，使用陀螺仪数据（GyroX）作为微分项的输入
    D_Out = GyroX * pidConf->Kd;

    // 限制比例输出的范围
    if (P_Out < pidConf->Kp_Min)
        P_Out = pidConf->Kp_Min;
    else if (P_Out > pidConf->Kp_Max)
        P_Out = pidConf->Kp_Max;

    // 限制积分输出的范围
    if (pidConf->Ki_Out < pidConf->Ki_Min)
        pidConf->Ki_Out = pidConf->Ki_Min;
    else if (pidConf->Ki_Out > pidConf->Ki_Max)
        pidConf->Ki_Out = pidConf->Ki_Max;

    // 限制微分输出的范围
    if (D_Out < pidConf->Kd_Min)
        D_Out = pidConf->Kd_Min;
    else if (D_Out > pidConf->Kd_Max)
        D_Out = pidConf->Kd_Max;

    // 计算PID总输出
    pidConf->PID_Out = P_Out + pidConf->Ki_Out + D_Out;

    // 限制PID总输出的范围
    if (pidConf->PID_Out > pidConf->outMax)
    {
        pidConf->PID_Out = pidConf->outMax;
    }
    else if (pidConf->PID_Out < pidConf->outMin)
    {
        pidConf->PID_Out = pidConf->outMin;
    }

    // 返回PID总输出值
    return pidConf->PID_Out;
}