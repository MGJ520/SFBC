#ifndef FOC_CODE_CONTROL_H
#define FOC_CODE_CONTROL_H

#include "include/precompiled.h"


extern enum Car {
    Disable_Output,//关闭输出
    Open_Output,//打开输出
    Error_Unknown,//错误
    Error_Init,
    Error_Power,
    Error_Mpu,
    Error_Encoder,
} System_Status;


// 卡尔曼滤波器
//extern SimpleKalmanFilter KalmanFilter_mpu;

// 低通滤波器
extern LowPassFilter lpf_speed;
extern LowPassFilter lpf_trun;
extern LowPassFilter lpf_run;

// 偏置参数
extern float Offset_parameters;

extern const float MAX_ALLOWED_SPEED;
extern int speed_count;
extern float Now_Speed;
extern float Now_Pitch;
extern float PID_Speed_Out;
extern float PID_Pitch_Out;
extern float C_Speed;
extern float C_Turn;
extern float Target_torque_A;
extern float Target_torque_B;

void AbnormalSpinDetect();
void LandingDetect();

void Control_Loop();
#endif //FOC_CODE_CONTROL_H
