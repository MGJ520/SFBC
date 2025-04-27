//
// Created by MGJ on 2025/4/26.
//

#ifndef FOC_CODE_PID_ADJUST_H
#define FOC_CODE_PID_ADJUST_H


typedef struct _PIDFloat_Obj_
{
    volatile float Kp;
    volatile float Ki;
    volatile float Kd;
    volatile float Ki_Out;
    volatile float Kp_Min;
    volatile float Kp_Max;
    volatile float Ki_Min;
    volatile float Ki_Max;
    volatile float Kd_Min;
    volatile float Kd_Max;
    volatile float outMin;
    volatile float outMax;
    volatile float PID_Out;
    float OutLimit;
} PID_Structure;

extern  PID_Structure UprightPID;
extern  PID_Structure SpeedPID;
extern  PID_Structure TurnPID;

void PID_parameters_Init();

float PID_Adjust(PID_Structure *handle, float Given, float Feedback);

float PID_Adjust_T(PID_Structure *handle, float Given, float Feedback, float Gyro);

#endif //FOC_CODE_PID_ADJUST_H
