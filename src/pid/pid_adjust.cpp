#include "pid_adjust.h"

PID_Structure UprightPID;
PID_Structure SpeedPID;
PID_Structure TurnPID;


#define UprightLimit 8.0f // 直立最大输出
#define TurnLimit    2.0f // 旋转最大输出
#define SpeedLimit   3.5f // 速度最大输出



void PID_parameters_Init()
{
/*************************************************************/
    SpeedPID.Kp   = 0.15f;
    SpeedPID.Ki   = 0.00001f;
    SpeedPID.Kd   = 0.00001f;

    UprightPID.Kp = 0.415f;
    UprightPID.Ki = 0.0f;
    UprightPID.Kd = 0.00001f;

    TurnPID.Kp = 0.07f;
    TurnPID.Ki = 0.0001f;
    TurnPID.Kd = 0.001f;


    SpeedPID.Ki_Out = 0.0f;

    SpeedPID.Kp_Min = -SpeedLimit;
    SpeedPID.Kp_Max = SpeedLimit;

    SpeedPID.Kd_Min = -SpeedLimit;
    SpeedPID.Kd_Max = SpeedLimit;

    SpeedPID.Ki_Min = -SpeedLimit / SpeedPID.Ki;
    SpeedPID.Ki_Max = SpeedLimit / SpeedPID.Ki;

    SpeedPID.outMin = -SpeedLimit / SpeedPID.Ki;
    SpeedPID.outMax = SpeedLimit / SpeedPID.Ki;

    SpeedPID.PID_Out = 0.0f;
    SpeedPID.OutLimit = SpeedLimit;

/*************************************************************/


/*************************************************************/


    UprightPID.Ki_Out = 0.0f;

    UprightPID.Kp_Min = -UprightLimit;
    UprightPID.Kp_Max = UprightLimit;

    UprightPID.Kd_Min = -UprightLimit;
    UprightPID.Kd_Max = UprightLimit;

    UprightPID.Ki_Min = -UprightLimit;
    UprightPID.Ki_Max = UprightLimit;

    UprightPID.outMin = -UprightLimit;
    UprightPID.outMax = UprightLimit;

    UprightPID.PID_Out = 0.0f;
/*************************************************************/
/*************************************************************/


    TurnPID.Ki_Out = 0.0f;

    TurnPID.Kp_Min = -TurnLimit;
    TurnPID.Kp_Max = TurnLimit;

    TurnPID.Kd_Min = -TurnLimit;
    TurnPID.Kd_Max = TurnLimit;

    TurnPID.Ki_Min = -TurnLimit;
    TurnPID.Ki_Max = TurnLimit;

    TurnPID.outMin = -TurnLimit;
    TurnPID.outMax = TurnLimit;

    TurnPID.PID_Out = 0.0f;
/*************************************************************/
}





/**
 * PID控制器函数，根据给定值和反馈值计算PID控制输出。
 *
 * @param handle 指向PID控制器结构体的指针，包含PID控制器的所有参数和状态。
 * @param Given 目标值（给定值），即PID控制器需要达到的目标值。
 * @param Feedback 反馈值，即从系统中测量到的实际值。
 *
 * @return 计算得到的PID控制输出值。
 */
float PID_Adjust(PID_Structure *handle, float Given, float Feedback) {
    float Error_value; // 误差值

    float P_Out; // 比例项输出

    // 计算误差值：目标值减去反馈值
    Error_value = Given - Feedback;

    // 计算比例项输出
    P_Out = Error_value * handle->Kp; // 比例项 = 误差值 * 比例增益 (Kp)

    // 累加积分项输出
    handle->Ki_Out += Error_value * handle->Ki; // 积分项 = 积分项 + 误差值 * 积分增益 (Ki)

    // 限制比例项输出，防止超出范围
    if (P_Out < handle->Kp_Min) P_Out = handle->Kp_Min; // 如果比例项小于最小值，设置为最小值
    else if (P_Out > handle->Kp_Max) P_Out = handle->Kp_Max; // 如果比例项大于最大值，设置为最大值

    // 限制积分项输出，防止积分饱和
    if (handle->Ki_Out < handle->Ki_Min) handle->Ki_Out = handle->Ki_Min; // 如果积分项小于最小值，设置为最小值
    else if (handle->Ki_Out > handle->Ki_Max) handle->Ki_Out = handle->Ki_Max; // 如果积分项大于最大值，设置为最大值

    // 计算总PID输出
    handle->PID_Out = P_Out + handle->Ki_Out; // PID输出 = 比例项 + 积分项

    // 限制PID输出，防止超出范围
    if (handle->PID_Out > handle->outMax) {
        handle->PID_Out = handle->outMax; // 如果PID输出大于最大值，设置为最大值
    } else if (handle->PID_Out < handle->outMin) {
        handle->PID_Out = handle->outMin; // 如果PID输出小于最小值，设置为最小值
    }

    return handle->PID_Out; // 返回计算得到的PID控制输出值
}


/**
 * 带微分项的PID控制器函数，根据给定值、反馈值和陀螺仪数据计算PID控制输出。
 *
 * @param handle 指向PID控制器结构体的指针，包含PID控制器的所有参数和状态。
 * @param Given 目标值（给定值），即PID控制器需要达到的目标值。
 * @param Feedback 反馈值，即从系统中测量到的实际值。
 * @param Gyro 陀螺仪数据，用于计算微分项。
 *
 * @return 计算得到的PID控制输出值。
 */
float PID_Adjust_T(PID_Structure *handle, float Given, float Feedback, float Gyro) {

    float Error_value; // 误差值
    float P_Out; // 比例项输出
    float D_Out; // 微分项输出

    // 计算误差值：目标值减去反馈值
    Error_value = Given - Feedback;
    // 计算比例项输出
    P_Out = Error_value * handle->Kp; // 比例项 = 误差值 * 比例增益 (Kp)
    // 累加积分项输出
    handle->Ki_Out += Error_value * handle->Ki; // 积分项 = 积分项 + 误差值 * 积分增益 (Ki)
    // 计算微分项输出
    D_Out = Gyro * handle->Kd; // 微分项 = 陀螺仪数据 * 微分增益 (Kd)


//   ---------------------------------------------------------------------------------- 限制输出
    // 限制比例项输出，防止超出范围
    if (P_Out < handle->Kp_Min) P_Out = handle->Kp_Min; // 如果比例项小于最小值，设置为最小值
    else if (P_Out > handle->Kp_Max) P_Out = handle->Kp_Max; // 如果比例项大于最大值，设置为最大值
    // 限制积分项输出，防止积分饱和
    if (handle->Ki_Out < handle->Ki_Min) handle->Ki_Out = handle->Ki_Min; // 如果积分项小于最小值，设置为最小值
    else if (handle->Ki_Out > handle->Ki_Max) handle->Ki_Out = handle->Ki_Max; // 如果积分项大于最大值，设置为最大值
    // 限制微分项输出，防止超出范围
    if (D_Out < handle->Kd_Min) D_Out = handle->Kd_Min; // 如果微分项小于最小值，设置为最小值
    else if (D_Out > handle->Kd_Max) D_Out = handle->Kd_Max; // 如果微分项大于最大值，设置为最大值
//   ---------------------------------------------------------------------------------- 限制输出

    // 计算总PID输出
    handle->PID_Out = P_Out + handle->Ki_Out + D_Out; // PID输出 = 比例项 + 积分项 + 微分项

//   ---------------------------------------------------------------------------------- 限制输出
    // 限制PID输出，防止超出范围
    if (handle->PID_Out > handle->outMax) {
        handle->PID_Out = handle->outMax; // 如果PID输出大于最大值，设置为最大值
    } else if (handle->PID_Out < handle->outMin) {
        handle->PID_Out = handle->outMin; // 如果PID输出小于最小值，设置为最小值
    }
//   ---------------------------------------------------------------------------------- 限制输出

    return handle->PID_Out; // 返回计算得到的PID控制输出值
}