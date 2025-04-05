#ifndef Upright_h
#define Upright_h

// 定义一个结构体，用于存储垂直平衡PID控制器的参数和输出
typedef struct upRightPID
{
    volatile float Kp;       // 比例系数，用于调整控制量与误差的线性关系
    volatile float Ki;       // 积分系数，用于消除稳态误差
    volatile float Kd;       // 微分系数，用于抑制快速变化的误差
    volatile float Limit = 60.0f;  // PID输出的最大限制值，默认为60.0

    volatile float Ki_Out = 0.0f;  // 积分项的输出值
    volatile float Kp_Min = -Limit;  // 比例项输出的最小值限制
    volatile float Kp_Max = Limit;   // 比例项输出的最大值限制

    volatile float Ki_Min = -Limit;  // 积分项输出的最小值限制
    volatile float Ki_Max = Limit;   // 积分项输出的最大值限制

    volatile float Kd_Min = -Limit;  // 微分项输出的最小值限制
    volatile float Kd_Max = Limit;   // 微分项输出的最大值限制

    volatile float outMin = -Limit;  // PID输出的最小值限制
    volatile float outMax = Limit;   // PID输出的最大值限制

    volatile float PID_Out = 0.0f;   // PID控制器的最终输出

} UprightPID_t;

// 初始化垂直平衡PID控制器的参数
void UprightPID_Init(UprightPID_t *pidConf, float Kp, float Ki, float Kd, float limit);

// 计算垂直平衡PID控制器的输出
// pidConf: PID控制器的配置结构体
// targetAngleX: 目标角度（期望值）
// angleX: 当前角度（实际值）
// GyroX: 陀螺仪的X轴角速度（用于微分项的计算）
float UprightPID(UprightPID_t *pidConf, float targetAngleX, float angleX, float GyroX);

#endif