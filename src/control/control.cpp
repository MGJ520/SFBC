#include "control.h"

#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "pid/pid_adjust.h"


//卡尔曼滤波        e_mea: 测量不确定性   e_est: 估计不确定性 q: 过程噪声
SimpleKalmanFilter KalmanFilter_mpu(0.2, 0.2, 0.20);

//滤波
LowPassFilter lpf_speed          = LowPassFilter(0.02);
LowPassFilter lpf_speed_error    = LowPassFilter(0.02);
LowPassFilter lpf_gyro_x         = LowPassFilter(0.02);
LowPassFilter lpf_control_steer  = LowPassFilter(0.02);
LowPassFilter lpf_control_speed  = LowPassFilter(0.02);

//速度限制
const float MAX_ALLOWED_SPEED = 100.0f;


//当前实际参数
float Now_Speed = 0;
float Now_Pitch = 0;
float Now_Speed_Erorr = 0;
float Offset_parameters = Mechanical_Zero_Point;

//目标速度
float Target_Speed = 0;
float Target_Steer = 0;

//控制输出
float Target_torque_A = 0;
float Target_torque_B = 0;

//速度环计数
int speed_count = 5;

//系统状态
Car System_Status = Open_Output;

//加速度提取保护开关
bool Acc_Protect= true;



void AbnormalSpinDetect() {
    static unsigned short count = 0;

    if (abs(Now_Speed) > MAX_ALLOWED_SPEED && System_Status == Open_Output) {
        Serial.println("[系统-动作]:速度过大,关闭输出");
        MotorClose();
    }


    if (abs(Now_Pitch) > 70 && System_Status == Open_Output) {
        Serial.println("[系统-动作]:小车倾倒,关闭输出");
        MotorClose();
    }

    if (Acc_Protect) {
        if (mpu6050.absAccZ > 1.55f && System_Status == Open_Output &&
            abs(Now_Speed) < 1 && abs(Now_Pitch) < 5) {
            Serial.println("[系统-动作]:小车提起(加速度),关闭输出");
            MotorClose();
        }
    }

    // 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
    if (System_Status == Open_Output) {
        if ((abs(Now_Speed) > 60 && abs(Now_Pitch) < 15)) {
            if (++count > 20) {
                count = 0;
                MotorClose();
                Serial.println("[系统-动作]:小车提起(速度),关闭输出");
            }
        } else {
            count = 0;
        }
    } else {
        count = 0;
    }

}

void LandingDetect() {
    static float lastCarAngle = 0;
    static unsigned short count = 0, count1 = 0;
    if (System_Status == Disable_Output) {
        // 小车角度5°~-5°启动检测
        if (abs(Now_Pitch) <= 5) {
            count1++;
            if (count1 >= 70) {//每隔250ms判断一次小车角度变化量，变化量小于0.8°或大于-0.8°判断为小车静止
                count1 = 0;
                if (abs((Now_Pitch - lastCarAngle) < 0.8)) {
                    count++;
                    if (count >= 4) {
                        count = 0;
                        count1 = 0;
                        MotorOpen();
                        Serial.println("[系统-动作]:小车着地,恢复动力");
                    }
                } else {
                    count = 0;
                }
                lastCarAngle = Now_Pitch;
            }
        } else {
            count1 = 0;
            count = 0;
        }
    }
}

void Control_Loop() {
    //如果foc没有初始化成功，不控制
    if (motor_A.motor_status != motor_ready || motor_B.motor_status != motor_ready) {
        return;
    }

    //===============================FOC控制==============================
    motor_A.loopFOC();
    motor_B.loopFOC();

    motor_A.move(Target_torque_A);
    motor_B.move(Target_torque_B*(-1));

    //===============================传感器==============================

    // 读取并更新MPU6050传感器数据
    mpu6050.update();

    //当前俯仰角
    Now_Pitch = mpu6050.getAngleX();

    //卡尔曼滤波
    Now_Pitch = KalmanFilter_mpu.updateEstimate(Now_Pitch);

    //小车当前速度
    Now_Speed = lpf_speed(-(motor_A.shaft_velocity + motor_B.shaft_velocity * (-1)) / 2);

    Now_Speed_Erorr=lpf_speed_error(motor_A.shaft_velocity + motor_B.shaft_velocity);



    //===============================PID==============================
    //PID速度环
    if (++speed_count >=2) {
        speed_count = 0;

        // 小车速度环
        PID_Adjust(&SpeedPID, 0.0f, (Now_Speed - Target_Speed * 60));

        // 小车旋转环
        PID_Adjust(&TurnPID, (Target_Steer * -55), Now_Speed_Erorr);

        //计算加速度
        mpu6050.calculateAbsoluteAcceleration();
    }

    //速度pid 控制设置
    if (Target_Speed != 0) { // 如果油门不为0
        SpeedPID.Ki_Out = 0; // 重置PID控制器的积分项
    }

    //旋转环pid 控制设置
    if ( Target_Steer!= 0) {
        TurnPID.Ki_Out = 0; // 重置PID控制器的积分项
    }


    //小车直立环
    PID_Adjust_T(&UprightPID, 0.0f, (Now_Pitch + Offset_parameters), lpf_gyro_x(mpu6050.getGyroXFV()));


    // 输出电机速度控制量
    Target_torque_A = UprightPID.PID_Out + SpeedPID.PID_Out + TurnPID.PID_Out;
    Target_torque_B = UprightPID.PID_Out + SpeedPID.PID_Out - TurnPID.PID_Out;




    //============================保护=================================
    //小车离地检查
    AbnormalSpinDetect();

    //小车着地检查
    LandingDetect();


}

//控制接口范围是-0.5~0.5  dead_zone是死区,可以默认0.1
bool Control_interface(float steer, float speed, float dead_zone)
{
    // 限制速度范围-0.5~0.5
    if (speed < -MAX_C_SPEED) {
        speed = -MAX_C_SPEED;
    }
    if (speed > MAX_C_SPEED) {
        speed = MAX_C_SPEED;
    }

    // 限制转向范围-0.5~0.5
    if (steer < -MAX_C_STEER) {
        steer = -MAX_C_STEER;
    }
    if (steer > MAX_C_STEER) {
        steer = MAX_C_STEER;
    }

    Target_Steer=steer;
    Target_Speed=speed;

    //添加死区
    if (fabs(Target_Speed) <= dead_zone) {
        Target_Speed = 0.0; // 如果在死区内，设置为零
    }
    if (fabs(Target_Steer) <= dead_zone) {
        Target_Steer = 0.0; // 如果在死区内，设置为零
    }

    //滤波
    Target_Steer = lpf_control_steer(Target_Steer); // 否则，继续进行滤波处理
    Target_Speed = lpf_control_speed(Target_Speed); // 否则，继续进行滤波处理

    return true;
}