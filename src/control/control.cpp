#include "control.h"

#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "pid/pid_adjust.h"


//// e_mea: 测量不确定性   e_est: 估计不确定性 q: 过程噪声
SimpleKalmanFilter KalmanFilter_mpu(0.2, 0.2, 0.20);

//速度滤波
LowPassFilter lpf_speed = LowPassFilter(0.02);
LowPassFilter lpf_speed_error = LowPassFilter(0.02);
LowPassFilter lpf_ag = LowPassFilter(0.02);



LowPassFilter lpf_out01 = LowPassFilter(0.05);


//速度滤波
LowPassFilter lpf_trun = LowPassFilter(0.02);

LowPassFilter lpf_run = LowPassFilter(0.02);


//偏置参数
float Offset_parameters = 2.45f;


const float MAX_ALLOWED_SPEED = 100.0f;

int speed_count = 5;

float Now_Speed_Erorr = 0;

float Now_Speed = 0;
float Now_Pitch = 0;

float C_Speed = 0;
float C_Turn = 0;

float PID_Speed_Out = 0;
float PID_Pitch_Out = 0;

float Target_torque_A = 0;
float Target_torque_B = 0;

Car System_Status = Open_Output;


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

    if (!xboxController.isConnected()) {
        if (mpu6050.absAccZ > 2.0f && System_Status == Open_Output &&
            abs(Now_Speed) < 1 && abs(Now_Pitch) < 5) {
            Serial.println("[系统-动作]:小车提起(加速度),关闭输出");
            MotorClose();
        }
    }

    // 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
    if (System_Status == Open_Output) {
        if ((abs(Now_Speed) > 50 && abs(Now_Pitch) < 10)) {
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
    motor_B.move(Target_torque_B*-1);

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
    //PID速度环
    if (++speed_count >=2) {
        speed_count = 0;
        // 小车速度环
        PID_Adjust(&SpeedPID, 0.0f, Now_Speed-C_Speed*60);

        // 小车旋转环
        PID_Adjust(&TurnPID,-C_Turn*55, Now_Speed_Erorr);

        //计算加速度
        mpu6050.calculateAbsoluteAcceleration();
    }

    PID_Adjust_T(&UprightPID, 0.0f, (Now_Pitch + Offset_parameters), lpf_ag(mpu6050.getGyroXFV())+Offset_parameters);

    Target_torque_A = UprightPID.PID_Out + SpeedPID.PID_Out + TurnPID.PID_Out;  // 输出电机速度控制量
    Target_torque_B = UprightPID.PID_Out + SpeedPID.PID_Out - TurnPID.PID_Out;  // 输出电机速度控制量


    //============================保护=================================
    AbnormalSpinDetect();

    LandingDetect();


}