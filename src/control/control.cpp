//
// Created by MGJ on 2025/4/23.
//

#include "control.h"
#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"


boolean Ready = true;
const float MAX_ALLOWED_SPEED = 100.0f;
int speed_count = 5;
float Now_Speed = 0;
float Now_Pitch = 0;
float PID_Speed_Out = 0;
float PID_Pitch_Out = 0;
float C_Speed = 0;
float C_Turn = 0;

float Target_torque_A = 0;
float Target_torque_B = 0;

Car System_Status;


void AbnormalSpinDetect() {
    static unsigned short count = 0;

    if (abs(Now_Speed) > MAX_ALLOWED_SPEED && System_Status == Open) {
        Serial.println("[系统-保护]:速度过大,关闭输出");
        MotorClose();
    }


    if (abs(Now_Pitch) > 70 && System_Status == Open) {
        Serial.println("[系统-保护]:小车倾倒,关闭输出");
        MotorClose();
    }

    if (!xboxController.isConnected()) {
        if (mpu6050.absAccZ > 1.35f && System_Status == Open &&
            abs(Now_Speed) < 1 && abs(Now_Pitch) < 5) {
            Serial.println("[系统-保护]:小车提起(加速度),关闭输出");
            MotorClose();
        }
    }

    // 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
    if (System_Status == Open) {
        if ((abs(Now_Speed) > 35 && abs(Now_Pitch) < 25)) {

            if (++count > 20) {
                count = 0;
                MotorClose();
                Serial.println("[系统-保护]:小车提起(速度),关闭输出");
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
    if (System_Status == Dis) {
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
                        Serial.println("[系统]:小车着地,恢复动力");
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

void Control_Loop()
{
    speed_count++;

    //===============================FOC控制==============================
    motor_A.loopFOC();
    motor_B.loopFOC();
    motor_A.move(Target_torque_A);
    motor_B.move(Target_torque_B);

    //===============================传感器==============================

    // 读取并更新MPU6050传感器数据
    mpu6050.update();

    //当前俯仰角
    Now_Pitch = mpu6050.getAngleX();

    //卡尔曼滤波
    Now_Pitch = KalmanFilter_mpu.updateEstimate(Now_Pitch);

    //小车当前速度
    Now_Speed = lpf_speed((motor_A.shaft_velocity + motor_B.shaft_velocity * (-1)) / 2);


    //==============================PID================================

    //PID直立环
    PID_Pitch_Out = pid_angle(Now_Pitch + Offset_parameters) * (-1);

    //PID速度环
    if (speed_count >= 3) {
        speed_count = 0;
        PID_Speed_Out = pid_speed(Now_Speed + C_Speed * 35);
        //计算加速度
        mpu6050.calculateAbsoluteAcceleration();
    }

    //PID直立环+速度环
    Target_torque_A = (PID_Pitch_Out + PID_Speed_Out - C_Turn * 2);
    Target_torque_B = (PID_Pitch_Out + PID_Speed_Out + C_Turn * 2) * (-1);


    //============================保护=================================
    AbnormalSpinDetect();
    LandingDetect();


}