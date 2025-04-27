#ifndef FOC_CODE_CONTROL_H
#define FOC_CODE_CONTROL_H

#include "include/precompiled.h"

#define MAX_C_SPEED  0.5
#define MAX_C_STEER  0.5

extern enum Car {
    Disable_Output,//关闭输出
    Open_Output,//打开输出
    Error_Unknown,//错误
    Error_Init,
    Error_Power,
    Error_Mpu,
    Error_Encoder,
} System_Status;


// 低通滤波器
extern LowPassFilter lpf_speed;
extern LowPassFilter lpf_control_steer;
extern LowPassFilter lpf_control_speed;

// 偏置参数
extern float Offset_parameters;

extern const float MAX_ALLOWED_SPEED;
extern int speed_count;
extern float Now_Speed;
extern float Now_Pitch;
extern float Target_Speed;
extern float Target_Steer;
extern float Target_torque_A;
extern float Target_torque_B;
extern bool Acc_Protect;

void AbnormalSpinDetect();
void LandingDetect();

void Control_Loop();

bool Control_interface(float steer, float speed, float dead_zone);


#endif //FOC_CODE_CONTROL_H



//void task_control(void *pvParameters)
//{
//    while (1)
//    {
//        angle_pitch = mpu.getAngleY();
//        // Serial.printf("d:%f\n",angle_pitch);
//        acc_Z = mpu.getAccZ();
//        if (running_state == STATE_STOP)
//        {
//            if (abs(angle_pitch - angle_pitch_offset) < 0.5)
//            {
//                running_state = STATE_RUNNING;
//            }
//        }
//        else if (running_state == STATE_RUNNING)
//        {
//            speed_average = (speed_right + speed_left) / 2;
//            if (abs(angle_pitch - angle_pitch_offset) > 50 )
//            {
//                speed_I_sum=0;//清空积分系数
//                running_state = STATE_PICKUP;
//            }
//            else
//            {
//                speed_target = translate_speed;
//
//                speed_error = speed_target - speed_average;
//
//                speed_I_sum+=(speed_error*speed_I);
//                if(speed_I_sum>15){
//                    speed_I_sum = 15;
//                }
//                if(speed_I_sum<-15){
//                    speed_I_sum = -15;
//                }
//                angle_target_pitch = speed_P * speed_error + speed_I_sum;
//
//                motor_output_left = angle_P * ((angle_pitch - angle_pitch_offset) - angle_target_pitch) + angle_D * mpu.getGyroY()+rotate_speed; //+ angle_D * mpu.getGyroY()
//                motor_output_right = angle_P * ((angle_pitch - angle_pitch_offset) - angle_target_pitch) + angle_D * mpu.getGyroY()-rotate_speed;
//                angle_last_pitch = angle_pitch;
//            }
//        }
//        else if (running_state == STATE_PICKUP)
//        {
//            motor_output_left = 0;
//            motor_output_right = 0;
//            if (speed_right > 3.14 && speed_left < -3.14)
//            {
//                running_state = STATE_STOP;
//            }
//        }
//        vTaskDelay(1);
//    }
//}
