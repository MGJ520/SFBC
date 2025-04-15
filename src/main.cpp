
#include "SimpleFOC.h"
#include <Arduino.h>
#include "BatteryAndButton.h"
#include "./Pre_System/Pre_System.h"
#include "BuzzerSound.h"
#include "APP.h"
#include "SimpleKalmanFilter.h"
#include "MPU6050.h"

MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C); // 磁传感器A
MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C); // 磁传感器B


TwoWire I2C_A = TwoWire(1); // I2C总线A，编号为1
TwoWire I2C_B = TwoWire(0); // I2C总线B，编号为0

MPU6050 mpu6050(I2C_B);

BLDCMotor motor_A = BLDCMotor(7); // 电机A
BLDCMotor motor_B = BLDCMotor(7); // 电机B


BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 13, 14); // 电机A的驱动器
BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10); // 电机B的驱动器


// 用于检测电机电流，参数分别为：电阻值、增益、电流检测引脚、电压检测引脚、备用引脚
LowsideCurrentSense cs_A = LowsideCurrentSense(0.005f, 50.0f, 2, 3, _NC); // 电机A的电流检测
LowsideCurrentSense cs_B = LowsideCurrentSense(0.005f, 50.0f, 4, 5, _NC); // 电机B的电流检测


//速度PID角度
PIDController pid_speed = PIDController(0.25, 0.00001, 0, 100000, 2.5);

//速度PID
PIDController pid_angle = PIDController(0.65, 0, 0.0001, 100000, 4);


LowPassFilter lpf_speed = LowPassFilter(0.025);


LowPassFilter lpf_mpu = LowPassFilter(0.01); //初始值0.5

// SimpleKalmanFilter(e_mea, e_est, q);
// e_mea: 测量不确定性
// e_est: 估计不确定性
// q: 过程噪声
SimpleKalmanFilter KalmanFilter_mpu(1.2, 1.2, 0.3);


TaskHandle_t Task1;
TaskHandle_t Task2;


float Offset_parameters = 3.5f; //偏置参数


Commander command = Commander(Serial);

void onTarget(char *cmd) {
    command.scalar(&Offset_parameters, cmd);
}

void Task1code(void *pvParameters) {
    Serial.println("A侧电机初始化...");
    motor_A.useMonitoring(Serial);

    driver_A.voltage_power_supply = 8.0;
    driver_A.init();
    sensor_A.init(&I2C_A);
    motor_A.linkSensor(&sensor_A);
    motor_A.linkDriver(&driver_A);
    cs_A.linkDriver(&driver_A);

    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_A.torque_controller = TorqueControlType::foc_current;
    motor_A.controller = MotionControlType::torque;


    motor_A.PID_current_d.P = 0.3;   // 降低P减少震荡
    motor_A.PID_current_d.I = 0.1;   // 增加I加速误差消除
    motor_A.PID_current_d.D = 0.0;   // 暂时禁用D项

    motor_A.PID_current_q.P = 0.3;   // 降低P减少震荡
    motor_A.PID_current_q.I = 0.1;   // 增加I加速误差消除
    motor_A.PID_current_q.D = 0.0;   // 暂时禁用D项

    motor_A.LPF_velocity.Tf = 0.09;   //速度滤波
    motor_A.LPF_current_d.Tf = 0.02;
    motor_A.LPF_current_q.Tf = 0.02;
    motor_A.LPF_angle.Tf = 0.05;


    motor_A.velocity_limit = 100;
    motor_A.voltage_limit = 0.4f;
    motor_A.current_limit = 3.5f;

    motor_A.KV_rating = 700;
    motor_A.sensor_direction = CW;
    motor_A.voltage_sensor_align = 0.6f;
    motor_A.velocity_index_search = 2.0f;

    motor_A.init();
    cs_A.init();

    cs_A.gain_a *= -1;
    cs_A.gain_b *= -1;
    cs_A.gain_c *= -1;

    motor_A.linkCurrentSense(&cs_A);
    motor_A.initFOC();
    vTaskDelete(Task1);
}


void Task2code(void *pvParameters) {
    Serial.println("B侧电机初始化...");
    motor_B.useMonitoring(Serial);

    driver_B.voltage_power_supply = 8.0;
    driver_B.init();
    sensor_B.init(&I2C_B);
    motor_B.linkSensor(&sensor_B);
    motor_B.linkDriver(&driver_B);
    cs_B.linkDriver(&driver_B);

    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.torque_controller = TorqueControlType::foc_current;
    motor_B.controller = MotionControlType::torque;


    motor_B.PID_current_d.P = 0.3;   // 降低P减少震荡
    motor_B.PID_current_d.I = 0.1;   // 增加I加速误差消除
    motor_B.PID_current_d.D = 0.0;   // 暂时禁用D项

    motor_B.PID_current_q.P = 0.3;   // 降低P减少震荡
    motor_B.PID_current_q.I = 0.1;   // 增加I加速误差消除
    motor_B.PID_current_q.D = 0.0;   // 暂时禁用D项

    motor_B.LPF_velocity.Tf = 0.09;  //速度滤波
    motor_B.LPF_current_d.Tf = 0.03;
    motor_B.LPF_current_q.Tf = 0.03;
    motor_B.LPF_angle.Tf = 0.05;

    motor_B.velocity_limit = 100;
    motor_B.voltage_limit = 0.4f;
    motor_B.current_limit = 3.0f;

    motor_B.KV_rating = 700;
    motor_B.sensor_direction = CW;
    motor_B.voltage_sensor_align = 0.6f;
    motor_B.velocity_index_search = 2.0f;

    motor_B.init();
    cs_B.init();

    cs_B.gain_a *= -1;
    cs_B.gain_b *= -1;
    cs_B.gain_c *= -1;

    motor_B.linkCurrentSense(&cs_B);
    motor_B.initFOC();
    buzzer.play(S_SIREN);
    mpu6050.calcGyroOffsets();
    vTaskDelete(Task2);
}


void setup() {
    //打开电源
    SetupCarSystem();
    //按键任务
    PowerAndButton.startTask();

    //I2C
    I2C_A.begin(37, 36, 400000UL);
    I2C_B.begin(8, 9, 400000UL);

    //MPU初始化
    mpu6050.begin();

    //创建任务
    xTaskCreatePinnedToCore(Task1code,
                            "Task1",
                            10000,
                            NULL,
                            3,
                            &Task1,
                            0);
    xTaskCreatePinnedToCore(Task2code,
                            "Task2",
                            10000,
                            NULL,
                            2,
                            &Task2,
                            1);


    command.add('T', onTarget, "target velocity");

}

// 定义速度限制参数
const float MAX_ALLOWED_SPEED = 150.0f;

int speed_count=5;


//传感器参数
float Now_Speed=0;
float Now_Pitch=0;


//Pid输出
float PID_Speed_Out=0;
float PID_Pitch_Out=0;

//最终输出力矩
float Target_torque_A = 0;
float Target_torque_B = 0;


void loop() {

    //判断初始化是否成功
    while (motor_A.motor_status == motor_ready && motor_B.motor_status == motor_ready) {
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
        Now_Pitch = mpu6050.getAngleX();// tockn的getangle，通过一阶置信计算

        //卡尔曼滤波
        Now_Pitch =KalmanFilter_mpu.updateEstimate(Now_Pitch);


        //小车当前速度
        Now_Speed = lpf_speed((motor_A.shaft_velocity + motor_B.shaft_velocity*(-1)) / 2 );




        //==============================PID================================
        //PID直立环
        PID_Pitch_Out = pid_angle(Now_Pitch + Offset_parameters)*(-1);

        //PID速度环
        if(speed_count>=2)
        {
            speed_count=0;
            PID_Speed_Out = pid_speed(Now_Speed);
            Serial.println(PID_Speed_Out);
        }

        //PID直立环+速度环
        Target_torque_A = (PID_Pitch_Out + PID_Speed_Out);
        Target_torque_B = (PID_Pitch_Out + PID_Speed_Out)*(-1);




        //============================保护=================================
        // 渐进式扭矩衰减
        if (Now_Speed > MAX_ALLOWED_SPEED) {
            balanceCarPowerOff();
        }
        // 如果速度超过限制，则降低力矩
        if (abs(motor_A.shaft_velocity) > 60) {
            Target_torque_A = 0;
            Target_torque_B = 0;
        }

        //暂时显示A
        //motor_A.monitor();
        //motor_B.monitor();
        command.run();
    }


}

