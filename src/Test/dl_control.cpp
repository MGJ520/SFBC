
#include <SimpleFOC.h>
#include <Arduino.h>
#include "BatteryAndButton.h"
#include "./Pre_System/Pre_System.h"
#include "MPU6050.h"
#include "BuzzerSound.h"
#include "APP.h"

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

PIDController pid_stb_A = PIDController(0.6, 0.5, 0.001, 100000, 4);
PIDController pid_vel_A = PIDController(0.6, 0.5, 0.001, 100000, _PI / 4);
LowPassFilter lpf_pitch_cmd_A = LowPassFilter(0.025);
LowPassFilter lpf_mpu = LowPassFilter(0.01); //初始值0.5


TaskHandle_t Task1;
TaskHandle_t Task2;

float target_torque_A = 0;
float target_torque_B = 0;

float Offset_parameters = -0.65; //偏置参数



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

    motor_A.PID_velocity.P = 0.3;   // 降低P减少震荡
    motor_A.PID_velocity.I = 0.2;   // 增加I加速误差消除
    motor_A.PID_velocity.D = 0.0;   // 暂时禁用D项

    motor_A.PID_current_d.P = 0.3;   // 降低P减少震荡
    motor_A.PID_current_d.I = 0.1;   // 增加I加速误差消除
    motor_A.PID_current_d.D = 0.0;   // 暂时禁用D项

    motor_A.PID_current_q.P = 0.3;   // 降低P减少震荡
    motor_A.PID_current_q.I = 0.1;   // 增加I加速误差消除
    motor_A.PID_current_q.D = 0.0;   // 暂时禁用D项

    motor_A.LPF_velocity.Tf = 0.09; // 增强速度滤波
    motor_A.LPF_current_d.Tf = 0.02; // 增强速度滤波
    motor_A.LPF_current_q.Tf = 0.02; // 增强速度滤波
    motor_A.LPF_angle.Tf = 0.05; // 增强速度滤波

    motor_A.PID_velocity.output_ramp = 1000;

    motor_A.velocity_limit = 100;
    motor_A.voltage_limit = 0.4f;
    motor_A.current_limit = 3.0f;

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

    motor_B.PID_velocity.P = 0.3;   // 降低P减少震荡
    motor_B.PID_velocity.I = 0.2;   // 增加I加速误差消除
    motor_B.PID_velocity.D = 0.0;   // 暂时禁用D项

    motor_B.PID_current_d.P = 0.3;   // 降低P减少震荡
    motor_B.PID_current_d.I = 0.1;   // 增加I加速误差消除
    motor_B.PID_current_d.D = 0.0;   // 暂时禁用D项

    motor_B.PID_current_q.P = 0.3;   // 降低P减少震荡
    motor_B.PID_current_q.I = 0.1;   // 增加I加速误差消除
    motor_B.PID_current_q.D = 0.0;   // 暂时禁用D项

    motor_B.LPF_velocity.Tf = 0.09; // 增强速度滤波
    motor_B.LPF_current_d.Tf = 0.03; // 增强速度滤波
    motor_B.LPF_current_q.Tf = 0.03; // 增强速度滤波
    motor_B.LPF_angle.Tf = 0.05; // 增强速度滤波

    motor_B.PID_velocity.output_ramp = 1000;

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

    mpu6050.begin();
    // 计算陀螺仪偏移量
    mpu6050.calcGyroOffsets();


// 创建任务1，并将其固定到核心0
    xTaskCreatePinnedToCore(
            Task1code,   // Task function. 任务函数，即任务的具体执行代码，这里是Task1code函数
            "Task1",     // name of task. 任务的名称，用于调试和日志记录，这里是"Task1"
            10000,       // Stack size of task. 任务的堆栈大小，单位是字节，这里是10000字节
            NULL,        // parameter of the task. 传递给任务函数的参数，这里是NULL，表示没有参数
            3,           // priority of the task. 任务的优先级，数字越大优先级越高，这里是3
            &Task1,      // Task handle to keep track of created task. 任务句柄，用于后续操作任务，这里是Task1
            1);          // pin task to core 0. 将任务固定到特定的核心，这里是核心0

// 创建任务2，并将其固定到核心1
    xTaskCreatePinnedToCore(
            Task2code,   // Task function. 任务函数，即任务的具体执行代码，这里是Task2code函数
            "Task2",     // name of task. 任务的名称，用于调试和日志记录，这里是"Task2"
            10000,       // Stack size of task. 任务的堆栈大小，单位是字节，这里是10000字节
            NULL,        // parameter of the task. 传递给任务函数的参数，这里是NULL，表示没有参数
            2,           // priority of the task. 任务的优先级，数字越大优先级越高，这里是2
            &Task2,      // Task handle to keep track of created task. 任务句柄，用于后续操作任务，这里是Task2
            1);          // pin task to core 1. 将任务固定到特定的核心，这里是核心1


    command.add('T', onTarget, "target velocity");

}

// 定义速度限制参数
const float MAX_ALLOWED_SPEED = 60.0f;
const float TORQUE_RAMP_RATE = 0.10f;  // 10% per cycle


void loop() {

    //判断初始化是否成功
    while (motor_A.motor_status == motor_ready && motor_B.motor_status == motor_ready) {

        motor_A.loopFOC();
        motor_B.loopFOC();

        motor_A.move(target_torque_A);
        motor_B.move(target_torque_B);

        mpu6050.update(); // 读取并更新MPU6050传感器数据

        // 当前俯仰角
        double mpu_pitch = lpf_mpu(mpu6050.getAngleX());// tockn的getangle，通过一阶置信计算

        // 计算目标俯仰角
        float target_pitch_A = lpf_pitch_cmd_A(pid_vel_A((motor_A.shaft_velocity + motor_B.shaft_velocity) / 2));

        target_torque_A = pid_stb_A(Offset_parameters - mpu_pitch + target_pitch_A);

        target_torque_B = (-target_torque_A);


        // 渐进式扭矩衰减
        if (abs(motor_A.shaft_velocity) > MAX_ALLOWED_SPEED) {
            float speed_error = motor_A.shaft_velocity - MAX_ALLOWED_SPEED;
            target_torque_A -= speed_error * TORQUE_RAMP_RATE;
        }

        // 渐进式扭矩衰减
        if (abs(motor_B.shaft_velocity) > MAX_ALLOWED_SPEED) {
            float speed_error = motor_B.shaft_velocity - MAX_ALLOWED_SPEED;
            target_torque_B -= speed_error * TORQUE_RAMP_RATE;
        }

        // 如果速度超过限制，则降低力矩
        if (abs(mpu_pitch) > 60) {
            target_torque_A = 0;
            target_torque_B = 0;
        }

        //暂时显示A
        motor_A.monitor();
        //motor_B.monitor();

        command.run();
    }


}

