

#include "foc_drive.h"
#include "task/freertos_task.h"
#include "control/control.h"
#include "nvs/nvs.h"
#include "power/BatteryAndButton.h"
#include "led/led.h"
#include "Init/init_System.h"
#include "I2C/I2C_Manage.h"
#include "mpu/MPU6050.h"

BLDCMotor motor_A = BLDCMotor(7); // 电机A
BLDCMotor motor_B = BLDCMotor(7); // 电机B


MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C);


#if   MPU_I2C==1
MPU6050 mpu6050(I2C_A);
#elif MPU_I2C==2
MPU6050 mpu6050(I2C_B);
#endif


BLDCDriver3PWM driver_A = BLDCDriver3PWM(PWM_A_1_GPIO, PWM_A_2_GPIO, PWM_A_3_GPIO); // 电机A的驱动器
BLDCDriver3PWM driver_B = BLDCDriver3PWM(PWM_B_1_GPIO, PWM_B_2_GPIO, PWM_B_3_GPIO); // 电机B的驱动器


// 用于检测电机电流，参数分别为：电阻值、增益、电流检测引脚、电压检测引脚、备用引脚
LowsideCurrentSense cs_A = LowsideCurrentSense(0.005f, 50.0f, ADC_A_1_GPIO, ADC_A_2_GPIO, _NC); // 电机A的电流检测
LowsideCurrentSense cs_B = LowsideCurrentSense(0.005f, 50.0f, ADC_B_1_GPIO, ADC_B_2_GPIO, _NC); // 电机B的电流检测


Commander command = Commander(Serial);


void onTarget(char *cmd) {
    command.scalar(&Offset_parameters, cmd);
}


//void on_SP(char *cmd) {
//    command.scalar(&pid_speed.P, cmd);
//}
//void on_SI(char *cmd) {
//    command.scalar(&pid_speed.I, cmd);
//}
//void on_SD(char *cmd) {
//    command.scalar(&pid_speed.D, cmd);
//}
//
//void on_VP(char *cmd) {
//    command.scalar(&pid_angle.P, cmd);
//}
//void on_VI(char *cmd) {
//    command.scalar(&pid_angle.I, cmd);
//}
//void on_VD(char *cmd) {
//    command.scalar(&pid_angle.D, cmd);
//}


void MotorOpen() {
    motor_A.enable();
    motor_B.enable();
    System_Status = Open_Output;
#ifdef LED_1_GPIO
    G_LED.doubleBlink();
    R_LED.off();
#endif
}


void MotorClose() {
    motor_A.disable();
    motor_B.disable();
    System_Status = Disable_Output;
#ifdef LED_1_GPIO
    G_LED.off();
    R_LED.blink();
#endif
}



void Foc_Parameters_init() {
    //==============================================================================
    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_A.torque_controller = TorqueControlType::foc_current;
    motor_A.controller = MotionControlType::torque;

    motor_A.PID_current_d.P = PID_CS_P;   // 降低P减少震荡
    motor_A.PID_current_d.I = PID_CS_I;   // 增加I加速误差消除
    motor_A.PID_current_d.D = PID_CS_D;   // 暂时禁用D项

    motor_A.PID_current_q.P = PID_CS_P;   // 降低P减少震荡
    motor_A.PID_current_q.I = PID_CS_I;   // 增加I加速误差消除
    motor_A.PID_current_q.D = PID_CS_D;   // 暂时禁用D项

    motor_A.LPF_velocity.Tf = LPF_velocity_Tf;   //速度滤波
    motor_A.LPF_current_d.Tf = LPF_current_d_Tf;
    motor_A.LPF_current_q.Tf = LPF_current_q_Tf;
    motor_A.LPF_angle.Tf = LPF_angle_Tf;


    motor_A.voltage_limit = 0.6f;
    motor_A.current_limit = 6.5f;

    motor_A.KV_rating = 700;

   // motor_A.phase_resistance = 0.1f;

//    motor_A.sensor_direction = CW;

    motor_A.velocity_index_search = 2.0f;
    driver_A.voltage_power_supply = 8.0;

    cs_A.gain_a *= -1;
    cs_A.gain_b *= -1;
    cs_A.gain_c *= -1;


    //==============================================================================
    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.torque_controller = TorqueControlType::foc_current;
    motor_B.controller = MotionControlType::torque;


    motor_B.PID_current_d.P = PID_CS_P;   // 降低P减少震荡
    motor_B.PID_current_d.I = PID_CS_I;   // 增加I加速误差消除
    motor_B.PID_current_d.D = PID_CS_D;   // 暂时禁用D项

    motor_B.PID_current_q.P = PID_CS_P;   // 降低P减少震荡
    motor_B.PID_current_q.I = PID_CS_I;   // 增加I加速误差消除
    motor_B.PID_current_q.D = PID_CS_D;   // 暂时禁用D项

    motor_B.LPF_velocity.Tf = LPF_velocity_Tf;  //速度滤波
    motor_B.LPF_current_d.Tf = LPF_current_d_Tf;
    motor_B.LPF_current_q.Tf = LPF_current_q_Tf;
    motor_B.LPF_angle.Tf = LPF_angle_Tf;


    motor_B.voltage_limit = 0.6f;
    motor_B.current_limit = 6.5f;

    motor_B.KV_rating = 700;
    //motor_B.phase_resistance = 0.1f;


//    motor_B.sensor_direction = CW;

    motor_B.velocity_index_search = 2.0f;
    driver_B.voltage_power_supply = 8.0;

    cs_B.gain_a *= -1;
    cs_B.gain_b *= -1;
    cs_B.gain_c *= -1;


}


void Foc_A_Initialize(void *pvParameters) {
#if MORE_FOC_INIT_INFO
    motor_A.useMonitoring(Serial);
#endif
    if (!driver_A.init()) {
        Serial.println("[驱动器A]:初始化失败");
    } else {
        Serial.println("[驱动器A]:初始化成功");
    }
    sensor_A.init(&I2C_A);
    motor_A.linkSensor(&sensor_A);
    motor_A.linkDriver(&driver_A);
    cs_A.linkDriver(&driver_A);
    Serial.println("[电流传感器A]:正在校准电流采样...");
    motor_A.init();
    if (!cs_A.init()) {
        Serial.println("[电流传感器A]:初始化失败");
    } else {
        Serial.println("[电流传感器A]:初始化成功");
    }
    motor_A.linkCurrentSense(&cs_A);
    Serial.println("[电机A]:正在对齐编码器,驱动相位...");
    if (!motor_A.initFOC()) {
        Serial.println("[电机A]:初始化失败");
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        systemInitialized = false; // 将系统初始化标志设置为false
        portEXIT_CRITICAL(&mux);
    } else {
        Serial.println("[电机A]:初始化成功");
    }
    vTaskDelete(Task1);
}

void Foc_B_Initialize(void *pvParameters) {
#if MORE_FOC_INIT_INFO
    motor_B.useMonitoring(Serial);
#endif
    if (!driver_B.init()) {
        Serial.println("[驱动器B]:初始化失败");
    } else {
        Serial.println("[驱动器B]:初始化成功");
    }
    sensor_B.init(&I2C_B);
    motor_B.linkSensor(&sensor_B);
    motor_B.linkDriver(&driver_B);
    cs_B.linkDriver(&driver_B);
    Serial.println("[电流传感器B]:正在校准电流采样...");
    motor_B.init();
    if (!cs_B.init()) {
        Serial.println("[电流传感器B]:初始化失败");
    } else {
        Serial.println("[电流传感器B]:初始化成功");
    }
    motor_B.linkCurrentSense(&cs_B);
    Serial.println("[电机B]:正在对齐编码器,驱动相位...");
    if (!motor_B.initFOC()) {
        Serial.println("[电机B]:初始化失败");
        portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
        portENTER_CRITICAL(&mux);
        systemInitialized = false; // 将系统初始化标志设置为false
        portEXIT_CRITICAL(&mux);
    } else {
        Serial.println("[电机B]:初始化成功");
    }
    command.add('T', onTarget, "T");
//    command.add('A', on_SP, "A");
//    command.add('B', on_SI, "B");
//    command.add('C', on_SD, "C");
//    command.add('D', on_VP, "D");
//    command.add('E', on_VI, "E");
//    command.add('F', on_VD, "F");
//    A 0.15
//    B 0.00001
//    C 0
//    D 0.25
//    E 0.0001
//    F 0.0001
//    T 1.3
    vTaskDelete(Task2);
}


bool check_encoder()
{
    TwoWire *wire_a = &I2C_A;
    TwoWire *wire_b = &I2C_B;
    uint8_t chipID;

    // 检查第一个编码器（AS5600_A）
    wire_a->beginTransmission(AS5600_ADDR);
    wire_a->write(CHIP_ID_REG);
    if (wire_a->endTransmission() != 0) { // I2C通信失败
        Serial.println("[AS5600_A]: 连接失败（通信错误）!");
        //没有等到电机变化，会有中断错误，默认跳过
        return false;
    }
    if (wire_a->requestFrom(AS5600_ADDR, 1) != 1) { // 请求数据失败
        Serial.println("[AS5600_A]: 数据读取失败!");
        return false;
    }
    Serial.println("[AS5600_A]: 连接成功!");

    // 检查第二个编码器（AS5600_B）
    wire_b->beginTransmission(AS5600_ADDR);
    wire_b->write(CHIP_ID_REG);
    if (wire_b->endTransmission() != 0) {
        Serial.println("[AS5600_B]: 连接失败（通信错误）!"); // 修正标签错误
        //没有等到电机变化，会有中断错误，默认跳过
        return false;
    }
    if (wire_b->requestFrom(AS5600_ADDR, 1) != 1) {
        Serial.println("[AS5600_B]: 数据读取失败!");
        return false;
    }
    Serial.println("[AS5600_B]: 连接成功!");

    return true; // 两个设备均通过校验
}
