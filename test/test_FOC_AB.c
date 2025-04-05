#include <Arduino.h>
#include "MPU6050.h"
#include "SimpleFOC.h"

#include "FocDriver.h"
#include "APP.h"
#include "BatteryAndButton.h"
#include "UprightPID.h"
#include "UserConfig.h"

// 定义磁传感器实例
// 创建两个磁传感器对象，分别连接到AS5600 I2C总线
MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C); // 磁传感器A
MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C); // 磁传感器B

// 定义两个I2C总线实例
// I2C_A 和 I2C_B 分别用于连接不同的设备
TwoWire I2C_A = TwoWire(1); // I2C总线A，编号为1
TwoWire I2C_B = TwoWire(0); // I2C总线B，编号为0

// 定义MPU6050传感器实例
// MPU6050连接到I2C_B总线
MPU6050 mpu6050(I2C_B);

// 定义BLDC电机和驱动器实例
// 创建两个无刷直流电机对象，每个电机有7极对
BLDCMotor motor_A = BLDCMotor(7); // 电机A
BLDCMotor motor_B = BLDCMotor(7); // 电机B

// 根据使用的硬件平台选择合适的驱动器引脚定义

// ESP32-S3芯片的引脚定义
// BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 34, 33); // 电机A的驱动器，控制引脚为35、34、33
// BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10); // 电机B的驱动器，控制引脚为12、11、10

// ESP32-S3模块的引脚定义
BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 13, 14); // 电机A的驱动器
BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10); // 电机B的驱动器

// 定义低边电流检测实例
// 用于检测电机电流，参数分别为：电阻值、增益、电流检测引脚、电压检测引脚、备用引脚
LowsideCurrentSense cs_A = LowsideCurrentSense(0.005f, 50.0f, 2, 3, _NC); // 电机A的电流检测
LowsideCurrentSense cs_B = LowsideCurrentSense(0.005f, 50.0f, 4, 5, _NC); // 电机B的电流检测
// -----------------------------------------------


// -----------------------------------------------
UprightPID_t upRightPIDConfig;
//PIDController Velocity_PID{.P = VELOCITY_Kp, .I = VELOCITY_Ki, .D = VELOCITY_Kd, .ramp = 1000, .limit = VELOCITY_LIMIT};
PIDController Velocity_PID(VELOCITY_Kp, VELOCITY_Ki, VELOCITY_Kd, 1000, VELOCITY_LIMIT);
//------------------------------------------------


boolean FocDriver::init() {
#if MONITOR_MODE
    motor_A.useMonitoring(Serial);
    motor_B.useMonitoring(Serial);
#endif

    //--------------------------------------------------------------------------------
    //I2c初始化
    if (!I2C_A.begin(37, 36, 400000UL)) {
        Serial.println("[A侧I2C]:初始化失败!");
        return false;
    } else {
        Serial.println("[A侧I2C]:初始化成功");
    }
    if (!I2C_B.begin(8, 9, 400000UL)) {
        Serial.println("[B侧I2C]:初始化失败!");
        return false;
    } else {
        Serial.println("[B侧I2C]:初始化成功");
    }


    //--------------------------------------------------------------------------------
    // 驱动器配置
    // 设置驱动器的电源电压为8.2V
    driver_A.voltage_power_supply = 8.2;
    driver_B.voltage_power_supply = 8.2;
    // 初始化驱动器A和B
    if (driver_A.init() == 0) {
        Serial.println("[A侧驱动器]:初始化失败!");
        return false;
    } else {
        Serial.println("[A侧驱动器]:初始化成功");
    }
    if (driver_B.init() == 0) {
        Serial.println("[B侧驱动器]:初始化失败!");
        return false;
    } else {
        Serial.println("[B侧驱动器]:初始化成功");
    }


    //--------------------------------------------------------------------------------
    // 初始化磁传感器A，将其连接到I2C总线A
    sensor_A.init(&I2C_A);
    // 初始化磁传感器B，将其连接到I2C总线B
    sensor_B.init(&I2C_B);
    // 将电机与传感器关联
    // 将电机A与磁传感器A关联，用于电机的位置和速度反馈
    motor_A.linkSensor(&sensor_A);
    // // 将电机B与磁传感器B关联，用于电机的位置和速度反馈
    motor_B.linkSensor(&sensor_B);
    // 将电机与驱动器关联
    // 将电机A与驱动器A关联，用于控制电机的PWM信号输出
    motor_A.linkDriver(&driver_A);
    // 将电机B与驱动器B关联，用于控制电机的PWM信号输出
    motor_B.linkDriver(&driver_B);


    //--------------------------------------------------------------------------------
    // 将电流检测模块与驱动器关联
    // 将电流检测模块A与驱动器A关联，用于检测电机电流
    cs_A.linkDriver(&driver_A);
    // 将电流检测模块B与驱动器B关联，用于检测电机电流
    cs_B.linkDriver(&driver_B);



    //--------------------------------------------------------------------------------
    // 电机配置
    // 设置电机A和B的FOC调制方式为“空间矢量PWM”（Space Vector PWM）
    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // 设置电机的运动控制回路类型
    // 设置电机A和B的转矩控制器为FOC电流控制模式
    motor_A.torque_controller = TorqueControlType::foc_current;
    motor_B.torque_controller = TorqueControlType::foc_current;

    // 设置电机A和B的运动控制器为速度控制模式
    motor_A.controller = MotionControlType::velocity;
    motor_B.controller = MotionControlType::velocity;

    // 配置电机A的速度PI控制器参数
    motor_A.PID_velocity.P = VEL_Kp;  // 比例增益
    motor_A.PID_velocity.I = VEL_Ki; // 积分增益
    motor_A.PID_velocity.output_ramp = 1000; // 输出斜率限制
    motor_A.voltage_sensor_align = 0.6f; // 电压传感器对齐参数
    // 配置电机B的速度PI控制器参数
    motor_B.PID_velocity.P = VEL_Kp;         // 比例增益
    motor_B.PID_velocity.I = VEL_Ki;         // 积分增益
    motor_B.PID_velocity.output_ramp = 1000; // 输出斜率限制
    motor_B.voltage_sensor_align = 0.6f;     // 电压传感器对齐参数


    // 配置电机A的电流环控制器参数（Q轴）
    motor_A.PID_current_q.P = C_Kp; // 比例增益
    motor_A.PID_current_q.I = C_Ki; // 积分增益
    motor_A.LPF_current_q.Tf = C_LF; // 低通滤波器时间常数
    motor_A.PID_current_q.output_ramp = 1000; // 输出斜率限制
    // 配置电机A的电流环控制器参数（D轴）
    motor_A.PID_current_d.P = C_Kp; // 比例增益
    motor_A.PID_current_d.I = C_Ki; // 积分增益
    motor_A.LPF_current_d.Tf = C_LF; // 低通滤波器时间常数
    motor_A.PID_current_d.output_ramp = 1000; // 输出斜率限制


    // 配置电机B的电流环控制器参数（Q轴）
    motor_B.PID_current_q.P = C_Kp;           // 比例增益
    motor_B.PID_current_q.I = C_Ki;           // 积分增益
    motor_B.LPF_current_q.Tf = C_LF;          // 低通滤波器时间常数
    motor_B.PID_current_q.output_ramp = 1000; // 输出斜率限制
    // 配置电机B的电流环控制器参数（D轴）
    motor_B.PID_current_d.P = C_Kp;           // 比例增益
    motor_B.PID_current_d.I = C_Ki;           // 积分增益
    motor_B.LPF_current_d.Tf = C_LF;          // 低通滤波器时间常数
    motor_B.PID_current_d.output_ramp = 1000; // 输出斜率限制


    // 设置电机A的电流环输出限制
    motor_A.PID_current_d.limit = 1.0f; // D轴电流限制
    motor_A.PID_current_q.limit = 1.0f; // Q轴电流限制
    // 设置电机B的电流环输出限制
    motor_B.PID_current_d.limit = 1.0f; // D轴电流限制
    motor_B.PID_current_q.limit = 1.0f; // Q轴电流限制

    // 设置电机的速度限制
    motor_A.velocity_limit = 10; // 最大速度限制
    motor_B.velocity_limit = 10; // 最大速度限制

    // 设置电机电流限制
    motor_A.current_limit = 0.2f; // 最大电流限制
    motor_B.current_limit = 0.2f; // 最大电流限制

    motor_A.voltage_limit = 0.3f;
    motor_B.voltage_limit = 0.3f;

    //700KV
    motor_A.KV_rating = 700;
    motor_B.KV_rating = 700;

    // 初始化电机A和B
    motor_A.init();
    motor_B.init();


    // 初始化电流检测模块A
    if (cs_A.init() == 0) {
        Serial.println("[A侧电流检测]:初始化失败!");
        return false;
    } else {
        Serial.println("[A侧电流检测]:初始化成功");
    }
    // 初始化电流检测模块B
    if (cs_B.init() == 0) {
        Serial.println("[B侧电流检测]:初始化失败!");
        return false;
    } else {
        Serial.println("[B侧电流检测]:初始化成功");
    }


    // 驱动器8302的增益在所有通道上都是反向的，因此需要调整增益符号
    cs_A.gain_a *= -1;
    cs_A.gain_b *= -1;
    cs_A.gain_c *= -1;
    // 驱动器8302的增益在所有通道上都是反向的，因此需要调整增益符号
    cs_B.gain_a *= -1;
    cs_B.gain_b *= -1;
    cs_B.gain_c *= -1;


    // 将电流检测模块A与电机A关联
    motor_A.linkCurrentSense(&cs_A);
    // 将电流检测模块B与电机B关联
    motor_B.linkCurrentSense(&cs_B);


    Serial.println("[电机A]:尝试转动,对齐编码器,电流相位...");
    // 初始化电机A和B的FOC（磁场定向控制）
    if (motor_A.initFOC() == 0) {
        Serial.println("[A侧FOC控制]:初始化失败!");
        return false;
    } else {
        Serial.println("[A侧FOC控制]:初始化成功");
    }


    Serial.println("[电机B]:尝试转动,对齐编码器,电流相位...");
    if (motor_B.initFOC() == 0) {
        Serial.println("[B侧FOC控制]:初始化失败!");
        return false;
    } else {
        Serial.println("[B侧FOC控制]:初始化成功");
    }


    // 初始化直立控制PID参数
    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);

    // 初始化MPU6050传感器
    mpu6050.begin();

    // 播放短促的蜂鸣声，表示FOC自检完成
    // buzzer.play(S_BEEP);

    // 计算陀螺仪偏移量
    mpu6050.calcGyroOffsets();

    return true;
}

CarControl_t carCTRL;
uint8_t velocityCount = 0;
uint8_t angleCount = 0;

const double max_speed = 0.2; // 最大速度
const double min_speed = -0.2; // 最小速度

float limitSpeed(double target, double min_speed, double max_speed) {
    if (target < min_speed) {
        return min_speed; // 如果目标速度小于最小速度，限制为最小速度
    } else if (target > max_speed) {
        return max_speed; // 如果目标速度大于最大速度，限制为最大速度
    } else {
        return target; // 如果目标速度在范围内，直接返回目标速度
    }
}

void FocDriver::running() {
    // 执行电机A和B的FOC（磁场定向控制）循环
    motor_A.loopFOC(); // 更新电机A的FOC控制逻辑
    motor_B.loopFOC(); // 更新电机B的FOC控制逻辑

    // 根据目标速度控制电机运动
    motor_A.move(motor_A.target); // 控制电机A运动到目标速度
    motor_B.move(motor_B.target); // 控制电机B运动到目标速度


    // 更新MPU6050传感器数据
    mpu6050.update(); // 读取并更新MPU6050传感器数据

    // 获取MPU6050的X轴角度和角速度
    carCTRL.MPUangleX = mpu6050.getAngleX(); // 获取X轴角度
    carCTRL.MPUgyroX = mpu6050.getGyroXFV(); // 获取X轴角速度（滤波值）

    // 每隔VEL_PID_UPDATE次循环更新一次速度PID控制
    if (++velocityCount == VEL_PID_UPDATE) {
        // 获取电机A和B的转轴速度
        carCTRL.MA_Velocity = motor_A.shaft_velocity; // 电机A的速度

        carCTRL.MB_Velocity = motor_B.shaft_velocity; // 电机B的速度

        // 限制目标速度在预设范围内
        carCTRL.TargetVelocity = constrain(appCTRL.Velocity, -TARGET_VEL_LIMIT, TARGET_VEL_LIMIT);

        // 计算速度误差
        carCTRL.VelocityError = carCTRL.TargetVelocity - (carCTRL.MB_Velocity + carCTRL.MA_Velocity) / 2.0f;

        // 使用速度PID控制器计算目标角度
        carCTRL.TargetAngle = Velocity_PID(carCTRL.VelocityError);

        // 重置速度计数器
        velocityCount = 0;
    }

    // 每隔UPRIGHT_PID_UPDATE次循环更新一次姿态PID控制
    if (++angleCount == UPRIGHT_PID_UPDATE) {
        // 限制转向速度在预设范围内
        carCTRL.SteerVelocity = constrain(appCTRL.SteerVelocity, -STR_LIMIT, STR_LIMIT);

        // 使用姿态PID控制器计算电机速度
        carCTRL.MotorVelocity = UprightPID(&upRightPIDConfig, carCTRL.TargetAngle, carCTRL.MPUangleX, carCTRL.MPUgyroX);

        // 重置角度计数器
        angleCount = 0;
    }

    // 根据控制逻辑设置电机的目标速度
    motor_A.target = carCTRL.SteerVelocity - carCTRL.MotorVelocity; // 电机A的目标速度
    motor_B.target = carCTRL.MotorVelocity + carCTRL.SteerVelocity; // 电机B的目标速度

}

FocDriver BalanceDriver;