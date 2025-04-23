//#include <Arduino.h>
//#include "SimpleFOC.h"
//#include "BatteryAndButton.h"
//#include "./Pre_System/Pre_System.h"
//
//BLDCMotor motor = BLDCMotor(7);
//
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//TwoWire I2C = TwoWire(0);
//
//#define  A 0
//#if A
//BLDCDriver3PWM driver = BLDCDriver3PWM(35, 13, 14);
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 50.0f, 2, 3, _NC); // 电机A的电流检测
//#else
//BLDCDriver3PWM driver = BLDCDriver3PWM(12, 11, 10);
//LowsideCurrentSense current_sense = LowsideCurrentSense(0.005f, 50.0f, 4, 5, _NC); // 电机B的电流检测
//#endif
//
//// commander实例化
//Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
//
//void setup() {
//    SetupCarSystem();
//    // 初始化编码传感器硬件
//    PowerAndButton.startTask();
//
//#if A
//    I2C.begin(37, 36, 400000UL);
//#else
//    I2C.begin(8, 9, 400000UL);
//#endif
//
//    sensor.init(&I2C);
//    // 连接电机和传感器
//    motor.linkSensor(&sensor);
//
//    // 配置驱动器
//    // 电源电压 [V]
//    driver.voltage_power_supply = 8;
//
//    driver.init();
//    // 连接驱动器
//    motor.linkDriver(&driver);
//
//    // 电流检测初始化和连接
//    current_sense.init();
//    motor.linkCurrentSense(&current_sense);
//
//
//    // 校准电压
//    motor.voltage_sensor_align = 0.5f;
//
//    // 力方式
//    motor.torque_controller = TorqueControlType::voltage;
//    // 运动方式
//    motor.controller = MotionControlType::velocity;
//
//    motor.voltage_limit = 0.3;
//
//    // 初始化电机
//    motor.init();
//    // 校准编码器，启用FOC
//    motor.initFOC();
//
//    // 设置初始目标值
//    motor.target = 0;
//
//    command.add('T', doTarget, "target voltage");
//    Serial.println(F("Motor ready."));
//    Serial.println(F("Set the target using serial terminal:"));
//    _delay(1000);
//}
//
//int i = 0;
//
//void loop() {
//
//    // FOC算法主函数
//    motor.loopFOC();
//    // 运动控制函数
//    motor.move(motor.target);
//    // 显示电流
//    motor.monitor();
//    // 用户通信
//    command.run();
//
//}