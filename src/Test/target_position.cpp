//#include <Arduino.h>
//#include "SimpleFOC.h"
//#include "BatteryAndButton.h"
//#include "./Pre_System/Pre_System.h"
//
//#include "FocDriver.h"
//
//
//BLDCMotor motor = BLDCMotor(7);
//
//#define  A 0
//#if A
//BLDCDriver3PWM driver = BLDCDriver3PWM(35, 13, 14);
//#else
//BLDCDriver3PWM driver = BLDCDriver3PWM(12, 11, 10);
//#endif
//// 目标变量
//float target_position = 0;
//
//// commander实例化
//Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
//
//void setup() {
//    //初始化系统
//    SetupCarSystem();
//    // 开启按键与电压检测任务
//    PowerAndButton.startTask();
//    // 配置驱动器
//    // 电源电压 [V]
//    driver.voltage_power_supply = 8;
//    driver.init();
//
//    // 连接电机和驱动器
//    motor.linkDriver(&driver);
//    motor.voltage_limit = 0.4;   // [V] - 如果相电阻没有定义
//
//    // 配置开环控制
//    motor.controller = MotionControlType::angle_openloop;
//
//    // 初始化电机
//    motor.init();
//
//    // 添加目标命令T
//    command.add('T', doTarget, "target velocity");
//
//    Serial.println("Motor ready!");
//    Serial.println("Set target velocity [rad/s]");
//
//    _delay(1000);
//}
//
//
//void loop() {
//    motor.move(motor.target);
//
//    motor.monitor();
//
//    command.run();
//
//}
