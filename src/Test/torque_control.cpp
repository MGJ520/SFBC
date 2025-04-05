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
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//TwoWire I2C = TwoWire(0);
//
//
////BLDCDriver3PWM driver = BLDCDriver3PWM(35, 13, 14);
//BLDCDriver3PWM driver = BLDCDriver3PWM(12, 11, 10);
//
//// commander实例化
//Commander command = Commander(Serial);
//void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
//
//void setup() {
//    SetupCarSystem();
//    // 初始化编码传感器硬件
//    PowerAndButton.startTask();
//    //I2C.begin(37, 36, 400000UL);
//      I2C.begin(8, 9, 400000UL);
//    sensor.init(&I2C);
//    // 连接电机和传感器
//    motor.linkSensor(&sensor);
//
//    // 配置驱动器
//    // 电源电压 [V]
//    driver.voltage_power_supply =8;
//    driver.init();
//    // 连接驱动器
//    motor.linkDriver(&driver);
//
//    // 校准电压
//    motor.voltage_sensor_align = 0.5f;
//
//    // 设置运动控制环
//    motor.torque_controller = TorqueControlType::voltage;
//    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
//    motor.controller = MotionControlType::angle;
//    motor.voltage_limit = 0.3;
//    // 配置位置环 PID 参数
//    motor.P_angle.P = 20;   // 比例增益
//    motor.P_angle.I = 1;    // 积分增益
//    motor.P_angle.D = 0 ;    // 微分增益
//    motor.P_angle.output_ramp=1000;
////    motor.LPF_angle.Tf = 0.05;
//
//
//    // 如果不需要，可以注释掉此行
//    motor.useMonitoring(Serial);
//
//    // 初始化电机
//    motor.init();
//
//    // 校准编码器，启用FOC
//    motor.initFOC();
//
//    // 设置电机初始目标
//    motor.target = 0; // Volts
//
//    command.add('T', doTarget, "target voltage");
//
//    Serial.println(F("Motor ready."));
//    Serial.println(F("Set the target using serial terminal:"));
//    _delay(1000);
//}
//int i=0;
//void loop() {
//
//    if (i>10000)
//    {
//        i=0;
//        Serial.println(sensor.getAngle());
//    }
//    i++;
//    // FOC算法主函数
//    motor.loopFOC();
//
//    // 运动控制函数
//    motor.move();
//
//    // 用户通信
//    command.run();
//}