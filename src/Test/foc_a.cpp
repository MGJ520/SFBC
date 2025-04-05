//
//#include <SimpleFOC.h>
//#include <Arduino.h>
//#include "BatteryAndButton.h"
//#include "./Pre_System/Pre_System.h"
//
//BLDCMotor motor = BLDCMotor(7);
////两个电机
//#define  A 1
//#if A
//BLDCDriver3PWM driver = BLDCDriver3PWM(35,13,14);
//#else
//BLDCDriver3PWM driver = BLDCDriver3PWM(12, 11, 10);
//#endif
//
//LowsideCurrentSense cs_A = LowsideCurrentSense(0.005f, 50.0f, 2, 3, _NC); // 电机A的电流检测
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
//TwoWire I2C_A = TwoWire(1);
//
//
//float target_velocity = 1;
//Commander command = Commander(Serial);
//
//void onTarget(char *cmd)
//{
//    command.scalar(&motor.target, cmd);
//}
//
//void setup() {
//    //打开电源
//    SetupCarSystem();
//
//    PowerAndButton.startTask();
//
//    motor.useMonitoring(Serial);
//
//
//#if A
//    I2C_A.begin(37, 36, 400000UL);
//#else
//    I2C_A.begin(8, 9, 400000UL);
//#endif
//
//
//
//    sensor.init(&I2C_A);
//    driver.voltage_power_supply = 8;
//    driver.init();
//
//    motor.linkSensor(&sensor);
//    motor.linkDriver(&driver);
//    cs_A.linkDriver(&driver);
//    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
//    motor.torque_controller = TorqueControlType::foc_current;
//    motor.controller = MotionControlType::torque;
//    motor.PID_velocity.P = 0.3;  // 降低P减少震荡
//    motor.PID_velocity.I = 0.2;   // 增加I加速误差消除
//    motor.PID_velocity.D = 0.0;   // 暂时禁用D项
//    motor.LPF_velocity.Tf = 0.09; // 增强速度滤波
//    motor.PID_velocity.output_ramp = 1000;
//
//    motor.PID_current_d.P = 0.2;  // 降低P减少震荡
//    motor.PID_current_d.I = 0.2;   // 增加I加速误差消除
//    motor.PID_current_d.D = 0.0;   // 暂时禁用D项
//
//    motor.LPF_current_d.Tf = 0.06; // 增强速度滤波
//
//    motor.PID_current_q.P = 0.2;  // 降低P减少震荡
//    motor.PID_current_q.I = 0.2;   // 增加I加速误差消除
//    motor.PID_current_q.D = 0.0;   // 暂时禁用D项
//
//    motor.LPF_current_q.Tf = 0.06; // 增强速度滤波
//
//    motor.LPF_angle.Tf = 0.05; // 增强速度滤波
//
//    motor.LPF_angle.Tf = 0.05f;  // 增强角度信号滤波
//    motor.voltage_limit = 1.0f;
//    motor.current_limit = 3.0f;
//    motor.voltage_sensor_align = 0.8f;
//
//
//    motor.init();
//    cs_A.init();
//    cs_A.gain_a *= -1;
//    cs_A.gain_b *= -1;
//    cs_A.gain_c *= -1;
//    motor.linkCurrentSense(&cs_A);
//    motor.initFOC();
//
//    command.add('T', onTarget, "target velocity");
//    Serial.println("Motor ready.");
//    Serial.println("Set the target velocity using serial terminal:");
////    _delay(1000);
//motor.target=1;
//}
//
//
//void loop() {
//    motor.loopFOC();
//    motor.move();
//    command.run();
//    motor.monitor();
//}
//
