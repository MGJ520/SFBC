//
//#include <SimpleFOC.h>
//#include <Arduino.h>
//#include "BatteryAndButton.h"
//#include "./Pre_System/Pre_System.h"
//
//// sensor instance
//MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C);
//MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C);
//
//TwoWire I2C_A = TwoWire(1);
//TwoWire I2C_B = TwoWire(0);
//
//// BLDC motor & driver instance
//BLDCMotor motor_A = BLDCMotor(7);
//BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 13, 14);
//
//BLDCMotor motor_B = BLDCMotor(7);
//BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10);
//
//// instantiate the commander
//Commander command = Commander(Serial);
//
//void doMotorA(char *cmd)
//{
//    command.motor(&motor_A, cmd);
//}
//
//
//
//void setup()
//{
//    SetupCarSystem();
//
//    // 初始化编码传感器硬件
//    PowerAndButton.startTask();
//
//    // initialise magnetic sensor hardware
//    I2C_A.begin(37, 36, 400000UL);
//    I2C_B.begin(8, 9, 400000UL);
//
//    // initialise magnetic sensor hardware
//    sensor_A.init(&I2C_A);
//    sensor_B.init(&I2C_B);
//
//    // link the motor to the sensor
//    motor_A.linkSensor(&sensor_A);
//    motor_B.linkSensor(&sensor_B);
//
//    // driver config
//    // power supply voltage [V]
//    driver_A.voltage_power_supply = 8.4;
//    driver_B.voltage_power_supply = 8.4;
//    driver_A.init();
//    driver_B.init();
//    // link the motor and the driver
//    motor_A.linkDriver(&driver_A);
//    motor_B.linkDriver(&driver_B);
//
//    // motor config
//    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
//    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
//
//    motor_B.modulation_centered = 1.0;
//    motor_A.modulation_centered = 1.0;
//
//    // set motion control loop to be used
//    motor_A.controller = MotionControlType::velocity;
//    motor_B.controller = MotionControlType::velocity;
//
//    // velocity PI controller parameters
//    motor_A.PID_velocity.P = 0.09f;
//    motor_A.PID_velocity.I = 3;
//    motor_A.PID_velocity.D = 0;
//    motor_A.LPF_velocity.Tf = 0.02f;
//    motor_A.PID_velocity.output_ramp = 1000;
//
//
//    motor_B.PID_velocity.P = 0.09f;
//    motor_B.PID_velocity.I = 3;
//    motor_B.PID_velocity.D = 0;
//    motor_B.LPF_velocity.Tf = 0.02f;
//    motor_B.PID_velocity.output_ramp = 1000;
//
//
//    motor_A.voltage_limit = 0.3;
//    motor_B.voltage_limit = 0.3;
//
//
//    // comment out if not needed
//    motor_A.useMonitoring(Serial);
//    motor_B.useMonitoring(Serial);
//    // initialize motor
//    motor_A.init();
//    motor_B.init();
//
//
//    // 在initFOC后添加状态检查
//    Serial.print("MotorA initFOC: ");
//    Serial.println(motor_A.initFOC() ? "OK" : "FAIL");
//    Serial.print("MotorB initFOC: ");
//    Serial.println(motor_B.initFOC() ? "OK" : "FAIL");
//
//
//    motor_A.target=10;
//    motor_B.target=10;
//
//    command.add('A', doMotorA, "motorA");
//    command.add('B', doMotorB, "motorB");
//
//    Serial.println(F("Motor ready."));
//    Serial.println(F("Set the target velocity using serial terminal:"));
//
//    _delay(1000);
//}
//
//void loop()
//{
//    motor_A.loopFOC();
//    motor_B.loopFOC();
//    motor_A.move();
//    motor_B.move();
//    motor_A.monitor();
//    motor_B.monitor();
//    command.run();
//}
