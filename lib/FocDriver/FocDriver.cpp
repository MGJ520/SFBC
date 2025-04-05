//#include <Arduino.h>
//#include "MPU6050.h"
//#include "SimpleFOC.h"
//
//#include "FocDriver.h"
//#include "APP.h"
//#include "BatteryAndButton.h"
//#include "UprightPID.h"
//#include "UserConfig.h"
//
//
//#include "APP.h"
//#include "UprightPID.h"
//#include "UserConfig.h"
//#include <Arduino.h>
//#include <MPU6050.h>
//#include <SimpleFOC.h>
//
//// sensor instance
//MagneticSensorI2C sensor_A = MagneticSensorI2C(AS5600_I2C);
//MagneticSensorI2C sensor_B = MagneticSensorI2C(AS5600_I2C);
//
//TwoWire I2C_A = TwoWire(1);
//TwoWire I2C_B = TwoWire(0);
//
//MPU6050 mpu6050(I2C_B);
//
//// BLDC motor & driver instance
//BLDCMotor motor_A = BLDCMotor(7);
//BLDCMotor motor_B = BLDCMotor(7);
//BLDCDriver3PWM driver_A = BLDCDriver3PWM(35, 13, 14);
//BLDCDriver3PWM driver_B = BLDCDriver3PWM(12, 11, 10);
//
//LowsideCurrentSense cs_A = LowsideCurrentSense(0.005f, 50.0f, 2, 3, _NC);
//LowsideCurrentSense cs_B = LowsideCurrentSense(0.005f, 50.0f, 4, 5, _NC);
//// -----------------------------------------------
//
//#if MONITOR_MODE
//Commander command = Commander(Serial);
//void doMotorA(char *cmd)
//{
//    command.motor(&motor_A, cmd);
//}
//
//void doMotorB(char *cmd)
//{
//    command.motor(&motor_B, cmd);
//}
//#endif
//// -----------------------------------------------
//UprightPID_t upRightPIDConfig;
//// PIDController Velocity_PID{.P = VELOCITY_Kp, .I = VELOCITY_Ki, .D = VELOCITY_Kd, .ramp = 1000, .limit =
//// VELOCITY_LIMIT};
//PIDController Velocity_PID(VELOCITY_Kp, VELOCITY_Ki, VELOCITY_Kd, 1000, VELOCITY_LIMIT);
////------------------------------------------------
//
//boolean FocDriver::init() {
//
//
//    I2C_A.begin(37, 36, 400000UL);
//    I2C_B.begin(8, 9, 400000UL);
//    sensor_A.init(&I2C_A);
//    sensor_B.init(&I2C_B);
//
//    // link the motor to the sensor
//    motor_A.linkSensor(&sensor_A);
//    motor_B.linkSensor(&sensor_B);
//
//    // driver config
//    // power supply voltage [V]
//    driver_A.voltage_power_supply = 8.2;
//    driver_B.voltage_power_supply = 8.2;
//    driver_A.init();
//    driver_B.init();
//    // link the motor and the driver
//    motor_A.linkDriver(&driver_A);
//    motor_B.linkDriver(&driver_B);
//    cs_A.linkDriver(&driver_A);
//    cs_B.linkDriver(&driver_B);
//    // motor config
//    motor_A.foc_modulation = FOCModulationType::SpaceVectorPWM;
//    motor_B.foc_modulation = FOCModulationType::SpaceVectorPWM;
//    // set motion control loop to be used
//    motor_A.torque_controller = TorqueControlType::foc_current;
//    motor_B.torque_controller = TorqueControlType::foc_current;
//    motor_A.controller = MotionControlType::velocity;
//    motor_B.controller = MotionControlType::velocity;
//
//    // velocity PI controller parameters
//    motor_A.PID_velocity.P = VEL_Kp;
//    motor_A.PID_velocity.I = VEL_Ki;
//    motor_A.PID_velocity.output_ramp = 1000;
//    motor_A.voltage_sensor_align = 0.6f;
//
//    motor_B.PID_velocity.P = VEL_Kp;
//    motor_B.PID_velocity.I = VEL_Ki;
//    motor_B.PID_velocity.output_ramp = 1000;
//    motor_B.voltage_sensor_align = 0.6f;
//
//    // --------------------------------------
//    motor_A.PID_current_q.P = C_Kp;
//    motor_A.PID_current_q.I = C_Ki;
//    motor_A.LPF_current_q.Tf = C_LF;
//    motor_A.PID_current_q.output_ramp = 1000;
//
//    motor_A.PID_current_d.P = C_Kp;
//    motor_A.PID_current_d.I = C_Ki;
//    motor_A.LPF_current_d.Tf = C_LF;
//    motor_A.PID_current_d.output_ramp = 1000;
//
//    motor_A.PID_current_d.limit = 3.0f;
//    motor_A.PID_current_q.limit = 3.0f;
//
//    motor_A.velocity_limit = 100;
//    motor_A.current_limit = 6.0f;
//    // ----------------------------------------
//    motor_B.PID_current_q.P = C_Kp;
//    motor_B.PID_current_q.I = C_Ki;
//    motor_B.LPF_current_q.Tf = C_LF;
//    motor_B.PID_current_q.output_ramp = 1000;
//
//    motor_B.PID_current_d.P = C_Kp;
//    motor_B.PID_current_d.I = C_Ki;
//    motor_B.LPF_current_d.Tf = C_LF;
//    motor_B.PID_current_d.output_ramp = 1000;
//
//    motor_B.PID_current_d.limit = 3.0f;
//    motor_B.PID_current_q.limit = 3.0f;
//
//    motor_B.velocity_limit = 100;
//    motor_B.current_limit = 6.0f;
//
//    // initialize motor
//    motor_A.init();
//    motor_B.init();
//
//    // ----------------------------------
//    cs_A.init();
//    // driver 8302 has inverted gains on all channels
//    cs_A.gain_a *= -1;
//    cs_A.gain_b *= -1;
//    cs_A.gain_c *= -1;
//    motor_A.linkCurrentSense(&cs_A);
//
//    cs_B.init();
//    // driver 8302 has inverted gains on all channels
//    cs_B.gain_a *= -1;
//    cs_B.gain_b *= -1;
//    cs_B.gain_c *= -1;
//    motor_B.linkCurrentSense(&cs_B);
//
//    motor_A.initFOC();
//    motor_B.initFOC();
//
//    // -------------------------------------------------------------------------------------
//    UprightPID_Init(&upRightPIDConfig, UPRIGHT_Kp, UPRIGHT_Ki, UPRIGHT_Kd, UPRIGHT_LIMIT);
//
//#if MONITOR_MODE
//    command.add('A', doMotorA, "motorA");
//    command.add('B', doMotorB, "motorB");
//#endif
//    mpu6050.begin();
//
//    mpu6050.calcGyroOffsets();
//    return true;
//}
//
//
//float limitSpeed(double target, double min_speed, double max_speed) {
//    if (target < min_speed) {
//        return min_speed; // 如果目标速度小于最小速度，限制为最小速度
//    } else if (target > max_speed) {
//        return max_speed; // 如果目标速度大于最大速度，限制为最大速度
//    } else {
//        return target; // 如果目标速度在范围内，直接返回目标速度
//    }
//}
//
//
//float Offset_parameters = 0; //偏置参数
//
//CarControl_t carCTRL;
//uint8_t velocityCount = 0;
//uint8_t angleCount = 0;
//
//void FocDriver::running() {
//
//    motor_A.loopFOC();
//    motor_B.loopFOC();
//    motor_A.move(motor_A.target);
//    motor_B.move(motor_B.target);
//
//    mpu6050.update();
//    carCTRL.MPUangleX = mpu6050.getAngleX();
//    carCTRL.MPUgyroX = mpu6050.getGyroXFV();
//
//#if MONITOR_MODE
//    motor_A.monitor();
//    motor_B.monitor();
//#endif
//
//    if (++velocityCount == VEL_PID_UPDATE)
//    {
//        carCTRL.MA_Velocity = motor_A.shaft_velocity;
//        carCTRL.MB_Velocity = motor_A.shaft_velocity;
//
//        carCTRL.TargetVelocity = constrain(appCTRL.Velocity, -TARGET_VEL_LIMIT, TARGET_VEL_LIMIT);
//
//        carCTRL.VelocityError = carCTRL.TargetVelocity - (carCTRL.MB_Velocity + carCTRL.MA_Velocity) / 2.0f;
//
//        carCTRL.TargetAngle = Velocity_PID(carCTRL.VelocityError);
//        velocityCount = 0;
//    }
//    if (++angleCount == UPRIGHT_PID_UPDATE)
//    {
//        carCTRL.SteerVelocity = constrain(appCTRL.SteerVelocity, -STR_LIMIT, STR_LIMIT);
//
//        carCTRL.MotorVelocity = UprightPID(&upRightPIDConfig, carCTRL.TargetAngle, carCTRL.MPUangleX, carCTRL.MPUgyroX);
//        angleCount = 0;
//    }
//
//    motor_A.target = carCTRL.SteerVelocity - carCTRL.MotorVelocity;
//    motor_B.target = carCTRL.MotorVelocity + carCTRL.SteerVelocity;
//
//}
//
//FocDriver BalanceDriver;