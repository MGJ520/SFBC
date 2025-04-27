//
// Created by MGJ on 2025/4/23.
//

#ifndef FOC_CODE_FOC_DRIVE_H
#define FOC_CODE_FOC_DRIVE_H


#include "mpu/MPU6050.h"


#define PID_CS_P          0.3
#define PID_CS_I          0.1
#define PID_CS_D          0.0

#define LPF_velocity_Tf   0.10
#define LPF_current_d_Tf  0.03
#define LPF_current_q_Tf  0.03
#define LPF_angle_Tf      0.05




#define AS5600_ADDR       0x36    // AS5600 的 I2C 地址
#define CHIP_ID_REG       0x0F    // 芯片 ID 寄存器地址



extern Commander command;


// 磁传感器
extern MagneticSensorI2C sensor_A;
extern MagneticSensorI2C sensor_B;



// MPU6050
extern MPU6050 mpu6050;

// 无刷电机
extern BLDCMotor motor_A;
extern BLDCMotor motor_B;

// 电机驱动器
extern BLDCDriver3PWM driver_A;
extern BLDCDriver3PWM driver_B;

// 电流检测
extern LowsideCurrentSense cs_A;
extern LowsideCurrentSense cs_B;


void MotorOpen();
void MotorClose();
void Foc_Parameters_init();
void onTarget(char *cmd);

bool check_encoder();


void Foc_A_Initialize(void *pvParameters);
void Foc_B_Initialize(void *pvParameters);



#endif //FOC_CODE_FOC_DRIVE_H
