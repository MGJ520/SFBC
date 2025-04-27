#ifndef FOCMOTOR_H
#define FOCMOTOR_H

#include "Arduino.h"
#include "Sensor.h"
#include "CurrentSense.h"

#include "../time_utils.h"
#include "../foc_utils.h"
#include "../defaults.h"
#include "../pid.h"
#include "../lowpass_filter.h"


// 监控位图（用于定义哪些参数需要监控）
#define _MON_TARGET 0b1000000 // 监控目标值
#define _MON_VOLT_Q 0b0100000 // 监控电压q分量值
#define _MON_VOLT_D 0b0010000 // 监控电压d分量值
#define _MON_CURR_Q 0b0001000 // 监控电流q分量值 - 如果有测量值
#define _MON_CURR_D 0b0000100 // 监控电流d分量值 - 如果有测量值
#define _MON_VEL    0b0000010 // 监控速度值
#define _MON_ANGLE  0b0000001 // 监控角度值


/**
 *  运动控制类型
 */
enum MotionControlType : uint8_t {
    torque            = 0x00,     //!< 力矩控制
    velocity          = 0x01,     //!< 速度运动控制
    angle             = 0x02,     //!< 位置/角度运动控制
    velocity_openloop = 0x03,     //!< 开环速度控制
    angle_openloop    = 0x04      //!< 开环角度控制
};


/**
 *  力矩控制类型
 */
enum TorqueControlType : uint8_t {
    voltage            = 0x00,     //!< 使用电压进行力矩控制
    dc_current         = 0x01,     //!< 使用直流电流进行力矩控制（单一电流幅值）
    foc_current        = 0x02,     //!< 使用dq电流进行力矩控制
};

/**
 *  FOC调制类型
 */
enum FOCModulationType : uint8_t {
    SinePWM            = 0x00,     //!< 正弦波脉宽调制（Sine PWM）
    SpaceVectorPWM     = 0x01,     //!< 空间矢量脉宽调制（Space Vector PWM）
    Trapezoid_120      = 0x02,     //!< 梯形波调制（120°）
    Trapezoid_150      = 0x03,     //!< 梯形波调制（150°）
};


enum FOCMotorStatus : uint8_t {
    motor_uninitialized = 0x00,     //!< 电机尚未初始化
    motor_initializing  = 0x01,     //!< 电机初始化正在进行中
    motor_uncalibrated  = 0x02,     //!< 电机已初始化，但未校准（可以进行开环控制）
    motor_calibrating   = 0x03,     //!< 电机校准正在进行中
    motor_ready         = 0x04,     //!< 电机已初始化且校准完成（可以进行闭环控制）
    motor_error         = 0x08,     //!< 电机处于错误状态（可恢复，例如过流保护激活）
    motor_calib_failed  = 0x0E,     //!< 电机校准失败（可能可恢复）
    motor_init_failed   = 0x0F,     //!< 电机初始化失败（不可恢复）
};

/**
 Generic motor class
*/
class FOCMotor
{
  public:
    /**
     * Default constructor - setting all variabels to default values
     */
    FOCMotor();

    /**  Motor hardware init function */
  	virtual void init()=0;
    /** Motor disable function */
  	virtual void disable()=0;
    /** Motor enable function */
    virtual void enable()=0;

    /**
     * Function linking a motor and a sensor 
     * 
     * @param sensor Sensor class  wrapper for the FOC algorihtm to read the motor angle and velocity
     */
    void linkSensor(Sensor* sensor);

    /**
     * Function linking a motor and current sensing 
     * 
     * @param current_sense CurrentSense class wrapper for the FOC algorihtm to read the motor current measurements
     */
    void linkCurrentSense(CurrentSense* current_sense);


    /**
     * Function initializing FOC algorithm
     * and aligning sensor's and motors' zero position 
     * 
     * - If zero_electric_offset parameter is set the alignment procedure is skipped
     */
    virtual int initFOC()=0;
    /**
     * Function running FOC algorithm in real-time
     * it calculates the gets motor angle and sets the appropriate voltages 
     * to the phase pwm signals
     * - the faster you can run it the better Arduino UNO ~1ms, Bluepill ~ 100us
     */
    virtual void loopFOC()=0;
    /**
     * Function executing the control loops set by the controller parameter of the BLDCMotor.
     * 
     * @param target  Either voltage, angle or velocity based on the motor.controller
     *                If it is not set the motor will use the target set in its variable motor.target
     * 
     * This function doesn't need to be run upon each loop execution - depends of the use case
     */
    virtual void move(float target = NOT_SET)=0;

    /**
    * Method using FOC to set Uq to the motor at the optimal angle
    * Heart of the FOC algorithm
    * 
    * @param Uq Current voltage in q axis to set to the motor
    * @param Ud Current voltage in d axis to set to the motor
    * @param angle_el current electrical angle of the motor
    */
    virtual void setPhaseVoltage(float Uq, float Ud, float angle_el)=0;

    // State calculation methods 
    /** Shaft angle calculation in radians [rad] */
    float shaftAngle();
    /**
     * Shaft angle calculation function in radian per second [rad/s]
     * It implements low pass filtering
     */
    float shaftVelocity();



    /**
     * Electrical angle calculation  
     */
    float electricalAngle();

    // 状态变量
    float target;            //!< 当前目标值 —— 取决于控制器的类型（角度、速度或转矩）
    float feed_forward_velocity = 0.0f; //!< 当前前馈速度
    float shaft_angle;       //!< 当前电机的机械角度
    float electrical_angle;  //!< 当前电机的电角度
    float shaft_velocity;    //!< 当前电机的速度rad/s
    float current_sp;        //!< 目标电流（q轴电流）
    float shaft_velocity_sp; //!< 当前目标速度
    float shaft_angle_sp;    //!< 当前目标角度
    DQVoltage_s voltage;     //!< 当前施加到电机的d轴和q轴电压
    DQCurrent_s current;     //!< 当前测量到的d轴和q轴电流
    float voltage_bemf;      //!< 估算的反电动势电压（如果提供了KV常数）

    // 电机配置参数
    float voltage_sensor_align; //!< 用于传感器与电机对齐的电压参数
    float velocity_index_search; //!< 用于索引搜索的目标速度

    // 电机物理参数
    float phase_resistance; //!< 电机的相电阻
    int pole_pairs; //!< 电机的极对数
    float KV_rating; //!< 电机的KV值（转速常数）
    float phase_inductance; //!< 电机的相电感

    // 限制变量
    float voltage_limit; //!< 电压限制变量 - 全局限制
    float current_limit; //!< 电流限制变量 - 全局限制
    float velocity_limit; //!< 速度限制变量 - 全局限制



// 电机状态变量
    int8_t enabled = 0; //!< 电机使能标志（0 = 禁用，1 = 使能）
    FOCMotorStatus motor_status = FOCMotorStatus::motor_uninitialized; //!< 电机状态

// PWM调制相关变量
    FOCModulationType foc_modulation; //!< 确定调制算法的参数
    int8_t modulation_centered = 1; //!< 调制方式标志（1）以驱动器限值/2为中心的调制，或（0）拉至0

// 配置结构体
    TorqueControlType torque_controller; //!< 确定转矩控制类型的参数
    MotionControlType controller; //!< 确定所使用的控制环路的参数

    // controllers and low pass filters
    PIDController PID_current_q{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};//!< parameter determining the q current PID config
    PIDController PID_current_d{DEF_PID_CURR_P,DEF_PID_CURR_I,DEF_PID_CURR_D,DEF_PID_CURR_RAMP, DEF_POWER_SUPPLY};//!< parameter determining the d current PID config
    LowPassFilter LPF_current_q{DEF_CURR_FILTER_Tf};//!<  parameter determining the current Low pass filter configuration
    LowPassFilter LPF_current_d{DEF_CURR_FILTER_Tf};//!<  parameter determining the current Low pass filter configuration
    PIDController PID_velocity{DEF_PID_VEL_P,DEF_PID_VEL_I,DEF_PID_VEL_D,DEF_PID_VEL_RAMP,DEF_PID_VEL_LIMIT};//!< parameter determining the velocity PID configuration
    PIDController P_angle{DEF_P_ANGLE_P,0,0,0,DEF_VEL_LIM};	//!< parameter determining the position PID configuration
    LowPassFilter LPF_velocity{DEF_VEL_FILTER_Tf};//!<  parameter determining the velocity Low pass filter configuration
    LowPassFilter LPF_angle{0.0};//!<  parameter determining the angle low pass filter configuration 
    unsigned int motion_downsample = DEF_MOTION_DOWNSMAPLE; //!< parameter defining the ratio of downsampling for move commad
    unsigned int motion_cnt = 0; //!< counting variable for downsampling for move commad

    // sensor related variabels
    float sensor_offset; //!< user defined sensor zero offset
    float zero_electric_angle = NOT_SET;//!< absolute zero electric angle - if available
    Direction sensor_direction = Direction::UNKNOWN; //!< default is CW. if sensor_direction == Direction::CCW then direction will be flipped compared to CW. Set to UNKNOWN to set by calibration

    /**
     * Function providing BLDCMotor class with the 
     * Serial interface and enabling monitoring mode
     * 
     * @param serial Monitoring Serial class reference
     */
    void useMonitoring(Print &serial);

    /**
     * Utility function intended to be used with serial plotter to monitor motor variables
     * significantly slowing the execution down!!!!
     */
    void monitor();
    unsigned int monitor_downsample = DEF_MON_DOWNSMAPLE; //!< 每调用 monitor_downsample 次才显示一次监控输出
    char monitor_start_char = '\0'; //!< 监控输出的起始字符
    char monitor_end_char = '\0'; //!< 监控输出的结束字符
    char monitor_separator = '\t'; //!< 监控输出的分隔符
    unsigned int monitor_decimals = 4; //!< 监控输出的小数位数
    // 初始监控将显示目标值、电压、速度和角度
    uint8_t monitor_variables = _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; //!< 位数组，用于存储用户希望监控的变量映射

    /** 
      * Sensor link:
      * - Encoder 
      * - MagneticSensor*
      * - HallSensor
    */
    Sensor* sensor;
    /** 
      * CurrentSense link
    */
    CurrentSense* current_sense;

    // monitoring functions
    Print* monitor_port; //!< Serial terminal variable if provided
  private:
    // monitor counting variable
    unsigned int monitor_cnt = 0 ; //!< counting variable
};


#endif
