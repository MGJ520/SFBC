#ifndef LOWSIDE_CS_LIB_H
#define LOWSIDE_CS_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/defaults.h"
#include "../common/base_classes/CurrentSense.h"
#include "../common/base_classes/FOCMotor.h"
#include "../common/lowpass_filter.h"
#include "hardware_api.h"


class LowsideCurrentSense : public CurrentSense {
public:
    /**
    LowsideCurrentSense 类构造函数
    @param shunt_resistor 采样电阻值
    @param gain 电流采样运放增益
    @param phA A 相 ADC 引脚
    @param phB B 相 ADC 引脚
    @param phC C 相 ADC 引脚（可选）
  */
    LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = _NC);

    /**
      LowsideCurrentSense 类构造函数
      @param mVpA 每安培对应的毫伏数（mV/A 比率）
      @param phA A 相 ADC 引脚
      @param phB B 相 ADC 引脚
      @param phC C 相 ADC 引脚（可选）
    */
    LowsideCurrentSense(float mVpA, int pinA, int pinB, int pinC = _NC);

    // 实现 CurrentSense 接口的函数

    // 初始化函数
    int init() override;

    // 获取三相电流
    PhaseCurrent_s getPhaseCurrents() override;

    // 驱动器对齐函数，参数为对齐电压
    int driverAlign(float align_voltage) override;


    // 每相的 ADC 测量增益
    // 支持不同相位的不同增益，更常见的是——反相电流
    // 这一部分应该在未来实现自动化
    float gain_a; //!< A 相增益
    float gain_b; //!< B 相增益
    float gain_c; //!< C 相增益

    // // 每相的低通滤波器
    // LowPassFilter lpf_a{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!< A 相电流低通滤波器
    // LowPassFilter lpf_b{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!< B 相电流低通滤波器
    // LowPassFilter lpf_c{DEF_LPF_PER_PHASE_CURRENT_SENSE_Tf}; //!< C 相电流低通滤波器

    float offset_ia; //!< A 相零电流时的电压值（ADC 读数的中心值）
    float offset_ib; //!< B 相零电流时的电压值（ADC 读数的中心值）
    float offset_ic; //!< C 相零电流时的电压值（ADC 读数的中心值）

private:

    // 硬件相关变量
    int pinA; //!< 用于电流测量的 A 相模拟引脚
    int pinB; //!< 用于电流测量的 B 相模拟引脚
    int pinC; //!< 用于电流测量的 C 相模拟引脚

    // 增益相关变量
    float shunt_resistor; //!< 采样电阻的阻值
    float amp_gain; //!< 放大器的增益值
    float volts_to_amps_ratio; //!< 伏特到安培的转换比率

    /**
     * 用于查找 ADC 零点偏移的函数
     */
    void calibrateOffsets();

};

#endif
