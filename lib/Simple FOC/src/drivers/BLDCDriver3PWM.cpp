#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
    if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
    if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
  if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
  if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);

}

// 初始化硬件引脚
int BLDCDriver3PWM::init() {
    // 配置PWM引脚为输出模式
    pinMode(pwmA, OUTPUT); // 设置pwmA引脚为输出模式
    pinMode(pwmB, OUTPUT); // 设置pwmB引脚为输出模式
    pinMode(pwmC, OUTPUT); // 设置pwmC引脚为输出模式

    // 如果定义了使能引脚，则将其配置为输出模式
    if(_isset(enableA_pin)) pinMode(enableA_pin, OUTPUT); // 如果定义了enableA_pin，则设置为输出模式
    if(_isset(enableB_pin)) pinMode(enableB_pin, OUTPUT); // 如果定义了enableB_pin，则设置为输出模式
    if(_isset(enableC_pin)) pinMode(enableC_pin, OUTPUT); // 如果定义了enableC_pin，则设置为输出模式

    // 检查电压限制配置是否合理
    if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
        voltage_limit = voltage_power_supply; // 如果未定义电压限制或电压限制大于电源电压，则将电压限制设置为电源电压

    // 设置PWM引脚的频率
    // 这是一个硬件特定的函数，具体取决于驱动器和微控制器
    params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC); // 配置3个PWM引脚的频率
    initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED); // 如果初始化成功，则将initialized设置为true
    return params != SIMPLEFOC_DRIVER_INIT_FAILED; // 返回初始化是否成功
}


// Set voltage to the pwm pin
void BLDCDriver3PWM::setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
  // disable if needed
  if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
    digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
    digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
    digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
  }
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}
