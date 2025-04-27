#include "../hardware_api.h"

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageInline(const int pinA, const void* cs_params){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * ((GenericCurrentSenseParams*)cs_params)->adc_voltage_conv;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  void* _configureADCInline(const void* driver_params, const int pinA,const int pinB,const int pinC){
  _UNUSED(driver_params);

  if( _isset(pinA) ) pinMode(pinA, INPUT);
  if( _isset(pinB) ) pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);

  GenericCurrentSenseParams* params = new GenericCurrentSenseParams {
    .pins = { pinA, pinB, pinC },
    .adc_voltage_conv = (5.0f)/(1024.0f)
  };

  return params;
}

// function reading an ADC value and returning the read voltage
__attribute__((weak))  float _readADCVoltageLowSide(const int pinA, const void* cs_params){
  return _readADCVoltageInline(pinA, cs_params);
}

// Configure low side for generic mcu
// cannot do much but 
__attribute__((weak))  void* _configureADCLowSide(const void* driver_params, const int pinA,const int pinB,const int pinC){
  return _configureADCInline(driver_params, pinA, pinB, pinC);
}

// 同步驱动器和模数转换器（ADC）
// 此函数用于在硬件驱动层面同步驱动器与ADC的操作，确保两者在数据采集和传输过程中保持一致。
__attribute__((weak)) void _driverSyncLowSide(void* driver_params, void* cs_params){
    // driver_params: 驱动器参数指针，用于传递驱动器相关的配置或状态信息。
    // cs_params: 控制信号参数指针，用于传递与控制信号相关的参数。
    _UNUSED(driver_params); // 表示此参数在当前实现中未被使用，避免编译器警告。
    _UNUSED(cs_params);     // 表示此参数在当前实现中未被使用，避免编译器警告。
}
__attribute__((weak)) void _startADC3PinConversionLowSide(){ }
