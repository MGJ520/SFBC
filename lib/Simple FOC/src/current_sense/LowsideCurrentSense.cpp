#include "LowsideCurrentSense.h"
#include "communication/SimpleFOCDebug.h"

// LowsideCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
LowsideCurrentSense::LowsideCurrentSense(float _shunt_resistor, float _gain, int _pinA, int _pinB, int _pinC) {
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    shunt_resistor = _shunt_resistor;
    amp_gain = _gain;
    volts_to_amps_ratio = 1.0f / _shunt_resistor / _gain; // volts to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}


LowsideCurrentSense::LowsideCurrentSense(float _mVpA, int _pinA, int _pinB, int _pinC) {
    pinA = _pinA;
    pinB = _pinB;
    pinC = _pinC;

    volts_to_amps_ratio = 1000.0f / _mVpA; // mV to amps
    // gains for each phase
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
}


// 低边电流传感器初始化函数
int LowsideCurrentSense::init() {
    // 检查是否已经链接了驱动器对象
    if (driver == nullptr) {
        SIMPLEFOC_DEBUG("CUR: Driver not linked!"); // 打印调试信息：未链接驱动器
        return 0; // 返回0表示初始化失败
    }

    // 配置ADC相关参数
    // 调用配置函数，传入驱动器参数和三个电流检测引脚号，返回配置后的ADC参数
    params = _configureADCLowSide(driver->params, pinA, pinB, pinC);

    // 如果配置失败，返回失败标志
    if (params == SIMPLEFOC_CURRENT_SENSE_INIT_FAILED) return 0;

    // 同步驱动器参数
    // 将配置好的ADC参数同步到驱动器中
    _driverSyncLowSide(driver->params, params);



    if (offset_flag)
    {
        SIMPLEFOC_DEBUG("CUR:calibrateOffsets"); // 打印调试信息：开始校准偏移量
        calibrateOffsets();
        offset_flag= false;

    } else
    {
        SIMPLEFOC_DEBUG("CUR:Skip Offset"); // 打印调试信息：初始化成功
    }


    // 设置初始化标志
    // 如果ADC参数配置成功（不等于初始化失败标志），则将初始化标志设置为true
    initialized = (params != SIMPLEFOC_CURRENT_SENSE_INIT_FAILED);

    // 返回成功标志
    SIMPLEFOC_DEBUG("CUR: CurrentSense Success!"); // 打印调试信息：初始化成功
    return 1;
}


// 函数声明：校准ADC的零偏移量
void LowsideCurrentSense::calibrateOffsets() {

    // 定义校准轮数，用于多次采样以提高精度
    const int calibration_rounds = 2000;

    // 初始化ADC偏移量为0
    offset_ia = 0; // 通道A的偏移量
    offset_ib = 0; // 通道B的偏移量
    offset_ic = 0; // 通道C的偏移量

    // 读取ADC电压多次（2000次），以找到零电流时的电压偏移量
    for (int i = 0; i < calibration_rounds; i++) {
        // 启动低边电流检测的ADC转换
        _startADC3PinConversionLowSide();

        // 如果通道A的引脚已设置，则读取通道A的ADC电压并累加到偏移量中
        if (_isset(pinA)) offset_ia += (_readADCVoltageLowSide(pinA, params));
        // 如果通道B的引脚已设置，则读取通道B的ADC电压并累加到偏移量中
        if (_isset(pinB)) offset_ib += (_readADCVoltageLowSide(pinB, params));
        // 如果通道C的引脚已设置，则读取通道C的ADC电压并累加到偏移量中
        if (_isset(pinC)) offset_ic += (_readADCVoltageLowSide(pinC, params));

        // 延时1ms，等待下一次ADC转换
        _delay(1);
    }

    // 计算每个通道的平均偏移量
    if (_isset(pinA)) offset_ia = offset_ia / calibration_rounds; // 计算通道A的平均偏移量
    if (_isset(pinB)) offset_ib = offset_ib / calibration_rounds; // 计算通道B的平均偏移量
    if (_isset(pinC)) offset_ic = offset_ic / calibration_rounds; // 计算通道C的平均偏移量

}


// read all three phase currents (if possible 2 or 3)
PhaseCurrent_s LowsideCurrentSense::getPhaseCurrents() {
    PhaseCurrent_s current;
    _startADC3PinConversionLowSide();
    current.a = (!_isset(pinA)) ? 0 : (_readADCVoltageLowSide(pinA, params) - offset_ia) * gain_a;// amps
    current.b = (!_isset(pinB)) ? 0 : (_readADCVoltageLowSide(pinB, params) - offset_ib) * gain_b;// amps
    current.c = (!_isset(pinC)) ? 0 : (_readADCVoltageLowSide(pinC, params) - offset_ic) * gain_c; // amps
    return current;
}

// Function aligning the current sense with motor driver
// if all pins are connected well none of this is really necessary! - can be avoided
// returns flag
// 0 - fail
// 1 - success and nothing changed
// 2 - success but pins reconfigured
// 3 - success but gains inverted
// 4 - success but pins reconfigured and gains inverted
int LowsideCurrentSense::driverAlign(float voltage) {

    int exit_flag = 1;
    if (skip_align) return exit_flag;

    if (!initialized) return 0;

    if (_isset(pinA)) {
        // set phase A active and phases B and C down
        driver->setPwm(voltage, 0, 0);
        //修改
        _delay(500);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the current 100 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a * 0.6f + 0.4f * c1.a;
            c.b = c.b * 0.6f + 0.4f * c1.b;
            c.c = c.c * 0.6f + 0.4f * c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        // align phase A
        float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
        float ac_ratio = c.c ? fabs(c.a / c.c) : 0;
        if (_isset(pinB) && ab_ratio > 1.5f) { // should be ~2
            gain_a *= _sign(c.a);
        } else if (_isset(pinC) && ac_ratio > 1.5f) { // should be ~2
            gain_a *= _sign(c.a);
        } else if (_isset(pinB) && ab_ratio < 0.7f) { // should be ~0.5
            // switch phase A and B
            int tmp_pinA = pinA;
            pinA = pinB;
            pinB = tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ib;
            offset_ib = tmp_offsetA;
            gain_a *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        } else if (_isset(pinC) && ac_ratio < 0.7f) { // should be ~0.5
            // switch phase A and C
            int tmp_pinA = pinA;
            pinA = pinC;
            pinC = tmp_pinA;
            float tmp_offsetA = offset_ia;
            offset_ia = offset_ic;
            offset_ic = tmp_offsetA;
            gain_a *= _sign(c.c);
            exit_flag = 2;// signal that pins have been switched
        } else {
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }

    if (_isset(pinB)) {
        // set phase B active and phases A and C down
        driver->setPwm(0, voltage, 0);
        _delay(200);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the current 50 times
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a * 0.6 + 0.4f * c1.a;
            c.b = c.b * 0.6 + 0.4f * c1.b;
            c.c = c.c * 0.6 + 0.4f * c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        float ba_ratio = c.a ? fabs(c.b / c.a) : 0;
        float bc_ratio = c.c ? fabs(c.b / c.c) : 0;
        if (_isset(pinA) && ba_ratio > 1.5f) { // should be ~2
            gain_b *= _sign(c.b);
        } else if (_isset(pinC) && bc_ratio > 1.5f) { // should be ~2
            gain_b *= _sign(c.b);
        } else if (_isset(pinA) && ba_ratio < 0.7f) { // it should be ~0.5
            // switch phase A and B
            int tmp_pinB = pinB;
            pinB = pinA;
            pinA = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ia;
            offset_ia = tmp_offsetB;
            gain_b *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        } else if (_isset(pinC) && bc_ratio < 0.7f) { // should be ~0.5
            // switch phase A and C
            int tmp_pinB = pinB;
            pinB = pinC;
            pinC = tmp_pinB;
            float tmp_offsetB = offset_ib;
            offset_ib = offset_ic;
            offset_ic = tmp_offsetB;
            gain_b *= _sign(c.c);
            exit_flag = 2; // signal that pins have been switched
        } else {
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }

    // if phase C measured
    if (_isset(pinC)) {
        // set phase C active and phases A and B down
        driver->setPwm(0, 0, voltage);
        _delay(200);
        PhaseCurrent_s c = getPhaseCurrents();
        // read the adc voltage 500 times ( arbitrary number )
        for (int i = 0; i < 100; i++) {
            PhaseCurrent_s c1 = getPhaseCurrents();
            c.a = c.a * 0.6 + 0.4f * c1.a;
            c.b = c.b * 0.6 + 0.4f * c1.b;
            c.c = c.c * 0.6 + 0.4f * c1.c;
            _delay(3);
        }
        driver->setPwm(0, 0, 0);
        float ca_ratio = c.a ? fabs(c.c / c.a) : 0;
        float cb_ratio = c.b ? fabs(c.c / c.b) : 0;
        if (_isset(pinA) && ca_ratio > 1.5f) { // should be ~2
            gain_c *= _sign(c.c);
        } else if (_isset(pinB) && cb_ratio > 1.5f) { // should be ~2
            gain_c *= _sign(c.c);
        } else if (_isset(pinA) && ca_ratio < 0.7f) { // it should be ~0.5
            // switch phase A and C
            int tmp_pinC = pinC;
            pinC = pinA;
            pinA = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ia;
            offset_ia = tmp_offsetC;
            gain_c *= _sign(c.a);
            exit_flag = 2; // signal that pins have been switched
        } else if (_isset(pinB) && cb_ratio < 0.7f) { // should be ~0.5
            // switch phase B and C
            int tmp_pinC = pinC;
            pinC = pinB;
            pinB = tmp_pinC;
            float tmp_offsetC = offset_ic;
            offset_ic = offset_ib;
            offset_ib = tmp_offsetC;
            gain_c *= _sign(c.b);
            exit_flag = 2; // signal that pins have been switched
        } else {
            // error in current sense - phase either not measured or bad connection
            return 0;
        }
    }

    if (gain_a < 0 || gain_b < 0 || gain_c < 0) exit_flag += 2;
    // exit flag is either
    // 0 - fail
    // 1 - success and nothing changed
    // 2 - success but pins reconfigured
    // 3 - success but gains inverted
    // 4 - success but pins reconfigured and gains inverted

    return exit_flag;
}
