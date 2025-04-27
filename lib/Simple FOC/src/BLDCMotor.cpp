#include "BLDCMotor.h"
#include "./communication/SimpleFOCDebug.h"


// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
// each is 60 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_120_map[6][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,_HIGH_IMPEDANCE,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,_HIGH_IMPEDANCE,-1} 
};

// see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
// each is 30 degrees with values for 3 phases of 1=positive -1=negative 0=high-z
int trap_150_map[12][3] = {
    {_HIGH_IMPEDANCE,1,-1},
    {-1,1,-1},
    {-1,1,_HIGH_IMPEDANCE},
    {-1,1,1},
    {-1,_HIGH_IMPEDANCE,1},
    {-1,-1,1},
    {_HIGH_IMPEDANCE,-1,1},
    {1,-1,1},
    {1,-1,_HIGH_IMPEDANCE},
    {1,-1,-1},
    {1,_HIGH_IMPEDANCE,-1},
    {1,1,-1} 
};

// BLDCMotor(int pp, float R, float KV, float L)
// - pp            - 极对数
// - R             - 电机相电阻
// - KV            - 电机KV值（每伏特转速）
// - L             - 电机相电感

BLDCMotor::BLDCMotor(int pp, float _R, float _KV, float _inductance)
        : FOCMotor()
{
    // 保存极对数
    pole_pairs = pp;
    // 保存相电阻值
    phase_resistance = _R;

    // 保存反电动势常数 KV = 1/KV
    // 乘以1/sqrt(2) - 取均方根值
    KV_rating = NOT_SET;
    if (_isset(_KV))
        KV_rating = _KV * _SQRT2;

    // 保存相电感值
    phase_inductance = _inductance;

    // 默认的转矩控制类型为电压控制
    torque_controller = TorqueControlType::voltage;
}

/**
	Link the driver which controls the motor
*/
void BLDCMotor::linkDriver(BLDCDriver* _driver) {
  driver = _driver;
}

// 初始化硬件引脚
void BLDCMotor::init() {
    // 检查驱动器是否已初始化
    if (!driver || !driver->initialized) {
        motor_status = FOCMotorStatus::motor_init_failed; // 设置电机状态为初始化失败
        SIMPLEFOC_DEBUG("MOT: 驱动器未初始化，无法初始化电机"); // 调试信息：驱动器未初始化，无法初始化电机
        return;
    }
    motor_status = FOCMotorStatus::motor_initializing; // 设置电机状态为正在初始化
    SIMPLEFOC_DEBUG("MOT: 开始初始化电机"); // 调试信息：开始初始化电机

    // 检查电压限制配置是否合理
    if(voltage_limit > driver->voltage_limit)
        voltage_limit =  driver->voltage_limit; // 如果设置的电压限制大于驱动器的电压限制，则将其调整为驱动器的电压限制
    // 限制传感器对齐时的电压
    if(voltage_sensor_align > voltage_limit)
        voltage_sensor_align = voltage_limit; // 如果传感器对齐电压大于电压限制，则将其调整为电压限制

    // 更新控制器的限制
    if(current_sense){
        // 如果使用电流检测，则电流控制环控制电压
        PID_current_q.limit = voltage_limit; // 设置电流控制环的上限为电压限制
        PID_current_d.limit = voltage_limit; // 设置电流控制环的下限为电压限制
    }
    if(_isset(phase_resistance) || torque_controller != TorqueControlType::voltage){
        // 如果已设置相电阻，或者扭矩控制器不是电压模式，则速度控制环控制电流
        PID_velocity.limit = current_limit; // 设置速度控制环的限制为电流限制
    }else{
        // 否则，速度控制环控制电压
        PID_velocity.limit = voltage_limit; // 设置速度控制环的限制为电压限制
    }
    P_angle.limit = velocity_limit; // 设置角度控制环的限制为速度限制

    // 如果使用开环控制且方向未设置，则默认设置为顺时针方向
    if ((controller == MotionControlType::angle_openloop
         || controller == MotionControlType::velocity_openloop)
        && (sensor_direction == Direction::UNKNOWN)) {
        sensor_direction = Direction::CW; // 设置传感器方向为顺时针
    }

    _delay(200); // 延时500毫秒
    // 启动电机
    SIMPLEFOC_DEBUG("MOT: 启用驱动器."); // 调试信息：启用驱动器
    enable(); // 启用电机驱动器
    _delay(200); // 延时500毫秒
    motor_status = FOCMotorStatus::motor_uncalibrated; // 设置电机状态为未校准
}

// disable motor driver
void BLDCMotor::disable()
{
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // disable the driver
  driver->disable();
  // motor status update
  enabled = 0;
}
// enable motor driver
void BLDCMotor::enable()
{
  // enable the driver
  driver->enable();
  // set zero to PWM
  driver->setPwm(0, 0, 0);
  // motor status update
  enabled = 1;
}
/**
  FOC 功能
*/
// FOC 初始化函数
int BLDCMotor::initFOC() {
    int exit_flag = 1; // 初始化退出标志为 1（表示成功）

    motor_status = FOCMotorStatus::motor_calibrating; // 设置电机状态为校准中

    // 如果需要，则对电机进行对齐
    // 对于编码器，对齐是必要的！
    // 传感器和电机对齐 - 可以通过设置 motor.sensor_direction 和 motor.zero_electric_angle 来跳过
    _delay(50); // 延时 500 毫秒
    if (sensor) { // 如果有传感器
        exit_flag *= alignSensor(); // 调用 alignSensor 函数进行传感器对齐，并将返回值与 exit_flag 相乘
        // 添加了转轴角度更新
        sensor->update(); // 更新传感器数据
        shaft_angle = shaftAngle(); // 获取转轴角度
    } else {
        exit_flag = 0; // 如果没有传感器，则无法进行 FOC，将退出标志设置为 0
        SIMPLEFOC_DEBUG("MOT: 没有编码器."); // 调试信息：没有传感器
    }

    // 对电流传感器进行对齐 - 可以跳过
    // 检查驱动器相位是否与电流检测相位一致
    // 并检查测量方向
    _delay(100); // 延时 500 毫秒
    if (exit_flag) { // 如果之前的对齐操作成功
        if (current_sense) { // 如果有电流检测
            if (!current_sense->initialized) { // 如果电流检测未初始化
                motor_status = FOCMotorStatus::motor_calib_failed; // 设置电机状态为校准失败
                SIMPLEFOC_DEBUG("MOT: 电流检测未初始化"); // 调试信息：电流检测未初始化
                exit_flag = 0; // 将退出标志设置为 0
            } else {
                exit_flag *= alignCurrentSense(); // 调用 alignCurrentSense 函数进行电流检测对齐，并将返回值与 exit_flag 相乘
            }
        } else {
            SIMPLEFOC_DEBUG("MOT: 没有电流传感器."); // 调试信息：没有电流检测
        }
    }

    if (exit_flag) { // 如果所有对齐操作成功
        SIMPLEFOC_DEBUG("MOT: 电机准备就绪."); // 调试信息：电机准备就绪
        motor_status = FOCMotorStatus::motor_ready; // 设置电机状态为准备就绪
    } else {
        SIMPLEFOC_DEBUG("MOT: FOC初始化失败."); // 调试信息：FOC 初始化失败
        motor_status = FOCMotorStatus::motor_calib_failed; // 设置电机状态为校准失败
        disable(); // 禁用电机
    }

    return exit_flag; // 返回退出标志
}

// 校准电机和电流检测相位
int BLDCMotor::alignCurrentSense() {
    int exit_flag = 1; // 初始化退出标志为 1（表示成功）

    SIMPLEFOC_DEBUG("MOT: 准备对齐相电流.");

    // 对齐电流检测和驱动器
    exit_flag = current_sense->driverAlign(voltage_sensor_align); // 调用电流检测的 driverAlign 方法进行对齐
    if (!exit_flag) {
        // 电流检测对齐失败，可能是相位未测量或连接不良
        SIMPLEFOC_DEBUG("MOT: 相电流对齐失败!"); // 调试信息：对齐失败
        exit_flag = 0; // 设置退出标志为 0（失败）
    } else {
        // 输出对齐状态标志
        SIMPLEFOC_DEBUG("MOT: 对齐成功: ", exit_flag); // 调试信息：对齐成功
    }

    return exit_flag > 0; // 返回对齐结果（如果 exit_flag 大于 0，则表示成功）
}
// 编码器对齐到电机的电气零角度
int BLDCMotor::alignSensor() {
    int exit_flag = 1; // 初始化退出标志为 1（表示成功）
    SIMPLEFOC_DEBUG("MOT: 准备对齐编码器..."); // 调试信息：开始对齐传感器

    // 检查传感器是否需要搜索绝对零点
    if (sensor->needsSearch())
        exit_flag = absoluteZeroSearch(); // 调用 absoluteZeroSearch 函数进行绝对零点搜索
    // 如果未找到索引，则停止初始化
    if (!exit_flag)
        return exit_flag;

    // 如果传感器的自然方向未知
    if (sensor_direction == Direction::UNKNOWN) {
        // 寻找自然方向
        // 向前移动一个电气周期
        for (int i = 0; i <= 500; i++) {
            float angle = _3PI_2 + _2PI * i / 500.0f; // 计算目标角度
            setPhaseVoltage(voltage_sensor_align, 0, angle); // 设置相电压
            sensor->update(); // 更新传感器数据
            _delay(2); // 延时 2 毫秒
        }
        // 获取中间角度
        sensor->update();
        float mid_angle = sensor->getAngle(); // 获取当前传感器角度
        // 向后移动一个电气周期
        for (int i = 500; i >= 0; i--) {
            float angle = _3PI_2 + _2PI * i / 500.0f; // 计算目标角度
            setPhaseVoltage(voltage_sensor_align, 0, angle); // 设置相电压
            sensor->update(); // 更新传感器数据
            _delay(2); // 延时 2 毫秒
        }
        sensor->update();
        float end_angle = sensor->getAngle(); // 获取最终角度
        setPhaseVoltage(0, 0, 0); // 停止所有电压
        _delay(150); // 延时 200 毫秒
        // 确定传感器的运动方向
        float moved = fabs(mid_angle - end_angle); // 计算角度变化
        if (moved < MIN_ANGLE_DETECT_MOVEMENT) { // 如果角度变化小于最小可检测值
            SIMPLEFOC_DEBUG("MOT: 未检测到电机运动"); // 调试信息：未检测到运动
            return 0; // 校准失败
        } else if (mid_angle < end_angle) {
            SIMPLEFOC_DEBUG("MOT: 电机方向为--CCW"); // 调试信息：传感器方向为逆时针
            sensor_direction = Direction::CCW; // 设置传感器方向为逆时针
        } else {
            SIMPLEFOC_DEBUG("MOT: 电机方向为--CW"); // 调试信息：传感器方向为顺时针
            sensor_direction = Direction::CW; // 设置传感器方向为顺时针
        }
        // 检查极对数
        if (fabs(moved * pole_pairs - _2PI) > 0.5f) { // 0.5f 是一个任意值，可以根据需要调整

            SIMPLEFOC_DEBUG("PP 检查失败 - 预估的极对数: ", _2PI / moved); // 调试信息：极对数校验失败
        } else {
            SIMPLEFOC_DEBUG("MOT: 极对数校验成功!"); // 调试信息：极对数校验成功
        }
    } else {
        SIMPLEFOC_DEBUG("MOT: 跳过编码器方向校准."); // 调试信息：跳过方向校准
    }

    // 如果电气零角度未知
    if (!_isset(zero_electric_angle)) {
        // 对齐电机和传感器的电气相位
        // 设置角度为 -90 度（270 度 = 3π/2）
        setPhaseVoltage(voltage_sensor_align, 0, _3PI_2); // 设置相电压
        _delay(150); // 延时 700 毫秒
        // 读取传感器数据
        sensor->update();
        // 获取当前电气零角度
        zero_electric_angle = 0;
        zero_electric_angle = electricalAngle(); // 获取电气角度
        // zero_electric_angle =  _normalizeAngle(_electricalAngle(sensor_direction*sensor->getAngle(), pole_pairs));
        _delay(20); // 延时 20 毫秒
        if (monitor_port) {
            SIMPLEFOC_DEBUG("MOT: 零点电角度:", zero_electric_angle); // 调试信息：电气零角度
        }
        // 停止所有电压
        setPhaseVoltage(0, 0, 0);
        _delay(150); // 延时 200 毫秒
    } else {
        SIMPLEFOC_DEBUG("MOT: 跳过零点电角度校准."); // 调试信息：跳过偏移校准
    }
    return exit_flag; // 返回退出标志
}

// 编码器对齐绝对零角度
// - 到索引位置
int BLDCMotor::absoluteZeroSearch() {
  // sensor precision: this is all ok, as the search happens near the 0-angle, where the precision
  //                    of float is sufficient.
  SIMPLEFOC_DEBUG("MOT: 正在探测位置...");
  // search the absolute zero with small velocity
  float limit_vel = velocity_limit;
  float limit_volt = voltage_limit;
  velocity_limit = velocity_index_search;
  voltage_limit = voltage_sensor_align;
  shaft_angle = 0;
  while(sensor->needsSearch() && shaft_angle < _2PI){
    angleOpenloop(1.5f*_2PI);
    // call important for some sensors not to loose count
    // not needed for the search
    sensor->update();
  }
  // disable motor
  setPhaseVoltage(0, 0, 0);
  // reinit the limits
  velocity_limit = limit_vel;
  voltage_limit = limit_volt;
  // check if the zero found
  if(monitor_port){
    if(sensor->needsSearch()) { SIMPLEFOC_DEBUG("MOT: Error: Not found!"); }
    else { SIMPLEFOC_DEBUG("MOT: Success!"); }
  }
  return !sensor->needsSearch();
}

// 迭代函数，循环运行FOC（磁场定向控制）算法，设置电机的电压矢量分量 Uq
// 这个函数运行得越快越好
void BLDCMotor::loopFOC() {
    // 更新传感器数据 —— 即使在开环模式下也要执行，因为用户可能会在不同模式之间切换，
    // 如果不更新，可能会丢失电机的完整旋转圈数信息。
    if (sensor) sensor->update();

    // 如果是开环控制模式（角度开环或速度开环），则直接返回，不执行后续操作
    if (controller == MotionControlType::angle_openloop || controller == MotionControlType::velocity_openloop) return;

    // 如果电机被禁用，则直接返回，不执行后续操作
    if (!enabled) return;

    // 需要先调用 update() 函数
    // 这个函数不会出现数值问题，因为它使用了 Sensor::getMechanicalAngle()，
    // 该函数返回的机械角度值范围是 0 到 2π。
    electrical_angle = electricalAngle();

    // 根据不同的转矩控制类型执行不同的操作
    switch (torque_controller) {
        case TorqueControlType::voltage:
            // 电压控制模式下，实际上不需要执行任何操作
            break;

        case TorqueControlType::dc_current:
            // 如果没有电流检测，则直接返回
            if (!current_sense) return;

            // 读取直流电流的幅值
            current.q = current_sense->getDCCurrent(electrical_angle);

            // 对电流值进行滤波处理
            current.q = LPF_current_q(current.q);

            // 计算相电压的 q 分量
            voltage.q = PID_current_q(current_sp - current.q);

            // 计算 d 分量的电压（用于滞后补偿）
            if (_isset(phase_inductance))
                voltage.d = _constrain(-current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
            else
                voltage.d = 0;
            break;

        case TorqueControlType::foc_current:
            // 如果没有电流检测，则直接返回
            if (!current_sense) return;

            // 读取 dq 分量的电流值
            current = current_sense->getFOCCurrents(electrical_angle);

            // 对电流值进行滤波处理
            current.q = LPF_current_q(current.q);
            current.d = LPF_current_d(current.d);

            // 计算相电压的 dq 分量
            voltage.q = PID_current_q(current_sp - current.q);
            voltage.d = PID_current_d(-current.d);

            // d 分量的电压（滞后补偿）—— 需要进一步验证
            // if (_isset(phase_inductance))
            //   voltage.d = _constrain(voltage.d - current_sp * shaft_velocity * pole_pairs * phase_inductance, -voltage_limit, voltage_limit);
            break;

        default:
            // 没有选择任何转矩控制方式
            SIMPLEFOC_DEBUG("MOT: no torque control selected!");
            break;
    }

    // 设置相电压 —— FOC算法的核心功能
    setPhaseVoltage(voltage.q, voltage.d, electrical_angle);
}

// 迭代函数，运行FOC算法的外环控制
// 此函数的行为由电机的motor.controller变量决定
// 它可以运行角度环、速度环或转矩环
// - 需要迭代调用，这是一个异步函数
// - 如果没有设置目标值，则使用motor.target的值
void BLDCMotor::move(float new_target) {
    // 下采样（可选）
    // 如果计数器未达到下采样阈值，则直接返回，不执行后续操作
    if(motion_cnt++ < motion_downsample) return;
    motion_cnt = 0;

    // 轴角和角速度需要先调用update()函数进行更新
    // 获取轴角
    // TODO 传感器精度：轴角实际上存储了完整的位置，包括完整旋转，作为一个浮点数
    //                        因此，当角度变大时，它并不精确。
    //                        此外，LPF对角度的作用方式是一个精度问题，当切换到2分量表示时，角度-LPF是一个问题
    if( controller!=MotionControlType::angle_openloop && controller!=MotionControlType::velocity_openloop )
        shaft_angle = shaftAngle(); // 即使电机被禁用，也要读取值以保持监控更新，但不在开环模式下
    // 获取角速度  TODO 角速度读取可能也不应该在开环模式下发生？
    shaft_velocity = shaftVelocity(); // 即使电机被禁用，也要读取值以保持监控更新

    // 如果电机被禁用，则什么也不做
    if(!enabled) return;
    // 设置内部目标变量
    if(_isset(new_target)) target = new_target;

    // 如果有KV额定值，计算反电动势电压 U_bemf = vel*(1/KV)
    if (_isset(KV_rating)) voltage_bemf = shaft_velocity/KV_rating/_RPM_TO_RADS;
    // 如果有相电阻且没有电流检测，则估算电机电流
    if(!current_sense && _isset(phase_resistance)) current.q = (voltage.q - voltage_bemf)/phase_resistance;

    // 根据控制模式更新基于电流的电压限制
    switch (controller) {
        case MotionControlType::torque:
            if(torque_controller == TorqueControlType::voltage){ // 如果是电压转矩控制
                if(!_isset(phase_resistance))  voltage.q = target;
                else  voltage.q =  target*phase_resistance + voltage_bemf;
                voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
                // 设置d分量（如果已知电感，则进行滞后补偿）
                if(!_isset(phase_inductance)) voltage.d = 0;
                else voltage.d = _constrain( -target*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
            }else{
                current_sp = target; // 如果是电流/FOC电流转矩控制
            }
            break;
        case MotionControlType::angle:
            // TODO 传感器精度：此计算在数值上不精确。目标值无法在角度较大时表达精确位置。这导致无法在高位置值时命令小的变化。
            //                        要解决此问题，必须以数值精确的方式计算角度差。
            // 角度设定值
            shaft_angle_sp = target;
            // 计算速度设定值
            shaft_velocity_sp = feed_forward_velocity + P_angle( shaft_angle_sp - shaft_angle );
            shaft_velocity_sp = _constrain(shaft_velocity_sp,-velocity_limit, velocity_limit);
            // 计算转矩指令 - 传感器精度：此计算是可接受的，但基于前面计算的不良值
            current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // 如果是电压转矩控制
            // 如果通过电压控制转矩
            if(torque_controller == TorqueControlType::voltage){
                // 如果未提供相电阻，则使用电压
                if(!_isset(phase_resistance))  voltage.q = current_sp;
                else  voltage.q =  _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
                // 设置d分量（如果已知电感，则进行滞后补偿）
                if(!_isset(phase_inductance)) voltage.d = 0;
                else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
            }
            break;
        case MotionControlType::velocity:
            // 速度设定值 - 传感器精度：此计算在数值上是精确的
            shaft_velocity_sp = target;
            // 计算转矩指令
            current_sp = PID_velocity(shaft_velocity_sp - shaft_velocity); // 如果是电流/FOC电流转矩控制
            // 如果通过电压控制转矩
            if(torque_controller == TorqueControlType::voltage){
                // 如果未提供相电阻，则使用电压
                if(!_isset(phase_resistance))  voltage.q = current_sp;
                else  voltage.q = _constrain( current_sp*phase_resistance + voltage_bemf , -voltage_limit, voltage_limit);
                // 设置d分量（如果已知电感，则进行滞后补偿）
                if(!_isset(phase_inductance)) voltage.d = 0;
                else voltage.d = _constrain( -current_sp*shaft_velocity*pole_pairs*phase_inductance, -voltage_limit, voltage_limit);
            }
            break;
        case MotionControlType::velocity_openloop:
            // 开环速度控制 - 传感器精度：此计算在数值上是精确的
            shaft_velocity_sp = target;
            voltage.q = velocityOpenloop(shaft_velocity_sp); // 返回设置到电机的电压
            voltage.d = 0;
            break;
        case MotionControlType::angle_openloop:
            // 开环角度控制 -
            // TODO 传感器精度：此计算在数值上不精确，且存在与闭环版本相同的问题，即在高角度时小设定值变化无法精确控制
            shaft_angle_sp = target;
            voltage.q = angleOpenloop(shaft_angle_sp); // 返回设置到电机的电压
            voltage.d = 0;
            break;
    }
}


// Method using FOC to set Uq and Ud to the motor at the optimal angle
// Function implementing Space Vector PWM and Sine PWM algorithms
//
// Function using sine approximation
// regular sin + cos ~300us    (no memory usage)
// approx  _sin + _cos ~110us  (400Byte ~ 20% of memory)
void BLDCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {

  float center;
  int sector;
  float _ca,_sa;

  switch (foc_modulation)
  {
    case FOCModulationType::Trapezoid_120 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 5
      // determine the sector
      sector = 6 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_120_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_120_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_120_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON);// disable phase if possible
      }else{
        Ua = trap_120_map[sector][0] * Uq + center;
        Ub = trap_120_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF);// disable phase if possible
      }

    break;

    case FOCModulationType::Trapezoid_150 :
      // see https://www.youtube.com/watch?v=InzXA7mWBWE Slide 8
      // determine the sector
      sector = 12 * (_normalizeAngle(angle_el + _PI_6 ) / _2PI); // adding PI/6 to align with other modes
      // centering the voltages around either
      // modulation_centered == true > driver.voltage_limit/2
      // modulation_centered == false > or Adaptable centering, all phases drawn to 0 when Uq=0
      center = modulation_centered ? (driver->voltage_limit)/2 : Uq;

      if(trap_150_map[sector][0]  == _HIGH_IMPEDANCE){
        Ua= center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_OFF, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][1]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_OFF, PhaseState::PHASE_ON); // disable phase if possible
      }else if(trap_150_map[sector][2]  == _HIGH_IMPEDANCE){
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_OFF); // disable phase if possible
      }else{
        Ua = trap_150_map[sector][0] * Uq + center;
        Ub = trap_150_map[sector][1] * Uq + center;
        Uc = trap_150_map[sector][2] * Uq + center;
        driver->setPhaseState(PhaseState::PHASE_ON, PhaseState::PHASE_ON, PhaseState::PHASE_ON); // enable all phases
      }

    break;

    case FOCModulationType::SinePWM :
    case FOCModulationType::SpaceVectorPWM :
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation
      _sincos(angle_el, &_sa, &_ca);

      // Inverse park transform
      Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // Clarke transform
      Ua = Ualpha;
      Ub = -0.5f * Ualpha + _SQRT3_2 * Ubeta;
      Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta;

      center = driver->voltage_limit/2;
      if (foc_modulation == FOCModulationType::SpaceVectorPWM){
        // discussed here: https://community.simplefoc.com/t/embedded-world-2023-stm32-cordic-co-processor/3107/165?u=candas1
        // a bit more info here: https://microchipdeveloper.com/mct5001:which-zsm-is-best
        // Midpoint Clamp
        float Umin = min(Ua, min(Ub, Uc));
        float Umax = max(Ua, max(Ub, Uc));
        center -= (Umax+Umin) / 2;
      } 

      if (!modulation_centered) {
        float Umin = min(Ua, min(Ub, Uc));
        Ua -= Umin;
        Ub -= Umin;
        Uc -= Umin;
      }else{
        Ua += center;
        Ub += center;
        Uc += center;
      }

      break;

  }

  // set the voltages in driver
  driver->setPwm(Ua, Ub, Uc);
}



// Function (iterative) generating open loop movement for target velocity
// - target_velocity - rad/s
// it uses voltage_limit variable
float BLDCMotor::velocityOpenloop(float target_velocity){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to achieve target velocity
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  // for display purposes
  shaft_velocity = target_velocity;

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)){
    Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
    // recalculate the current  
    current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}

// Function (iterative) generating open loop movement towards the target angle
// - target_angle - rad
// it uses voltage_limit and velocity_limit variables
float BLDCMotor::angleOpenloop(float target_angle){
  // get current timestamp
  unsigned long now_us = _micros();
  // calculate the sample time from last call
  float Ts = (now_us - open_loop_timestamp) * 1e-6f;
  // quick fix for strange cases (micros overflow + timestamp not defined)
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  // calculate the necessary angle to move from current position towards target angle
  // with maximal velocity (velocity_limit)
  // TODO sensor precision: this calculation is not numerically precise. The angle can grow to the point
  //                        where small position changes are no longer captured by the precision of floats
  //                        when the total position is large.
  if(abs( target_angle - shaft_angle ) > abs(velocity_limit*Ts)){
    shaft_angle += _sign(target_angle - shaft_angle) * abs( velocity_limit )*Ts;
    shaft_velocity = velocity_limit;
  }else{
    shaft_angle = target_angle;
    shaft_velocity = 0;
  }

  // use voltage limit or current limit
  float Uq = voltage_limit;
  if(_isset(phase_resistance)){
    Uq = _constrain(current_limit*phase_resistance + fabs(voltage_bemf),-voltage_limit, voltage_limit);
    // recalculate the current  
    current.q = (Uq - fabs(voltage_bemf))/phase_resistance;
  }
  // set the maximal allowed voltage (voltage_limit) with the necessary angle
  // sensor precision: this calculation is OK due to the normalisation
  setPhaseVoltage(Uq,  0, _electricalAngle(_normalizeAngle(shaft_angle), pole_pairs));

  // save timestamp for next call
  open_loop_timestamp = now_us;

  return Uq;
}
