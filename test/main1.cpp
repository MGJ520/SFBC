// 更换蓝牙库
#include <Arduino.h> // 引入Arduino基础库
#include <SimpleFOC.h> // 引入SimpleFOC库，用于无刷电机控制
#include <MPU6050_tockn.h> // 引入MPU6050传感器库，用于获取姿态信息

#define gyroY_OFF -0.56 // 定义陀螺仪Y轴偏移量，用于校准

// 定义两个I2C通信接口，用于连接不同的传感器
TwoWire I2Cone = TwoWire(0); // I2C接口1
TwoWire I2Ctwo = TwoWire(1); // I2C接口2

// 定义两个磁性传感器，用于无刷电机的编码器反馈
MagneticSensorI2C sensor0 = MagneticSensorI2C(AS5600_I2C); // 传感器0
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C); // 传感器1

// 声明一个函数，用于将加速度计数据转换为旋转角度，供卡尔曼滤波使用
double acc2rotation(double x, double z);

// 初始化MPU6050传感器，连接到I2C接口2
MPU6050 mpu60501(I2Ctwo);

// 定义两个无刷电机及其驱动器
BLDCMotor motor0 = BLDCMotor(7); // 电机0
BLDCDriver3PWM driver0 = BLDCDriver3PWM(35, 13, 14); // 电机0的驱动器

BLDCMotor motor1 = BLDCMotor(7); // 电机1
BLDCDriver3PWM driver1  = BLDCDriver3PWM(12, 11, 10); // 电机1的驱动器

// 定义自稳PID控制器，初始参数：P=0.8, I=5.5, D=0, ramp=100000, limit=8
PIDController pid_stb = PIDController(0.8, 5.5, 0, 100000, 8);

// 定义速度PID控制器，初始参数：P=1.5, I=0, D=0.01, ramp=10000, limit=π/4
PIDController pid_vel = PIDController(1.5, 0, 0.01, 10000, _PI/4);

// 定义速度控制滤波器，滤波时间常数为0.07
LowPassFilter lpf_pitch_cmd = LowPassFilter(0.07);

// 定义油门和转向滤波器，时间常数分别为0.5和0.1
LowPassFilter lpf_throttle = LowPassFilter(0.5); // 油门滤波器
LowPassFilter lpf_steering = LowPassFilter(0.1); // 转向滤波器

// 定义控制变量
float steering = 0; // 转向值
float throttle = 0; // 油门值
float new_steering; // 新的转向值
float new_throttle; // 新的油门值
float max_throttle = 20; // 最大油门值，单位为rad/s
float max_steering = 0.8; // 最大转向值，单位为V
float Offset_parameters = -9.3; // 偏置参数，用于调整控制逻辑
int Checkcomma; // 用于判断是否含有逗号分隔的变量

// 初始化Commander对象，用于接收和解析串口指令
Commander commander = Commander(Serial);

// 定义回调函数，用于处理Commander接收到的指令
void cntStab(char* cmd) {  commander.pid(&pid_stb, cmd);} // 处理自稳PID指令
void cntMove(char* cmd) {  commander.pid(&pid_vel, cmd);} // 处理速度PID指令
void lpfPitch(char* cmd) {  commander.lpf(&lpf_pitch_cmd, cmd);} // 处理速度控制滤波指令
void lpfSteering(char* cmd) {  commander.lpf(&lpf_steering, cmd);} // 处理转向滤波指令
void lpfThrottle(char* cmd) {  commander.lpf(&lpf_throttle, cmd);} // 处理油门滤波指令

void setup(){
    // 初始化系统
    SetupCarSystem(); // 调用SetupCarSystem函数初始化车辆系统
    // 开启按键与电压检测任务
    PowerAndButton.startTask(); // 启动电源和按钮检测任务

    _delay(750); // 延时750ms，等待系统稳定

    // 初始化I2C接口
    I2Cone.begin(37,36, 400000UL); // 初始化I2C接口1
    I2Ctwo.begin(8,9, 400000UL); // 初始化I2C接口2

    // 初始化磁性传感器
    sensor0.init(&I2Cone); // 初始化传感器0
    sensor1.init(&I2Ctwo); // 初始化传感器1

    // 初始化MPU6050传感器
    mpu60501.begin(); // 初始化MPU6050
    mpu60501.calcGyroOffsets(true); // 计算陀螺仪偏移量

    // 将电机与传感器关联
    motor0.linkSensor(&sensor0); // 将电机0与传感器0关联
    motor1.linkSensor(&sensor1); // 将电机1与传感器1关联

    // 设置电机速度环PID参数
    motor1.PID_velocity.P = 0.05; // 电机1速度环P值
    motor1.PID_velocity.I = 1; // 电机1速度环I值
    motor1.PID_velocity.D = 0; // 电机1速度环D值

    motor0.PID_velocity.P = 0.05; // 电机0速度环P值
    motor0.PID_velocity.I = 1; // 电机0速度环I值
    motor0.PID_velocity.D = 0; // 电机0速度环D值

    // 设置电机电压传感器参数
    motor0.voltage_sensor_align = 6; // 电机0电压传感器对齐参数
    driver0.voltage_power_supply = 8; // 电机0驱动器电源电压
    driver0.init(); // 初始化电机0驱动器
    motor0.linkDriver(&driver0); // 将电机0与驱动器0关联

    motor1.voltage_sensor_align = 6; // 电机1电压传感器对齐参数
    driver1.voltage_power_supply = 8; // 电机1驱动器电源电压
    driver1.init(); // 初始化电机1驱动器
    motor1.linkDriver(&driver1); // 将电机1与驱动器1关联

    // 设置电机控制模式
    motor0.torque_controller = TorqueControlType::voltage; // 电机0采用电压控制模式
    motor1.torque_controller = TorqueControlType::voltage; // 电机1采用电压控制模式
    motor0.controller = MotionControlType::torque; // 电机0采用力矩控制模式
    motor1.controller = MotionControlType::torque; // 电机1采用力矩控制模式

    // 设置电机电压限制
    motor0.voltage_limit=1; // 电机0电压限制
    motor1.voltage_limit=1; // 电机1电压限制

    // 启用电机监控功能
    motor1.useMonitoring(Serial); // 启用电机1监控
    motor0.useMonitoring(Serial); // 启用电机0监控

    // 初始化电机
    motor1.init(); // 初始化电机1
    motor0.init(); // 初始化电机0

    // 初始化FOC控制
    motor1.initFOC(); // 初始化电机1FOC控制
    motor0.initFOC(); // 初始化电机0FOC控制

    // 将回调函数添加到Commander对象中
    commander.add('A', cntStab, "pid stab"); // 添加自稳PID指令回调
    commander.add('B', cntMove, "pid vel"); // 添加速度PID指令回调
    commander.add('C', lpfThrottle, "lpf vel command"); // 添加油门滤波指令回调
    commander.add('D', lpfPitch, "lpf throttle"); // 添加速度控制滤波指令回调
    commander.add('E', lpfSteering, "lpf steering"); // 添加转向滤波指令回调
}

void loop(){
    // 运行FOC控制循环
    motor1.loopFOC(); // 电机1FOC控制循环
    motor0.loopFOC(); // 电机0FOC控制循环

    // 执行电机的运动控制
    motor1.move(); // 控制电机1运动
    motor0.move(); // 控制电机0运动

    // 更新MPU6050传感器数据
    mpu60501.update(); // 更新MPU6050数据

    // 获取MPU6050计算的俯仰角
    double mpu_pitch = mpu60501.getAngleY(); // 获取Y轴的俯仰角

    // 计算目标俯仰角
    float target_pitch = lpf_pitch_cmd(pid_vel((motor0.shaft_velocity + motor1.shaft_velocity) / 2 - lpf_throttle(throttle)));
    // 说明：
    // 1. (motor0.shaft_velocity + motor1.shaft_velocity) / 2：计算两个电机的平均转速
    // 2. lpf_throttle(throttle)：对油门值进行滤波
    // 3. pid_vel(...)：使用速度PID控制器计算目标速度误差
    // 4. lpf_pitch_cmd(...)：对目标速度误差进行滤波，得到目标俯仰角

    // 打印目标俯仰角，用于调试
    Serial.println(target_pitch);

    // 计算电压控制信号
    float voltage_control = pid_stb(Offset_parameters - mpu_pitch + target_pitch);
    // 说明：
    // 1. Offset_parameters：偏置参数，用于调整控制逻辑
    // 2. mpu_pitch：当前俯仰角
    // 3. target_pitch：目标俯仰角
    // 4. pid_stb(...)：使用自稳PID控制器计算电压控制信号

    // 计算转向调整值
    float steering_adj = lpf_steering(steering);
    // 说明：
    // 1. steering：当前转向值
    // 2. lpf_steering(...)：对转向值进行滤波，得到转向调整值

    // 设置电机的目标电压
    motor0.target = voltage_control - steering_adj; // 电机0的目标电压
    motor1.target = voltage_control + steering_adj; // 电机1的目标电压

    // 运行Commander，处理串口指令
    commander.run(); // 检查串口指令并执行相应的回调函数
}