
#include "init_System.h"


#include "power/BatteryAndButton.h"
#include "nvs/nvs.h"
#include "foc/foc_drive.h"
#include "led/led.h"
#include "buzzer/BuzzerSound.h"
#include "control/control.h"
#include "pid/pid_adjust.h"

boolean Ready = true;

constexpr int SHUTDOWN_DELAY_MS = 1000; // 定义关机倒计时的延迟时间，单位是毫秒（1秒）
constexpr int ERROR_REPEAT_COUNT = 10; // 定义错误重复次数，用于关机倒计时的次数


// 系统状态
bool systemInitialized = true; // 用于标记系统是否初始化成功的标志，默认为true


// 初始化系统
void SetupCarSystem() {
    //串口初始化
    Serial.begin(SERIAL_BAUDRATE);

    Serial.println("\n================ 自检开始 =================");

    //LED初始化
    init_led();

    //电池源初始化
    LogInitialization("电池模块", PowerAndButton.init());

    //nvs初始化
    //Read_Data();

    //I2C初始化
    I2C_A.begin(SDA_A_GPIO, SCL_A_GPIO, 400000UL);
    I2C_B.begin(SDA_B_GPIO, SCL_B_GPIO, 400000UL);

    //MPU初始化
    mpu6050.begin();

    //FOC参数初始化
    Foc_Parameters_init();

    //PID参数初始化
    PID_parameters_Init();

}


void Init_Loop() {
    if (motor_A.motor_status == motor_ready && motor_B.motor_status == motor_ready && Ready) {
        Ready = false;
        buzzer.play(S_SIREN);
        MotorClose();
        Serial.println("\n================ 自检完成 =================");
        PrintTestResult(); // 打印最终的自检结果
    } else if (motor_A.motor_status == motor_init_failed && motor_B.motor_status == motor_init_failed && Ready
    ||motor_A.motor_status == motor_init_failed && motor_B.motor_status == motor_ready && Ready
    ||motor_A.motor_status == motor_ready && motor_B.motor_status == motor_init_failed && Ready
    ) {
        systemInitialized= false;
        Serial.println("\n================ 自检完成 =================");
        PrintTestResult(); // 打印最终的自检结果
    }

}


// 记录模块初始化的结果
void LogInitialization(const char *module, bool result) {
    Serial.printf("\n\n[初始化] %s - ", module); // 打印模块名称
    Serial.println(result ? "成功\n" : "失败!\n"); // 根据初始化结果打印成功或失败

    // 如果初始化失败
    if (!result) {
        systemInitialized = false; // 将系统初始化标志设置为false
        Serial.printf("[错误] %s初始化失败，请检查硬件连接\n", module); // 提示用户检查硬件连接
    }
}


// 打印测试结果
void PrintTestResult() {
    Serial.println("\n================ 自检结果 ================="); // 打印自检结果的标题

    // 如果系统初始化失败
    if (!systemInitialized) {
        Serial.println("[严重错误] 系统初始化失败，即将关机..."); // 提示系统初始化失败并准备关机
        EmitShutdownSequence(); // 执行关机序列
        balanceCarPowerOff(); // 关闭平衡车电源（假设这是关闭电源的函数）
    } else {
        Serial.println("[状态] 系统硬件功能正常,准备就绪"); // 提示系统初始化成功，硬件功能正常
    }

    Serial.println("==========================================\n"); // 打印分隔线
}


// 执行关机序列
void EmitShutdownSequence() {
    // 倒计时关机
    for (int i = ERROR_REPEAT_COUNT; i > 0; --i) { // 从ERROR_REPEAT_COUNT开始倒计时
        Serial.printf("[关机] 剩余倒计时 %d 秒...\n", i); // 打印剩余倒计时时间
        delay(SHUTDOWN_DELAY_MS); // 等待1秒
    }
}


