#include <Arduino.h>
#include "init_System.h"
#include "power/BatteryAndButton.h"
#include "buzzer/BuzzerSound.h"
#include "foc/foc_drive.h"
#include "control/control.h"
#include "eeprom/eeprom.h"

// 常量定义
constexpr uint32_t SERIAL_BAUDRATE = 115200; // 定义串口通信的波特率，这里是115200
constexpr int SHUTDOWN_DELAY_MS = 1000; // 定义关机倒计时的延迟时间，单位是毫秒（1秒）
constexpr int ERROR_REPEAT_COUNT = 10; // 定义错误重复次数，用于关机倒计时的次数

// 系统状态
bool systemInitialized = true; // 用于标记系统是否初始化成功的标志，默认为true

// 函数声明
void PrintTestResult(); // 声明打印测试结果的函数
void LogInitialization(const char* module, bool result); // 声明记录初始化结果的函数
void EmitShutdownSequence(); // 声明执行关机序列的函数



// 初始化系统
void SetupCarSystem() {
    // 初始化串口通信
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("\n================ 自检开始 =================");


    // 检查电池模块是否初始化成功
    LogInitialization("电池模块", PowerAndButton.init());


    // 检查驱动模块是否初始化成功
    //LogInitialization("驱动模块", BalanceDriver.init());



    Serial.println("\n================ 自检完成 =================");
    PrintTestResult(); // 打印最终的自检结果
}







// 记录模块初始化的结果
void LogInitialization(const char* module, bool result) {
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

void Ready_Time() {
    if (motor_A.motor_status == motor_ready && motor_B.motor_status == motor_ready && Ready) {
        buzzer.play(S_SIREN);
        Ready = false;
        MotorClose();
        if (First_Ready) {

            if (_isset(cs_A.pinA)) motor_data.A_Offset_ia = cs_A.offset_ia;
            if (_isset(cs_A.pinB)) motor_data.A_Offset_ib = cs_A.offset_ib;
            if (_isset(cs_A.pinC)) motor_data.A_Offset_ic = cs_A.offset_ic;

            if (_isset(cs_B.pinA)) motor_data.B_Offset_ia = cs_B.offset_ia;
            if (_isset(cs_B.pinB)) motor_data.B_Offset_ib = cs_B.offset_ib;
            if (_isset(cs_B.pinC)) motor_data.B_Offset_ic = cs_B.offset_ic;
            eeprom_writer_task_run();
        }
    }
}
