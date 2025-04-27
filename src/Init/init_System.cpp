
#include "init_System.h"


#include "power/BatteryAndButton.h"
#include "nvs/nvs.h"
#include "foc/foc_drive.h"
#include "led/led.h"
#include "buzzer/BuzzerSound.h"
#include "control/control.h"
#include "pid/pid_adjust.h"
#include "I2C/I2C_Manage.h"
#include "web/web.h"


// 定义一个字符串数组，存储多个硬件版本
const char* hardwareVersions[] = {
        "ESP32_S3_WROOM",
        "ESP32_S3_CHIP",
        "ESP32_S3_CHIP_MPU"
};



//是否启动完成
bool Ready = true;

// 用于标记系统是否初始化成功的标志，默认为true
bool systemInitialized = true;

// 关机倒计时
constexpr int ERROR_REPEAT_COUNT = 10;



void PrintSystemVer()
{
    Serial.println("\n================  SYSTEM INFO  =================");
    // 输出编译时间
    Serial.print("Compile    Time:");
    Serial.print(__DATE__);  // 编译日期
    Serial.print(" ");
    Serial.println(__TIME__);  // 编译时间

    // 输出软件版本
    Serial.print("Software Version: ");
    Serial.println(SOFTWARE_VERSION);

    // 输出硬件版本
    Serial.print("Hardware Version: ");
    Serial.println(hardwareVersions[HARDWARE_VERSION]);

}
// 初始化系统
void SetupCarSystem() {
    //串口初始化
    Serial.begin(SERIAL_BAUDRATE);

    //输出系统版本
    PrintSystemVer();

    Serial.println("\n================ 自检开始 =================");

    //LED初始化
    init_led();

    //电池源初始化
    LogInitialization("电池模块", PowerAndButton.init());

    //nvs初始化
    //Read_Data();

    //I2C初始化
    LogInitialization("I2C模块", I2C_init());

    //MPU初始化
    LogInitialization("MPU6050模块", mpu6050.begin());

    //编码器
    LogInitialization("AS5600模块", check_encoder());

    //Wifi初始化
    wifi_init();

    //FOC参数初始化
    Foc_Parameters_init();

    //PID参数初始化
    PID_parameters_Init();

    //foc初始化
    if (!systemInitialized)
    {
        //FOC,如果前面不通过，启动foc比较危险
        Serial.println("[FOC]: 跳过电机初始化启动,请你先解决前面的硬件错误!");
        LogInitialization("FOC驱动", false);
        // 打印最终的自检结果
       // PrintTestResult();
    }

}


void Init_Loop() {
    if (motor_A.motor_status == motor_ready && motor_B.motor_status == motor_ready && Ready) {
        Ready = false;
        //提示启动完成
        buzzer.play(S_SIREN);
        //启动默认关闭电机
        MotorClose();
        // 打印最终的自检结果
        PrintTestResult();
    } else if (motor_A.motor_status == motor_init_failed && motor_B.motor_status == motor_init_failed && Ready
    ||motor_A.motor_status == motor_init_failed && motor_B.motor_status == motor_ready && Ready
    ||motor_A.motor_status == motor_ready && motor_B.motor_status == motor_init_failed && Ready
    ) {
        systemInitialized= false;
        // 打印最终的自检结果
        PrintTestResult();
    }

}


// 记录模块初始化的结果
void LogInitialization(const char *module, bool result) {
    Serial.printf("\n[初始化] %s - ", module); // 打印模块名称
    Serial.println(result ? "成功" : "失败,请检查硬件连接!");
    // 如果初始化失败
    if (!result) {
        systemInitialized = false;
    }
    Serial.println("-------------------------------------------");
}


// 打印测试结果
void PrintTestResult() {
    Serial.println("\n================ 自检完成 =================");
    // 如果系统初始化失败
    if (!systemInitialized) {
        Serial.println("[硬件错误] 系统初始化失败，即将关机...");
        Serial.println("==========================================\n");
        // 执行关机序列
        EmitShutdownSequence();
        // 关闭平衡车电源
        balanceCarPowerOff();
        //如果没有关机就重启
        esp_restart();
    } else {
        Serial.println("[状态] 系统硬件功能正常,准备就绪");
        Serial.println("==========================================\n");
    }

}


// 执行关机序列
void EmitShutdownSequence() {
    for (int i = ERROR_REPEAT_COUNT; i > 0; --i) {
        Serial.printf("[关机] 剩余倒计时 %d 秒...\n", i);
        delay(1000); // 等待1秒
    }
}


