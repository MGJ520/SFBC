#include "BatteryAndButton.h"

#include "buzzer/BuzzerSound.h"
#include "OneButton.h"
#include "led/led.h"

// 定义一个OneButton对象，用于处理按钮事件，绑定到BUTTON_PIN引脚，并启用消抖功能
OneButton bnt(BUTTON_PIN, true);

#ifdef BUTTON_WIFI_GPIO
OneButton bnt_wifi(BUTTON_WIFI_GPIO, true);
#endif
// 定义一个变量用于存储电池电压值
uint16_t batteryVoltage_raw = 0;

uint16_t battery_percent = 0;

float vcc_voltage_out = 0;

uint16_t battery_low = 0;

uint16_t s_count = 500;

// 预定义的电压和电量百分比数组（全局常量）
const double voltages[] = {6.9, 7.36, 7.48, 7.58, 7.74, 7.86, 8.4};

const int percentages[] = {5, 10, 20, 40, 60, 80, 100};

// 编译时静态断言确保数组长度一致
static_assert(
        sizeof(voltages) / sizeof(voltages[0]) == sizeof(percentages) / sizeof(percentages[0]),
        "Voltage and percentage arrays must have equal length");

double calculate_battery_percentage(double voltage) {
    const int numEntries = sizeof(voltages) / sizeof(voltages[0]);
    // 处理边界情况
    if (voltage <= voltages[0])
        return percentages[0];
    if (voltage >= voltages[numEntries - 1])
        return percentages[numEntries - 1];

    // 查找电压所在区间
    int upperIndex = 0;
    while (upperIndex < numEntries && voltages[upperIndex] <= voltage) {
        upperIndex++;
    }
    int lowerIndex = upperIndex - 1;

    // 计算线性插值
    const double voltageRange = voltages[upperIndex] - voltages[lowerIndex];
    const double percentRange = percentages[upperIndex] - percentages[lowerIndex];
    const double ratio = (voltage - voltages[lowerIndex]) / voltageRange;

    return percentages[lowerIndex] + ratio * percentRange;
}

// 定义一个函数用于开启平衡车电源
void balanceCarPowerOn() {
    // 将电源使能引脚设置为高电平，开启电源
    digitalWrite(POWEREN_PIN, HIGH);
}

// 定义一个函数用于关闭平衡车电源
void balanceCarPowerOff() {
    if (battery_low == 0) {
        // 播放关机音效
        // buzzer.play(powerOffID);
        battery_low = 1;
    }

#ifdef LED_1_GPIO
    R_LED.on();
#endif

    // 将电源使能引脚设置为低电平，关闭电源
    digitalWrite(POWEREN_PIN, LOW);
}

// 处理电池不安全状态
void BatteryVoltageNotSafety() {
//    if (battery_low == 0) {
//        // 播放警报音效
//        buzzer.play(S_SIREN);
//    }
    // 关闭平衡车电源
    balanceCarPowerOff();
}

// 定义一个函数用于检查电池电压是否安全
void CheckBatteryVoltageForSafety() {
    // 通过模拟输入引脚读取电池电压值
    batteryVoltage_raw = analogRead(BAT_ADC_PIN);

    // 计算的ADC电压
    double adc_voltage_out = ((double) batteryVoltage_raw / 4095) * voltage_3v3;
    vcc_voltage_out = adc_voltage_out * (((resistance_vcc + resistance_gnd) / resistance_gnd) + resistance_fix);

    // 计算的百分比
    battery_percent = (uint16_t) calculate_battery_percentage(vcc_voltage_out);

    if (batteryVoltage_raw < 2000) {
        if (++s_count>500)
        {
            s_count=0;
            Serial.println("[电池-动作]:已触发保护,过载或电池电压低");
            Serial.print("\n[电池]:原始数据: ");
            Serial.println(batteryVoltage_raw);
            Serial.print("[电池]:计算ADC电压: ");
            Serial.println(adc_voltage_out);
            Serial.print("[电池]:计算VCC电压: ");
            Serial.println(vcc_voltage_out);
            Serial.print("[电池]:计算电池电量: ");
            Serial.println(battery_percent);

        }
        BatteryVoltageNotSafety();
    }
}



[[noreturn]] void ButtonBatteryTask(void *pvParameters) {
    while (true) {
        bnt.tick();
#ifdef BUTTON_WIFI_GPIO
        bnt_wifi.tick();
#endif
        // 任务延时5ms
        vTaskDelay(5);
        // 检查电池电压是否安全
        CheckBatteryVoltageForSafety();
    }
};



boolean PWR_AND_BNT::init(void) {

#ifdef PWM_MPU_GPIO
    pinMode(PWM_MPU_GPIO, OUTPUT);
    digitalWrite(PWM_MPU_GPIO, LOW);
#endif

    // 设置电池电压检测引脚为输入模式
    pinMode(BAT_ADC_PIN, INPUT);

    // 设置电源使能引脚为输出模式
    pinMode(POWEREN_PIN, OUTPUT);

    // 开启平衡车电源
    balanceCarPowerOn();

    delay(100);

    // 初始化蜂鸣器，绑定到BUZZER_PIN引脚
    buzzer.Init(BUZZER_PIN);

    // 重置按钮对象的状态
    bnt.reset();

#ifdef BUTTON_WIFI_GPIO
    bnt_wifi.reset();
#endif

    // 为按钮绑定长按事件
    // bnt.attachLongPressStart(balanceCarPowerOff);

    // 注释掉的代码：绑定双击事件
    // bnt.attachDoubleClick(turnOffRGB);

    // 为按钮绑定短按事件
    bnt.attachClick(balanceCarPowerOff);

#ifdef BUTTON_WIFI_GPIO
    bnt_wifi.attachClick(balanceCarPowerOff);
#endif

    CheckBatteryVoltageForSafety();

    Serial.print("\n[电池]:原始数据: ");
    Serial.println(batteryVoltage_raw);
    Serial.print("[电池]:计算VCC电压: ");
    Serial.println(vcc_voltage_out);
    Serial.print("[电池]:计算电池电量: ");
    Serial.println(battery_percent);

    delay(100);

    if (vcc_voltage_out < 0 || vcc_voltage_out > 13) {
        Serial.print("[电池]:电压范围错误");
        return false;
    }

    if (vcc_voltage_out < 6.2) {
        Serial.print("[电池]:低电压");
        return false;
    }

    Serial.print("[电池]:电池电压正常");

    //返回检查结果
    return true;
}


PWR_AND_BNT PowerAndButton;
