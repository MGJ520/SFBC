#ifndef BatteryAndButton_h
#define BatteryAndButton_h

#include <Arduino.h>

// ----------------------
#define BUTTON_PIN     BUTTON_DIS_GPIO         // 按键引脚
#define BUZZER_PIN     BEEP_GPIO        // 蜂鸣器引脚
#define POWEREN_PIN    PWR_EN_GPIO        // 电源引脚
#define BAT_ADC_PIN    BAT_ADC_GPIO         // 电源电压引脚

//#define voltage_3v3    3.28   // 3.3V供电校准
//#define resistance_fix (0.51)  //电阻值修正


#define voltage_3v3    3.295   // 3.3V供电校准
#define resistance_fix (0.45)  //电阻值修正


#define resistance_vcc 20     // 电阻值K（VCC端）
#define resistance_gnd 10     // 电阻值K（GND端）

//--------------------- SOUND ID ---------------------
#define changeColorID  11
#define powerOnID      4
#define powerOffID     5
#define powerOffledID  1

// ---------------------- button class -------------------------
class PWR_AND_BNT
{
  public:
    void startTask();
    boolean init();
};

void balanceCarPowerOn();
void balanceCarPowerOff();

extern PWR_AND_BNT PowerAndButton;
extern uint16_t batteryVoltage_raw;
extern float vcc_voltage_out;
#endif