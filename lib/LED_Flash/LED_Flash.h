//
// Created by MGJ on 2025/4/26.
//

#ifndef FOC_CODE_LED_FLASH_H
#define FOC_CODE_LED_FLASH_H

#include <Arduino.h>

class LED {
public:
    LED(uint8_t pin); // 构造函数

    void begin();     // 初始化LED引脚
    void off();       // 关闭LED
    void on();        // 开启LED
    void blink(uint16_t interval = 500);           // 单闪模式
    void doubleBlink(uint16_t speed = 200, uint16_t pause = 800); // 双闪模式
    void update();    // 需要放在主循环中持续更新

private:
    uint8_t _pin;
    enum class Mode { Off, On, Blink, DoubleBlink };
    Mode _mode = Mode::Off;

    // 状态控制相关
    unsigned long _lastMillis = 0;
    bool _ledState = false;

    // 单闪参数
    uint16_t _blinkInterval = 500;

    // 双闪参数和状态机
    uint16_t _doubleBlinkSpeed = 200;
    uint16_t _doubleBlinkPause = 800;
    uint8_t _blinkCounter = 0;
};

class LEDManager {
public:
    static void addLED(LED* led);
    static void updateAll();
    static void offALL();
private:
    static constexpr uint8_t MAX_LEDS = 10;
    static LED* instances[MAX_LEDS];
    static uint8_t count;


};


#endif //FOC_CODE_LED_H
