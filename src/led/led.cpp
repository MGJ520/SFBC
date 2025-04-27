//
// Created by MGJ on 2025/4/23.
//

#include "led.h"
#include "control/control.h"

#ifdef LED_1_GPIO
LED B_LED(LED_1_GPIO);
LED G_LED(LED_2_GPIO);
LED R_LED(LED_3_GPIO);
#endif


void init_led() {
#ifdef LED_1_GPIO
    LEDManager::addLED(&R_LED);
    LEDManager::addLED(&G_LED);
    LEDManager::addLED(&B_LED);
    R_LED.begin();
    G_LED.begin();
    B_LED.begin();

    R_LED.on();
#endif
}

void Led_Loop() {
    LEDManager::updateAll(); // 统一更新所有LED
}