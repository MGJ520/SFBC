//
// Created by MGJ on 2025/4/23.
//

#include "led_flash.h"
#include "control/control.h"

int led_count = 0;

void Led_flash() {
    if (System_Status != Open) {
#ifdef LED_3_GPIO
        digitalWrite(LED_3_GPIO, HIGH);
        digitalWrite(LED_2_GPIO, LOW);
#endif
    } else {
#ifdef LED_3_GPIO
        if (++led_count >= 800) {
            digitalWrite(LED_3_GPIO, LOW);
            digitalWrite(LED_2_GPIO, HIGH);
        } else {
            digitalWrite(LED_3_GPIO, LOW);
            digitalWrite(LED_2_GPIO, LOW);
        }
        if (led_count >= 850) {
            led_count = 0;
        }
#endif
    }
}
