//
// Created by MGJ on 2025/4/26.
//

#include "LED_Flash.h"


LED::LED(uint8_t pin) : _pin(pin) {}

void LED::begin() {
    pinMode(_pin, OUTPUT);
    off();
}

void LED::off() {
    _mode = Mode::Off;
    digitalWrite(_pin, LOW);
    _ledState = false;
}

void LED::on() {
    _mode = Mode::On;
    digitalWrite(_pin, HIGH);
    _ledState = true;
}

void LED::blink(uint16_t interval) {
    _mode = Mode::Blink;
    _blinkInterval = interval;
    _lastMillis = millis();
}

void LED::doubleBlink(uint16_t speed, uint16_t pause) {
    _mode = Mode::DoubleBlink;
    _doubleBlinkSpeed = speed;
    _doubleBlinkPause = pause;
    _blinkCounter = 0;
    _lastMillis = millis();
}

void LED::update() {
    unsigned long currentMillis = millis();

    switch(_mode) {
        case Mode::Blink:
            if (currentMillis - _lastMillis >= _blinkInterval) {
                _ledState = !_ledState;
                digitalWrite(_pin, _ledState);
                _lastMillis = currentMillis;
            }
            break;

        case Mode::DoubleBlink:
            if (_blinkCounter < 2) {
                // 双闪激活阶段
                if (currentMillis - _lastMillis >= _doubleBlinkSpeed) {
                    _ledState = !_ledState;
                    digitalWrite(_pin, _ledState);
                    _lastMillis = currentMillis;
                    if (!_ledState) _blinkCounter++;
                }
            } else {
                // 暂停阶段
                if (currentMillis - _lastMillis >= _doubleBlinkPause) {
                    _blinkCounter = 0;
                    _ledState = true;
                    digitalWrite(_pin, _ledState);
                    _lastMillis = currentMillis;
                }
            }
            break;

        default:
            // Off和On模式不需要处理
            break;
    }
}


// LEDManager实现
LED* LEDManager::instances[LEDManager::MAX_LEDS];


uint8_t LEDManager::count = 0;

void LEDManager::addLED(LED* led) {
    if (count < MAX_LEDS) {
        instances[count++] = led;
    }
}

void LEDManager::updateAll() {
    for (uint8_t i = 0; i < count; i++) {
        instances[i]->update();
    }
}

void LEDManager::offALL() {
    for (uint8_t i = 0; i < count; i++) {
        instances[i]->off();
    }
}