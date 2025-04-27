//
// Created by MGJ on 2025/4/23.
//

#ifndef FOC_CODE_LED_H
#define FOC_CODE_LED_H

#include "include/precompiled.h"

extern LED R_LED;  // 声明R_LED为全局变量
extern LED G_LED;  // 声明G_LED为全局变量
extern LED B_LED;  // 声明B_LED为全局变量


void init_led();
void Led_Loop();

#endif //FOC_CODE_LED_H
