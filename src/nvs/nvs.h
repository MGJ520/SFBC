#ifndef FOC_CODE_PREFERENCES_H
#define FOC_CODE_PREFERENCES_H

#include "include/precompiled.h"


typedef struct Preferences_Data {
    uint8_t version = DATA_VERSION;  // 版本号
    uint8_t version_time=0;
    float A_Offset_ia=0, A_Offset_ib=0;
    float B_Offset_ia=0, B_Offset_ib=0;
}  Preferences_Data;

extern bool save_requested;
// 声明全局变量
extern Preferences_Data motor_data;
extern bool First_Ready;
extern Preferences prefs;
// DRAM_ATTR 可选

void Read_Data();

void writePreferencesTask(void *pvParameters);
#endif