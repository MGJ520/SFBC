#ifndef FOC_CODE_PREFERENCES_H
#define FOC_CODE_PREFERENCES_H

#include "include/precompiled.h"


typedef struct Preferences_Data {
    uint8_t version = DATA_VERSION;  // 版本号
    uint8_t version_time=0;
    float A_Offset_ia=0, A_Offset_ib=0, A_Offset_ic=0;
    float B_Offset_ia=0, B_Offset_ib=0, B_Offset_ic=0;
}  Preferences_Data;


// 声明全局变量
extern Preferences_Data motor_data;
extern bool First_Ready;

void Save_Data();
void Read_Data();


#endif