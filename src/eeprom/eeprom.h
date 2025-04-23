#ifndef FOC_CODE_EEPROM_H
#define FOC_CODE_EEPROM_H

#include "include/precompiled.h"



#pragma pack(push, 1)
typedef struct EEPROM_Data {
    uint8_t version = DATA_VERSION;  // 版本号
    int new_time=0;
    float A_Offset_ia=0, A_Offset_ib=0, A_Offset_ic=0;
    float B_Offset_ia=0, B_Offset_ib=0, B_Offset_ic=0;
    uint32_t crc=0;  // CRC校验值

}  EEPROM_Data;
#pragma pack(pop)


#define EEPROM_SIZE    sizeof(EEPROM_Data)


// 声明全局变量
extern EEPROM_Data motor_data;
extern bool First_Ready;



void EEPROM_Read_Data();
void EEPROM_Write_Data(void *pvParameters);

#endif // FOC_CODE_EEPROM_H