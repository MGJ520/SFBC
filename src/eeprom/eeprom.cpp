//
// Created by MGJ on 2025/4/22.
//

#include <EEPROM.h>
#include "eeprom.h"
#include "freertos/FreeRTOS.h"
#include "foc/foc_drive.h"

EEPROM_Data motor_data;

bool First_Ready;

int EEPROM_Addr = 0;

// CRC校验函数
uint32_t calculateCRC(const EEPROM_Data &data) {
    uint32_t crc = 0;
    const uint8_t *bytes = (uint8_t*)&data;
    for (size_t i = 0; i < sizeof(data) - sizeof(data.crc); i++) {
        crc ^= (bytes[i] << (i % 24));
    }
    return crc;
}

void EEPROM_Init() {
    if (!EEPROM.begin(EEPROM_SIZE)) {
        Serial.println("EEPROM初始化失败!");
    }
}


void EEPROM_Read_Data() {
    EEPROM_Init();  // 调用初始化函数
    EEPROM.get(EEPROM_Addr, motor_data);

    // 校验版本和CRC
    if (motor_data.version != DATA_VERSION ||
        motor_data.crc != calculateCRC(motor_data)) {
        Serial.println("[eeprom]:数据无效或版本不匹配，重置为默认值");
    }

    if (motor_data.new_time != DEFAULT_TIME) {
        Serial.println("[eeprom]:首次启动");
        First_Ready = true;
        motor_data.new_time = DEFAULT_TIME;
        motor_data.version = DATA_VERSION;
    } else {
        Serial.println("[eeprom]:存在数据,非首次启动");
        First_Ready = false;
    }




    if (First_Ready) {
        Serial.println("[eeprom]:没有EEprom数据");
        cs_A.offset_flag = true;
        cs_B.offset_flag = true;
    } else {
        cs_A.offset_flag = false;
        cs_B.offset_flag = false;
        if (_isset(cs_A.pinA)) cs_A.offset_ia = motor_data.A_Offset_ia;
        if (_isset(cs_A.pinB)) cs_A.offset_ib = motor_data.A_Offset_ib;
        if (_isset(cs_A.pinC)) cs_A.offset_ic = motor_data.A_Offset_ic;
        if (_isset(cs_B.pinA)) cs_B.offset_ia = motor_data.B_Offset_ia;
        if (_isset(cs_B.pinB)) cs_B.offset_ib = motor_data.B_Offset_ib;
        if (_isset(cs_B.pinC)) cs_B.offset_ic = motor_data.B_Offset_ic;
    }

}

void EEPROM_Write_Data(void *pvParameters)  {

    motor_data.crc = calculateCRC(motor_data);
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    EEPROM.put(EEPROM_Addr, motor_data);
    bool success = EEPROM.commit();
    portEXIT_CRITICAL(&mux);

    if (success) {
        Serial.println("数据保存成功");
    } else {
        Serial.println("保存失败!");
    }

}

