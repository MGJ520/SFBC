//
// Created by MGJ on 2025/4/22.
//

#include <EEPROM.h>
#include "eeprom.h"
#include "freertos/FreeRTOS.h"
#include "foc/foc_drive.h"
#include "task/freertos_task.h"

#include "esp_task_wdt.h" // 需要包含看门狗头文件



Preferences prefs;
const char* NAMESPACE = "motor_cfg";


Preferences_Data motor_data;

bool First_Ready;

void Read_Data() {
    prefs.begin(NAMESPACE, true); // 以只读模式打开命名空间

    // 检查是否存在有效数据
    size_t dataSize = prefs.getBytesLength("motor_data");
    if(dataSize == sizeof(Preferences_Data)) {
        // 读取结构体数据
        prefs.getBytes("motor_data", &motor_data, sizeof(Preferences_Data));

        // 校验版本
        if (motor_data.version != DATA_VERSION) {
            Serial.println("[nvs]:数据版本不匹配，重置为默认值");
            First_Ready = true;
        }

        // 检查时间戳
        if (motor_data.version_time != DEFAULT_TIME) {
            Serial.println("[nvs]:首次启动");
            First_Ready = true;
        } else {
            Serial.println("[nvs]:存在数据,非首次启动");
            First_Ready = false;
        }
    } else {
        Serial.println("[nvs]:没有数据");
        First_Ready = true;
    }

    prefs.end();

    // 应用偏移量数据
    if (First_Ready) {
        cs_A.offset_flag = true;
        cs_B.offset_flag = true;
    } else {
        // 应用存储的偏移量
        if (_isset(cs_A.pinA)) cs_A.offset_ia = motor_data.A_Offset_ia;
        if (_isset(cs_A.pinB)) cs_A.offset_ib = motor_data.A_Offset_ib;
        if (_isset(cs_A.pinC)) cs_A.offset_ic = motor_data.A_Offset_ic;
        if (_isset(cs_B.pinA)) cs_B.offset_ia = motor_data.B_Offset_ia;
        if (_isset(cs_B.pinB)) cs_B.offset_ib = motor_data.B_Offset_ib;
        if (_isset(cs_B.pinC)) cs_B.offset_ic = motor_data.B_Offset_ic;

        cs_A.offset_flag = false;
        cs_B.offset_flag = false;
    }
}



void Save_Data() {
    esp_task_wdt_init(10, true); // 5秒超时
    esp_task_wdt_delete(NULL);
    prefs.begin(NAMESPACE, false); // 以读写模式打开命名空间
    // 更新版本时间戳
    motor_data.version_time = DEFAULT_TIME;
    // 保存结构体数据
    prefs.putBytes("motor_data", &motor_data, sizeof(Preferences_Data));
    // 提交并关闭
    prefs.end();
    esp_task_wdt_init(10, true); // 5秒超时
    Serial.println("[nvs]:数据保存成功");
}