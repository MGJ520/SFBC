//
// Created by MGJ on 2025/4/22.
//


#include <esp_int_wdt.h>
#include "nvs.h"

#include "freertos/FreeRTOS.h"
#include "foc/foc_drive.h"
#include "task/freertos_task.h"

#include "esp_task_wdt.h" // 需要包含看门狗头文件
#include "power/BatteryAndButton.h"
#include "nvs_flash.h"


Preferences prefs;

DRAM_ATTR Preferences_Data motor_data;
bool First_Ready= true;


void Read_Data() {

    prefs.begin("motor_data", true); // 以只读模式打开命名空间
    size_t dataSize = prefs.getBytesLength("motor_data");
    if (dataSize >2) {
        // 读取结构体数据
        prefs.getInt("time"  , motor_data.version_time);
        prefs.getInt("v"     , motor_data.version);
        prefs.getFloat("a_ia", motor_data.A_Offset_ia);
        prefs.getFloat("a_ib", motor_data.A_Offset_ib);
        prefs.getFloat("b_ia", motor_data.B_Offset_ia);
        prefs.getFloat("b_ib", motor_data.B_Offset_ib);

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
        cs_A.offset_ia = motor_data.A_Offset_ia;
        cs_A.offset_ib = motor_data.A_Offset_ib;
        cs_B.offset_ia = motor_data.B_Offset_ia;
        cs_B.offset_ib = motor_data.B_Offset_ib;

        cs_A.offset_flag = false;
        cs_B.offset_flag = false;
    }
}

//        if (First_Ready) {
//            // 在需要写入时启动任务
//            xTaskCreate(writePreferencesTask,
//                        "WritePrefs",
//                        30000,
//                        NULL,
//                        0,
//                        &Task4
//            );
//        }


void writePreferencesTask(void *pvParameters) {
    Serial.println("写入数据...");
    motor_data.version_time = DEFAULT_TIME;
    motor_data.A_Offset_ia = cs_A.offset_ia;
    motor_data.A_Offset_ib = cs_A.offset_ib;
    motor_data.B_Offset_ia = cs_B.offset_ia;
    motor_data.B_Offset_ib = cs_B.offset_ib;

    esp_int_wdt_cpu_init();
    esp_int_wdt_init(); // 默认超时为 5ms
    esp_task_wdt_init(30, true); // 设置超时时间为30秒

    prefs.begin("motor_data", false);

    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    Serial.println("开始写入数据...");
    Serial.println("写入时间...");
    portENTER_CRITICAL(&mux);
    prefs.putInt("time", motor_data.version_time);
    Serial.println("写入版本...");
    prefs.putInt("v", motor_data.version);
    esp_task_wdt_reset();
    Serial.println("写入A_OFFSET_IA...");
    prefs.putFloat("a_ia", motor_data.A_Offset_ia);
    Serial.println("写入A_OFFSET_IB...");
    prefs.putFloat("a_ib", motor_data.A_Offset_ib);
    esp_task_wdt_reset();
    Serial.println("写入B_OFFSET_IA...");
    prefs.putFloat("b_ia", motor_data.B_Offset_ia);
    Serial.println("写入B_OFFSET_IB...");
    prefs.putFloat("b_ib", motor_data.B_Offset_ib);
    esp_task_wdt_reset();
    portEXIT_CRITICAL(&mux);
    Serial.println("写入完成");
    prefs.end();
    vTaskDelete(Task4);
}