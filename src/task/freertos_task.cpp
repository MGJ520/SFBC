//
// Created by MGJ on 2025/4/23.
//

#include "freertos_task.h"


#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "power/BatteryAndButton.h"
#include "eeprom/eeprom.h"

#include "esp_task_wdt.h"

TaskHandle_t Task0;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

void Start_Task()
{

    xTaskCreatePinnedToCore(ButtonBatteryTask,
                            "Button_Event",
                            2048,
                            NULL,
                            2,
                            &Task0,
                            0);


    xTaskCreatePinnedToCore(Foc_A_Initialize,
                            "Foc_A_Initialize",
                            2048,
                            NULL,
                            3,
                            &Task1,
                            0);


    xTaskCreatePinnedToCore(Foc_B_Initialize,
                            "Foc_B_Initialize,",
                            2048,
                            NULL,
                            4,
                            &Task2,
                            0);



    xTaskCreatePinnedToCore(Handle_control_tasks,
                            "Handle_control",
                            4096,
                            NULL,
                            2,
                            &Task3,
                            1);

}
void eeprom_writer_task_run()
{

    vTaskSuspend(Task0);
    vTaskSuspend(Task1);
    vTaskSuspend(Task2);
    vTaskSuspend(Task3);


    Serial.println("[nvs]:写数据,保存电流offset参数");

    xTaskCreatePinnedToCore(Save_Data,
                            "Save_Data",
                            10000,
                            NULL,
                            tskIDLE_PRIORITY,
                            &Task4,
                            0);
}