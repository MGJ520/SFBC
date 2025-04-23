//
// Created by MGJ on 2025/4/23.
//

#include "freertos_task.h"
#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "power/BatteryAndButton.h"
#include "eeprom/eeprom.h"


TaskHandle_t Task1;
TaskHandle_t Task2;


void Start_Task()
{

    xTaskCreatePinnedToCore(BatteryVoltageCheckTask,
                            "Battery_Voltage_Check",
                            2048,
                            NULL,
                            9,
                            NULL,
                            1);


    xTaskCreatePinnedToCore(ButtonEventTask,
                            "Button_Event",
                            2048,
                            NULL,
                            2,
                            NULL,
                            1);


    xTaskCreatePinnedToCore(Foc_A_Initialize,
                            "Task1",
                            10000,
                            NULL,
                            10,
                            &Task1,
                            1);


    xTaskCreatePinnedToCore(Foc_B_Initialize,
                            "Task2",
                            10000,
                            NULL,
                            11,
                            &Task2,
                            1);


    xTaskCreatePinnedToCore(Handle_control_tasks,
                            "Task4",
                            10000,
                            NULL,
                            0,
                            NULL,
                            0);

}
void eeprom_writer_task_run()
{
    Serial.println("[EEprom]:写EEprom数据,保存电流offset参数");
    xTaskCreatePinnedToCore(EEPROM_Write_Data,
                            "Task3",
                            10000,
                            NULL,
                            11,
                            NULL,
                            1);
}