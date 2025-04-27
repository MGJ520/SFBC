#include "freertos_task.h"


#include "foc/foc_drive.h"
#include "handle/xbox_controls.h"
#include "power/BatteryAndButton.h"
#include "nvs/nvs.h"

#include "esp_task_wdt.h"
#include "Init/init_System.h"

TaskHandle_t Task0;
TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;
TaskHandle_t Task4;

void Start_Task() {
//=====================================核心0=========================================

    //Foc_A初始化任务
    xTaskCreatePinnedToCore(Foc_A_Initialize,
                            "Foc_A_Initialize",
                            2048,
                            NULL,
                            3,
                            &Task1,
                            0);

    //Foc_B初始化任务
    xTaskCreatePinnedToCore(Foc_B_Initialize,
                            "Foc_B_Initialize,",
                            2048,
                            NULL,
                            4,
                            &Task2,
                            0);

    //按键处理任务
    xTaskCreatePinnedToCore(ButtonBatteryTask,
                            "Button_Event",
                            2048,
                            NULL,
                            2,
                            &Task0,
                            0);

//=====================================核心1=========================================
//    //手柄任务
//    xTaskCreatePinnedToCore(Handle_control_tasks,
//                            "Handle_control",
//                            4096,
//                            NULL,
//                            2,
//                            &Task3,
//                            1);

}
