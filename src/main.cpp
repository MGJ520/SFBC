#include <esp_task_wdt.h>
#include "./Include/project_config.h"


void setup() {
    //初始化系统
    SetupCarSystem();
    //开始任务
    Start_Task();
}


void loop() {

    //第一次启动
    Ready_Time();

    //判断初始化是否成功
    if (motor_A.motor_status == motor_ready && motor_B.motor_status == motor_ready) {
        //Foc控制
        Control_Loop();
        //刷新Led
        Led_flash();
        //串口命令
        command.run();
    }

}

