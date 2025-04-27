#include "./Include/project_config.h"



void setup() {
    //初始化系统
    SetupCarSystem();

    //开始任务
    Start_Task();
}

void loop() {

    //初次启动
    Init_Loop();

    //FOC控制
    Control_Loop();

    //LED显示
    Led_Loop();

    //web控制
    Web_loop();

    //串口控制
    command.run();

}

