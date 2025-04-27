#include "./Include/project_config.h"


void setup() {
    //初始化系统
    SetupCarSystem();

    //开始任务
    Start_Task();
}

int mpu_count=0;

void loop() {

    //初次启动
    Init_Loop();

    //FOC控制
    Control_Loop();

    //LED显示
    Led_Loop();

    //串口运行
    command.run();

//    if (++mpu_count > 200)
//    {
//        mpu_count=0;
//        Serial.print("mpu:");
//        Serial.print(mpu6050.getAngleX());
//        Serial.print(",");
//        Serial.print(mpu6050.getAngleY());
//        Serial.print(",");
//        Serial.print(mpu6050.getAngleZ());
//        Serial.print(",");
//        Serial.println(mpu6050.temperature);
//    }

}

