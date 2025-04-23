#include "./Include/project_config.h"

void setup() {

    //打开电源
    SetupCarSystem();

    //读取eeprom数据
    EEPROM_Read_Data();

    //I2C
    I2C_A.begin(SDA_A_GPIO, SCL_A_GPIO, 400000UL);
    I2C_B.begin(SDA_B_GPIO, SCL_B_GPIO, 400000UL);

    //MPU初始化
    mpu6050.begin();

    //Motor初始化参数
    Foc_Parameters_init();

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

        //暂时显示A
        // motor_A.monitor();
        //motor_B.monitor();

        command.run();
    }


}

