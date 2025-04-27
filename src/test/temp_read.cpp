////#include "driver/temp_sensor.h"  // 引入温度传感器驱动库
////#include "MPU6050.h"
////#include "Arduino.h"
////
////
////TwoWire I2C_A = TwoWire(1); // I2C总线A，编号为1
////MPU6050 mpu6050(I2C_A);
////
////void setup()  // 初始化函数，程序启动时执行一次
////{
////    Serial.begin(115200);  // 初始化串口通信，波特率设置为115200
////
////    // 定义温度传感器的配置结构体
////    temp_sensor_config_t temp_sensor = {
////            .dac_offset = TSENS_DAC_L2,  // 设置DAC偏移量，使用TSENS_DAC_L2作为偏移量
////            .clk_div = 6,                // 设置时钟分频因子为6
////    };
////    temp_sensor_set_config(temp_sensor);  // 将上述配置应用到温度传感器
////    temp_sensor_start();                  // 启动温度传感器
////
////
////    I2C_A.begin(SDA_A_GPIO, SCL_A_GPIO, 400000UL);
////    //MPU初始化
////    mpu6050.begin();
////
////}
////
////float tsens_out;
////
////void loop()
////{
////    delay(500);
////
////    mpu6050.update();
////
////    temp_sensor_read_celsius(&tsens_out);
////    Serial.printf("ESP32: %0.0f,MPU: %0.2f\n", tsens_out,mpu6050.temperature);    // 通过串口输出温度值，格式为浮点数
////
////}
//<<<<<<< HEAD
//>>>>>>> 06efc59 (first commit):src/test/temp_read.cpp
//=======
//
//>>>>>>> 590c94a (first commit)
