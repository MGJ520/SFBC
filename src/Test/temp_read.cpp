//#include "driver/temp_sensor.h"  // 引入温度传感器驱动库
//#include "Arduino.h"            // 引入Arduino基础库
//
//void setup()  // 初始化函数，程序启动时执行一次
//{
//    Serial.begin(115200);  // 初始化串口通信，波特率设置为115200
//    // 定义温度传感器的配置结构体
//    temp_sensor_config_t temp_sensor = {
//            .dac_offset = TSENS_DAC_L2,  // 设置DAC偏移量，使用TSENS_DAC_L2作为偏移量
//            .clk_div = 6,                // 设置时钟分频因子为6
//    };
//    temp_sensor_set_config(temp_sensor);  // 将上述配置应用到温度传感器
//    temp_sensor_start();                  // 启动温度传感器
//}
//
//void loop()  // 主循环函数，程序启动后不断重复执行
//{
//    float tsens_out;  // 定义一个浮点型变量，用于存储温度传感器的输出值
//    temp_sensor_read_celsius(&tsens_out);  // 读取温度传感器的温度值（摄氏度），并存储到tsens_out中
//    Serial.printf("%0.0f\r\n", tsens_out);    // 通过串口输出温度值，格式为浮点数
//    delay(500);                            // 每次读取后延迟500毫秒
//}
