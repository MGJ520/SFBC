//// welcome to lingshunlab.com
//// 加载EEPROM的库
//#include <EEPROM.h>
//
//// 定义EEPROM的大小
//#define EEPROM_SIZE 1  // 这里定义1个字节的大小
//
//int read_value = 0;
//
//void setup() {
//    Serial.begin(115200);
//
//    // 初始化EEPROM为预习定义的大小
//    EEPROM.begin(EEPROM_SIZE);
//}
//
//void loop() {
//    read_value = EEPROM.read(0); // 读区 EEPROM 第0位的数据
//    Serial.println(read_value);
//
//    read_value++;   // read_value+1 ，但EEPROM只接受0～255的数值，超出的将会是255的取余值
//    EEPROM.write(0, read_value); // 把 变量read_value的数值写入第0位
//    EEPROM.commit(); // 需要提交才能正真地把数据写入EEPROM
//
//    delay(1000);
//}
//
//#include "Arduino.h"
//#include <Preferences.h>  // 引入Preferences库，用于操作NVS
//
//Preferences preferences;  // 创建Preferences对象
//
//void setup() {
//    Serial.begin(115200);  // 初始化串口通信
//    delay(1000);           // 稍作延迟，等待串口连接
//
//    // 打开Preferences，指定命名空间为"counterStorage"
//    preferences.begin("counterStorage", false);
//
//    // 读取计数器
//    int counter = preferences.getInt("counter", -1);  // 如果键不存在，则返回默认值0
//    Serial.print("读取到的计数器值: ");
//    Serial.println(counter);
//
//    // 更新计数器
//    counter=3;
//    preferences.putInt("counter", counter);  // 将新的计数值写入NVS
//    Serial.print("更新后的计数器值: ");
//    Serial.println(counter);
//
//    // 提交更改（在Preferences中，更改会自动提交，无需手动调用commit）
//    Serial.println("更改已提交");
//
//    // 关闭Preferences
//    preferences.end();
//}
//
//void loop() {
//    // 周期性地验证存储的数据
//    delay(1000);  // 每秒验证一次
//
//    // 打开Preferences，指定命名空间为"counterStorage"
//    preferences.begin("counterStorage", false);
//
//    // 读取计数器
//    int counter = preferences.getInt("counter", -1);  // 如果键不存在，则返回默认值0
//    Serial.print("验证读取到的计数器值: ");
//    Serial.println(counter);
//
//    // 关闭Preferences
//    preferences.end();
//}