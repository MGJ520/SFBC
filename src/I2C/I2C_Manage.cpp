//
// Created by MGJ on 2025/5/1.
//

#include "I2C_Manage.h"

TwoWire I2C_A = TwoWire(1); // I2C总线A，编号为1
TwoWire I2C_B = TwoWire(0); // I2C总线B，编号为0

bool I2C_init()
{
    Serial.println("[I2C]:正在初始化I2C");

    Serial.print("[I2CA]:SDA:");
    Serial.print(SDA_A_GPIO);  // 打印 SDA_A_GPIO 的值
    Serial.print(" SCL:");
    Serial.println(SCL_A_GPIO);  // 打印 SCL_A_GPIO 的值

    Serial.print("[I2CB]:SDA:");
    Serial.print(SDA_B_GPIO);  // 打印 SDA_A_GPIO 的值
    Serial.print(" SCL:");
    Serial.println(SCL_B_GPIO);  // 打印 SCL_A_GPIO 的值

    I2C_A.begin(SDA_A_GPIO, SCL_A_GPIO, 400000UL);
    I2C_B.begin(SDA_B_GPIO, SCL_B_GPIO, 400000UL);

    return true;
}