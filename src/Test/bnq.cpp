#include <Arduino.h>
#include <SimpleFOC.h>
#include <cmath> // 包含 cmath 头文件以使用 fmod 函数
// 分别测试编码器AB是否工作正常

MagneticSensorI2C sensor_A1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor_B1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Ctwo = TwoWire(1); // 编码器A
TwoWire I2Cone = TwoWire(0); // 编码器B

void setup()
{
    Serial.begin(115200);

    I2Ctwo.begin(SDA_A_GPIO, SCL_A_GPIO, 400000UL); // 编码器A
    sensor_A1.init(&I2Ctwo);           // 编码器A

    I2Cone.begin(SDA_B_GPIO, SCL_B_GPIO, 400000UL); // 编码器B
    sensor_B1.init(&I2Cone); // 编码器B

    Serial.println("Sensor ready");
    delay(1000);
}
int i = 0;
void loop()
{

    sensor_A1.update();
    double angle_A = sensor_A1.getAngle();
    sensor_B1.update();
    double angle_B = sensor_B1.getAngle();
    if (i >  300)
    {
        i = 0;
        double result_A = abs(fmod(angle_A, 2 * PI)); // 对 2π 取模
        Serial.print("angle_A=");
        Serial.print(result_A * 57.32);
        Serial.print(",");
        Serial.print("Velocity_A=");
        Serial.print(sensor_A1.getVelocity());
        Serial.print(",");

        double result_B =  abs(fmod(angle_B, 2 * PI)); // 对 2π 取模
        Serial.print("angle_B=");
        Serial.print(result_B * 57.32);
        Serial.print(",");
        Serial.print("Velocity_B=");
        Serial.print(sensor_B1.getVelocity());
        Serial.print("\r\n");
    }
    i++;
}