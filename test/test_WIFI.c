#include <Arduino.h>
#include <WiFi.h>

const char *ssid = "Harold";        // 替换为你的 WiFi 名称
const char *password = "123456789"; // 替换为你的 WiFi 密码

void setup()
{
    Serial.begin(115200);

    // 启动 WiFi 连接
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // 初始信号强度检测
    int rssi = WiFi.RSSI();
    Serial.print("Initial RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void loop()
{
    static unsigned long previousMillis = 0;
    const long interval = 3000; // 检测间隔（3秒）

    // 非阻塞式定时检测
    if (millis() - previousMillis >= interval)
    {
        previousMillis = millis();

        if (WiFi.status() == WL_CONNECTED)
        {
            int rssi = WiFi.RSSI();
            Serial.print("RSSI: ");
            Serial.print(rssi);
            Serial.print(" dBm | ");

            // 信号质量分级
            if (rssi >= -50)
            {
                Serial.println("极强信号");
            }
            else if (rssi >= -60)
            {
                Serial.println("强信号");
            }
            else if (rssi >= -70)
            {
                Serial.println("中等信号");
            }
            else
            {
                Serial.println("弱信号");
            }
        }
        else
        {
            Serial.println("WiFi 连接已断开!");
        }
    }

    delay(100); // 维持系统稳定性
}