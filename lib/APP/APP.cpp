#include "APP.h"
#include "BatteryAndButton.h"
#include "BuzzerSound.h"
#include "UserConfig.h"

const char *ssid = USER_SSID;
const char *password = USER_PASSWORD;

AppControl_t appCTRL;

WebServer appServer(80);

#if STATIC_IP_MODE /*静态IP配置*/

// 使用这些宏来初始化IPAddress实例
IPAddress staticIP(STATIC_IP_FIRST_OCTET, STATIC_IP_SECOND_OCTET, STATIC_IP_THIRD_OCTET, STATIC_IP_FOURTH_OCTET);
IPAddress gateway(GATEWAY_FIRST_OCTET, GATEWAY_SECOND_OCTET, GATEWAY_THIRD_OCTET, GATEWAY_FOURTH_OCTET);
IPAddress subnet(SUBNET_FIRST_OCTET, SUBNET_SECOND_OCTET, SUBNET_THIRD_OCTET, SUBNET_FOURTH_OCTET);
#endif

static void AppServerTask(void *pvParameters)
{
    
#if STATIC_IP_MODE /*静态IP配置*/
    if (WiFi.config(staticIP, gateway, subnet) == false)
    {
        Serial.println(" Static IP Config Failed.");
    }
#endif


    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    // #if !STATIC_IP_MODE
    Serial.println(" ");
    Serial.println("********************************************************");
    Serial.println(WiFi.localIP());
    Serial.println("********************************************************");
    // #endif

    appServer.on("/connect", appConnectHandler);
    appServer.on("/voltage", appVoltageHandler);
    appServer.on("/mpuset", appMPUsetHandler);
    appServer.on("/move", appMoveHandler);
    appServer.on("/poweroff", appPowerOffHandler);
    appServer.begin();
    Serial.println("Balance Car AppServer Started");

    for (;;)
    {
        appServer.handleClient();
        vTaskDelay(10);
    }
};

static void CarBrakeTask(void *pvParameters)
{
    for (;;)
    {
        if (appCTRL.Direction == "stop")
        {
            if (appCTRL.Velocity > appCTRL.MPUOffset)
            {
                appCTRL.Velocity -= BRAKE_DECAY;
                appCTRL.Velocity = (appCTRL.Velocity < appCTRL.MPUOffset) ? appCTRL.MPUOffset : appCTRL.Velocity;
            }
            if (appCTRL.Velocity < appCTRL.MPUOffset)
            {
                appCTRL.Velocity += BRAKE_DECAY;
                appCTRL.Velocity = (appCTRL.Velocity > appCTRL.MPUOffset) ? appCTRL.MPUOffset : appCTRL.Velocity;
            }
            // -----------------
            appCTRL.SteerVelocity = 0.0f;
        }
        else if (appCTRL.Direction == "mpu")
        {
            appCTRL.Velocity = appCTRL.MPUOffset;
        }
        vTaskDelay(10);
    }
};

void AppTaskInit::startTask()
{
    xTaskCreatePinnedToCore(AppServerTask, "App Sever Task", 6144, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(CarBrakeTask, "Brake Car Task", 2048, NULL, 2, NULL, 0);
};

/*-------------------------------------------------------------------------*/
// 应用程序连接处理器
// 当客户端连接到服务器时调用
static void appConnectHandler()
{
    appServer.send(200, "text/plain", "1"); // 向客户端发送状态码200和文本响应"1"，表示连接成功
};

// 电池电压处理器
// 当客户端请求电池电压时调用
static void appVoltageHandler()
{
    appServer.send(200, "text/plain", String(batteryVoltage_raw)); // 将电池电压值以文本形式发送给客户端
};

// MPU设置处理器
// 当客户端发送MPU设置请求时调用
static void appMPUsetHandler()
{
    appCTRL.Direction = appServer.arg("direction"); // 从请求中获取"direction"参数并赋值给appCTRL.Direction
    appCTRL.MPUOffset = appServer.arg("distance").toFloat(); // 从请求中获取"distance"参数并转换为浮点数，赋值给appCTRL.MPUOffset
};

// 移动控制处理器
// 根据客户端发送的方向指令调整速度和转向
static void appMoveHandler()
{
    appCTRL.Direction = appServer.arg("direction"); // 获取客户端发送的方向指令

    if (appCTRL.Direction == "up") // 如果方向为"up"
    {
        appCTRL.Velocity += MOVE_VEL; // 增加移动速度
    }

    if (appCTRL.Direction == "down") // 如果方向为"down"
    {
        appCTRL.Velocity -= MOVE_VEL; // 减少移动速度
    }

    if (appCTRL.Direction == "left") // 如果方向为"left"
    {
        // 增加转向速度，但不超过限制值STR_LIMIT
        appCTRL.SteerVelocity = (appCTRL.SteerVelocity > STR_LIMIT) ? STR_LIMIT : appCTRL.SteerVelocity + STR_VEL;
        appCTRL.Velocity = (appCTRL.Velocity > STR_LIMIT) ? STR_LIMIT : appCTRL.Velocity + STR_VEL;
    }

    if (appCTRL.Direction == "right") // 如果方向为"right"
    {
        // 减少转向速度，但不低于限制值-STR_LIMIT
        appCTRL.SteerVelocity = (appCTRL.SteerVelocity < -STR_LIMIT) ? -STR_LIMIT : appCTRL.SteerVelocity - STR_VEL;
        appCTRL.Velocity = (appCTRL.Velocity < -STR_LIMIT) ? -STR_LIMIT : appCTRL.Velocity - STR_VEL;
    }
};

// 关闭电源处理器
// 当客户端请求关闭电源时调用
static void appPowerOffHandler()
{
    balanceCarPowerOff(); // 调用关闭电源的函数
}


// 初始化应用程序任务
AppTaskInit APP;