//
// Created by MGJ on 2025/5/1.
//

#include "web.h"
#include "control/control.h"
//#include "control/control.h"
#include <ESPAsyncWebServer.h>

#include <ArduinoJson.h>
#include <ESPmDNS.h>      //用于设备域名 MDNS.begin("esp32")
#include <DNSServer.h>


// AP配置
const char* ssid = "MyCar";
const char* password = "";

DNSServer dnsServer;                       //创建dnsServer实例
const int DNS_PORT = 53;                  //设置DNS端口号


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

IPAddress apIP(192, 168, 1, 1);            //设置AP的IP地址

// HTML页面内容
const char *htmlContent = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 摇杆控制</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        #joystick-container {
            width: 300px;
            height: 300px;
            background: #f0f0f0;
            border-radius: 50%;
            margin: 50px auto;
            position: relative;
            touch-action: none;
        }

        #data {
            margin-bottom: 150px;
        }

        #joystick {
            width: 60px;
            height: 60px;
            background: #4CAF50;
            border-radius: 50%;
            position: absolute;
            left: 120px;
            top: 120px;
            transition: transform 0.1s linear; /* 添加基础过渡效果 */
        }

        .status-box {
            text-align: center;
            font-family: Arial, sans-serif;
            margin: 10px;
            padding: 8px;
            border-radius: 5px;
        }

        #connection-status {
            background-color: #ff4444;
            color: white;
        }

        #latency {
            margin-top: 100px;
            background-color: #4CAF50;
            color: white;
        }
    </style>
</head>
<body>


<div id="latency" class="status-box">延迟: -- ms</div>

<div id="connection-status" class="status-box">状态: 断开连接</div>

<div id="data" class="status-box">X: 0.00, Y: 0.00</div>
<div id="joystick-container">
    <div id="joystick"></div>
</div>


<script>
    const joystick = document.getElementById('joystick');
    const container = document.getElementById('joystick-container');
    const dataDiv = document.getElementById('data');
    const statusDiv = document.getElementById('connection-status');
    const latencyDiv = document.getElementById('latency');
    let ws;
    let pingInterval;
    let lastPingTime;

    function updateConnectionStatus(status, isError) {
        statusDiv.textContent = `状态: ${status}`;
        statusDiv.style.backgroundColor = isError ? '#ff4444' : '#4CAF50';
    }

    function connectWebSocket() {
        ws = new WebSocket('ws://' + window.location.hostname + '/ws');

        ws.onopen = () => {
            console.log('WebSocket连接已建立');
            updateConnectionStatus('已连接', false);
            pingInterval = setInterval(sendPing, 1000);
        };

        ws.onerror = (error) => {
            console.log('WebSocket错误:', error);
            updateConnectionStatus('连接错误', true);
        };

        ws.onclose = () => {
            console.log('WebSocket断开连接');
            updateConnectionStatus('断开连接', true);
            clearInterval(pingInterval);
            setTimeout(connectWebSocket, 3000);
            latencyDiv.textContent = `延迟: ~ ms`;
        };

        ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'pong') {
                const latency = Date.now() - data.clientTime;
                latencyDiv.textContent = `延迟: ${latency} ms`;
            }
        };
    }

    function sendPing() {
        if (ws.readyState === WebSocket.OPEN) {
            const pingData = {
                type: 'ping',
                clientTime: Date.now()
            };
            ws.send(JSON.stringify(pingData));
        }
    }

    function initJoystick() {
        let isDragging = false;
        const rect = container.getBoundingClientRect();
        const centerX = rect.width / 2;
        const centerY = rect.height / 2;
        const maxDistance = 100;
        let currentX = 0;
        let currentY = 0;
        let animationFrameId = null;

        function updatePosition(clientX, clientY) {
            const rect = container.getBoundingClientRect();
            const x = clientX - rect.left - centerX;
            const y = clientY - rect.top - centerY;

            const distance = Math.min(maxDistance, Math.sqrt(x * x + y * y));
            const angle = Math.atan2(y, x);

            const normalizedX = (distance * Math.cos(angle)) / maxDistance;
            const normalizedY = (distance * Math.sin(angle)) / maxDistance;

            currentX = normalizedX;
            currentY = normalizedY;

            joystick.style.transform = `translate(
                    ${normalizedX * maxDistance}px,
                    ${normalizedY * maxDistance}px
                )`;

            dataDiv.textContent = `X: ${normalizedX.toFixed(2)}, Y: ${normalizedY.toFixed(2)}`;

            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({
                    x: normalizedX,
                    y: normalizedY
                }));
            }
        }

        function animateToCenter() {
            const startTime = Date.now();
            const duration = 200; // 动画持续时间300ms
            const startX = currentX;
            const startY = currentY;

            function animate() {
                const progress = Math.min(1, (Date.now() - startTime) / duration);

                const currentPosX = startX * (1 - progress);
                const currentPosY = startY * (1 - progress);

                joystick.style.transform = `translate(
                        ${currentPosX * maxDistance}px,
                        ${currentPosY * maxDistance}px
                    )`;

                dataDiv.textContent = `X: ${currentPosX.toFixed(2)}, Y: ${currentPosY.toFixed(2)}`;

                if (ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({
                        x: currentPosX,
                        y: currentPosY
                    }));
                }

                if (progress < 1) {
                    animationFrameId = requestAnimationFrame(animate);
                } else {
                    // 确保最终位置准确
                    joystick.style.transform = 'translate(0, 0)';
                    dataDiv.textContent = 'X: 0.00, Y: 0.00';
                    ws.send(JSON.stringify({x: 0, y: 0}));
                    animationFrameId = null;
                }
            }

            animationFrameId = requestAnimationFrame(animate);
        }

        // 触摸事件处理
        container.addEventListener('touchstart', (e) => {
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
                animationFrameId = null;
            }
            isDragging = true;
            updatePosition(e.touches[0].clientX, e.touches[0].clientY);
            e.preventDefault();
        });

        container.addEventListener('touchmove', (e) => {
            if (isDragging) {
                updatePosition(e.touches[0].clientX, e.touches[0].clientY);
                e.preventDefault();
            }
        });

        container.addEventListener('touchend', () => {
            if (isDragging) {
                isDragging = false;
                animateToCenter();
            }
        });

        // 鼠标事件处理
        container.addEventListener('mousedown', (e) => {
            if (animationFrameId) {
                cancelAnimationFrame(animationFrameId);
                animationFrameId = null;
            }
            isDragging = true;
            updatePosition(e.clientX, e.clientY);
        });

        document.addEventListener('mousemove', (e) => {
            if (isDragging) {
                updatePosition(e.clientX, e.clientY);
            }
        });

        document.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                animateToCenter();
            }
        });
    }

    window.onload = () => {
        connectWebSocket();
        initJoystick();
    };
</script>
</body>
</html>
)rawliteral";



/*
 * 开启DNS服务器
 */
void initDNS()
{
    if (dnsServer.start(DNS_PORT, "*", apIP))   //判断将所有地址映射到esp32的ip上是否成功
    {
        Serial.println("start dnsserver success.");
    } else {
        Serial.println("start dnsserver failed.");
    }
}


void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch(type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket客户端 #%u 连接成功\n", client->id());
            Acc_Protect= false;
            break;
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket客户端 #%u 断开连接\n", client->id());
            Acc_Protect= true;
            break;
        case WS_EVT_DATA: {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if(info->final && info->index == 0 && info->len == len){
                if(info->opcode == WS_TEXT) {
                    data[len] = 0;
                    JsonDocument doc;
                    DeserializationError error = deserializeJson(doc, data);

                    if(error) {
                        Serial.print("JSON解析失败: ");
                        Serial.println(error.c_str());
                        return;
                    }

                    // 处理摇杆数据
                    if (doc["x"].is<float>() && doc["y"].is<float>()) {
                        float x = doc["x"];
                        float y = doc["y"];

                        Serial.printf("收到数据 - X: %.2f, Y: %.2f\n", x, y);
                        //控制接口 0.03是死区
                        Control_interface(
                                x/2,
                                y/2,
                                0.03
                        );
                    }
                    // 处理ping请求
                    else if (doc["type"].is<const char*>() && strcmp(doc["type"], "ping") == 0) {
                        JsonDocument resDoc;
                        resDoc["type"] = "pong";
                        resDoc["clientTime"] = doc["clientTime"];
                        String response;
                        serializeJson(resDoc, response);
                        client->text(response);
                    }
                }
            }
            break;
        }
        default:
            break;
    }
}


void wifi_init() {
    if (MDNS.begin("esp32"))      //给设备设定域名esp32,完整的域名是esp32.local
    {
        Serial.println("MDNS responder started");
    }

    // 创建AP
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   //设置AP热点IP和子网掩码
    Serial.println("AP已启动");
    Serial.print("IP地址: ");
    Serial.println(WiFi.softAPIP());

    initDNS();

    // 配置WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // 配置HTTP服务器
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", htmlContent);
    });

    // 启动服务器
    server.begin();
}

void Web_loop() {
    ws.cleanupClients();
    dnsServer.processNextRequest();   //检查客户端DNS请求
}