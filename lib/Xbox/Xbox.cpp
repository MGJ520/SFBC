#include "Xbox.h"

#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

XboxSeriesXControllerESP32_asukiaaa::Core xboxController("98:7a:14:a4:65:f1");

float joyLHorizon, joyLVertical, joyRHorizon, joyRVertical, trigLT, trigRT;
bool btnA, btnB, btnX, btnY;
bool btnShare, btnStart, btnSelect, btnXbox;
bool btnLB, btnRB, btnLS, btnRS;
bool btnDirUp, btnDirLeft, btnDirRight, btnDirDown;

/// 用于解析Xbox手柄的要输入，将他们转化成0-1之前的小数
void xboxController_parse()
{

    joyLHorizon = (float)(xboxController.xboxNotif.joyLHori);
    joyLVertical = (float)(xboxController.xboxNotif.joyLVert);
    joyRHorizon = (float)(xboxController.xboxNotif.joyRHori);
    joyRVertical = (float)(xboxController.xboxNotif.joyRVert);
    trigLT = (float)(xboxController.xboxNotif.trigLT);
    trigRT = (float)(xboxController.xboxNotif.trigRT);
    btnA = xboxController.xboxNotif.btnA;
    btnB = xboxController.xboxNotif.btnB;
    btnX = xboxController.xboxNotif.btnX;
    btnY = xboxController.xboxNotif.btnY;
    btnShare = xboxController.xboxNotif.btnShare; // 我这个手柄没有这个键貌似是最新版手柄的
    btnStart = xboxController.xboxNotif.btnStart;
    btnSelect = xboxController.xboxNotif.btnSelect;
    btnXbox = xboxController.xboxNotif.btnXbox;
    btnLB = xboxController.xboxNotif.btnLB;
    btnRB = xboxController.xboxNotif.btnRB;
    btnLS = xboxController.xboxNotif.btnLS; // 左摇杆按下键
    btnRS = xboxController.xboxNotif.btnRS; // 右摇杆按下键
    btnDirUp = xboxController.xboxNotif.btnDirUp;
    btnDirLeft = xboxController.xboxNotif.btnDirLeft;
    btnDirRight = xboxController.xboxNotif.btnDirRight;
    btnDirDown = xboxController.xboxNotif.btnDirDown;
}

void xbox_init()
{
    xboxController.begin();
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        xboxController_parse();
    }
    else
    {
        if (xboxController.getCountFailedConnection() > 2)
        {
            ESP.restart();
        }
    }
}

void xbox_server()
{
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        xboxController_parse();
        if (xboxController.isWaitingForFirstNotification())
        {
            Serial.println("waiting for first notification");
        }
        else
        {
            Serial.println("Address: " + xboxController.buildDeviceAddressStr());
            Serial.print(xboxController.xboxNotif.toString());
            unsigned long receivedAt = xboxController.getReceiveNotificationAt();
            uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
            Serial.print("joyLHori rate: ");
            Serial.println((float)xboxController.xboxNotif.joyLHori / joystickMax);
            Serial.print("joyLVert rate: ");
            Serial.println((float)xboxController.xboxNotif.joyLVert / joystickMax);
            Serial.println("battery " + String(xboxController.battery) + "%");
            Serial.println("received at " + String(receivedAt));
        }
    }
    else
    {
        if (xboxController.getCountFailedConnection() > 2)
        {
            ESP.restart();
        }
    }
}
