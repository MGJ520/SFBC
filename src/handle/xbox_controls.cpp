//
// Created by MGJ on 2025/4/23.
//

#include "xbox_controls.h"
#include "control/control.h"
#include "foc/foc_drive.h"
#include "buzzer/BuzzerSound.h"
#include "power/BatteryAndButton.h"

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;



void Handle_control_tasks(void *pvParameters) {
    xboxController.begin();
    while (1)
    {
        xboxController.onLoop();
        if (xboxController.isConnected()) {
            if (!xboxController.isWaitingForFirstNotification()) {
                uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
                C_Speed = ((float) xboxController.xboxNotif.joyLVert / (float) joystickMax) - 0.5f;
                C_Turn = ((float) xboxController.xboxNotif.joyRHori / (float) joystickMax) - 0.5f;
                if (( xboxController.xboxNotif.btnB) && System_Status == Open) {
                    MotorClose();
                    buzzer.play(S_JUMP);
                }
                if (xboxController.xboxNotif.btnY) {
                    balanceCarPowerOff();
                    buzzer.play(S_JUMP);
                }
                if ((xboxController.xboxNotif.btnLB || xboxController.xboxNotif.btnRB) && System_Status == Dis) {
                    MotorOpen();
                    buzzer.play(S_MODE1);
                }
                C_Speed = lpf_run(C_Speed);
                C_Turn = lpf_trun(C_Turn);
            }
        }
        vTaskDelay(1);
    }
}