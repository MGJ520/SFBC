#include "xbox_controls.h"
#include "control/control.h"
#include "foc/foc_drive.h"
#include "buzzer/BuzzerSound.h"
#include "power/BatteryAndButton.h"

#include "led/led.h"

XboxSeriesXControllerESP32_asukiaaa::Core xboxController;


[[noreturn]] void Handle_control_tasks(void *pvParameters) {
    xboxController.begin();
    uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;

    while (true) {
        xboxController.onLoop();
        vTaskDelay(pdMS_TO_TICKS(1)); // 延时1000ms

        if (xboxController.isConnected()) {
            Acc_Protect= false;
            if (!xboxController.isWaitingForFirstNotification()) {

                //控制接口
                Control_interface(
                        ((float) xboxController.xboxNotif.joyLVert / (float) joystickMax) - 0.5f,
                        ((float) xboxController.xboxNotif.joyRHori / (float) joystickMax) - 0.5f,
                        0.1
                );

                //打开保护
                if ((xboxController.xboxNotif.btnB) && System_Status == Open_Output) {
                    MotorClose();
                    buzzer.play(S_JUMP);
                }
                //关机
                if (xboxController.xboxNotif.btnY) {
                    balanceCarPowerOff();
                    buzzer.play(S_JUMP);
                }
                //关闭保护
                if ((xboxController.xboxNotif.btnLB || xboxController.xboxNotif.btnRB) &&
                    System_Status == Disable_Output) {
                    MotorOpen();
                    buzzer.play(S_MODE1);
                }
            }


#ifdef LED_1_GPIO
            B_LED.on();
#endif
        } else {
            Acc_Protect= true;
#ifdef LED_1_GPIO
            B_LED.off();
#endif
        }
    }


}