//#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
//
//XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
//
//void setup() {
//    Serial.begin(115200);
//    Serial.println("Starting NimBLE Client");
//    xboxController.begin();
//}
//
//void loop() {
//    xboxController.onLoop();
//    if (xboxController.isConnected()) {
//        if (xboxController.isWaitingForFirstNotification()) {
//            Serial.println("waiting for first notification");
//        } else {
//            uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
//            //左右
//            Serial.print("joyRHori rate: ");
//            float C_Speed = ((float)xboxController.xboxNotif.joyRHori / (float)joystickMax)-0.5f;
//            //前后
//            Serial.println(C_Speed);
//            Serial.print("joyLVert rate: ");
//            float C_Turn = ((float)xboxController.xboxNotif.joyLHori / (float)joystickMax)-0.5f;
//            Serial.println(C_Turn);
//        }
//    } else {
//        Serial.println("not connected");
//    }
//    delay(300);
//}