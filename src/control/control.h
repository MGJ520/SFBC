#ifndef FOC_CODE_CONTROL_H
#define FOC_CODE_CONTROL_H

#include "include/precompiled.h"


extern enum Car {
    Dis, Open, Err
} System_Status;


extern boolean Ready;
extern const float MAX_ALLOWED_SPEED;
extern int speed_count;
extern float Now_Speed;
extern float Now_Pitch;
extern float PID_Speed_Out;
extern float PID_Pitch_Out;
extern float C_Speed;
extern float C_Turn;
extern float Target_torque_A;
extern float Target_torque_B;

void AbnormalSpinDetect();
void LandingDetect();

void Control_Loop();
#endif //FOC_CODE_CONTROL_H
