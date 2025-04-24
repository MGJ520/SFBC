//
// Created by MGJ on 2025/4/23.
//

#ifndef FOC_CODE_XBOX_CONTROLS_H
#define FOC_CODE_XBOX_CONTROLS_H

#include "include/precompiled.h"

extern XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

[[noreturn]] void Handle_control_tasks(void *pvParameters);


#endif //FOC_CODE_XBOX_CONTROLS_H
