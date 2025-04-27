#ifndef COMMON_DEFS_H
#define COMMON_DEFS_H

// 公共定义（如宏、常量等）


//===================================nvs==========================================
//nvs数据版本
#define DATA_VERSION   ((__DATE__[8] - '0') * 10 + (__DATE__[9] - '0'))

//nvs更新变量
#define DEFAULT_TIME   1


//=============================================================================
//串口
#define SERIAL_BAUDRATE 115200

#endif  // COMMON_DEFS_H