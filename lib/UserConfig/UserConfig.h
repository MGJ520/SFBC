
#ifndef UserConfig_h
#define UserConfig_h

/*--------------------Phone WiFi Config------------------------
 * 这部分配置用于设置手机WiFi的名称和密码。
 * 如果设备需要连接到手机创建的WiFi热点，需要将以下SSID和密码
 * 设置为手机热点的名称和密码。
 */
#define USER_SSID "ABC"    // 将手机WiFi名称改为它
#define USER_PASSWORD "123456789" // 将手机WiFi密码改为它


#define STATIC_IP_MODE 0 // 0:DHCP 1:静态IP
#define YOUR_IP 229

/*--------------------------PID Config------------------------*/
#define VEL_Kp 0.082f // MOTOR-FOC
#define VEL_Ki 2.8f
#define C_Kp 0.6f
#define C_Ki 0.8f
#define C_LF 0.1f // MOTOR-FOC

#define VELOCITY_Kp 0.22f
#define VELOCITY_Ki 0.008f
#define VELOCITY_Kd 0.0f
#define VELOCITY_LIMIT 45.0f

#define UPRIGHT_Kp 1.0f
#define UPRIGHT_Ki 0.08f
#define UPRIGHT_Kd 0.02f
#define UPRIGHT_LIMIT 100.0f

#define TARGET_VEL_LIMIT 55.0f

#define STR_LIMIT 30.0f

#define VEL_PID_UPDATE 4
#define UPRIGHT_PID_UPDATE 2

/*------------------------MPU-----------------------------------*/
#define GYRO_COEF 0.975f
#define GYRO_COEFX 0.85f
#define PRE_GYRO_COEFX 0.25f
/*------------------------Monitor Mode--------------------------*/

#define MONITOR_MODE 0 // 0:关闭 1:开启

/*-------------------------------------------------------------*/
#endif