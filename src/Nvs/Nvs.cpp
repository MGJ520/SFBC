//
// Created by MGJ on 2025/4/19.
//

#include "Nvs.h"
//    Preferences preferences;  // 创建Preferences对象
//    // 打开Preferences，指定命名空间为"counterStorage"
//    preferences.begin("CS2_OFST", false);
//
//    // 读取计数器
//    if (preferences.getInt("new", -1) == 4) {
//        SIMPLEFOC_DEBUG("CUR: Find CurrentSense Offset !");
//        if (_isset(cs_B.pinA)) cs_B.offset_ia = preferences.getFloat("offset_ia", 0); // 计算通道A的平均偏移量
//        if (_isset(cs_B.pinB)) cs_B.offset_ib = preferences.getFloat("offset_ib", 0); // 计算通道B的平均偏移量
//        if (_isset(cs_B.pinC)) cs_B.offset_ic = preferences.getFloat("offset_ic", 0); // 计算通道C的平均偏移量
//        cs_B.cs_num = 0;
//    }

//    if (preferences.getInt("new", -1) != 4) {
//        preferences.putInt("new", 3);  // 将新的计数值写入NVS
//        if (_isset(cs_B.pinA)) preferences.putFloat("offset_ia", cs_B.offset_ia);  // 将新的计数值写入NVS
//        if (_isset(cs_B.pinB)) preferences.putFloat("offset_ib", cs_B.offset_ib);  // 将新的计数值写入NVS
//        if (_isset(cs_B.pinC)) preferences.putFloat("offset_ic", cs_B.offset_ic);  // 将新的计数值写入NVS
//        cs_B.cs_num = 1;
//    }
//    // 关闭Preferences
//    preferences.end();