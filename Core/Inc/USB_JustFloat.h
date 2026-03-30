//
// Created by lxcy on 2025/10/23.
//

#ifndef FOC_SENSORLESS_USB_JUSTFLOAT_H
#define FOC_SENSORLESS_USB_JUSTFLOAT_H
#include "stdint.h"


//收到数据
void receiveDataHandler(uint8_t* Buf, uint32_t *Len);
//记录电机运行和系统状态，用于发送给上位机
void recordRunningData();
#endif //FOC_SENSORLESS_USB_JUSTFLOAT_H