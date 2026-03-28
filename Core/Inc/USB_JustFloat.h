//
// Created by lxcy on 2025/10/23.
//

#ifndef FOC_SENSORLESS_USB_JUSTFLOAT_H
#define FOC_SENSORLESS_USB_JUSTFLOAT_H
#include "stdint.h"

#define singleDataLength 9 //单次记录需要的数据长度,注意这里要算上帧尾(以float为单位)
#define USB_HalfDataCount 100//填充数据的一半数量
#define USB_HalfDataLength (singleDataLength*USB_HalfDataCount)//填充数据的一半长度
#define USB_MaxDataRecordLength (USB_HalfDataLength*2)//当数据每被填满一半，就发送一次数据
//收到数据
void receiveDataHandler(uint8_t* Buf, uint32_t *Len);
//记录电机运行和系统状态，用于发送给上位机
void recordRunningData();
#endif //FOC_SENSORLESS_USB_JUSTFLOAT_H