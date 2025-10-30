//
// Created by lxcy on 2025/10/23.
//

#ifndef FOC_SENSORLESS_USB_JUSTFLOAT_H
#define FOC_SENSORLESS_USB_JUSTFLOAT_H
#include "stdint.h"

#define singleDataLength 7 //单次记录需要的数据长度(以float为单位)
#define USB_HalfDataCount 1000//填充数据的一半数量
#define USB_HalfDataLength (singleDataLength*USB_HalfDataCount)//填充数据的一半长度
#define USB_MaxDataRecordLength (USB_HalfDataLength*2)//当数据每被填满一半，就发送一次数据
extern uint32_t Serial_DataRecordIndex;//表示已经记录数据的下标
extern float Serial_data[USB_MaxDataRecordLength+1000];//长度为USB_MaxDataRecordLength+1000，为了留有一定裕量，防止超过数组下标(理论上不会超过太多)
//收到数据
void receiveDataHandler(uint8_t* Buf, uint32_t *Len);
//记录电机运行和系统状态，用于发送给上位机
void recordRunningData();
#endif //FOC_SENSORLESS_USB_JUSTFLOAT_H