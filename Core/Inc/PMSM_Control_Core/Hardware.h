//
// Created by lxcy on 2025/10/10.
// 与硬件相关的参数

#ifndef FOC_SENSORLESS_HARDWARE_H
#define FOC_SENSORLESS_HARDWARE_H
#include <stdint.h>
extern float VCC_3V3; //稳压模块输出电压
extern uint16_t IA_REF;  //Ia偏置电压ADC值(12位ADC),根据电阻网络计算得到
extern uint16_t IB_REF; //Ib偏置电压ADC值(12位ADC),根据电阻网络计算得到
extern const float IA_K;//增益，由电阻网络计算得到(取正值)，单位V/A
extern const float IB_K;//增益，由电阻网络计算得到(取正值)，单位V/A

static const uint8_t POLE_PAIRS=7; //极对数
void Offline_IabVcc_Adjust();
#endif //FOC_SENSORLESS_HARDWARE_H