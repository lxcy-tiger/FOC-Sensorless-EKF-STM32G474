//
// Created by lxcy on 2025/10/10.
// 与硬件相关的参数

#ifndef FOC_SENSORLESS_HARDWARE_H
#define FOC_SENSORLESS_HARDWARE_H
#include <stdint.h>
extern float VCC_3V3; //稳压模块输出电压
extern uint16_t IA_REF; //Ia偏置电压ADC值(12位ADC)
extern uint16_t IB_REF; //Ib偏置电压ADC值(12位ADC)
extern const float IA_K;//增益，测得41.3mA时的输出为1.603 V，ADC测量得到的电压=IAREF-IA_K*IA，单位V/A
extern const float IB_K;//增益，测得41.1mA时的输出为1.638V，ADC测量得到的电压=IBREF-IB_K*IB，单位V/A

static const uint8_t POLE_PAIRS=7; //极对数
void Offline_IabVcc_Adjust();
#endif //FOC_SENSORLESS_HARDWARE_H