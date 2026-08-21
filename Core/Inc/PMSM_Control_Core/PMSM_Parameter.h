//
// Created by lxcy on 2026/1/10.
//

#ifndef FOC_SENSORLESS_PMSM_PARAMETER_H
#define FOC_SENSORLESS_PMSM_PARAMETER_H
#include "main.h"
static const float Rs=5.5f/2 + 1*1.0f; //电机相电阻以及后面的板子导通电阻和采样电阻补偿项(Ohm)
static const float Ls= 2.e-3f/2 + 1*0.0006f; //电机相电感以及后面离线参数辨识得到的实际相电感(H)
static const float flux =0.00386335f; //磁通(V·s或V/(rad/s))
static const float J=4.5e-6f;//机械转动惯量(kg·m^2)
static const float T_s=5e-5f;//观测器执行周期(s)

static const uint8_t POLE_PAIRS=7; //极对数

/*
    死区补偿参数，当某一相的电流，比如A相大于0(流入电机)，
    其实际占空比会比设定的占空比要小(由硬件和芯片设置共同决定，不一定是小，有可能是大，但是在我的硬件平台上，是要小）
    因此对于ia>0，需要将占空比增大，也就是将比较器的值减小(反之亦然)，已知我的硬件L6230的死区是1us，因此补偿值为：
    HRTIM的PERIOD值(34000) * 死区时间(1us) / 周期时间(50us)
 */
static const int32_t DeadTimeComp=34000*1.f/50;
#endif //FOC_SENSORLESS_PMSM_PARAMETER_H