//
// Created by lxcy on 2026/1/10.
//

#ifndef FOC_SENSORLESS_PMSM_PARAMETER_H
#define FOC_SENSORLESS_PMSM_PARAMETER_H
#include "main.h"
static const float Rs=5.5f/2; //电阻(Ohm)
static const float Ls= 2.e-3f/2; //电感(H)
static const float flux =0.00386335f; //磁通(V·s)
static const float J=4.5e-6f;//机械转动惯量(kg·m^2)
static const float T_s=5e-5f;//观测器执行周期(s)

static const uint8_t POLE_PAIRS=7; //极对数
#endif //FOC_SENSORLESS_PMSM_PARAMETER_H