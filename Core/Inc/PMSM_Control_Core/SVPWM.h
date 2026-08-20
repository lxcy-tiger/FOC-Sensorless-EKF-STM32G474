//
// Created by lxcy on 2025/9/30.
//


#ifndef FOC_SENSORLESS_SVPWM_H
#define FOC_SENSORLESS_SVPWM_H
#include "main.h"
#include "hrtim.h"
#include "other.h"
//更新Udc和DivUdc唯一接口(暂时未用到)
//void SetUdc(float newUdc);
//获取Udc唯一接口(暂时未用到)
//float GetUdc();
//计算SVPWM并更新到HRTIM的计数值(内部实现简单非线性限幅，建议在外部对Valpha和Vbeta进行等比例限幅，以避免过调制)
void SVPWM_Calculate_Set(float Valpha, float Vbeta,float ia,float ib,float ic);
#endif //FOC_SENSORLESS_SVPWM_H