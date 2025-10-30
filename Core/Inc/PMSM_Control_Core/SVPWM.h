//
// Created by lxcy on 2025/9/30.
//


#ifndef FOC_SENSORLESS_SVPWM_H
#define FOC_SENSORLESS_SVPWM_H
#include "main.h"
#include "hrtim.h"
#include "other.h"
//更新Udc和DivUdc唯一接口
void SetUdc(float newUdc);
//获取Udc唯一接口
float GetUdc();
//设定PWM比较值，这里ABC三相分别对应TIMER ACD三个计时器。(-O2自动内联)
static void HRTIM_Set_PWMCompare(uint16_t A,uint16_t B,uint16_t C) {
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, A);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, B);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_D, HRTIM_COMPAREUNIT_1, C);
}
//计算SVPWM并更新到HRTIM的计数值(内部实现简单非线性限幅，建议在外部对Valpha和Vbeta进行等比例限幅，以避免过调制)
void SVPWM_Calculate_Set(float Valpha, float Vbeta);
#endif //FOC_SENSORLESS_SVPWM_H