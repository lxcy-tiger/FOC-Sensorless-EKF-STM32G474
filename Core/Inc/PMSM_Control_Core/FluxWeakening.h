//
// Created by lxcy on 2026/3/15.
//

#ifndef FOC_SENSORLESS_FLUXWEAKENING_H
#define FOC_SENSORLESS_FLUXWEAKENING_H
#include "other.h"
#include "stdint.h"
//获取Id的给定,用于弱磁(注:此函数可以通过useWeakening开关,所以说不是调用了该函数就一定弱磁)
float GetIdSet_Weak(const float SpeedSet,const float SpeedMeasure);
#endif //FOC_SENSORLESS_FLUXWEAKENING_H