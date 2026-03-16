//
// Created by lxcy on 2026/3/15.
//
#include "PMSM_Control_Core/FluxWeakening.h"
static const uint8_t useWeakening=1;//使用弱磁策略以获得更高的转速
float GetIdSet_Weak(const float SpeedSet,const float SpeedMeasure) {
    if (useWeakening)
        return sat(my_abs(SpeedMeasure),1000,2500,0,-0.5);
    else return 0.f;
}