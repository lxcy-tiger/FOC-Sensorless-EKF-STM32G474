//
// Created by lxcy on 2025/10/8.
//

#ifndef FOC_SENSORLESS_CLARKE_PARK_H
#define FOC_SENSORLESS_CLARKE_PARK_H
#include "other.h"
#include "arm_math.h"
typedef struct Clarke_Park_t{
    struct Clarke_t{
        float Ia_I;
        float Ib_I;
        float Ic_I;//无用
        float Ialpha_O;
        float Ibeta_O;
    }clarke;
    struct IClarke_t {
        float Valpha_I;
        float Vbeta_I;
        float Va_O;
        float Vb_O;
        float Vc_O;
    }iclarke;
    struct Park_t {
        float Ialpha_I;
        float Ibeta_I;
        float Theta_I;//弧度制
        float Id_O;
        float Iq_O;
    }park;
    struct IPark_t {
        float Vd_I;
        float Vq_I;
        float Theta_I;
        float Valpha_O;
        float Vbeta_O;
    }ipark;
};
extern struct Clarke_Park_t clarke_park_transform;
#define ClarkePark clarke_park_transform //简短的变量名定义

void Clarke_transform(struct Clarke_t* clarke);
void IClarke_transform(struct IClarke_t* iclarke);
void Park_transform(struct Park_t* park);
void IPark_transform(struct IPark_t* ipark);

#endif //FOC_SENSORLESS_CLARKE_PARK_H