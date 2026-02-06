//
// Created by lxcy on 2026/1/27.
//

#ifndef FOC_SENSORLESS_ST_SMO_PLL_H
#define FOC_SENSORLESS_ST_SMO_PLL_H
#include "other.h"
struct ST_SMO_PLL_t {
    float Valpha_I;//输入Valpha(V)
    float Vbeta_I;//输入Vbeta(V)
    float Ialpha_I;//输入Ialpha(A)
    float Ibeta_I;//输入Ibeta(A)

    float E_alpha_O;//输出反电动势E_alpha(V)
    float E_beta_O;//输出反电动势E_beta(V)
    float Espeed_O;//输出电角速度Espeed(rad/s)
    float Etheta_O;//输出电角度Etheta(rad)
};
extern struct ST_SMO_PLL_t st_smo_pll_est;
void ST_SMO_PLL_update(struct ST_SMO_PLL_t*st_smo_pll);
#endif //FOC_SENSORLESS_ST_SMO_PLL_H