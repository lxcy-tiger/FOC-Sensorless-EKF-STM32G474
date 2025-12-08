//
// Created by lxcy on 2025/12/7.
//

#ifndef FOC_SENSORLESS_FluxObserver_PLL_H
#define FOC_SENSORLESS_FluxObserver_PLL_H
#include "other.h"
struct FluxObserver_PLL_t {
    float Valpha_I;//输入Valpha(V)
    float Vbeta_I;//输入Vbeta(V)
    float Ialpha_I;//输入Ialpha(A)
    float Ibeta_I;//输入Ibeta(A)

    float Flux_alpha_O;//输出转子磁链Flux_alpha(V·s)
    float Flux_beta_O;//输出转子磁链Flux_beta(V·s)
    float Espeed_O;//输出电角速度Espeed(rad/s)
    float Etheta_O;//输出电角度Etheta(rad)
};
extern struct FluxObserver_PLL_t fluxObserver_pll_est;

void FluxObserver_PLL_update(struct FluxObserver_PLL_t*fluxObserver_pll_est);

#endif //FOC_SENSORLESS_SMO_PLL_H