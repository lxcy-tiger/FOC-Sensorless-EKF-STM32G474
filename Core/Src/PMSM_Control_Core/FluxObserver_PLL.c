//
// Created by lxcy on 2025/12/7.
//
/* 磁链观测器与PLL，参考自论文
 * 《Performance Improvement of Nonlinear Flux Observer for Sensorless Control of PMSM》
 *                                  Jiali Liu and Yongchang Zhang , Senior Member, IEEE
 */
#include "PMSM_Control_Core/FluxObserver_PLL.h"

static const float T_s=5e-5;//采样时间(SMO执行周期)(s)
static const float Rs=5.5f/2;//电阻(Ohm)
static const float Ls=2.e-3f/2;;//电感(H)
static float flux_s[2]={0.0050628f,0.f};//定子磁通预测值(V·s)
static float flux_r[2]={0.f,0.f};//转子磁通预测值(V·s)
static const float flux_e=0.0050628f;//转子磁通幅值(定值)
static const float lambda=600.f;//增益
static float Etheta=0.f;//电角度预测值(rad)
static float Espeed=0.f;//电角速度预测值(rad/s)
static const float k=1.f;//切向增益

struct FluxObserver_PLL_t fluxObserver_pll_est;

static void FluxObserver_update(struct FluxObserver_PLL_t*fluxObserver_pll_est) {
    const float ualpha=fluxObserver_pll_est->Valpha_I;
    const float ubeta=fluxObserver_pll_est->Vbeta_I;
    const float ialpha=fluxObserver_pll_est->Ialpha_I;
    const float ibeta=fluxObserver_pll_est->Ibeta_I;
    flux_r[0]=flux_s[0]-Ls*ialpha;
    flux_r[1]=flux_s[1]-Ls*ibeta;

    flux_s[0]=T_s*(
        ualpha-Rs*ialpha+
            lambda/2*(flux_r[0]-k*flux_r[1])
            *(flux_e*flux_e-flux_r[0]*flux_r[0]-flux_r[1]*flux_r[1])
        )
        +flux_s[0];
    flux_s[1]=T_s*(
        ubeta-Rs*ibeta+
            lambda/2*(flux_r[1]+k*flux_r[0])
            *(flux_e*flux_e-flux_r[0]*flux_r[0]-flux_r[1]*flux_r[1])
        )
        +flux_s[1];
    fluxObserver_pll_est->Flux_alpha_O=flux_r[0];
    fluxObserver_pll_est->Flux_beta_O=flux_r[1];
}

static void PLL_update(struct FluxObserver_PLL_t*fluxObserver_pll_est) {
    float c_theta,s_theta;
    static const float RAD_TO_DEG = 180.0f / PI;
    arm_sin_cos_f32(Etheta * RAD_TO_DEG, &s_theta, &c_theta);
    const float err=fluxObserver_pll_est->Flux_beta_O*c_theta-fluxObserver_pll_est->Flux_alpha_O*s_theta;
    FluxObserver_Speed_PIstate.error=err;
    FluxObserver_Speed_PI_update(&FluxObserver_Speed_PIstate);
    Espeed=FluxObserver_Speed_PIstate.Output;
    Etheta+=Espeed*T_s;
    while (Etheta >= PI2) Etheta -= PI2;
    while (Etheta < 0.f) Etheta += PI2;
    fluxObserver_pll_est->Espeed_O=Espeed;
    fluxObserver_pll_est->Etheta_O=Etheta;
}

void FluxObserver_PLL_update(struct FluxObserver_PLL_t*fluxObserver_pll_est) {
    FluxObserver_update(fluxObserver_pll_est);
    PLL_update(fluxObserver_pll_est);
}


