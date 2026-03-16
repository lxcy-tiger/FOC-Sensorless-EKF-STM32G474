//
// Created by lxcy on 2025/12/7.
//
/* 磁链观测器与PLL，参考自论文
 * 《Performance Improvement of Nonlinear Flux Observer for Sensorless Control of PMSM》
 *                                  Jiali Liu and Yongchang Zhang , Senior Member, IEEE
 */
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "PMSM_Control_Core/PMSM_Parameter.h"
static float flux_s[2]={0.f,0.f};//定子磁通预测值(V·s)
static float flux_r[2]={0.f,0.f};//转子磁通预测值(V·s)
static float flux_e=0.f;//转子磁通幅值(定值)
/*
 * 增益,仿真结果说明:太小时事实上就会使得磁链观测器事实上是无效的(甚至和lambda=0没有区别)
 * 可以查看
 * USB_data[USB_DataRecordIndex++]=fluxObserver_pll_est.Flux_alpha_O;
 * 和USB_data[USB_DataRecordIndex++]=fluxObserver_pll_est.Flux_beta_O;
 * 以alpha和beta的幅值对齐为妙(即alpha和beta的最大值和最小值都是相等的,而不是有一个偏置)
 * 但是不要过度放大，否则float乘积会变成NAN
 */
static const float lambda=1.e8f;
static float Etheta=0.f;//电角度预测值(rad)
static float Espeed=0.f;//电角速度预测值(rad/s)
static const float k=1.f;//切向增益

struct FluxObserver_PLL_t fluxObserver_pll_est;
void FluxObserver_init() {
    flux_e=flux;
    flux_s[0]=flux;
}
static void FluxObserver_update(struct FluxObserver_PLL_t*fluxObserver_pll) {
    const float ualpha=fluxObserver_pll->Valpha_I;
    const float ubeta=fluxObserver_pll->Vbeta_I;
    const float ialpha=fluxObserver_pll->Ialpha_I;
    const float ibeta=fluxObserver_pll->Ibeta_I;
    flux_r[0]=flux_s[0]-Ls*ialpha;
    flux_r[1]=flux_s[1]-Ls*ibeta;

    const float k_sign=Speed_PIstate.Set>0?k*1.f:k*-1.f;//给定负转速时，必须要将k取相反数，否则导致磁链观测器无法收敛
    flux_s[0]=T_s*(
        ualpha-Rs*ialpha+
            lambda/2*(flux_r[0]-k_sign*flux_r[1])
            *(flux_e*flux_e-flux_r[0]*flux_r[0]-flux_r[1]*flux_r[1])
        )
        +flux_s[0];
    flux_s[1]=T_s*(
        ubeta-Rs*ibeta+
            lambda/2*(flux_r[1]+k_sign*flux_r[0])
            *(flux_e*flux_e-flux_r[0]*flux_r[0]-flux_r[1]*flux_r[1])
        )
        +flux_s[1];
    fluxObserver_pll->Flux_alpha_O=flux_r[0];
    fluxObserver_pll->Flux_beta_O=flux_r[1];
}

static void PLL_update(struct FluxObserver_PLL_t*fluxObserver_pll) {
    float c_theta,s_theta;
    static const float RAD_TO_DEG = 180.0f / PI;
    arm_sin_cos_f32(Etheta * RAD_TO_DEG, &s_theta, &c_theta);
    const float err=fluxObserver_pll->Flux_beta_O*c_theta-fluxObserver_pll->Flux_alpha_O*s_theta;
    FluxObserver_PLLSpeed_PIstate.error=err;
    FluxObserver_PLLSpeed_PI_update(&FluxObserver_PLLSpeed_PIstate);
    Espeed=FluxObserver_PLLSpeed_PIstate.Output;
    Etheta+=Espeed*T_s;
    while (Etheta >= PI2) Etheta -= PI2;
    while (Etheta < 0.f) Etheta += PI2;
    static const uint8_t useLPF_Speed=1;//使用低通滤波得到更平滑的转速输出(500hz截止频率)
    if (useLPF_Speed) {
        fluxObserver_pll->Espeed_O=lowPass_filter_flux_Espeed(Espeed);
    }
    else {
        fluxObserver_pll->Espeed_O=Espeed;
    }
    fluxObserver_pll->Etheta_O=Etheta;
}

void FluxObserver_PLL_update(struct FluxObserver_PLL_t*fluxObserver_pll) {
    FluxObserver_update(fluxObserver_pll);
    PLL_update(fluxObserver_pll);
}


