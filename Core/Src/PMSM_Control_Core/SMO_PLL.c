//
// Created by lxcy on 2026/1/4.
//
#include "PMSM_Control_Core/SMO_PLL.h"
#include "PMSM_Control_Core/PMSM_Parameter.h"
static float Etheta=0.f;//电角度预测值(rad)
static float Espeed=0.f;//电角速度预测值(rad/s)
static const float k=3.f;//增益
static float Ealpha_smo=0.f; //SMO的Ealpha开关项(k乘以sgn或sat函数)
static float Ebeta_smo=0.f;//SMO的Ebeta开关项(k乘以sgn或sat函数)
static float ialpha_est=0.f;//SMO预估ialpha
static float ibeta_est=0.f;//SMO预估ibeta
struct SMO_PLL_t smo_pll_est;

static void SMO_update(struct SMO_PLL_t*smo_pll) {
    const float ualpha=smo_pll->Valpha_I;
    const float ubeta=smo_pll->Vbeta_I;
    const float ialpha=smo_pll->Ialpha_I;
    const float ibeta=smo_pll->Ibeta_I;

    static const uint8_t useSat=1;//使用饱和函数替代符号函数
    if (useSat) {
        static const float maxErr=0.1f;
        Ealpha_smo=k*sat(ialpha_est-ialpha,-maxErr,maxErr,-1,1);
        Ebeta_smo=k*sat(ibeta_est-ibeta,-maxErr,maxErr,-1,1);
    }else {
        Ealpha_smo=k*sign(ialpha_est-ialpha);
        Ebeta_smo=k*sign(ibeta_est-ibeta);
    }

    ialpha_est=ialpha_est+T_s*(1/Ls*ualpha-Rs/Ls*ialpha_est-1/Ls*Ealpha_smo);
    ibeta_est=ibeta_est+T_s*(1/Ls*ubeta-Rs/Ls*ibeta_est-1/Ls*Ebeta_smo);


    smo_pll->E_alpha_O=lowPass_filter_SMO_Ealpha(Ealpha_smo);//高频开关项必须要经过低通滤波才能得到准确的反电动势项
    smo_pll->E_beta_O=lowPass_filter_SMO_Ebeta(Ebeta_smo);//高频开关项必须要经过低通滤波才能得到准确的反电动势项
    //smo_pll->E_alpha_O=IIR_filter2Ealpha(Ealpha_smo);//高频开关项必须要经过低通滤波才能得到准确的反电动势项
    //smo_pll->E_beta_O=IIR_filter2Ebeta(Ebeta_smo);//高频开关项必须要经过低通滤波才能得到准确的反电动势项
}

static void PLL_update(struct SMO_PLL_t*smo_pll) {
    float c_theta,s_theta;
    static const float RAD_TO_DEG = 180.0f / PI;
    arm_sin_cos_f32(Etheta * RAD_TO_DEG, &s_theta, &c_theta);
    const float err=-smo_pll->E_alpha_O*c_theta-smo_pll->E_beta_O*s_theta;
    SMO_PLLSpeed_PIstate.error = Speed_PIstate.Set > 0 ? err:-err;
    SMO_PLLSpeed_PI_update(&SMO_PLLSpeed_PIstate);
    Espeed=SMO_PLLSpeed_PIstate.Output;

    static const uint8_t useLPF_Speed=1;//使用低通滤波得到更平滑的转速输出
    if (useLPF_Speed) {
        smo_pll->Espeed_O=lowPass_filter_SMO_Espeed(Espeed);
        //smo_pll->Espeed_O=IIR_filter2SMO_Speed(Espeed);
    }
    else {
        smo_pll->Espeed_O=Espeed;
    }

    Etheta+=Espeed*T_s;
    while (Etheta >= PI2) Etheta -= PI2;
    while (Etheta < 0.f) Etheta += PI2;

    static const float wc=SMO_E_lowPass_Fc*PI2;//低通滤波器截止频率(rad/s)
    smo_pll->Etheta_O=Etheta+atan2f(smo_pll->Espeed_O,wc);
    while (smo_pll->Etheta_O >= PI2) smo_pll->Etheta_O -= PI2;
    while (smo_pll->Etheta_O < 0.f) smo_pll->Etheta_O += PI2;
}

void SMO_PLL_update(struct SMO_PLL_t*smo_pll) {
    SMO_update(smo_pll);
    PLL_update(smo_pll);
}