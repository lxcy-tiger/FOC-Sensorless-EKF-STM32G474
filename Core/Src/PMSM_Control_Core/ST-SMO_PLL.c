//
// Created by lxcy on 2026/1/27.
//

#include "PMSM_Control_Core/ST-SMO_PLL.h"
#include "PMSM_Control_Core/PMSM_Parameter.h"

static float Etheta=0.f;//电角度预测值(rad)
static float Espeed=0.f;//电角速度预测值(rad/s)
static const float k1=0.6f;//增益1
static const float k2=10000.f;//增益2
static float Ealpha_st_smo=0.f; //ST-SMO的Ealpha项
static float Ebeta_st_smo=0.f;//ST-SMO的Ebeta项
static float ialpha_est=0.f;//ST-SMO预估ialpha
static float ibeta_est=0.f;//ST-SMO预估ibeta
static float signAlpha_addup=0.f;//积分项
static float signBeta_addup=0.f;//积分项
static const float sign_addup_MAX=5;//积分项限幅
static float E_amplitude;//计算当前反电动势幅值，除以它，将反电动势误差项归一化，防止不同速度下，k1,k2,以及PLL的作用不一致
struct ST_SMO_PLL_t st_smo_pll_est;

void ST_SMO_update(struct ST_SMO_PLL_t*st_smo_pll) {
    const float ualpha=st_smo_pll->Valpha_I;
    const float ubeta=st_smo_pll->Vbeta_I;
    const float ialpha=st_smo_pll->Ialpha_I;
    const float ibeta=st_smo_pll->Ibeta_I;

    const float ialpha_err=ialpha_est-ialpha;
    const float ibeta_err=ibeta_est-ibeta;

    float sqrt_abs_ialpha_err;// 根号|ialpha_err|
    float sqrt_abs_ibeta_err;// 根号|ibeta_err|
    arm_sqrt_f32(my_abs(ialpha_err),&sqrt_abs_ialpha_err);
    arm_sqrt_f32(my_abs(ibeta_err),&sqrt_abs_ibeta_err);

    arm_sqrt_f32(st_smo_pll->E_alpha_O * st_smo_pll->E_alpha_O
        + st_smo_pll->E_beta_O * st_smo_pll->E_beta_O,&E_amplitude);

    //大转速大E需要更大的校正力度，小转速小E校正力度小一点
    if (E_amplitude>0.1) {
        sqrt_abs_ialpha_err = sqrt_abs_ialpha_err * E_amplitude;
        sqrt_abs_ibeta_err = sqrt_abs_ibeta_err * E_amplitude;
    }
    else {
        sqrt_abs_ialpha_err = sqrt_abs_ialpha_err * 0.1;
        sqrt_abs_ibeta_err = sqrt_abs_ibeta_err * 0.1;
    }


    const float signAlpha_addup_unsat=signAlpha_addup+k2*T_s*sqrt_abs_ialpha_err*sign(ialpha_err);//sign(ialpha_err);
    const float signBeta_addup_unsat=signBeta_addup+k2*T_s*sqrt_abs_ibeta_err*sign(ibeta_err);//sign(ibeta_err);

    signAlpha_addup = clamp_float(signAlpha_addup_unsat,-sign_addup_MAX,sign_addup_MAX);
    signBeta_addup = clamp_float(signBeta_addup_unsat,-sign_addup_MAX,sign_addup_MAX);

    Ealpha_st_smo=k1*sqrt_abs_ialpha_err*sign(ialpha_err)+signAlpha_addup;
    Ebeta_st_smo=k1*sqrt_abs_ibeta_err*sign(ibeta_err)+signBeta_addup;

    ialpha_est=ialpha_est+T_s*(1/Ls*ualpha-Rs/Ls*ialpha_est-1/Ls*Ealpha_st_smo);
    ibeta_est=ibeta_est+T_s*(1/Ls*ubeta-Rs/Ls*ibeta_est-1/Ls*Ebeta_st_smo);


    static const uint8_t useLPF_SMO_E=1;//使用低通滤波滤除高频分量，截止频率设得很高，相位误差比较小，这里忽略相位误差
    if (useLPF_SMO_E) {
        st_smo_pll->E_alpha_O=lowPass_filter_ST_SMO_Ealpha(Ealpha_st_smo);
        st_smo_pll->E_beta_O=lowPass_filter_ST_SMO_Ebeta(Ebeta_st_smo);
    }
    else {
        st_smo_pll->E_alpha_O=Ealpha_st_smo;
        st_smo_pll->E_beta_O=Ebeta_st_smo;
    }
}
static void PLL_update(struct ST_SMO_PLL_t*st_smo_pll) {
    float c_theta,s_theta;
    static const float RAD_TO_DEG = 180.0f / PI;
    arm_sin_cos_f32(Etheta * RAD_TO_DEG, &s_theta, &c_theta);
    const float err=-st_smo_pll->E_alpha_O*c_theta-st_smo_pll->E_beta_O*s_theta;
    //大转速E大误差大，小转速E小误差小，所以同时除以幅值，现在大小转速的误差都差不多了
    if (E_amplitude > 1.f) {
        ST_SMO_Speed_PIstate.error=err/E_amplitude;
    }
    else {
        ST_SMO_Speed_PIstate.error=err/1.01f;
    }
    ST_SMO_Speed_PI_update(&ST_SMO_Speed_PIstate);
    Espeed=ST_SMO_Speed_PIstate.Output;

    static const uint8_t useLPF_Speed=1;//使用低通滤波得到更平滑的转速输出
    if (useLPF_Speed) {
        st_smo_pll->Espeed_O=lowPass_filter_ST_SMO_Espeed(Espeed);
    }
    else {
        st_smo_pll->Espeed_O=Espeed;
    }
    Etheta+=Espeed*T_s;
    while (Etheta >= PI2) Etheta -= PI2;
    while (Etheta < 0.f) Etheta += PI2;

    st_smo_pll->Etheta_O=Etheta;
}

void ST_SMO_PLL_update(struct ST_SMO_PLL_t*st_smo_pll) {
    ST_SMO_update(st_smo_pll);
    PLL_update(st_smo_pll);
}