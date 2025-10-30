//
// Created by lxcy on 2025/10/8.
//

#include "PMSM_Control_Core/Clarke_Park.h"

//Clarke和Park变换的所有变量，使用一个结构体全部包含(I表示输入,O表示输出)
struct Clarke_Park_t clarke_park_transform={0};

//Clarke变换
void Clarke_transform(struct Clarke_t* clarke) {
    clarke->Ialpha_O = clarke->Ia_I;
    clarke->Ibeta_O = (clarke->Ia_I+2*clarke->Ib_I)*Div_Sqrt_3;
}

//反Clarke变换
void IClarke_transform(struct IClarke_t* iclarke) {
    iclarke->Va_O=iclarke->Valpha_I;
    iclarke->Vb_O=(-iclarke->Valpha_I+Sqrt_3*iclarke->Vbeta_I)/2;
    iclarke->Vc_O=(-iclarke->Valpha_I-Sqrt_3*iclarke->Vbeta_I)/2;
}

//Park变换
void Park_transform(struct Park_t* park) {
    float cosTheta,sinTheta;
    arm_sin_cos_f32(park->Theta_I*180/PI,&sinTheta,&cosTheta);
    park->Id_O=cosTheta*park->Ialpha_I+sinTheta*park->Ibeta_I;
    park->Iq_O=-sinTheta*park->Ialpha_I+cosTheta*park->Ibeta_I;
}

//反Park变换
void IPark_transform(struct IPark_t* ipark) {
    float cosTheta,sinTheta;
    arm_sin_cos_f32(ipark->Theta_I*180/PI,&sinTheta,&cosTheta);
    ipark->Valpha_O=cosTheta*ipark->Vd_I-sinTheta*ipark->Vq_I;
    ipark->Vbeta_O=sinTheta*ipark->Vd_I+cosTheta*ipark->Vq_I;
}