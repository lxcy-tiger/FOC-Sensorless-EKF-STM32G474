//
// Created by lxcy on 2025/10/9.
//
#include "PMSM_Control_Core/EKF.h"

static float Rs; //电阻(Ohm)
static float Ls; //电感(H)
static float flux; //磁通(V·s)
static float T_s;//采样时间(EKF执行周期)(s)
static float TsDivLs;//数值上等于T_s/Ls，用于加速计算过程
static float x_k_k_prev[4]; //先验估计
float x_k_k[4]; //后验估计
static const float Q_0_0=0.03,Q_1_1=0.03,Q_2_2=100,Q_3_3=0.00003; //过程噪声方差矩阵
static const float R_0_0=0.05,R_1_1=0.05; //测量噪声方差矩阵
static float P_k_k_prev_0_0,P_k_k_prev_0_1,P_k_k_prev_0_2,P_k_k_prev_0_3,
        P_k_k_prev_1_1,P_k_k_prev_1_2,P_k_k_prev_1_3,
        P_k_k_prev_2_2,P_k_k_prev_2_3
        ,P_k_k_prev_3_3; //先验方差矩阵
static float P_k_k_0_0=1,P_k_k_0_1,P_k_k_0_2,P_k_k_0_3,
        P_k_k_1_1=1,P_k_k_1_2,P_k_k_1_3,
        P_k_k_2_2=1,P_k_k_2_3
        ,P_k_k_3_3=1;  //后验方差矩阵
static float K_k[4][2]; //卡尔曼增益
//static float C[2][4] = {
//    {1, 0, 0, 0},
//    {0, 1, 0, 0}
//}; //输出矩阵，事实上代码不需要使用该变量


struct EKF_t ekf_est;
void EKF_init() {
    Rs = 5.5f/2;
    Ls = 2.e-3f/2;
    flux =0.00386335f;
    T_s=5e-5f;
    TsDivLs=T_s/Ls;
}

void EKF_update(struct EKF_t*ekf) {
    const float i_alpha_k_prev = x_k_k[0];
    const float i_beta_k_prev = x_k_k[1];
    const float w_e_k_prev = x_k_k[2];
    const float theta_e_k_prev = x_k_k[3];
    float c_theta_e_k_prev,s_theta_e_k_prev;//不能为const
    static const float RAD_TO_DEG = 180.0f / PI;
    arm_sin_cos_f32(theta_e_k_prev * RAD_TO_DEG, &s_theta_e_k_prev, &c_theta_e_k_prev);
    const float w_flux = w_e_k_prev * flux;
    x_k_k_prev[0] = i_alpha_k_prev +
                    (-Rs  * i_alpha_k_prev + w_flux *s_theta_e_k_prev+  ekf->Valpha_I )*TsDivLs;
    x_k_k_prev[1] = i_beta_k_prev +
                    (-Rs * i_beta_k_prev - w_flux * c_theta_e_k_prev +  ekf->Vbeta_I)*TsDivLs;
    x_k_k_prev[2] = w_e_k_prev;
    x_k_k_prev[3] = theta_e_k_prev + T_s * w_e_k_prev;


    const float A_k_0_0=1 - Rs*TsDivLs;
    const float A_k_1_1=1 - Rs*TsDivLs;
    const float A_k_3_2=T_s;
    const float TsDivLsFlux=TsDivLs*flux;
    const float A_k_0_2= TsDivLsFlux*s_theta_e_k_prev;
    const float A_k_1_2= - TsDivLsFlux * c_theta_e_k_prev;
    const float A_k_0_3= -A_k_1_2*w_e_k_prev;
    const float  A_k_1_3 = A_k_0_2*w_e_k_prev;


    //计算P_k_k_prev
    P_k_k_prev_0_0=A_k_0_0*A_k_0_0*P_k_k_0_0+2*A_k_0_0*(A_k_0_2*P_k_k_0_2+A_k_0_3*P_k_k_0_3)
                    +A_k_0_2*A_k_0_2*P_k_k_2_2+2*A_k_0_2*A_k_0_3*P_k_k_2_3+A_k_0_3*A_k_0_3*P_k_k_3_3+Q_0_0;
    P_k_k_prev_0_1=A_k_0_0*A_k_1_1*P_k_k_0_1+A_k_1_1*(A_k_0_2*P_k_k_1_2+A_k_0_3*P_k_k_1_3)
                    +A_k_0_0*(A_k_1_2*P_k_k_0_2+A_k_1_3*P_k_k_0_3)
                    +A_k_1_2*(A_k_0_2*P_k_k_2_2+A_k_0_3*P_k_k_2_3)
                    +A_k_1_3*(A_k_0_2*P_k_k_2_3+A_k_0_3*P_k_k_3_3);
    P_k_k_prev_1_1=A_k_1_1*A_k_1_1*P_k_k_1_1+2*A_k_1_1*(A_k_1_2*P_k_k_1_2+A_k_1_3*P_k_k_1_3)
                    +A_k_1_2*A_k_1_2*P_k_k_2_2+2*A_k_1_2*A_k_1_3*P_k_k_2_3
                    +A_k_1_3*A_k_1_3*P_k_k_3_3+Q_1_1;
    P_k_k_prev_2_2=P_k_k_2_2+Q_2_2;
    P_k_k_prev_3_3=A_k_3_2*A_k_3_2*P_k_k_2_2+2*A_k_3_2*P_k_k_2_3+P_k_k_3_3+Q_3_3;
    P_k_k_prev_2_3=A_k_3_2*P_k_k_2_2+P_k_k_2_3;

    const float APpBP_0_0=A_k_0_0*P_k_k_0_2+A_k_0_2*P_k_k_2_2+A_k_0_3*P_k_k_2_3
    ,APpBP_0_1=A_k_0_0*P_k_k_0_3+A_k_0_2*P_k_k_2_3+A_k_0_3*P_k_k_3_3
    ,APpBP_1_0=A_k_1_1*P_k_k_1_2+A_k_1_2*P_k_k_2_2+A_k_1_3*P_k_k_2_3
    ,APpBP_1_1=A_k_1_1*P_k_k_1_3+A_k_1_2*P_k_k_2_3+A_k_1_3*P_k_k_3_3;
    P_k_k_prev_0_2=APpBP_0_0;
    P_k_k_prev_1_2=APpBP_1_0;
    P_k_k_prev_0_3=A_k_3_2*APpBP_0_0+APpBP_0_1;
    P_k_k_prev_1_3=A_k_3_2*APpBP_1_0+APpBP_1_1;

    //计算Kk
    const float PR00=P_k_k_prev_0_0+R_0_0;
    const float PR11=P_k_k_prev_1_1+R_1_1;
    const float PR01=P_k_k_prev_0_1*P_k_k_prev_0_1;
    const float det=1.f/(PR00*PR11-PR01);
    K_k[0][0]=det*(P_k_k_prev_0_0*PR11-PR01);
    K_k[0][1]=det*(P_k_k_prev_0_1*R_0_0);
    K_k[1][0]=det*P_k_k_prev_0_1*R_1_1;
    K_k[1][1]=det*(-PR01+P_k_k_prev_1_1*PR00);
    K_k[2][0]=det*(P_k_k_prev_0_2*PR11-P_k_k_prev_1_2*P_k_k_prev_0_1);
    K_k[2][1]=det*(-P_k_k_prev_0_1*P_k_k_prev_0_2+P_k_k_prev_1_2*PR00);
    K_k[3][0]=det*(P_k_k_prev_0_3*PR11-P_k_k_prev_1_3*P_k_k_prev_0_1);
    K_k[3][1]=det*(-P_k_k_prev_0_3*P_k_k_prev_0_1+P_k_k_prev_1_3*PR00);


    //ypCx=y-Cx~(k-1)=y-Cf(xk-1|k-1,uk,0)=Cxk|k-1
    float ypCx[2] = {
        ekf->Ialpha_I - x_k_k_prev[0],
        ekf->Ibeta_I - x_k_k_prev[1]
    };
    for (uint8_t a = 0; a < 4; ++a)
        x_k_k[a] = x_k_k_prev[a] + K_k[a][0] * ypCx[0] + K_k[a][1] * ypCx[1];
    //更新协方差矩阵
    P_k_k_0_0=P_k_k_prev_0_0-K_k[0][0]*P_k_k_prev_0_0-K_k[0][1]*P_k_k_prev_0_1;
    P_k_k_0_1=P_k_k_prev_0_1-K_k[0][0]*P_k_k_prev_0_1-K_k[0][1]*P_k_k_prev_1_1;
    P_k_k_0_2=P_k_k_prev_0_2-K_k[0][0]*P_k_k_prev_0_2-K_k[0][1]*P_k_k_prev_1_2;
    P_k_k_0_3=P_k_k_prev_0_3-K_k[0][0]*P_k_k_prev_0_3-K_k[0][1]*P_k_k_prev_1_3;

    P_k_k_1_1=P_k_k_prev_1_1-K_k[1][0]*P_k_k_prev_0_1-K_k[1][1]*P_k_k_prev_1_1;
    P_k_k_1_2=P_k_k_prev_1_2-K_k[1][0]*P_k_k_prev_0_2-K_k[1][1]*P_k_k_prev_1_2;
    P_k_k_1_3=P_k_k_prev_1_3-K_k[1][0]*P_k_k_prev_0_3-K_k[1][1]*P_k_k_prev_1_3;

    P_k_k_2_2=P_k_k_prev_2_2-K_k[2][0]*P_k_k_prev_0_2-K_k[2][1]*P_k_k_prev_1_2;
    P_k_k_2_3=P_k_k_prev_2_3-K_k[2][0]*P_k_k_prev_0_3-K_k[2][1]*P_k_k_prev_1_3;

    P_k_k_3_3=P_k_k_prev_3_3-K_k[3][0]*P_k_k_prev_0_3-K_k[3][1]*P_k_k_prev_1_3;


    while (x_k_k[3] >= PI2) x_k_k[3] -= PI2;
    while (x_k_k[3] < 0.f) x_k_k[3] += PI2;

    ekf->Ialpha_O=x_k_k[0];
    ekf->Ibeta_O=x_k_k[1];
    ekf->Espeed_O=x_k_k[2];
    ekf->Etheta_O=x_k_k[3];
}