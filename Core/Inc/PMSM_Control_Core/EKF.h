//
// Created by lxcy on 2025/10/9.
//

#ifndef FOC_SENSORLESS_EKF_H
#define FOC_SENSORLESS_EKF_H
#include "other.h"
typedef struct EKF_t {
    float Valpha_I;//输入Valpha(V)
    float Vbeta_I;//输入Vbeta(V)
    float Ialpha_I;//输入Ialpha(A)
    float Ibeta_I;//输入Ibeta(A)

    float Ialpha_O;//输出预测值Ialpha(A)
    float Ibeta_O;//输出预测值Ibeta(A)
    float Espeed_O;//输出电角速度Espeed(rad/s)
    float Etheta_O;//输出电角度Etheta(rad)
};
extern struct EKF_t ekf_est;
void EKF_init();//初始化EKF,启动前必须执行一次
void EKF_update(struct EKF_t*ekf);//更新EKF
#endif //FOC_SENSORLESS_EKF_H