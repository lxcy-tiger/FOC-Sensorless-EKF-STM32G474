//
// Created by lxcy on 2026/3/9.
//

#ifndef FOC_SENSORLESS_EKF5_H
#define FOC_SENSORLESS_EKF5_H
#include "other.h"
struct EKF5_t {
    float Valpha_I;//输入Valpha(V)
    float Vbeta_I;//输入Vbeta(V)
    float Ialpha_I;//输入Ialpha(A)
    float Ibeta_I;//输入Ibeta(A)

    float Ialpha_O;//输出预测值Ialpha(A)
    float Ibeta_O;//输出预测值Ibeta(A)
    float Espeed_O;//输出电角速度Espeed(rad/s)
    float Etheta_O;//输出电角度Etheta(rad)
    float Tm_O;//输出负载转矩(N·m)
};
extern struct EKF5_t ekf5_est;
void EKF5_init();//初始化EKF,启动前必须执行一次
void EKF5_update(struct EKF5_t*ekf5);//更新EKF
#endif //FOC_SENSORLESS_EKF5_H