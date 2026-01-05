//
// Created by lxcy on 2025/9/30.
// 用于定义杂项数据与函数的文件
#include "other.h"

//自动生成两个电流滤波函数
//GenerateFunction_MedianFilter3(Ia)
//GenerateFunction_MedianFilter3(Ib)
// 自动生成两个电流的5点中值滤波函数
GenerateFunction_MedianFilter5(Ia)
GenerateFunction_MedianFilter5(Ib)

float IIR_filter(float input,const float a[3],const float b[3],float w[3]) {
    //初始化
    w[0]=w[1];
    w[1]=w[2];
    //求状态变量
    w[2]=a[0]*input-a[1]*w[1]-a[2]*w[0];
    //求输出
    return b[0]*w[2]+b[1]*w[1]+b[2]*w[0];
}

float IIR_filter2Ia(float input) {
    static float w[3];
    const float A[3] = {0.505001029045877625200944294192595407367,
        0.747789178258503328677875288121867924929,
 0.272214937925007283148204351164167746902 };
    const float B[3] = {1.f,2.f,1.f};
    return IIR_filter(input,A,B,w);
}
float IIR_filter2Ib(float input) {
    static float w[3];
    const float A[3] = {0.505001029045877625200944294192595407367,
        0.747789178258503328677875288121867924929,
 0.272214937925007283148204351164167746902};
    const float B[3] = {1.f,2.f,1.f};
    return IIR_filter(input,A,B,w);
}

float IIR_filter2Ealpha(float input) {
    static float w[3];
    const float A[3] = {0.005542717210280681712497585778010034119,
        -1.778631777824584814240438390697818249464,
 0.800802646665707551498769589670700952411 };
    const float B[3] = {1.f,2.f,1.f};
    return IIR_filter(input,A,B,w);
}
float IIR_filter2Ebeta(float input) {
    static float w[3];
    const float A[3] = {0.005542717210280681712497585778010034119,
        -1.778631777824584814240438390697818249464,
 0.800802646665707551498769589670700952411 };
    const float B[3] = {1.f,2.f,1.f};
    return IIR_filter(input,A,B,w);
}

float IIR_filter2SMO_Speed(float input) {
    static float w[3];
    const float A[3] = {0.005542717210280681712497585778010034119,
        -1.778631777824584814240438390697818249464,
 0.800802646665707551498769589670700952411 };
    const float B[3] = {1.f,2.f,1.f};
    return IIR_filter(input,A,B,w);
}