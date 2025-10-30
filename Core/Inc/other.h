//
// Created by lxcy on 2025/9/30.
// 用于定义杂项数据与函数的头文件

#ifndef FOC_SENSORLESS_OTHER_H
#define FOC_SENSORLESS_OTHER_H
#include "arm_math.h"
#include "main.h"
#include "PMSM_Control_Core/Hardware.h"
#include "PMSM_Control_Core/PI_Controller.h"
// 数学类常数与函数
static const float Sqrt_3=1.7320508075688772935274463415059f;//根号三(编译器自动截断)
static const float Sqrt_3_Half=Sqrt_3/2;//二分之根号三
static const float Div_Sqrt_3=1/Sqrt_3;//倒数_根号三(优化除法为乘法)
static const float PI2=PI*2;
//对u32数值进行限幅
static inline uint32_t clamp_u32(uint32_t input,uint32_t min,uint32_t max) {
    if (input < min)return min;
    if (input > max)return max;
    return input;
}
/*
 * median_3和median_filter_3构成中值滤波器(3个最近值)函数
 */
//取中位数
static inline float median_3(float a, float b, float c) {
    if ((a >= b && a <= c) || (a >= c && a <= b)) return a;
    if ((b >= a && b <= c) || (b >= c && b <= a)) return b;
    return c;
}
//自动生成函数
#define GenerateFunction_MedianFilter3(NAME) \
    static float NAME##_buf[3] = {0}; \
    static uint8_t NAME##_buf_index = 0;\
    float median_filter_##NAME##_3(float NAME##_input){\
    NAME##_buf[NAME##_buf_index] = NAME##_input;\
    NAME##_buf_index = (NAME##_buf_index + 1) % 3;\
    return median_3(NAME##_buf[0], NAME##_buf[1], NAME##_buf[2]);\
    }\

float median_filter_Ia_3(float Ia_input);
float median_filter_Ib_3(float Ib_input);

// 取5个数的中位数（第3小的数）
static inline float median_5(float a, float b, float c, float d, float e) {
    // 使用比较网络找出中位数，避免排序
    if (a > b) { float t = a; a = b; b = t; }
    if (c > d) { float t = c; c = d; d = t; }
    if (a > c) { float t = a; a = c; c = t; }
    if (b > d) { float t = b; b = d; d = t; }
    if (b > c) { float t = b; b = c; c = t; }
    // 此时 a <= b <= c <= d，e 是独立的
    if (e < b) {
        if (e < a) return b;
        else return e;
    } else {
        if (e < c) return e;
        else return c;
    }
}
#define GenerateFunction_MedianFilter5(NAME) \
static float NAME##_buf[5] = {0}; \
static uint8_t NAME##_buf_index = 0; \
float median_filter_##NAME##_5(float NAME##_input) { \
NAME##_buf[NAME##_buf_index] = NAME##_input; \
NAME##_buf_index = (NAME##_buf_index + 1) % 5; \
return median_5(NAME##_buf[0], NAME##_buf[1], NAME##_buf[2], NAME##_buf[3], NAME##_buf[4]); \
}
float median_filter_Ia_5(float Ia_input);
float median_filter_Ib_5(float Ib_input);

//给定电角速度
static inline void GiveESpeed(float espeed) {
    Speed_PIstate.Set=espeed;
}
//给定电角速度(rpm)
static inline void GiveESpeedRpm(float espeed) {
    GiveESpeed(espeed*PI2/60);
}
//给定机械角速度
static inline void GiveSpeed(float espeed) {
    GiveESpeed(espeed*POLE_PAIRS);
}
//给定机械角速度(rpm)
static inline void GiveSpeedRpm(float espeed) {
    GiveESpeedRpm(espeed*POLE_PAIRS);
}

float IIR_filter(float input,const float a[3],const float b[3],float w[3]);
float IIR_filter10A(float input);
float IIR_filter10B(float input);
float IIR_filter2A(float input);
float IIR_filter2B(float input);
#endif //FOC_SENSORLESS_OTHER_H