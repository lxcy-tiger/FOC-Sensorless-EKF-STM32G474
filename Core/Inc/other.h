//
// Created by lxcy on 2025/9/30.
// 用于定义杂项数据与函数的头文件

#ifndef FOC_SENSORLESS_OTHER_H
#define FOC_SENSORLESS_OTHER_H
#include "arm_math.h"
#include "main.h"
#include "PMSM_Control_Core/PMSM_Parameter.h"
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
//对float数值进行限幅
static inline float clamp_float(float input,float min,float max) {
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

//给定电角速度(rad/s)
static inline void GiveESpeed(float espeed) {
    Speed_PIstate.Set=espeed;
}
//给定电角速度(rpm)
static inline void GiveESpeedRpm(float espeed) {
    GiveESpeed(espeed*PI2/60);
}
//给定机械角速度(rad/s)
static inline void GiveSpeed(float espeed) {
    GiveESpeed(espeed*POLE_PAIRS);
}
//给定机械角速度(rpm)
static inline void GiveSpeedRpm(float espeed) {
    GiveESpeedRpm(espeed*POLE_PAIRS);
}

float IIR_filter2Ia(float input);
float IIR_filter2Ib(float input);
static inline float my_abs(float input) {
    if (input < 0) return -input;
    else return input;
}
static inline float sign(float input) {
    if (input < 0) return -1;
    if (input > 0) return 1;
    return 0;
}
static inline float sat(const float input,const float minx,const float maxx,const float miny,const float maxy) {
    if (input < minx) return miny;
    if (input > maxx) return maxy;
    return (maxy-miny)/(maxx-minx)*(input-minx)+miny;
}

//SMO的IIR滤波器(已弃用，改为简单的低通滤波器)
float __attribute__((unused)) IIR_filter2Ealpha(float input);
float __attribute__((unused)) IIR_filter2Ebeta(float input);
float __attribute__((unused)) IIR_filter2SMO_Speed(float input);

//自动生成函数(简单的一阶低通滤波器,参数NAME:函数名称,f_c截止频率(hz))
#define GenerateFunction_LowPassFilter(NAME,f_c) \
    static float lowPass_filter_##NAME(float NAME##_input){\
    static float y_old=0;\
    const float k= PI2*T_s*f_c;\
    y_old=k*NAME##_input+(1-k)*y_old;\
    return y_old;\
    }

GenerateFunction_LowPassFilter(flux_Espeed,500)

//SMO的反电动势进行低通滤波的一阶滤波器的截止频率(hz)
#define SMO_E_lowPass_Fc 250
GenerateFunction_LowPassFilter(SMO_Ealpha,SMO_E_lowPass_Fc)
GenerateFunction_LowPassFilter(SMO_Ebeta,SMO_E_lowPass_Fc)
GenerateFunction_LowPassFilter(SMO_Espeed,15)

//ST-SMO的反电动势进行低通滤波的一阶滤波器的截止频率(hz)
#define ST_SMO_E_lowPass_Fc 400
GenerateFunction_LowPassFilter(ST_SMO_Ealpha,ST_SMO_E_lowPass_Fc)
GenerateFunction_LowPassFilter(ST_SMO_Ebeta,ST_SMO_E_lowPass_Fc)
GenerateFunction_LowPassFilter(ST_SMO_Espeed,25)

GenerateFunction_LowPassFilter(Ia,2000)
GenerateFunction_LowPassFilter(Ib,2000)

GenerateFunction_LowPassFilter(Id,3000)
GenerateFunction_LowPassFilter(Iq,3000)

#endif //FOC_SENSORLESS_OTHER_H/