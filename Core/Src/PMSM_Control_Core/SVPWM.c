//
// Created by lxcy on 2025/9/30.
//
#include "PMSM_Control_Core/SVPWM.h"


static float Udc = 12;
static float DivUdc = 1.f/12;//母线电压倒数(随时更新)
void SetUdc(float newUdc){
    Udc=newUdc;
    DivUdc=1.f/newUdc;
}//更新Udc和DivUdc唯一接口
float GetUdc() {
    return Udc;
}//获取Udc唯一接口
static const uint16_t HRTIM_Period = 34000; //HRTIM的四个时钟的周期值(MASTER、A、C、D)
//将Ts设为计数值的两倍，那么A、C、D的比较值恰好等于计算出来的Ton，注意，这里要用U32，不然会溢出
static const uint32_t Ts = HRTIM_Period * 2;
static const uint16_t CompareMin = 100; //计数器最小计数值


//计算SVPWM并更新到HRTIM的计数值(内部实现简单非线性限幅，建议在外部对Valpha和Vbeta进行等比例限幅，以避免过调制)
void SVPWM_Calculate_Set(float Valpha, float Vbeta) {
    if (Udc == 0)return; //避免后续除法导致错误。

    //扇区(Sector)判断，A=Vbeta,B=(sqrt(3)*Valpha-Vbeta)/2,C=(-sqrt(3)*Valpha-Vbeta)/2
    //N=A+2B+4C
    //Sector(N)=1(3),2(1),3(5),4(4),5(6),6(2)
    const uint8_t Sector_Table[7] = {0, 2, 6, 1, 4, 3, 5}; //查表确定扇区，第一个值没用
    const float A = Vbeta, B = (Sqrt_3 * Valpha - Vbeta) / 2, C = (-Sqrt_3 * Valpha - Vbeta) / 2; //(-O2优化下自动提取公共部分)
    uint8_t N = 0;
    N += A > 0 ? 1 : 0;
    N += B > 0 ? 2 : 0;
    N += C > 0 ? 4 : 0;
    const uint8_t Sector = Sector_Table[N];
    const float sq3_Ts_divUdc = Sqrt_3 * Ts / Udc;
    const float X = sq3_Ts_divUdc * A, Y = -sq3_Ts_divUdc * C, Z = -sq3_Ts_divUdc * B;
    float Tx, Ty;
    switch (Sector) {
        case 1:
            Tx = -Z;
            Ty = X;
            break;
        case 2:
            Tx = Z;
            Ty = Y;
            break;
        case 3:
            Tx = X;
            Ty = -Y;
            break;
        case 4:
            Tx = -X;
            Ty = Z;
            break;
        case 5:
            Tx = -Y;
            Ty = -Z;
            break;
        case 6:
            Tx = Y;
            Ty = -X;
            break;
        default:
            Tx = 0;
            Ty = 0;
            break;
    }
    //-O2会优化的(浮点乘法)
    uint32_t small = (Ts - Tx - Ty) / 4, medium = (Ts + Tx - Ty) / 4, big = (Ts + Tx + Ty) / 4;
    small = clamp_u32(small, CompareMin, HRTIM_Period - CompareMin);
    medium = clamp_u32(medium, CompareMin, HRTIM_Period - CompareMin);
    big = clamp_u32(big, CompareMin, HRTIM_Period - CompareMin);
    switch (Sector) {
        case 1:
            HRTIM_Set_PWMCompare(small, medium, big);
            break;
        case 2:
            HRTIM_Set_PWMCompare(medium, small, big);
            break;
        case 3:
            HRTIM_Set_PWMCompare(big, small, medium);
            break;
        case 4:
            HRTIM_Set_PWMCompare(big, medium, small);
            break;
        case 5:
            HRTIM_Set_PWMCompare(medium, big, small);
            break;
        case 6:
            HRTIM_Set_PWMCompare(small, big, medium);
            break;
        default:
            //未知情况，将输出减到最小。
            HRTIM_Set_PWMCompare(HRTIM_Period - CompareMin, HRTIM_Period - CompareMin, HRTIM_Period - CompareMin);
            break;
    }
}
