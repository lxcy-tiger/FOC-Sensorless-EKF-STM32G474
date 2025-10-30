//
// Created by lxcy on 2025/10/10.
//
#include "PMSM_Control_Core/Hardware.h"

#include "adc.h"
#include "stm32g4xx_hal_adc.h"

float VCC_3V3=3.314f; //稳压模块输出电压
uint16_t IA_REF=4095*680/(680+2200)*2; //Ia偏置电压ADC值(12位ADC),根据电阻网络计算得到
uint16_t IB_REF=4095*680/(680+2200)*2; //Ib偏置电压ADC值(12位ADC),根据电阻网络计算得到
const float IA_K=0.33*2200/(680+2200)*2;//增益，由电阻网络计算得到(取正值)，单位V/A
const float IB_K=0.33*2200/(680+2200)*2;//增益，由电阻网络计算得到(取正值)，单位V/A
/*获取实际电流的步骤：
 * (ADC测量值-ADC偏置值)*VCC_3V3/4095.f === V0 为采样电阻电压
 * -V0/IA_K === IA 为A相电压(以流入电机为正方向)
*/

//校正VCC_3V3,IA_REF,IB_REF
void Offline_IabVcc_Adjust() {
    uint32_t IarefAddUp=0;
    uint32_t IbrefAddUp=0;
    uint32_t VrefAddUp=0;
    uint16_t ADCres[3];
    static const uint16_t Repeat=10000;
    //12位的数据，乘以16位计数，小于32位的数据(4095*10000<U32_MAX)
    for (uint16_t i=0;i<Repeat;i++) {
        HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADCres,3);
        while (HAL_DMA_GetState(hadc1.DMA_Handle) == HAL_DMA_STATE_BUSY);
        HAL_ADC_Stop_DMA(&hadc1);
        IarefAddUp+=ADCres[0];
        IbrefAddUp+=ADCres[1];
        VrefAddUp+=ADCres[2];
    }
    IA_REF=IarefAddUp/Repeat;
    IB_REF=IbrefAddUp/Repeat;
    VCC_3V3=(*VREFINT_CAL_ADDR)*3.f*Repeat/VrefAddUp;
//#define GIVEMORE
#ifdef GIVEMORE
    IA_REF-=70;
    IB_REF-=80;
#endif

}
