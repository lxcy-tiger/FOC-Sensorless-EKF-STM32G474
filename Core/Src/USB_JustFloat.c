//
// Created by lxcy on 2025/10/23.
//
#include "USB_JustFloat.h"

#include "usbd_cdc_if.h"
#include "PMSM_Control_Core/Clarke_Park.h"
#include "PMSM_Control_Core/EKF.h"
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "PMSM_Control_Core/SMO_PLL.h"
#include "PMSM_Control_Core/ST-SMO_PLL.h"

// 采样分频系数：1表示每次都记录，2表示每2次记录1次
// (经测试,发送长度为9是USB传输速度的极限,但是我们可以通过每两次记录一次的方式,使得我们可以同时查看更多的变量)
#define USB_RECORD_DIVIDER 2


#if USB_RECORD_DIVIDER==1
#define singleDataLength 9 //单次记录需要的数据长度,注意这里要算上帧尾(以float为单位)
#define USB_HalfDataCount 100//填充数据的一半数量
#define USB_HalfDataLength (singleDataLength*USB_HalfDataCount)//填充数据的一半长度
#define USB_MaxDataRecordLength (USB_HalfDataLength*2)//当数据每被填满一半，就发送一次数据
static uint32_t USB_DataRecordIndex=0;//表示已经记录数据的下标
static float USB_data[USB_MaxDataRecordLength+1000];//长度为USB_MaxDataRecordLength+1000，留有一定裕量，防止超过数组下标(理论上不会超过)
unsigned char tail[4]={0x00,0x00,0x80,0x7f};
//记录电机运行和系统状态，用于发送给上位机
void recordRunningData() {
    USB_data[USB_DataRecordIndex++]=Iq_PIstate.Set;
    USB_data[USB_DataRecordIndex++]=Iq_PIstate.Measure;
    USB_data[USB_DataRecordIndex++]=Id_PIstate.Set;
    USB_data[USB_DataRecordIndex++]=Id_PIstate.Measure;
    USB_data[USB_DataRecordIndex++]=Speed_PIstate.Set;
    USB_data[USB_DataRecordIndex++]=Speed_PIstate.Measure;
    USB_data[USB_DataRecordIndex++]=ClarkePark.ipark.Valpha_O;
    USB_data[USB_DataRecordIndex++]=st_smo_pll_est.E_alpha_O;
    USB_data[USB_DataRecordIndex++]=*((float*)&tail[0]);
    if (USB_DataRecordIndex==USB_HalfDataLength) {
        CDC_Transmit_FS((uint8_t*)USB_data,USB_HalfDataLength*4);
    }
    else if (USB_DataRecordIndex==USB_MaxDataRecordLength) {
        CDC_Transmit_FS((uint8_t*)(USB_data+USB_HalfDataLength),USB_HalfDataLength*4);
        USB_DataRecordIndex=0;
    }
}
#elif USB_RECORD_DIVIDER==2

#define singleDataLength 18 //单次记录需要的数据长度,注意这里要算上帧尾(以float为单位)
#define USB_HalfDataCount 100//填充数据的一半数量
#define USB_HalfDataLength (singleDataLength*USB_HalfDataCount)//填充数据的一半长度
#define USB_MaxDataRecordLength (USB_HalfDataLength*2)//当数据每被填满一半，就发送一次数据
static uint32_t USB_DataRecordIndex=0;//表示已经记录数据的下标
static float USB_data[USB_MaxDataRecordLength+1000];//长度为USB_MaxDataRecordLength+1000，留有一定裕量，防止超过数组下标(理论上不会超过)
unsigned char tail[4]={0x00,0x00,0x80,0x7f};
//记录电机运行和系统状态，用于发送给上位机
void recordRunningData() {
    static int divCount=0;//每两次记录一次,这里加个变量记录到底达到两次没有
    divCount=(divCount+1)%USB_RECORD_DIVIDER;
    if (divCount==0) {
        USB_data[USB_DataRecordIndex++]=Iq_PIstate.Set;
        USB_data[USB_DataRecordIndex++]=Iq_PIstate.Measure;
        USB_data[USB_DataRecordIndex++]=Iq_PIstate.Output;

        USB_data[USB_DataRecordIndex++]=Id_PIstate.Set;
        USB_data[USB_DataRecordIndex++]=Id_PIstate.Measure;
        USB_data[USB_DataRecordIndex++]=Id_PIstate.Output;

        USB_data[USB_DataRecordIndex++]=Speed_PIstate.Set;
        USB_data[USB_DataRecordIndex++]=Speed_PIstate.Measure;

        USB_data[USB_DataRecordIndex++]=ClarkePark.ipark.Valpha_O;
        USB_data[USB_DataRecordIndex++]=ClarkePark.ipark.Vbeta_O;

        USB_data[USB_DataRecordIndex++]=smo_pll_est.Espeed_O;
        USB_data[USB_DataRecordIndex++]=smo_pll_est.Etheta_O;
        extern uint8_t IF_Start_Step;
        extern float IF_ETheta;//IF角度
        extern float IF_ESpeed;//IF速度
        USB_data[USB_DataRecordIndex++]=IF_Start_Step;
        USB_data[USB_DataRecordIndex++]=IF_ETheta;
        USB_data[USB_DataRecordIndex++]=IF_ESpeed;

        USB_data[USB_DataRecordIndex++]=ClarkePark.clarke.Ia_I;
        USB_data[USB_DataRecordIndex++]=ClarkePark.ipark.Theta_I;


        USB_data[USB_DataRecordIndex++]=*((float*)&tail[0]);
        if (USB_DataRecordIndex==USB_HalfDataLength) {
            CDC_Transmit_FS((uint8_t*)USB_data,USB_HalfDataLength*4);
        }
        else if (USB_DataRecordIndex==USB_MaxDataRecordLength) {
            CDC_Transmit_FS((uint8_t*)(USB_data+USB_HalfDataLength),USB_HalfDataLength*4);
            USB_DataRecordIndex=0;
        }
    }
}

#endif
