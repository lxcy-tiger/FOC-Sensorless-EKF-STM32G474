//
// Created by lxcy on 2025/10/23.
//
#include "USB_JustFloat.h"

#include "usbd_cdc_if.h"
#include "PMSM_Control_Core/Clarke_Park.h"
#include "PMSM_Control_Core/EKF.h"
uint32_t USB_DataRecordIndex=0;//表示已经记录数据的下标
float USB_data[USB_MaxDataRecordLength+1000];//长度为USB_MaxDataRecordLength+1000，留有一定裕量，防止超过数组下标(理论上不会超过)
unsigned char tail[4]={0x00,0x00,0x80,0x7f};
//记录电机运行和系统状态，用于发送给上位机
void recordRunningData() {
    USB_data[USB_DataRecordIndex++]=ClarkePark.park.Ialpha_I;
    USB_data[USB_DataRecordIndex++]=ClarkePark.park.Ibeta_I;
    USB_data[USB_DataRecordIndex++]=Speed_PIstate.Set;
    USB_data[USB_DataRecordIndex++]=Speed_PIstate.Measure;
    USB_data[USB_DataRecordIndex++]=ClarkePark.ipark.Valpha_O;
    USB_data[USB_DataRecordIndex++]=ClarkePark.ipark.Vbeta_O;
    USB_data[USB_DataRecordIndex++]=ekf_est.Etheta_O;
    USB_data[USB_DataRecordIndex++]=*((float*)&tail[0]);
    if (USB_DataRecordIndex==USB_HalfDataLength) {
        CDC_Transmit_FS((uint8_t*)USB_data,USB_HalfDataLength*4);
    }
    else if (USB_DataRecordIndex==USB_MaxDataRecordLength) {
        CDC_Transmit_FS((uint8_t*)(USB_data+USB_HalfDataLength),USB_HalfDataLength*4);
        USB_DataRecordIndex=0;
    }
}