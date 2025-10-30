/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "USB_JustFloat.h"
#include "PMSM_Control_Core/Clarke_Park.h"
#include "PMSM_Control_Core/EKF.h"
#include "PMSM_Control_Core/Hardware.h"
#include "PMSM_Control_Core/PI_Controller.h"
#include "PMSM_Control_Core/SVPWM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_FS;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
  /*
   *获取ABC相电流
   */
  const uint32_t va_source=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  const uint32_t vb_source=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  //必须显式强转有符号整数，否则结果会被隐式转化为无符号整数
  const float Ia_raw=(IA_REF-(int32_t)va_source)*VCC_3V3/IA_K/4095.f;
  const float Ib_raw=(IB_REF-(int32_t)vb_source)*VCC_3V3/IB_K/4095.f;


  //中值滤波去毛刺
  const float Ia = median_filter_Ia_5(Ia_raw);
  const float Ib = median_filter_Ib_5(Ib_raw);
  //const float Ia=IIR_filter2A(Ia_median);
  //const float Ib=IIR_filter2B(Ib_median);
  //const float Ia = Ia_raw;
  //const float Ib = Ib_raw;
  const float Ic=-(Ia+Ib);

  /*
   * 执行一次Clarke，获取静止三相电流
   */
  ClarkePark.clarke.Ia_I=Ia;
  ClarkePark.clarke.Ib_I=Ib;
  ClarkePark.clarke.Ic_I=Ic;
  Clarke_transform(&ClarkePark.clarke);

  /*
   * 执行一次EKF,获取转子角度和速度
   * 注意这里输入电压取上一次中断时计算的电压(事实上应当为上上次中断时计算的电压)
   * 因为这次计算的电压，将会在下个周期作用
   */
  ekf_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
  ekf_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
  ekf_est.Valpha_I=ClarkePark.ipark.Valpha_O;
  ekf_est.Vbeta_I=ClarkePark.ipark.Vbeta_O;
  EKF_update(&ekf_est);

//#define ALL_OPENCIRCLE
#ifdef ALL_OPENCIRCLE
/*
 * 全部开环测试
 */
  static float Etheta=0;
  //500rpm*7*2*PI/60/20KHZ

  static const float Etheta_Add=500*7*PI2/60/20000;
  Etheta+=Etheta_Add;
  if(Etheta>PI2)Etheta-=PI2;

  ClarkePark.ipark.Vd_I=0;
  ClarkePark.ipark.Vq_I=2.1f;
  ClarkePark.ipark.Theta_I=Etheta;
  IPark_transform(&ClarkePark.ipark);
  SVPWM_Calculate_Set(ClarkePark.ipark.Valpha_O,ClarkePark.ipark.Vbeta_O);
  recordRunningData();
  return;
#endif
//#define THETA_OPENCIRCLE
#ifdef THETA_OPENCIRCLE
  /*
   * 角度开环测试
   */
  static float Etheta=0;
  //500rpm*7*2*PI/60/20KHZ

  static const float Etheta_Add=500*7*PI2/60/20000;
  Etheta+=Etheta_Add;
  if(Etheta>PI2)Etheta-=PI2;

  Speed_PIstate.Measure=ekf_est.Espeed_O;
  Speed_PI_update(&Speed_PIstate);

  ClarkePark.park.Ialpha_I=ClarkePark.clarke.Ialpha_O;
  ClarkePark.park.Ibeta_I=ClarkePark.clarke.Ibeta_O;
  ClarkePark.park.Theta_I=Etheta;
  Park_transform(&ClarkePark.park);

  Id_PIstate.Measure=ClarkePark.park.Id_O;
  Id_PI_update(&Id_PIstate);
  Iq_PIstate.Set=Speed_PIstate.Output;
  Iq_PIstate.Measure=ClarkePark.park.Iq_O;
  Iq_PI_update(&Iq_PIstate);

  ClarkePark.ipark.Vd_I=Id_PIstate.Output;
  ClarkePark.ipark.Vq_I=Iq_PIstate.Output;
  ClarkePark.ipark.Theta_I=Etheta;

  IPark_transform(&ClarkePark.ipark);
  SVPWM_Calculate_Set(ClarkePark.ipark.Valpha_O,ClarkePark.ipark.Vbeta_O);
  return;
#endif
  /*
   * 执行一次转速环，获取Q电流环给定
   */
  Speed_PIstate.Measure=ekf_est.Espeed_O;
  Speed_PI_update(&Speed_PIstate);

  /*
   * 执行一次Park，获取dq电流
   */
  ClarkePark.park.Ialpha_I=ClarkePark.clarke.Ialpha_O;
  ClarkePark.park.Ibeta_I=ClarkePark.clarke.Ibeta_O;
  ClarkePark.park.Theta_I=ekf_est.Etheta_O;
  Park_transform(&ClarkePark.park);

  /*
   * 执行一次dq电流环，获取Udq电压给定
   */
  Id_PIstate.Measure=ClarkePark.park.Id_O;
  Id_PI_update(&Id_PIstate);
  Iq_PIstate.Set=Speed_PIstate.Output;
  Iq_PIstate.Measure=ClarkePark.park.Iq_O;
//#define SpeedOpenCircle
#ifdef SpeedOpenCircle
  Iq_PIstate.Set=0.2f;
#endif
  Iq_PI_update(&Iq_PIstate);

  /*
   * 执行一次反park，获取Ualpha和Ubeta
   */
  ClarkePark.ipark.Vd_I=Id_PIstate.Output;
  ClarkePark.ipark.Vq_I=Iq_PIstate.Output;
  //#define CurrentOpenCircle
  #ifdef CurrentOpenCircle
  static float minous=0.f;
  if (minous>-.5f)minous-=3e-6f;
  ClarkePark.ipark.Vd_I=0;
  static float Addupppppp=2.1f;
  if (Addupppppp<6.f)Addupppppp+=1e-5f;
  ClarkePark.ipark.Vq_I=Addupppppp;
  #endif
  ClarkePark.ipark.Theta_I=ekf_est.Etheta_O;
  IPark_transform(&ClarkePark.ipark);

  /*
   * 对计算所得的矢量进行限幅(6.5V)
   */
  float modulus;
  arm_sqrt_f32(ClarkePark.ipark.Valpha_O*ClarkePark.ipark.Valpha_O
     +ClarkePark.ipark.Vbeta_O*ClarkePark.ipark.Vbeta_O,&modulus);
  if (modulus>6.5f) {
    ClarkePark.ipark.Valpha_O*=6.5f/modulus;
    ClarkePark.ipark.Vbeta_O*=6.5f/modulus;
  }

  /*
   * 执行一次SVPWM，更新计数值
   */
  SVPWM_Calculate_Set(ClarkePark.ipark.Valpha_O,ClarkePark.ipark.Vbeta_O);

  //记录数据
  recordRunningData();

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles USB low priority interrupt remap.
  */
void USB_LP_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_IRQn 0 */

  /* USER CODE END USB_LP_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_IRQn 1 */

  /* USER CODE END USB_LP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
