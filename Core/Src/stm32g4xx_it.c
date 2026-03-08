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
#include "PMSM_Control_Core/FluxObserver_PLL.h"
#include "PMSM_Control_Core/SMO_PLL.h"
#include "PMSM_Control_Core/ST-SMO_PLL.h"
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
extern ADC_HandleTypeDef hadc2;
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
  static volatile uint8_t adc1_done = 0;
  static volatile uint8_t adc2_done = 0;
  if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS))
  {
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
    adc1_done = 1;
  }
  if (__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS))
  {
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS);
    adc2_done = 1;
  }
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
  //HRTIM每次更新时同时触发ADC1,ADC2采样Ia,Ib,因此此中断一个周期触发两次，必须要等两路电流都采集完,才进行控制
  if (adc1_done && adc2_done)
  {
    adc1_done = 0;
    adc2_done = 0;
  }
  else return;
  /*
   *获取ABC相电流
   */
  const uint32_t va_source=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  const uint32_t vb_source=HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
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
  //const float Ia =lowPass_filter_Ia(Ia_raw);
  //const float Ib =lowPass_filter_Ib(Ib_raw);
  const float Ic=-(Ia+Ib);

  /*
   * 执行一次Clarke，获取静止三相电流
   */
  ClarkePark.clarke.Ia_I=Ia;
  ClarkePark.clarke.Ib_I=Ib;
  ClarkePark.clarke.Ic_I=Ic;
  Clarke_transform(&ClarkePark.clarke);

  const int Observer=0;//0表示使用EKF,1表示使用FluxObserver-PLL,2表示使用SMO-PLL,3表示使用ST-SMO_PLL
  float Espeed=0;
  float Etheta=0;
  /*
  * 注意这里输入电压取上一次中断时计算的电压,事实上应当取上上次中断时计算的电压
  * 时刻图:
  * 中断时刻0 周期0 中断时刻1 周期1 中断时刻2 周期2
  * 中断时刻0计算的电压(此时已经进入周期0),将会在周期1生效
  * 所以说我们在中断时刻2应当取作用在周期1的电压
  * 即 中断时刻0计算的电压
  * 但是我们的ClarkePark.ipark变量只记录了前一个周期即中断时刻1计算的电压
  * 由于中断时刻0和中断时刻1计算的电压不会有太大的差异，所以将就一下也能用
  */
  if (Observer==0) {
    /*
    * 执行一次EKF,获取转子角度和速度
    */
    ekf_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    ekf_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    ekf_est.Valpha_I=ClarkePark.ipark.Valpha_O;
    ekf_est.Vbeta_I=ClarkePark.ipark.Vbeta_O;
    EKF_update(&ekf_est);
    Espeed=ekf_est.Espeed_O;
    Etheta=ekf_est.Etheta_O;
  }
  else if (Observer>=1){
    /*
    * 执行一次FluxObserver-PLL,获取转子角度和速度
    */
    fluxObserver_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    fluxObserver_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    fluxObserver_pll_est.Valpha_I=ClarkePark.ipark.Valpha_O;
    fluxObserver_pll_est.Vbeta_I=ClarkePark.ipark.Vbeta_O;
    FluxObserver_PLL_update(&fluxObserver_pll_est);
    Espeed=fluxObserver_pll_est.Espeed_O;
    Etheta=fluxObserver_pll_est.Etheta_O;
  }
  //注:if从此处断开，请留意

  if (Observer==2){
    /*
    * 执行一次SMO-PLL,获取转子角度和速度,注意SMO低速性能差,这里先使用磁链观测器把速度提上来再切换到SMO
    */
    smo_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    smo_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    smo_pll_est.Valpha_I=ClarkePark.ipark.Valpha_O;
    smo_pll_est.Vbeta_I=ClarkePark.ipark.Vbeta_O;
    SMO_PLL_update(&smo_pll_est);
    static int StartCount=0;
    if (StartCount>=3*20000) {//三秒后切换到SMO
      Espeed=smo_pll_est.Espeed_O;
      Etheta=smo_pll_est.Etheta_O;
    }
    else StartCount++;
  }
  else if (Observer==3) {
    /*
    * 执行一次ST-SMO-PLL,获取转子角度和速度,注意ST-SMO低速性能差,这里先使用磁链观测器把速度提上来再切换到ST-SMO
    */
    st_smo_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    st_smo_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    st_smo_pll_est.Valpha_I=ClarkePark.ipark.Valpha_O;
    st_smo_pll_est.Vbeta_I=ClarkePark.ipark.Vbeta_O;
    ST_SMO_PLL_update(&st_smo_pll_est);
    static int StartCount=0;
    if (StartCount>=3*20000) {//三秒后切换到ST-SMO
      Espeed=st_smo_pll_est.Espeed_O;
      Etheta=st_smo_pll_est.Etheta_O;
    }
    else StartCount++;
  }


  /*
   * 每隔10次执行一次转速环(即20khz/10=2khz)，获取Q电流环给定
   */
  static int Speed_RunCount=0;
  Speed_RunCount++;
  if (Speed_RunCount>=10) {
    Speed_PIstate.Measure=Espeed;
    Speed_PI_update(&Speed_PIstate);
    Speed_RunCount=0;
  }

  /*
   * 执行一次Park，获取dq电流
   */
  ClarkePark.park.Ialpha_I=ClarkePark.clarke.Ialpha_O;
  ClarkePark.park.Ibeta_I=ClarkePark.clarke.Ibeta_O;
  ClarkePark.park.Theta_I=Etheta;
  Park_transform(&ClarkePark.park);

  /*
   * 低通滤波滤除id,iq高频分量
   */
  float id_lowpass = lowPass_filter_Id(ClarkePark.park.Id_O);
  float iq_lowpass = lowPass_filter_Iq(ClarkePark.park.Iq_O);
  /*
   * 执行一次dq电流环，获取Udq电压给定
   */
  Id_PIstate.Measure=id_lowpass;
  Id_PI_update(&Id_PIstate);
  Iq_PIstate.Set=Speed_PIstate.Output;
  Iq_PIstate.Measure=iq_lowpass;
  Iq_PI_update(&Iq_PIstate);

  /*
  * dq电流解耦与反电动势前馈
  * 根据公式：
  * ud=Rsid+Ld(did/dt)-weLqiq
  * uq=Rsiq+Lq(diq/dt)+we(Ldid+flux)
  * 可知这里得到的dq电流环输出ud,uq，并不能准确反映id或者iq的大小,通常这里可以减去耦合项we*xxx
  */
  const float ud_decoupling=-Espeed*Ls*Iq_PIstate.Measure;
  const float uq_decoupling=Espeed*(Ls*Id_PIstate.Measure+flux);

  /*
   * 执行一次反park，获取Ualpha和Ubeta
   */
  ClarkePark.ipark.Vd_I=Id_PIstate.Output+ud_decoupling;
  ClarkePark.ipark.Vq_I=Iq_PIstate.Output+uq_decoupling;
  ClarkePark.ipark.Theta_I=Etheta;
  IPark_transform(&ClarkePark.ipark);

  /*
   * 对计算所得的矢量进行限幅(6.5V)
   */
  float modulus;
  arm_sqrt_f32(ClarkePark.ipark.Valpha_O*ClarkePark.ipark.Valpha_O
     +ClarkePark.ipark.Vbeta_O*ClarkePark.ipark.Vbeta_O,&modulus);
  if (modulus>6.5f) {
    ClarkePark.ipark.Valpha_O *= 6.5f / modulus;
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
