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
uint8_t IF_startStep=1;//3步骤启动
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

  const int Observer=2;//0表示使用EKF,1表示使用FluxObserver-PLL,2表示使用SMO-PLL
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
  else if (Observer==1){
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
  else {
    /*
    * 执行一次SMO-PLL,获取转子角度和速度,注意SMO低速性能差,需要强拖启动
    * 强拖启动设计为3步骤：
    * 1.完全由强拖决定角度和速度(期间也要进行SMO估计)
    * 2.由强拖和SMO共同决定角度和速度(强拖的角度权重由1降为0,SMO的权重从0升到1)
    * 3.闭环到SMO
    */
    if (Speed_PIstate.Set==0)return;//在main函数发出转速给定前,先不要执行后面的代码,因为后面的强拖代码暂时设计为只能启动一次(以后会改为可多次启动)
    smo_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    smo_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    smo_pll_est.Valpha_I=ClarkePark.ipark.Valpha_O;
    smo_pll_est.Vbeta_I=ClarkePark.ipark.Vbeta_O;
    SMO_PLL_update(&smo_pll_est);
    static float IF_speed=0.0f;//IF启动速度
    static float IF_theta=0.0f;//IF启动角度
    static const float IF_speedAccelerate=0.005f;//IF启动加速度
    static const float IF_StartCurrent=0.3f;//IF启动电流
    static const float T_s=5e-5f;//SMO执行周期(20khz)
    static float SMO_thetaRecord=0;//记录切换时的SMO角度值(1切到2)
    static float end_CurrentSet=0;//阶段2的最终电流
    static const float Iq_subSpeed=0.00005f;//降低Iq的速率
    static const float Iq_addSpeed=0.0001f;//降低Iq的速率
    if (IF_startStep==1) {
      IF_speed+=IF_speedAccelerate;
      IF_theta+=IF_speed*T_s;
      while (IF_theta>=PI2)IF_theta-=PI2;
      while (IF_theta<0)IF_theta+=PI2;
      Espeed=IF_speed;
      Speed_PIstate.Measure=Espeed;
      Etheta=IF_theta;
      /*
      * 执行一次Park，获取dq电流
      */
      ClarkePark.park.Ialpha_I=ClarkePark.clarke.Ialpha_O;
      ClarkePark.park.Ibeta_I=ClarkePark.clarke.Ibeta_O;
      ClarkePark.park.Theta_I=Etheta;
      Park_transform(&ClarkePark.park);

      /*
       * 执行一次dq电流环，获取Udq电压给定
       */
      Id_PIstate.Measure=ClarkePark.park.Id_O;
      Id_PI_update(&Id_PIstate);
      Iq_PIstate.Set=IF_StartCurrent;
      Iq_PIstate.Measure=ClarkePark.park.Iq_O;
      Iq_PI_update(&Iq_PIstate);

      /*
       * 执行一次反park，获取Ualpha和Ubeta
       */
      ClarkePark.ipark.Vd_I=Id_PIstate.Output;
      ClarkePark.ipark.Vq_I=Iq_PIstate.Output;
      ClarkePark.ipark.Theta_I=Etheta;
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

      if (IF_speed>=Speed_PIstate.Set) {
        IF_startStep=2;
        SMO_thetaRecord=smo_pll_est.Etheta_O;
        static const float Speed_I_Ts_VAL=0.0000003f;
        Speed_PIstate.AddUp=Iq_PIstate.Set/Speed_I_Ts_VAL;
      }
      return;
    }
    else if (IF_startStep==2) {
      static const int32_t Smooth_ThetaTotalCounts=20000;//平滑角度切换到smo闭环
      static int32_t Smooth_ThetaCounts=0;//平滑角度切换到smo闭环
      IF_theta+=smo_pll_est.Espeed_O*T_s;
      SMO_thetaRecord+=smo_pll_est.Espeed_O*T_s;
      //前面不需要将它弄到0~2pi范围
      Etheta=(IF_theta*(Smooth_ThetaTotalCounts-Smooth_ThetaCounts)
        +SMO_thetaRecord*Smooth_ThetaCounts)/Smooth_ThetaTotalCounts;
      Espeed=smo_pll_est.Espeed_O;
      Speed_PIstate.Measure=Espeed;
      while (Etheta>=PI2)Etheta-=PI2;
      while (Etheta<0)Etheta+=PI2;
      /*
      * 执行一次转速环，获取Q电流环给定
      */
      Speed_PIstate.Measure=Espeed;
      Speed_PI_update(&Speed_PIstate);

      /*
      * 执行一次Park，获取dq电流
      */
      ClarkePark.park.Ialpha_I=ClarkePark.clarke.Ialpha_O;
      ClarkePark.park.Ibeta_I=ClarkePark.clarke.Ibeta_O;
      ClarkePark.park.Theta_I=Etheta;
      Park_transform(&ClarkePark.park);

      /*
       * 执行一次dq电流环，获取Udq电压给定
       */
      Id_PIstate.Measure=ClarkePark.park.Id_O;
      Id_PI_update(&Id_PIstate);
      Iq_PIstate.Set=Speed_PIstate.Output;
      Iq_PIstate.Measure=ClarkePark.park.Iq_O;
      Iq_PI_update(&Iq_PIstate);

      /*
       * 执行一次反park，获取Ualpha和Ubeta
       */
      ClarkePark.ipark.Vd_I=Id_PIstate.Output;
      ClarkePark.ipark.Vq_I=Iq_PIstate.Output;
      ClarkePark.ipark.Theta_I=Etheta;
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
      if (Smooth_ThetaCounts>=Smooth_ThetaTotalCounts && my_abs(smo_pll_est.Espeed_O-Speed_PIstate.Set)<20) {
        IF_startStep=3;
      }
      if (Smooth_ThetaCounts<Smooth_ThetaTotalCounts)Smooth_ThetaCounts++;
      return;
    }
    else {
      Espeed=smo_pll_est.Espeed_O;
      Etheta=smo_pll_est.Etheta_O;
    }
  }
  /*
   * 执行一次转速环，获取Q电流环给定
   */
  Speed_PIstate.Measure=Espeed;
  Speed_PI_update(&Speed_PIstate);

  /*
   * 执行一次Park，获取dq电流
   */
  ClarkePark.park.Ialpha_I=ClarkePark.clarke.Ialpha_O;
  ClarkePark.park.Ibeta_I=ClarkePark.clarke.Ibeta_O;
  ClarkePark.park.Theta_I=Etheta;
  Park_transform(&ClarkePark.park);

  /*
   * 执行一次dq电流环，获取Udq电压给定
   */
  Id_PIstate.Measure=ClarkePark.park.Id_O;
  Id_PI_update(&Id_PIstate);
  Iq_PIstate.Set=Speed_PIstate.Output;
  Iq_PIstate.Measure=ClarkePark.park.Iq_O;
  Iq_PI_update(&Iq_PIstate);

  /*
   * 执行一次反park，获取Ualpha和Ubeta
   */
  ClarkePark.ipark.Vd_I=Id_PIstate.Output;
  ClarkePark.ipark.Vq_I=Iq_PIstate.Output;
  ClarkePark.ipark.Theta_I=Etheta;
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
