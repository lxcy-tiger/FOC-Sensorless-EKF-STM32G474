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
#include "PMSM_Control_Core/FluxWeakening.h"
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
/*
 *  IF强拖切闭环步骤
 *  数字0:IF强拖步骤,速度不停增加(IdIq电流环开启,转速环开启,观测器开启,角度:由IF给定)
 *        数字0->数字1:单周期坐标变换，瞬间切换
 *  数字1:IF强拖结束并闭环转速环和观测器,减小Id给定进行收尾(IdIq电流环开启,转速环开启,观测器开启,角度:由观测器给定)
*/
uint8_t IF_Start_Step=0;
static const float IF_IqCurrentTarget=0.6f;//IF强拖电流最终给定(给定Iq电流环最终值,单位A)
static const float IF_IqCurrentAcceleration=0.5f;//IF强拖电流增加速度(A/s)
float IF_IqCurrent=0.0f;//IF强拖电流实际值(Iq给定值以每秒IF_IqCurrentAcceleration的速度增加,慢慢上升直到IF_IqCurrentTarget)
static const float IF_Acceleration=100.f;//IF加速度(表示强拖速度增加的速度，单位rad/s^2)
static const float IF_Target_Speed=500.f;//IF目标速度(表示结束强拖应该达到的速度,即IF_Start_Step何时变为1的阈值,单位rad/s)
float IF_ETheta=0.0f;//IF角度
float IF_ESpeed=0.0f;//IF速度
float OB_ETheta=0.0f;//观测器角度
float OB_ESpeed=0.0f;//观测器速度
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


  //低通滤波去毛刺(虽然中值滤波能很好去除电流采样毛刺,但是会恶化控制效果,这里使用低通滤波代替了)
  //const float Ia = median_filter_Ia_5(Ia_raw);
  //const float Ib = median_filter_Ib_5(Ib_raw);
  //const float Ia=IIR_filter2A(Ia_median);
  //const float Ib=IIR_filter2B(Ib_median);
  //const float Ia = Ia_raw;
  //const float Ib = Ib_raw;
  const float Ia =lowPass_filter_Ia(Ia_raw);
  const float Ib =lowPass_filter_Ib(Ib_raw);
  const float Ic=-(Ia+Ib);

  /*
   * 执行一次Clarke，获取静止两相电流
   */
  ClarkePark.clarke.Ia_I=Ia;
  ClarkePark.clarke.Ib_I=Ib;
  ClarkePark.clarke.Ic_I=Ic;
  Clarke_transform(&ClarkePark.clarke);

  const int Observer=1;//0表示使用EKF,1表示使用FluxObserver-PLL,2表示使用SMO-PLL,3表示使用ST-SMO_PLL
  float Espeed=0;
  float Etheta=0;
  static float Valpha_last=0.0f;
  static float Vbeta_last=0.0f;
  /*
  * 注意这里输入电压取上一次中断时计算的电压,事实上应当取上上次中断时计算的电压
  * 时刻图:
  * 中断时刻0 周期0 中断时刻1 周期1 中断时刻2 周期2
  * 中断时刻0计算的电压(此时已经进入周期0),将会在周期1生效
  * 所以说我们在中断时刻2应当取作用在周期1的电压
  * 即 中断时刻0计算的电压
  * 但是我们的ClarkePark.ipark变量只记录了前一个周期即中断时刻1计算的电压
  * 我们使用ClarkePark.ipark的电压会有偏差
  * 所以我们使用了Valpha_last和Vbeta_last记录了上上次计算的电压，更为精确
  * 中断时刻0，ipark计算前，记录Valpha_last和Vbeta_last=ipark(即中断时刻-1的ipark)，计算ipark(中断时刻0的ipark)
  * 中断时刻1，ipark计算前，记录Valpha_last和Vbeta_last=ipark(即中断时刻0的ipark)，
  * 中断时刻2，执行观测器，使用Valpha_last和Vbeta_last(即中断时刻0的ipark)，
  */
  if (Observer==0) {
    /*
    * 执行一次EKF,获取转子角度和速度
    */
    ekf_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    ekf_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    ekf_est.Valpha_I=Valpha_last;
    ekf_est.Vbeta_I=Vbeta_last;
    EKF_update(&ekf_est);
    OB_ESpeed=ekf_est.Espeed_O;
    OB_ETheta=ekf_est.Etheta_O;
  }
  else if (Observer==1){
    /*
    * 执行一次FluxObserver-PLL,获取转子角度和速度
    */
    fluxObserver_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    fluxObserver_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    fluxObserver_pll_est.Valpha_I=Valpha_last;
    fluxObserver_pll_est.Vbeta_I=Vbeta_last;
    FluxObserver_PLL_update(&fluxObserver_pll_est);
    OB_ESpeed=fluxObserver_pll_est.Espeed_O;
    OB_ETheta=fluxObserver_pll_est.Etheta_O;
  }
  else if (Observer==2){
    /*
    * 执行一次SMO-PLL,获取转子角度和速度,注意SMO低速性能差,这里统一强拖切闭环
    */
    smo_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    smo_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    smo_pll_est.Valpha_I=Valpha_last;
    smo_pll_est.Vbeta_I=Vbeta_last;
    SMO_PLL_update(&smo_pll_est);
    OB_ESpeed=smo_pll_est.Espeed_O;
    OB_ETheta=smo_pll_est.Etheta_O;
  }
  else if (Observer==3) {
    /*
    * 执行一次ST-SMO-PLL,获取转子角度和速度,注意ST-SMO低速性能差,这里统一强拖切闭环
    */
    st_smo_pll_est.Ialpha_I=ClarkePark.clarke.Ialpha_O;
    st_smo_pll_est.Ibeta_I=ClarkePark.clarke.Ibeta_O;
    st_smo_pll_est.Valpha_I=Valpha_last;
    st_smo_pll_est.Vbeta_I=Vbeta_last;
    ST_SMO_PLL_update(&st_smo_pll_est);

    OB_ESpeed=st_smo_pll_est.Espeed_O;
    OB_ETheta=st_smo_pll_est.Etheta_O;
  }

  if (IF_Start_Step==0) {
    IF_ESpeed+=IF_Acceleration*T_s;
    IF_ETheta+=IF_ESpeed*T_s;
    while (IF_ETheta>=PI2)IF_ETheta-=PI2;
    while (IF_ETheta<0)IF_ETheta+=PI2;
    Espeed=IF_ESpeed;
    Etheta=IF_ETheta;
    if (IF_ESpeed>=IF_Target_Speed) {
      //强拖过渡阶段，进行两次坐标变换切换闭环
      IF_Start_Step=1;
      ClarkePark.park.Ialpha_I=Id_PIstate.Set;
      ClarkePark.park.Ibeta_I=Iq_PIstate.Set;
      ClarkePark.park.Theta_I=OB_ETheta-IF_ETheta;
      Park_transform(&ClarkePark.park);
      Id_PIstate.Set=ClarkePark.park.Id_O;
      Iq_PIstate.Set=ClarkePark.park.Iq_O;

      ClarkePark.park.Ialpha_I=Id_PIstate.Output;
      ClarkePark.park.Ibeta_I=Iq_PIstate.Output;
      ClarkePark.park.Theta_I=OB_ETheta-IF_ETheta;
      Park_transform(&ClarkePark.park);
      Id_PIstate.Output=ClarkePark.park.Id_O;
      Iq_PIstate.Output=ClarkePark.park.Iq_O;

      Id_PIstate.AddUp=Id_PIstate.Output*(1/Current_I_Ts_VAL);
      Iq_PIstate.AddUp=Iq_PIstate.Output*(1/Current_I_Ts_VAL);

      Speed_PIstate.AddUp=(Iq_PIstate.Set-(Speed_PIstate.Set-Speed_PIstate.Measure)*Speed_P_VAL)*(1/Speed_I_Ts_VAL);
      Speed_PIstate.Output=Iq_PIstate.Set;

      Etheta=OB_ETheta;
    }
  }
  else {
    Espeed=OB_ESpeed;
    Etheta=OB_ETheta;
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
   * (这里注意强拖时坐标变换切换闭环,低通滤波可能会导致Set与Measure有一些差异,
   *    但是由于低通滤波截止频率比较大(k≈0.94),所以误差并不大,可以忽略,
   *    当然我也不知道截止频率那么大的低通滤波在这里有什么用(或许该把截止频率弄小一点),但是既然写了,也懒得删除了)
   */
  float id_lowpass = lowPass_filter_Id(ClarkePark.park.Id_O);
  float iq_lowpass = lowPass_filter_Iq(ClarkePark.park.Iq_O);
  /*
   * 执行一次dq电流环，获取Udq电压给定
   */
  if (Id_PIstate.Set>=0.f)Id_PIstate.Set-=0.00001f;
  else Id_PIstate.Set=0.f;
  //使用暂时不要使用弱磁，Id_PIstate.Set=GetIdSet_Weak(Speed_PIstate.Set,Speed_PIstate.Measure);
  Id_PIstate.Measure=id_lowpass;
  Id_PI_update(&Id_PIstate);
  if (IF_Start_Step==0) {
    if (IF_IqCurrent<IF_IqCurrentTarget)IF_IqCurrent+=IF_IqCurrentAcceleration*T_s;
    Iq_PIstate.Set=IF_IqCurrent;
  }
  else {
    Iq_PIstate.Set=Speed_PIstate.Output;
  }
  Iq_PIstate.Measure=iq_lowpass;
  Iq_PI_update(&Iq_PIstate);

  /*
   * 执行反park前，将上次设置的Ualpha和Ubeta记录下来，用于下一次中断时的观测器预测
   */
  Valpha_last=ClarkePark.ipark.Valpha_O;
  Vbeta_last=ClarkePark.ipark.Vbeta_O;

  /*
  * dq电流解耦与反电动势前馈
  * 根据公式：
  * ud=Rsid+Ld(did/dt)-weLqiq
  * uq=Rsiq+Lq(diq/dt)+we(Ldid+flux)
  * 可知这里得到的dq电流环输出ud,uq，并不能准确反映id或者iq的大小,通常这里可以减去耦合项we*xxx
  */
  static const uint8_t use_decoupling=0;//是否使用前馈解耦
  const float ud_decoupling=use_decoupling ? -Espeed*Ls*Iq_PIstate.Measure : 0;
  const float uq_decoupling=use_decoupling ? Espeed*(Ls*Id_PIstate.Measure+flux) : use_decoupling;

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
  SVPWM_Calculate_Set(ClarkePark.ipark.Valpha_O,ClarkePark.ipark.Vbeta_O,Ia,Ib,Ic);

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
