#include "app_Routine.h"
#include "stm32f3xx_it.h"

#include "func_CurrentCtrl.h"
#include "drv_UartSendCtrl.h"

#include "mod_NTCSensor.h"

#include "tim.h"

#include <stdio.h>

#include "hrtim.h"
// #include "usart.h"

#include "drv_BridgeHW.h"
#include "drv_ADCSense.h"

#include "func_PowerCtrl.h"
#include "func_ChargeSafety.h"
#include "func_LED.h"
#include "func_Observer.h"
#include "drv_SetPlanning.h"
#include "func_Sense.h"
#include "drv_utils.h"
#include "func_SafetyProtect.h"
#include "drv_ADCSense.h"

#include "drv_Time.h"

#include "drv_Monitor.h"

#include "GeneralMacros.h"

#include "mod_ADCProcess.h"

#include "drv_Monitor.h"

#include "stm32f3xx_hal_hrtim.h"

#define DBGMCU_APB1_FZ_REG (*(__IO uint32_t *)0xE0042008)
#define IWDG_HALT_DISABLE() (DBGMCU_APB1_FZ_REG |= (1 << 12))
#define IWDG_HALT_ENABLE() (DBGMCU_APB1_FZ_REG &= (~(1 << 12)))

float set_current = -2.f;

Key_S Key;

// TODO: 对于电感电流做单独监测，根据电感实际电流计算现在是不是存在过流情况
/**
 * @brief 用于处理最高优先级的任务, 触发频率大约 6000Hz
 * @author fwlh
 */
float test_p = 0;
float calculated_p = 0;
uint32_t inttime = 0;
static void TaskList1(void)
{

    // 计数器累加
    Time_Run();
    inttime++;
#if (WATCHDOG_ENABLED == 1)
    // 喂看门狗
    MOnitor_Objcet_Feed(HRTIM_Working);
#endif /* WATCHDOG_ENABLED */

    // 处理 ADC 的原始数据
    ADC_Routine(Read_Current_Time());
		
#if (DEBUG)
     if ((Read_Now_Data(DataType_U_CAP, ADC_Filted_Data) > 27.3f))
     {
         set_current = -2.f;
     }
     else if ((Read_Now_Data(DataType_U_CAP, ADC_Filted_Data) < 13))
         set_current = 2.f;

    // 修改电流设定值，调试用
    Current_FreshSet(set_current);
    //BridgeHW_TGGLWrite(0.05, 1.01, 0.2, 0.31);
		//hhrtim1.Instance->sCommonRegs.OENR |= 0x0F;
    //BridgeHW_SetPWM(Read_Now_Data(DataType_I_BH, ADC_Filted_Data));
#endif
    // 电流环的计算
    CurrentCtrl_Routine(Read_Current_Time());
    //    GPIOB->BSRR = ((GPIOB->ODR & GPIO_PIN_7) << 16U) | (~(GPIOB->ODR) & GPIO_PIN_7);
}

/**
 * @brief 用于处理优先级第二高的任务, 触发频率大约 1000Hz
 * @author fwlh
 */
static void TaskList2(void)
{
#if (WATCHDOG_ENABLED == 1)
    // 喂看门狗
    MOnitor_Objcet_Feed(TIM2_Working);
#endif /* WATCHDOG_ENABLED */

    // fault检测轮询
    FaultTrigger_Routine();

    // TODO: 尝试测试一下老二代电容的观测器程序
    SCAP_Observer(Read_Now_Data(DataType_U_CAP, ADC_Filted_Data), Read_Current_Time());

    // 刷新按键状态
    Key_Rec(&Key, !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));

#if (!DEBUG)
    // 功率环控制轮询
    Power_Ctrl_Routine();
#else
    Battery_Cal();
#endif
}

/**
 * @brief 用于处理优先级第三高的任务, 触发频率大约 10Hz
 * @author fwlh
 */
static void TaskList3(void)
{
#if (WATCHDOG_ENABLED == 1)
    // 喂看门狗
    MOnitor_Objcet_Feed(TIM7_Working);
#endif /* WATCHDOG_ENABLED */
    // led轮询
    LED_Routine();

#if (!DEBUG)
    // 底盘断电保护
    PowerOutage_Routine();
#endif
#if (WATCHDOG_ENABLED == 1)
    // 检测看门狗
    Monitor_Objcet_Check();
#endif /* WATCHDOG_ENABLED */
}

// TIMA 的复位事件回调函数
// void HAL_HRTIM_CounterResetCallback(HRTIM_HandleTypeDef *hhrtim,
//                                     uint32_t TimerIdx)
// {

//     __HAL_HRTIM_TIMER_DISABLE_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_TIM_IT_RST);
// }

/**
 * @brief Init the General Ctrl status.
 * @author fwlh
 */
void GeneralCtrl_Init(void)
{

    // Start the HRTIM Timer.
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
	  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 |
                                                HRTIM_OUTPUT_TA2 |
                                                HRTIM_OUTPUT_TB1 |
                                                HRTIM_OUTPUT_TB2);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_E);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);

    __HAL_HRTIM_TIMER_DISABLE_IT(&hhrtim1, 0, HRTIM_TIM_IT_RST);
    // Start the TIM2 timer.
    LL_TIM_EnableIT_UPDATE(TIM2);
    LL_TIM_EnableCounter(TIM2);
    TIM2_UPDATE_INT_Callback_Register(TaskList2);
    TIM6_UPDATE_INT_Callback_Register(TaskList3);

    // Set the ADC-DMA interupt callback.
    ADC_Sense_SetCallback(TaskList1);

    LL_TIM_EnableIT_UPDATE(TIM15);
    LL_TIM_EnableCounter(TIM15);
    TIM15_UPDATE_INT_Callback_Register(TIM15_ADC_TRIGGER);

    // 初始化按键
    Key_Init(&Key, 20, 2000, GPIO_PIN_13);

    // 初始化fault检测
    FaultTrigger_Init();

#if (WATCHDOG_ENABLED == 1)
    MX_IWDG_Init();
#if (!DEBUG)
    IWDG_HALT_ENABLE();
#else
    IWDG_HALT_DISABLE();
#endif /* WATCHDOG_ENABLED */

#endif
}
