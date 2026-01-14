#include <rtthread.h>
#include <board.h>
#include "can_com_thread.h"
#include "app_RefSystem.h"
#include "SuperCap_Com.h"
#include "mod_GimbalComTx.h"
#include "app_ChassisCtrl.h"
#include "ThreadCtrl.h"
#include "func_MonHandling.h"
#include "app_UI.h"
#include "custom_message.h"
#ifdef BSP_CHASSIS_USE_SPI_CAN
#include "drv_spiCanMaster.h"
#endif
#include "app_monitor.h"
#ifdef BSP_USING_RGB_LIGHT
#include "func_RGB_Display.h"
#endif
#ifdef BSP_USING_BUZZER
#include "drv_buzzer.h"
#endif
#if defined(CORE_USING_REMOTE) || defined(BSP_USING_VOFA)
#include "onlyChassis.h"
#endif

#ifdef BSP_USING_VISUAL_USART
#include "func_Visual_Com.h"
#include "sentry_cmd.h"
#endif
#include "protect.h"
#include "fun_odometer.h"

#ifdef BSP_USING_SD_MASTER
#include "func_sd_master.h"
#endif

#define LOG_TAG     "main"
#define LOG_LVL     LOG_LVL_DBG
#include <ulog.h>
#include "mod_Log.h"

int main(void)
{
    int result=0;
    // 上电提示音
#ifdef BSP_USING_BUZZER
    set_buzzer(2000, 1);
    rt_thread_mdelay(120);
    set_buzzer(0, 1);
#endif
#ifdef BSP_USING_SD_MASTER
    sd_master_init(THREAD_PRIO_SD_MASTER);
    sd_master_vofa_create(15, "power&heat.csv", 6, "Time", "Power", "Pwr_Lim", "Buffer", "Heat", "Ht_Lim");
    sd_master_mkFile(0, "Result.txt");
    sd_master_mkFile(1, "Pos.txt");
	
    sd_master_mkFile(2, "HP.txt");
    sd_master_mkFile(3, "LowFreq_Status.txt");
    sd_master_mkFile(4, "debug.txt");
#endif
    if (MONITOR_INIT(THREAD_STACK_MONITOR, THREAD_PRIO_MONITOR, THREAD_TICK_MONITOR) != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#ifdef BSP_USING_RGB_LIGHT
    RGB_Display_Init();
#endif

    /* SoftMod */
    result = CAN_Init();
    if (result!= RT_EOK)
    {
        LOG_E("CAN Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else 
    {
        LOG_I("CAN Init Success!");
    }

    /* Communication */
#ifdef BSP_REF_USART_NAME
    result = DJI_Init();
    if (result != RT_EOK)
    {
        LOG_E("DJI Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else 
    {
        LOG_I("DJI Init Success!");
    }
#endif
    if (Scpr_Com_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (Gimbal_Com_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);

    /* Main Module */
    if (Chassis_Ctrl_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);

#ifdef BSP_USING_CHASSIS_IMU
    if (ChassisIMU_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#endif

#ifdef BSP_USING_VISUAL_USART
    if (VisualInit() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#endif

#ifdef CORE_USING_REMOTE
    if (OnlyChassis_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#endif

#ifdef CORE_USING_SENTRY
    if (Sentry_Data_Send_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#else
    if (UI_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
    if (Custom_Message_Data_Send_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#endif

    if (protct_ThreadInit() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);

    return RT_EOK;
}
