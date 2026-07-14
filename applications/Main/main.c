#include <rtthread.h>
#include <board.h>

#include "SuperCap_Com_new.h"
#include "drv_Gimbal_Com.h"
#include "app_ChassisCtrl.h"
#include "func_MonHandling.h"
//#include "app_UI.h"
#include "app_RefSystem.h"

#ifdef BSP_USING_RGB_LIGHT
#include "func_RGB_Display.h"
#endif
#if defined(CORE_USING_REMOTE) || defined(BSP_USING_VOFA)
#include "onlyChassis.h"
#endif
#ifdef BSP_USING_VISUAL_USART
#include "func_Visual_Com.h"
#endif
#include "protect.h"
#include "drv_IMU_Can.h"

/*公共库模块*/
#include "app_monitor.h"
#include "mod_Log.h"
#include "can_com_thread.h"
#ifdef BSP_USING_BUZZER
#include "drv_buzzer.h"
#endif
#ifdef USE_SPI_CAN_SLAVE
#include "drv_SPICAN.h"
#endif
#ifdef BSP_USING_REF
#include "app_RefSystem.h"
#endif
#ifdef BSP_USING_LOG
#define LOG_TAG "main"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#endif
int main(void){
    int result = 0;
    /*上电提示音*/
#ifdef BSP_USING_BUZZER
    set_buzzer(2000, 1);
    rt_thread_mdelay(120);
    set_buzzer(0, 1);
#endif
    /*监视器*/
    if (MONITOR_INIT(THREAD_STACK_MONITOR, THREAD_PRIO_MONITOR, THREAD_TICK_MONITOR) != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);	
    /*灯效控制*/
#ifdef BSP_USING_RGB_LIGHT
    RGB_Display_Init();
#endif

    /* 通信 */
    result = CAN_Init();
    if (result != RT_EOK)
    {
        LOG_E("CAN Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("CAN Init Success!");
    }
#ifdef BSP_USING_REF
    /*裁判系统*/
    result = Ref_Init();
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
    /*超电通信*/
    result = Scpr_Com_Init();
    if (result != RT_EOK)
    {
        LOG_E("Scpr_Com Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("Scpr_Com Init Success!");
    }

    /*云台通信*/
   result = Gimbal_Com_Init();
    if (result != RT_EOK)
    {
        LOG_E("Gimbal_Com Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("Gimbal_Com Init Success!");
    }

    // /*IMU CAN接收*/
    result = IMU_Can_Init();
    if (result != RT_EOK)
    {
        LOG_E("IMU_Can_Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("IMU_Can_Init Success!");
    }

    /* 控制 */
    result = Chassis_Ctrl_Init();
    if (result != RT_EOK)
    {
        LOG_E("Chassis_Ctrl Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("Chassis_Ctrl Init Success!");
    }

#ifdef BSP_USING_CHASSIS_IMU
    // if (ChassisIMU_Init() != RT_EOK)
    //     Robot_Chassis_Reset(RT_TRUE);
#endif

#ifdef BSP_USING_VISUAL_USART
    result = VisualInit();
    if (result != RT_EOK)
    {
        LOG_E("Visual_com Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("Visual_com Init Success!");
    }
#endif

#ifdef CORE_USING_REMOTE
    if (OnlyChassis_Init() != RT_EOK)
        Robot_Chassis_Reset(RT_TRUE);
#endif

    result = protct_ThreadInit();
    if (result != RT_EOK)
    {
        LOG_E("protct_Thread Init Failes! Result is %d", result);
        LOG_E("Robot Reset!");
        Robot_Chassis_Reset(RT_TRUE);
    }
    else
    {
        LOG_I("protct_Thread Init Success!");
    }

    return RT_EOK;
}
