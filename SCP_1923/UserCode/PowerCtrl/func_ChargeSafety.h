#ifndef __FUNC_CHARGESAFETY_H__
#define __FUNC_CHARGESAFETY_H__

#include "main.h"

typedef struct
{
    unsigned Charge_Permission_Flag : 1;     // 是否允许充电
    unsigned DisCharge_Attempt_Judge : 1;    // 用于标志本次判断是不是进入尝试断电请求
    unsigned DisCharge_Attempt_Status : 1;   // 当前是否处于尝试断电状态
    uint16_t Charge_UnPermission_Times;      // 连续触发断电条件的次数
    Time_Struct_t *DisCharge_Attempt_Time;   // 记录本次开始断电尝试的时间
    Time_Struct_t *DisCharge_LastCheck_Time; // 记录上一次底盘断电状态的判断时间
} Charge_System_Permission_Check_Structure;  // 用于记录底盘供电状态的结构体类型

extern void Charge_System_Check_Init(void (*ChargeSystemStop)(void *param),
                                     void (*ChargeSystemRestart)(void *param));
extern int8_t Power_On_Self_Test(void);

/**
 * @brief 进行充电允许性的判断
 * @author fwlh
 * @param  I_cap            超级电容端的电流, 超级电容输出为正方向
 * @param  I_referee        裁判系统端的电流, 裁判系统输出为正方向
 * @return uint8_t          返回 0 说明不允许充电
 */
extern uint8_t Charge_System_Permission_Check(float I_cap, float I_referee);

extern uint8_t Inquire_PowerOnSelfTest_Result(void);
extern uint8_t Key_Protect_Charge(void);

#endif /* __FUNC_CHARGESAFETY_H__ */
