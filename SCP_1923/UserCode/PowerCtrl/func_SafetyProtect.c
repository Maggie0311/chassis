#include "func_SafetyProtect.h"

#include "func_CurrentCtrl.h"
#include "mod_ADCProcess.h"

#include "func_LED.h"

/**
 * @brief 异常情况监测
 * @param PWMCount 时刻
 * @return int 0：正常；1：异常
 */
uint32_t OCAP_Count, OBAT_Count, UBAT_Count, OIBH_Count, UCAP_Count;
int BreakSituation_Protect(uint32_t PWMCount)
{
    static uint8_t OCAP_Flag, OBAT_Flag, UBAT_Flag, OIBH_Flag, UCAP_Flag;
    

    // 工作状态检查

    // 电容过压
    if ((Read_Now_Data(DataType_U_CAP, ADC_Filted_Data) > SET_OCAP) && (OCAP_Flag == 0))
    {
        OCAP_Flag = 1;
        OCAP_Count = PWMCount;
    }

    if (OCAP_Flag == 1)
        if (Read_Now_Data(DataType_U_CAP, ADC_Filted_Data) < SET_OCAP - 1.0f)
            OCAP_Flag = 0;
        else
            OCAP_Count = PWMCount;
		// 电容欠压
    if ((Read_Now_Data(DataType_U_CAP, ADC_Filted_Data) < SET_UCAP) && (UCAP_Flag == 0))
    {
//        UCAP_Flag = 1;
//        UCAP_Count = PWMCount;
    }

    if (UCAP_Flag == 1)
        if (Read_Now_Data(DataType_U_CAP, ADC_Filted_Data) > SET_UCAP + 1.0f)
            UCAP_Flag = 0;
        else
            UCAP_Count = PWMCount;
		
    // 电源过压
    if ((Read_Now_Data(DataType_U_IN, ADC_Filted_Data) > SET_OBAT) && (OBAT_Flag == 0))
    {
        OBAT_Flag = 1;
        OBAT_Count = PWMCount;
    }

    if (OBAT_Flag == 1)
        if (Read_Now_Data(DataType_U_IN, ADC_Filted_Data) < BAT_UMAX)
            OBAT_Flag = 0;
        else
            OBAT_Count = PWMCount;

    // 电源欠压
    if ((Read_Now_Data(DataType_U_IN, ADC_Filted_Data) < SET_UBAT) && (UBAT_Flag == 0))
    {
        UBAT_Flag = 1;
        UBAT_Count = PWMCount;
    }

    if (UBAT_Flag == 1)
        if (Read_Now_Data(DataType_U_IN, ADC_Filted_Data) > BAT_UMIN)
            UBAT_Flag = 0;
        else
            UBAT_Count = PWMCount;

    // 电流过流
    if ((Read_Now_Data(DataType_I_BH, ADC_Filted_Data) > SET_OIBH) && (OIBH_Flag == 0))
    {
        OIBH_Flag = 1;
        OIBH_Count = PWMCount;
    }

    if (OIBH_Flag == 1)
        if (Read_Now_Data(DataType_I_BH, ADC_Filted_Data) < IBH_MAX)
            OIBH_Flag = 0;
        else
            OIBH_Count = PWMCount;

    if ((PWMCount - OCAP_Count < 200) || (PWMCount - OBAT_Count < 200) ||
        (PWMCount - UBAT_Count < 200) || (PWMCount - OIBH_Count < 200) || (PWMCount - UCAP_Count < 200))
    {
        PWM_Output_Ctrl(SAFETYPROTECT, 0);
#ifdef LED_DEBUG
        LED_Ctrl(1, 2, LED_ON, 1);
#endif

        return 1;
    }

    // 恢复正常工作
    PWM_Output_Ctrl(SAFETYPROTECT, 1);

#ifdef LED_DEBUG
    LED_Ctrl(1, 2, LED_OFF, 1);
#endif

    return 0;
}
