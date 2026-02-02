#include "drv_BridgeHW.h"
#include "app_Routine.h"
// C语言文件
#include <string.h>
// 单片机文件
#include "hrtim.h"

// 这里注意，因为主定时器和子定时器频率不一样，子定时器的频率是主定时器的四倍，所以这里的周期数也被设置为了主定时器中一个触发周期的四倍
#define HRTIM_PERIOD_COUNT_MAX __GLOBAL_HRTIM_PERIOD_COUNT_MAX
// 死区时间200nm左右
#define HRTIM_DEADTIME_COUNT __GLOBAL_HRTIM_DEADTIME_COUNT

typedef struct
{
    float TGGLA[2]; // A 侧两处翻转点比例 未考虑死区 (分别代表下降沿位置和上升沿位置)
    float TGGLB[2]; // B 侧两处翻转点比例 未考虑死区 (分别代表下降沿位置和上升沿位置)
} Bridge_HW_s;

Bridge_HW_s Bridge_HW;

// 主要考虑死区问题和PWM边缘细节 按照给定的设定参数对寄存器进行操作
/**
 * @brief 按照TGGLA与TGGLB修改PWM占空比
 *
 * @param I_B   目前B端电流值（关注正负）
 */
static int Temp[8];
void BridgeHW_SetPWM(float I_B)
{

    int fori;

    // // TODO: test,恒占空比标定用

    // Bridge_HW.TGGLA[0] = 0.95f;
    // Bridge_HW.TGGLA[1] = 1.01f;
    // Bridge_HW.TGGLB[0] = 0.45f;
    // Bridge_HW.TGGLB[1] = 1.01f;

    Temp[0] = (int)((Bridge_HW.TGGLA[0]) * HRTIM_PERIOD_COUNT_MAX); // 输入下管的上升沿
    Temp[1] = (int)((Bridge_HW.TGGLA[1]) * HRTIM_PERIOD_COUNT_MAX); // 输入下管的下降沿
    Temp[2] = (int)((Bridge_HW.TGGLB[0]) * HRTIM_PERIOD_COUNT_MAX); // 输出下管的上升沿
    Temp[3] = (int)((Bridge_HW.TGGLB[1]) * HRTIM_PERIOD_COUNT_MAX); // 输出下管的下降沿

    for (fori = 0; fori < 4; fori++)
        if (Temp[fori] <= 10)
            Temp[fori] = 10;
				
    // 由于A定时器控制的是算法中的B端，为了算法的一致性，将TGGLA的值赋给B定时器，反之亦然
#if (DEBUG)
//        hhrtim1.Instance->sTimerxRegs[0].CMP1xR = HRTIM_PERIOD_COUNT_MAX * 0.95;
//        hhrtim1.Instance->sTimerxRegs[0].CMP2xR = HRTIM_PERIOD_COUNT_MAX * 1.01;

//        hhrtim1.Instance->sTimerxRegs[1].CMP1xR = HRTIM_PERIOD_COUNT_MAX * 0.82;
//        hhrtim1.Instance->sTimerxRegs[1].CMP2xR = HRTIM_PERIOD_COUNT_MAX * 1.01;

#endif
}

/**
 * @brief 给定时器的比较单元赋值
 *
 */
void Bridge_TIM_Set_Comp(void)
{
    hhrtim1.Instance->sTimerxRegs[0].CMP1xR = Temp[2];
    hhrtim1.Instance->sTimerxRegs[0].CMP2xR = Temp[3];

    hhrtim1.Instance->sTimerxRegs[1].CMP1xR = Temp[0];
    hhrtim1.Instance->sTimerxRegs[1].CMP2xR = Temp[1];
}
/**
 * @brief 初始化
 *
 * @return int
 */
int BridgeHW_Init(void)
{
    // 结构体初始值置0
    memset(&Bridge_HW, 0, sizeof(Bridge_HW));
    return 0;
}

/**
 * @brief 修改PWM参数
 *
 * @param TGGLA1 A 侧两处翻转点比例 未考虑死区 (分别代表下降沿位置和上升沿位置)
 * @param TGGLA2
 * @param TGGLB1 B 侧两处翻转点比例 未考虑死区 (分别代表下降沿位置和上升沿位置)
 * @param TGGLB2
 */
void BridgeHW_TGGLWrite(float TGGLA1, float TGGLA2, float TGGLB1, float TGGLB2)
{
    Bridge_HW.TGGLA[0] = TGGLA1;
    Bridge_HW.TGGLA[1] = TGGLA2;
    Bridge_HW.TGGLB[0] = TGGLB1;
    Bridge_HW.TGGLB[1] = TGGLB2;
}

/**
 * @brief 读取当前设定的TGGL值
 *
 * @param AorB      1：读取A端；0:读取B端
 * @param Num       0:读取第一个；1：读取第二个
 * @return float    返回该值
 */
float BridgeHW_TGGLRead(int8_t AorB, int8_t Num)
{
    if (AorB)
        return Bridge_HW.TGGLA[Num];
    else
        return Bridge_HW.TGGLB[Num];
}
