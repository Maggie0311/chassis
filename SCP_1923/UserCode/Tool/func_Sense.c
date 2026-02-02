#include "func_Sense.h"

// 初始化一个按键
void Key_Init(Key_S *Key, int ConfirmMax, int ReleaseCount, uint16_t GPIO_Pin)
{
    Key->ConfirmCount = 0;
    Key->ConfirmMax = ConfirmMax;
    Key->Press = 0;
    Key->ReleaseCount_Set = ReleaseCount;
    Key->ReleaseCount_Now = 0;
    Key->GPIO_Pin_In = GPIO_Pin;
}

// 记录按键状态
void Key_Rec(Key_S* Key, char Press_In)
{
    char toggle; // 记录是否将要翻转

    if (Key->Press)
    {
        if (Press_In)
        {
            toggle = 0;
        }
        else
        {
            toggle = 1;
        }
    }
    else
    {
        if (Key->ReleaseCount_Now > 0)
        { // 如果有触发释抑，则暂不处理
            Key->ReleaseCount_Now--;
            return;
        }
        if (Press_In)
        {
            toggle = 1;
        }
        else
        {
            toggle = 0;
        }
    }

    if (toggle)
    {
        // 出现变化
        Key->ConfirmCount++;
        if (Key->ConfirmCount >= Key->ConfirmMax)
        {
            Key->Press = !Key->Press;
            Key->ConfirmCount = 0;
        }
        if(Key->Press)
        {
            // 设置触发释抑
            Key->ReleaseCount_Now = Key->ReleaseCount_Set;
        }
    }
    else
    {
        // 没有出现变化
        if (Key->ConfirmCount > 0)
        {
            Key->ConfirmCount--;
        }
        if (Key->ConfirmCount > 0)
        {
            Key->ConfirmCount--;
        }
    }
}

// 可以通过在Clear传入1 将本函数设置为查询到1就清空 可配合触发释抑使用
int8_t Key_GetPress(Key_S* Key, char Clear)
{
    if(Clear)
    {
        if (Key->Press)
        {
            Key->Press = 0;
            return 1;
        }
    }
    return Key->Press;
}
