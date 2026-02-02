#include "func_LED.h"

#define LINE 2
#define ROW 4

typedef struct
{
    GPIO_TypeDef *GPIO_Port; // GPIO端口
    uint16_t GPIO_Pin;       // GPIO PIN
    LED_State_e State;       // LED状态
    uint8_t Flash_Period;    // 闪烁周期
} LED_s;

Time_Struct_t Time_LED = {.Count_Per_Second = 1000, .Timer_Count_Max = (uint32_t)(-1)};
LED_s LED_Strust[2][4];

/**
 * @brief
 * @author dty
 * @param a 第a行
 * @param b 第b列
 * @param ctrl LED_ON/LED_OFF/LED_Flash
 * @param period 闪烁周期（100ms整数倍）
 */
void LED_Ctrl(uint8_t a, uint8_t b, LED_State_e ctrl, uint16_t period)
{
    assert_param((a <= LINE) && (a > 0));
    assert_param((b <= ROW) && (b > 0));
    if (period < 100)
    {
        period = 100;
    }

    LED_Strust[a - 1][b - 1].State = ctrl;
    LED_Strust[a - 1][b - 1].Flash_Period = (period / 100) * 100;
}

/**
 * @brief 关闭所有LED
 * @author dty
 */
void LED_AllOff(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

/**
 * @brief 开启所有LED
 * @author dty
 */
void LED_AllOn(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
}

/**
 * @brief 初始化
 * @author dty
  */
void LED_Init(void)
{
    LED_Strust[0][0].GPIO_Port = GPIOB;
    LED_Strust[0][0].GPIO_Pin = GPIO_PIN_4;
    LED_Strust[0][0].State = LED_OFF;
    LED_Strust[0][0].Flash_Period = 0;

    LED_Strust[0][1].GPIO_Port = GPIOB;
    LED_Strust[0][1].GPIO_Pin = GPIO_PIN_3;
    LED_Strust[0][1].State = LED_OFF;
    LED_Strust[0][1].Flash_Period = 0;

    LED_Strust[0][2].GPIO_Port = GPIOA;
    LED_Strust[0][2].GPIO_Pin = GPIO_PIN_11;
    LED_Strust[0][2].State = LED_OFF;
    LED_Strust[0][2].Flash_Period = 0;

    LED_Strust[0][3].GPIO_Port = GPIOA;
    LED_Strust[0][3].GPIO_Pin = GPIO_PIN_9;
    LED_Strust[0][3].State = LED_OFF;
    LED_Strust[0][3].Flash_Period = 0;

    LED_Strust[1][0].GPIO_Port = GPIOB;
    LED_Strust[1][0].GPIO_Pin = GPIO_PIN_5;
    LED_Strust[1][0].State = LED_OFF;
    LED_Strust[1][0].Flash_Period = 0;

    LED_Strust[1][1].GPIO_Port = GPIOB;
    LED_Strust[1][1].GPIO_Pin = GPIO_PIN_15;
    LED_Strust[1][1].State = LED_OFF;
    LED_Strust[1][1].Flash_Period = 0;

    LED_Strust[1][2].GPIO_Port = GPIOC;
    LED_Strust[1][2].GPIO_Pin = GPIO_PIN_14;
    LED_Strust[1][2].State = LED_OFF;
    LED_Strust[1][2].Flash_Period = 0;

    LED_Strust[1][3].GPIO_Port = GPIOC;
    LED_Strust[1][3].GPIO_Pin = GPIO_PIN_15;
    LED_Strust[1][3].State = LED_OFF;
    LED_Strust[1][3].Flash_Period = 0;
    LED_AllOff();
}

/**
 * @brief LED轮询
 * @author dty
 */
void LED_Routine(void)
{
    int i, j;
    for (i = 0; i < LINE; i++)
    {
        for (j = 0; j < ROW; j++)
        {
            switch (LED_Strust[i][j].State)
            {
            case LED_ON:
                HAL_GPIO_WritePin(LED_Strust[i][j].GPIO_Port, LED_Strust[i][j].GPIO_Pin, GPIO_PIN_RESET);
                break;
            case LED_OFF:
                HAL_GPIO_WritePin(LED_Strust[i][j].GPIO_Port, LED_Strust[i][j].GPIO_Pin, GPIO_PIN_SET);
                break;
            case LED_Flash:
                if ((Time_LED.Timer_Count % LED_Strust[i][j].Flash_Period) == 0)
                {
                    HAL_GPIO_TogglePin(LED_Strust[i][j].GPIO_Port, LED_Strust[i][j].GPIO_Pin);
                }
                break;
            default:
                break;
            }
        }
    }
    Time_LED.Timer_Count += 100;
}
