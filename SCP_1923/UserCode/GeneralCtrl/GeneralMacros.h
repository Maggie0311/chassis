#ifndef __GENERALMACROS_H__
#define __GENERALMACROS_H__

// 启动串口输出波形 VOFA 调试
// 写 0 关闭调试, 1 发送原始 ADC 数据, 2 处理后的 ADC 数据, 3 电流控制数据
#define SERIAL_USE_VOFA_DEBUG (0)
#if (SERIAL_USE_VOFA_DEBUG == 0)
#undef ADC_RAWDATA_SEND
#undef ADC_DATA_SEND
#undef CURRENTCTRL_DATA_SEND
#elif (SERIAL_USE_VOFA_DEBUG == 1)
#define ADC_RAWDATA_SEND
#undef ADC_DATA_SEND
#undef CURRENTCTRL_DATA_SEND
#elif (SERIAL_USE_VOFA_DEBUG == 2)
#undef ADC_RAWDATA_SEND
#define ADC_DATA_SEND
#undef CURRENTCTRL_DATA_SEND
#elif (SERIAL_USE_VOFA_DEBUG == 3)
#undef ADC_RAWDATA_SEND
#undef ADC_DATA_SEND
#define CURRENTCTRL_DATA_SEND
#endif

// 写 1 关闭 PWM 输出，AB 两侧固定低侧导通
#define USE_STOPPWM_H (0)

// 写 1 默认自检通过
#define SELFCHECK_ALWAYS_PASS (1)

// 是否开启电容容量测试模式(定义为真此宏会进入电容容量测试模式)
#define CAPACITY_TEST_MODE (0)

// 是否进入数据标定模式(定义为真此宏会进入数据标定模式)
#define CAPACITY_CALIBRATING_MODE (0)

// 是否要开启看门狗(0/1 表示关闭/开启开门狗)
#define WATCHDOG_ENABLED (0)

#endif /* __GENERALMACROS_H__ */
