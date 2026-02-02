#ifndef __PID_H__
#define __PID_H__

typedef struct
{
    float kp; // pid三个参数
    float ki;
    float kd;
    float kdi; // 将 D 分量添加到 I 中的系数

    float last_set; // 上次pid的设定值
    float set;      // pid的设定值
    float err;      // 偏差值
    float err_old;  // 上次偏差值

    float i_value; // 现在的积分值（该积分已经乘了ki参数了
    float i_limit; // 积分限幅
    char I_Dis;    // 为1时会停止修改I_Value

    float pout;
    float iout;
    float dout;

    float out;            // pid的输出
    float out_limit_up;   // 输出上限幅
    float out_limit_down; // 输出下限幅
} pid_t;
// 电机PID闭环结构体

extern void pid_clear(pid_t *target);

extern void PID_Init(pid_t *pid,
                     float kp, float ki, float kd, float kdi,
                     float i_limit,
                     float out_limit_up,
                     float out_limit_down);
extern void PID_Calc(pid_t *pid, float Error);

#endif
