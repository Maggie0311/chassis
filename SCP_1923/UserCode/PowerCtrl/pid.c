#include "pid.h"

/**
 * @brief：清除PID的状态
 * @param [in]	target:要清零的PID结构体指针
 * @return：		无
 * @author：mqy
 */
void pid_clear(pid_t *target)
{
    // 状态归零
    target->err = 0;
    target->err_old = 0;
    target->i_value = 0;
    target->out = 0;
    target->set = 0;
    target->last_set = 0;
    target->I_Dis = 0;
    target->pout = 0;
    target->iout = 0;
    target->dout = 0;
}

void PID_Init(pid_t *pid,
              float kp, float ki, float kd, float kdi,
              float i_limit,
              float out_limit_up,
              float out_limit_down)
{
    // 参数设置
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kdi = kdi;
    pid->i_limit = i_limit;
    pid->out_limit_up = out_limit_up;
    pid->out_limit_down = out_limit_down;
    // 状态归零
    pid_clear(pid);
}

void PID_Calc(pid_t *pid, float Error)
{
    pid->err_old = pid->err;
    pid->err = Error;

    pid->pout = pid->kp * pid->err;
    pid->dout = pid->kd * (pid->err - pid->err_old);

    if (!pid->I_Dis)
    {
        pid->iout += pid->ki * (pid->err + pid->kdi * pid->dout);

        if (pid->iout > pid->i_limit)
        {
            pid->iout = pid->i_limit;
        }
        else if (pid->iout < -pid->i_limit)
        {
            pid->iout = -pid->i_limit;
        }
    }

    pid->out = pid->pout + pid->dout + pid->iout;

    if (pid->out > pid->out_limit_up)
    {
        pid->out = pid->out_limit_up;
    }
    else if (pid->out < pid->out_limit_down)
    {
        pid->out = pid->out_limit_down;
    }

    pid->i_value = pid->iout;
}
