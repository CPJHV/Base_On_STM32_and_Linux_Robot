#include "pid.h"

void pid_init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0;
    pid->feedback = 0;
    pid->err = 0;
    pid->err_last = 0;
    pid->out = 0;
}

// 这个函数名必须和调用的一模一样
float increment_pid_ctrl(PID_TypeDef *pid, float feedback)
{
    float increment;

    pid->feedback = feedback;
    pid->err = pid->target - pid->feedback;

    increment = pid->kp * (pid->err - pid->err_last)
              + pid->ki * pid->err
              + pid->kd * (pid->err - 2 * pid->err_last);

    pid->out += increment;
    pid->err_last = pid->err;

    // 限幅
    if(pid->out > 150)  pid->out = 150;
    if(pid->out < -150) pid->out = -150;

    return pid->out;
}