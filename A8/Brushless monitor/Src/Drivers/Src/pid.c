#include "pid.h"
#include "bldc.h"

PID_TypeDef  g_current_pid;                 /* 电流PID参数结构体 */
PID_TypeDef  g_speed_pid;                   /* 速度PID参数结构体 */
/**
 * @brief       初始化PID结构体成员
 * @param       无
 * @retval      无
 */
void pid_init(void)
{
    /* 设定电流目标2000mA(空载最小电流400mA左右)较高的转速对应的电流较大，力矩较大，可适应较高的转速调节*/
    /* 【注意】如设置的转速对应的电流超过了电流设定值，将导致PID转至电流环调节，转速将无法继续提升 */
    g_current_pid.SetPoint = 2000.0;
    g_current_pid.ActualValue = 0.0;        /*设定目标Desired Value*/
    g_current_pid.LastError = 0.0;          /* Error[1]*/
    g_current_pid.LastError = 0.0;          /* Error[-1]*/
    g_current_pid.PrevError = 0.0;          /* Error[-2]*/
    g_current_pid.Proportion = C_KP;        /* 比例常数 Proportional Const*/
    g_current_pid.Integral = C_KI;          /* 积分常数 Integral Const*/
    g_current_pid.Derivative = C_KD;        /* 微分常数 Derivative Const*/
    g_current_pid.IngMax = 9000;
    g_current_pid.IngMin = 600;
    g_current_pid.OutMin = 600;
    g_current_pid.OutMax = 9000;
    
    g_speed_pid.SetPoint = 0;               /* 设定目标Desired Value*/
    g_speed_pid.ActualValue = 0.0;          /* 设定目标Desired Value*/
    g_speed_pid.Ui = 0.0;
    g_speed_pid.Up = 0.0;
    g_speed_pid.Ud = 0.0;
    g_speed_pid.Error = 0.0;                /* Error[1]*/
    g_speed_pid.LastError = 0.0;            /* Error[-1]*/
    g_speed_pid.PrevError = 0.0;            /* Error[-2]*/
    g_speed_pid.Proportion = S_KP;          /* 比例常数 Proportional Const*/
    g_speed_pid.Integral = S_KI;            /* 积分常数 Integral Const*/
    g_speed_pid.Derivative = S_KD;          /* 微分常数 Derivative Const*/ 
    g_speed_pid.IngMax = 9000;
    g_speed_pid.IngMin = -9000;
    g_speed_pid.OutMax = 9000;              /* 输出限制 */
    g_speed_pid.OutMin = -9000;
}

/**
 * @brief       位置式PID算法
 * @param       *PID：PID结构体句柄所对应的目标值
 * @param       Feedback_value ： 实际值
 * @retval      目标控制量
 */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);   /* 速度档位偏差 */
    PID->Up = PID->Proportion * PID->Error;
    PID->Ui += (PID->Error * PID->Integral);
    LIMIT_OUT(PID->Ui,PID->IngMax,PID->IngMin);
    PID->Ud = PID->Derivative * (PID->Error - PID->LastError);
    PID->ActualValue = PID->Up + PID->Ui + PID->Ud;
    LIMIT_OUT(PID->ActualValue,PID->OutMax,PID->OutMin);
    PID->LastError = PID->Error;
    return ((int32_t)(PID->ActualValue));                   /* 返回实际控制数值 */
}
