#include "pid.h"

Pid_ pidx;
/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */
 void Pid_::pid_init(void)
{
    g1_speed_pid.SetPoint = 0;
    g1_speed_pid.ActualValue = 0.0;
    g1_speed_pid.SumError = 0.0;
    g1_speed_pid.Error = 0.0;
    g1_speed_pid.LastError = 0.0;
    g1_speed_pid.PrevError = 0.0;
    g1_speed_pid.Proportion = S1_KP;
    g1_speed_pid.Integral = S1_KI;
    g1_speed_pid.Derivative = S1_KD;

    g1_current_pid.SetPoint = 0.0;
    g1_current_pid.ActualValue = 0.0;
    g1_current_pid.SumError = 0.0;
    g1_current_pid.Error = 0.0;
    g1_current_pid.LastError = 0.0;
    g1_current_pid.PrevError = 0.0;
    g1_current_pid.Proportion = C1_KP;
    g1_current_pid.Integral = C1_KI;
    g1_current_pid.Derivative = C1_KD;

    /************************* 电机 2 *************************/
    g2_speed_pid.SetPoint = 0;
    g2_speed_pid.ActualValue = 0.0;
    g2_speed_pid.SumError = 0.0;
    g2_speed_pid.Error = 0.0;
    g2_speed_pid.LastError = 0.0;
    g2_speed_pid.PrevError = 0.0;
    g2_speed_pid.Proportion = S2_KP;
    g2_speed_pid.Integral = S2_KI;
    g2_speed_pid.Derivative = S2_KD;

    g2_current_pid.SetPoint = 0.0;
    g2_current_pid.ActualValue = 0.0;
    g2_current_pid.SumError = 0.0;
    g2_current_pid.Error = 0.0;
    g2_current_pid.LastError = 0.0;
    g2_current_pid.PrevError = 0.0;
    g2_current_pid.Proportion = C2_KP;
    g2_current_pid.Integral = C2_KI;
    g2_current_pid.Derivative = C2_KD;

    /************************* 电机 3 *************************/
    g3_speed_pid.SetPoint = 0;
    g3_speed_pid.ActualValue = 0.0;
    g3_speed_pid.SumError = 0.0;
    g3_speed_pid.Error = 0.0;
    g3_speed_pid.LastError = 0.0;
    g3_speed_pid.PrevError = 0.0;
    g3_speed_pid.Proportion = S3_KP;
    g3_speed_pid.Integral = S3_KI;
    g3_speed_pid.Derivative = S3_KD;

    g3_current_pid.SetPoint = 0.0;
    g3_current_pid.ActualValue = 0.0;
    g3_current_pid.SumError = 0.0;
    g3_current_pid.Error = 0.0;
    g3_current_pid.LastError = 0.0;
    g3_current_pid.PrevError = 0.0;
    g3_current_pid.Proportion = C3_KP;
    g3_current_pid.Integral = C3_KI;
    g3_current_pid.Derivative = C3_KD;

    /************************* 电机 4 *************************/
    g4_speed_pid.SetPoint = 0;
    g4_speed_pid.ActualValue = 0.0;
    g4_speed_pid.SumError = 0.0;
    g4_speed_pid.Error = 0.0;
    g4_speed_pid.LastError = 0.0;
    g4_speed_pid.PrevError = 0.0;
    g4_speed_pid.Proportion = S4_KP;
    g4_speed_pid.Integral = S4_KI;
    g4_speed_pid.Derivative = S4_KD;

    g4_current_pid.SetPoint = 0.0;
    g4_current_pid.ActualValue = 0.0;
    g4_current_pid.SumError = 0.0;
    g4_current_pid.Error = 0.0;
    g4_current_pid.LastError = 0.0;
    g4_current_pid.PrevError = 0.0;
    g4_current_pid.Proportion = C4_KP;
    g4_current_pid.Integral = C4_KI;
    g4_current_pid.Derivative = C4_KD;
		
		//角度控制的
		angle_pid.SetPoint = 0;
    angle_pid.ActualValue = 0;
    angle_pid.SumError = 0;
    angle_pid.Error = 0;
    angle_pid.LastError = 0;
    angle_pid.PrevError = 0;

    angle_pid.Proportion = 2.2f;    // 角度环P
    angle_pid.Integral = 0.06f;     // 角度环I
    angle_pid.Derivative = 0.15f;   // 角度环D
}

/**
 * @brief       pid闭环控制
 * @param       *PID：PID结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
int32_t Pid_::increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* 计算偏差 */
    
#if  INCR_LOCT_SELECT                                                       /* 增量式PID */
    //uk=kp*（ek-e（k-1））+ki*ek+Kd*（ek-2*e（k-1）+e（k-2））
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* 比例环节 */
                        + (PID->Integral * PID->Error)                                             /* 积分环节 */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分环节 */
    
    PID->PrevError = PID->LastError;                                        /* 存储偏差，用于下次计算 */
    PID->LastError = PID->Error;
    
#else                                                                       /* 位置式PID */
    //公式：Kp*e误差+Ki*求和误差+Kd（这次误差与上一次误差的差值）
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* 比例环节 */
                       + (PID->Integral * PID->SumError)                    /* 积分环节 */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* 微分环节 */
    PID->LastError = PID->Error;
    
#endif
    return ((int32_t)(PID->ActualValue));                                   /* 返回计算后输出的数值 */
}

/**
 * @brief       积分限幅
 * @param       *PID：PID结构体变量地址
 * @param       max_limit：最大值
 * @param       min_limit：最小值
 * @retval      无
 */
void Pid_::integral_limit( PID_TypeDef *PID , float max_limit, float min_limit )
{
    if (PID->SumError >= max_limit)                           /* 超过限幅 */
    {
        PID->SumError = max_limit;                            /* 限制积分 */
    }
    else if (PID->SumError <= min_limit)                      /* 超过限幅 */
    {
        PID->SumError = min_limit;
    }
}
