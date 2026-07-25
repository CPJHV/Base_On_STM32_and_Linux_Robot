
#ifndef __PID_H
#define __PID_H

#include "main.h"

/******************************************************************************************/

/*定义PID参数相关宏*/
#define  S_KP      0.008000f        /* 速度环的P参数需远小于电流环的P */
#define  S_KI      0.00025f         /* I参数*/
#define  S_KD      0.000250f        /* D参数*/
/*定义PID参数相关宏*/
#define  C_KP      2.00f            /* P参数*/
#define  C_KI      0.20f            /* I参数*/
#define  C_KD      0.01f            /* D参数*/
#define SMAPLSE_PID_SPEED  40       /* 采样率 单位ms*/
/*定义位置PID参数相关宏*/
/*PID结构体*/
typedef struct
{
    __IO float  SetPoint;           /* 设定目标 */
    __IO float  ActualValue;        /* 实际值*/
    __IO float  SumError;           /* 误差累计*/
    __IO float  Up;                 /* 比例项 */
    __IO float  Ui;                 /* 积分项 */
    __IO float  Ud;                 /* 微分项 */
    __IO float  Proportion;         /* 比例常数 P*/
    __IO float  Integral;           /* 积分常数 I*/
    __IO float  Derivative;         /* 微分常数 D*/
    __IO float  Error;              /* Error[-1]*/
    __IO float  LastError;          /* Error[-1]*/
    __IO float  PrevError;          /* Error[-2]*/
    __IO float  IngMin;
    __IO float  IngMax;
    __IO float  OutMin;
    __IO float  OutMax;
} PID_TypeDef;

#define LIMIT_OUT(var,max,min) {(var)=(var)>(max)?(max):(var);(var)=(var)<(min)?(min):(var);}
extern PID_TypeDef  g_current_pid;  /* 电流PID参数结构体*/
extern PID_TypeDef  g_speed_pid;    /* 速度PID参数结构体*/
extern float *user_setpoint;
/******************************************************************************************/
/* 外部接口函数 */
void pid_init(void);
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);
#endif
