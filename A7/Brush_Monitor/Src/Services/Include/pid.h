#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  0         /* 0：位置式，1：增量式 */

/* 注意：双环控制的时候，外环PID参数调节幅度不要太大，这对于整个曲线的影响很大 */

#if INCR_LOCT_SELECT

/* 调试模式 PID 参数 */
#define S1_KP      1.500f
#define S1_KI      0.023f
#define S1_KD      0.010f

#define C1_KP      1.00f
#define C1_KI      3.00f
#define C1_KD      0.00f

#define S2_KP      1.500f
#define S2_KI      0.023f
#define S2_KD      0.010f

#define C2_KP      1.00f
#define C2_KI      3.00f
#define C2_KD      0.00f

#define S3_KP      1.500f
#define S3_KI      0.023f
#define S3_KD      0.010f

#define C3_KP      1.00f
#define C3_KI      3.00f
#define C3_KD      0.00f

#define S4_KP      1.500f
#define S4_KI      0.023f
#define S4_KD      0.010f

#define C4_KP      1.00f
#define C4_KI      3.00f
#define C4_KD      0.00f

#else

/* 正常模式 PID 参数 */
#define S1_KP      1.500f
#define S1_KI      0.023f
#define S1_KD      0.002f

#define C1_KP      1.00f
#define C1_KI      3.75f
#define C1_KD      0.00f

#define S2_KP      1.500f
#define S2_KI      0.023f
#define S2_KD      0.002f

#define C2_KP      1.00f
#define C2_KI      3.75f
#define C2_KD      0.00f

#define S3_KP      1.500f
#define S3_KI      0.023f
#define S3_KD      0.002f

#define C3_KP      1.00f
#define C3_KI      3.75f
#define C3_KD      0.00f

#define S4_KP      1.500f
#define S4_KI      0.023f
#define S4_KD      0.002f

#define C4_KP      1.00f
#define C4_KI      3.75f
#define C4_KD      0.00f



/* 采样周期 */
#define SMAPLSE_PID_SPEED1  50
#define SMAPLSE_PID_SPEED2  50
#define SMAPLSE_PID_SPEED3  50
#define SMAPLSE_PID_SPEED4  50
#endif

/*PID结构体*/
typedef struct
{
    __IO float  SetPoint;            /* 目标值 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
} PID_TypeDef;


class Pid_{
	public:
		void pid_init(void);                 /* pid初始化 */
		int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);              /* pid闭环控制 */
		void integral_limit( PID_TypeDef *PID , float max_limit, float min_limit ); 	/* 积分限幅 */
		PID_TypeDef  g1_speed_pid;       /* 速度环PID参数结构体 */
		PID_TypeDef  g1_current_pid;     /* 电流环PID参数结构体 */
		
		PID_TypeDef  g2_speed_pid;       /* 速度环PID参数结构体 */
		PID_TypeDef  g2_current_pid;     /* 电流环PID参数结构体 */
		
		PID_TypeDef  g3_speed_pid;       /* 速度环PID参数结构体 */
		PID_TypeDef  g3_current_pid;     /* 电流环PID参数结构体 */
	
		PID_TypeDef  g4_speed_pid;       /* 速度环PID参数结构体 */
		PID_TypeDef  g4_current_pid;     /* 电流环PID参数结构体 */
	
		//转动角度PID控制
		PID_TypeDef angle_pid; 
	private:
		
};
extern Pid_ pidx;

#endif