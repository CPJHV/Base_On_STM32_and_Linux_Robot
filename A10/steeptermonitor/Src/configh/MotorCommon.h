#ifndef __MOTOR_COMMON_H
#define __MOTOR_COMMON_H

#include "main.h"

// ==================== 公共宏定义 ====================
#define PULSE_REV             1600.0f
#define MAX_STEP_ANGLE        1.8f

#define TIM_FREQ              168000000U
#define T1_FREQ               (TIM_FREQ / 168)

#define FSPR                  200//整步下，电机转一圈需要的脉冲数量
#define MICRO_STEP            8//细分程度，一步细分成8微小步
#define SPR                   (FSPR * MICRO_STEP)//一圈总的脉冲数

#define ROUNDPS_2_STEPPS(rpm)   ((rpm) * SPR / 60)//输入的是rpm是转每分钟，这里转成每秒钟需要输出多少脉冲才可以达到这个速度
#define MIDDLEVELOCITY(vo,vt)   ( ( (vo) + (vt) ) / 2 )
#define INCACCEL(vo,v,t)        ( ( 2 * ((v) - (vo)) ) / pow((t),2) )
#define INCACCELSTEP(j,t)       ( ( (j) * pow( (t) , 3 ) ) / 6.0f )
#define ACCEL_TIME(t)           ( (t) / 2 )
#define SPEED_MIN               (T1_FREQ / 65535.0f)

#ifndef TRUE
#define TRUE                    1
#endif

#ifndef FALSE
#define FALSE                   0
#endif

// ==================== 公共枚举 ====================
typedef enum
{
    CW  = 0,
    CWN = 1
} DIR_STATE;

typedef enum
{
    STATE_ACCEL    = 1,
    STATE_AVESPEED = 2,
    STATE_DECEL    = 3,
    STATE_STOP     = 0,
    STATE_IDLE     = 4,
} motor_state_typedef;

// ==================== 公共结构体 ====================
typedef struct {
    int32_t vo;
    int32_t vt;
    int32_t accel_step;
    int32_t decel_step;
    float   *accel_tab;
    float   *decel_tab;
    float   *ptr;
    int32_t dec_point;
    int32_t step;
    int32_t step_pos;
} speed_calc_t;

typedef struct
{
    uint8_t state;
    uint8_t dir;
    float speed;
    float setspeed;
    int32_t location;
} Motor_TypeDef;

#endif