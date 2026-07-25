#ifdef _BLDC_WU_

#ifndef __BLDC_TIM_H
#define __BLDC_TIM_H

#include "main.h"
/******************************************************************************************/
/* 高级定时器 定义 */
extern TIM_HandleTypeDef g_atimx_handle;      /* 定时器x句柄 */
 /* TIMX PWM 定义 
 * 注意: 通过修改这几个宏定义, 可以支持TIM1/TIM8定时器
 */
#define ATIM_TIMX_PWM_CH1_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH1_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define ATIM_TIMX_PWM_CH2_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH2_GPIO_PIN             GPIO_PIN_9
#define ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define ATIM_TIMX_PWM_CH3_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH3_GPIO_PIN             GPIO_PIN_10
#define ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

/* 互补通道IO */
#define M1_LOW_SIDE_U_PORT                      GPIOB
#define M1_LOW_SIDE_U_PIN                       GPIO_PIN_13
#define M1_LOW_SIDE_U_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define M1_LOW_SIDE_V_PORT                      GPIOB
#define M1_LOW_SIDE_V_PIN                       GPIO_PIN_14
#define M1_LOW_SIDE_V_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define M1_LOW_SIDE_W_PORT                      GPIOB
#define M1_LOW_SIDE_W_PIN                       GPIO_PIN_15
#define M1_LOW_SIDE_W_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define ATIM_TIMX_PWM_CHY_GPIO_AF               GPIO_AF1_TIM1

#define ATIM_TIMX_PWM                           TIM1
#define ATIM_TIMX_PWM_IRQn                      TIM1_UP_TIM10_IRQn
#define ATIM_TIMX_PWM_IRQHandler                TIM1_UP_TIM10_IRQHandler
#define ATIM_TIMX_PWM_CH1                       TIM_CHANNEL_1                                   /* 通道1 */
#define ATIM_TIMX_PWM_CH2                       TIM_CHANNEL_2                                   /* 通道2 */
#define ATIM_TIMX_PWM_CH3                       TIM_CHANNEL_3                                   /* 通道3 */
#define ATIM_TIMX_PWM_CHY_CLK_ENABLE()          do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)      /* TIM1 时钟使能 */

extern TIM_HandleTypeDef g_atimx_handle;        /* 定时器x句柄 */
/******************************************************************************************/
/* 基本定时器 定义 */

/* TIMX 中断定义 
 * 默认是针对TIM6/TIM7
 */
 
#define BTIM_TIMX_INT                       TIM6
#define BTIM_TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)          /* TIM6 时钟使能 */

/******************************************************************************************/

void btim_timx_int_init(uint16_t arr, uint16_t psc);                                            /* 基本定时器 定时中断初始化函数 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc);                                         /* 高级定时器 PWM初始化函数 */
#endif

#endif
















