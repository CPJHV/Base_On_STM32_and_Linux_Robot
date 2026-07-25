#ifndef __PWM_H
#define __PWM_H

#include "stm32f4xx_hal.h"

/******************************************************************************************/
/*                          第 1 组 电机 TIM 互补输出
/*                          PWMH=PA8  PWML=PB13
/******************************************************************************************/
#define PWM1_H_GPIO_PORT            GPIOA
#define PWM1_H_GPIO_PIN             GPIO_PIN_8
#define PWM1_H_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define PWM1_L_GPIO_PORT           GPIOB
#define PWM1_L_GPIO_PIN            GPIO_PIN_13
#define PWM1_L_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define PWM1_TIM_GPIO_AF              GPIO_AF1_TIM1
#define PWM1_TIME_SOURCE              TIM1
#define PWM1_TIME_CHANNEL             TIM_CHANNEL_1
#define PWM1_TIME_CCR                 PWM1_TIME_SOURCE->CCR1
#define PWM1_TIM_CLK_ENABLE()         do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)

/******************************************************************************************/
/*                          第 2 组 电机
/*                          PWMH=PA9  PWML=PB14
/******************************************************************************************/
#define PWM2_H_GPIO_PORT            GPIOA
#define PWM2_H_GPIO_PIN             GPIO_PIN_9
#define PWM2_H_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define PWM2_L_GPIO_PORT           GPIOB
#define PWM2_L_GPIO_PIN            GPIO_PIN_14
#define PWM2_L_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define PWM2_TIM_GPIO_AF              GPIO_AF1_TIM1
#define PWM2_TIME_SOURCE              TIM1
#define PWM2_TIME_CHANNEL             TIM_CHANNEL_2
#define PWM2_TIME_CCR                 PWM2_TIME_SOURCE->CCR2
#define PWM2_TIM_CLK_ENABLE()         do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)

/******************************************************************************************/
/*                          第 3 组 电机
/*                          PWMH=PA10 PWML=PB15
/******************************************************************************************/
#define PWM3_H_GPIO_PORT            GPIOA
#define PWM3_H_GPIO_PIN             GPIO_PIN_10
#define PWM3_H_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define PWM3_L_GPIO_PORT           GPIOB
#define PWM3_L_GPIO_PIN            GPIO_PIN_15
#define PWM3_L_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define PWM3_TIM_GPIO_AF              GPIO_AF1_TIM1
#define PWM3_TIME_SOURCE              TIM1
#define PWM3_TIME_CHANNEL             TIM_CHANNEL_3
#define PWM3_TIME_CCR                 PWM3_TIME_SOURCE->CCR3
#define PWM3_TIM_CLK_ENABLE()         do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)

/******************************************************************************************/
/*                          第 4 组 电机
/*                          PWMH=PC6  PWML=PA5
/******************************************************************************************/
#define PWM4_H_GPIO_PORT            GPIOC
#define PWM4_H_GPIO_PIN             GPIO_PIN_6
#define PWM4_H_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define PWM4_L_GPIO_PORT           GPIOA
#define PWM4_L_GPIO_PIN            GPIO_PIN_5
#define PWM4_L_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define PWM4_TIM_GPIO_AF              GPIO_AF2_TIM3
#define PWM4_TIME_SOURCE              TIM3
#define PWM4_TIME_CHANNEL             TIM_CHANNEL_1
#define PWM4_TIME_CCR                 PWM4_TIME_SOURCE->CCR1
#define PWM4_TIM_CLK_ENABLE()         do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)

#ifdef __cplusplus
class Pwm_ {
public:
    

    // 4个电机PWM初始化
    void pwm1_tim_cplm_pwm_init(uint16_t arr, uint16_t psc);
    void pwm2_tim_cplm_pwm_init(uint16_t arr, uint16_t psc);
    void pwm3_tim_cplm_pwm_init(uint16_t arr, uint16_t psc);
    void pwm4_tim_cplm_pwm_init(uint16_t arr, uint16_t psc);

    // 单个通道启动(主/互补)
    void PWM1_HorL_START(uint8_t param);
    void PWM2_HorL_START(uint8_t param);
    void PWM3_HorL_START(uint8_t param);
    void PWM4_HorL_START(uint8_t param);

    // 分别停止4个输出通道
    void PWM1_STOP(void);
    void PWM2_STOP(void);
    void PWM3_STOP(void);
    void PWM4_STOP(void);
private:
		//4个电机的PWM定时器的句柄
		TIM_HandleTypeDef g_pwm1_cplm_pwm_handle;
    TIM_HandleTypeDef g_pwm2_cplm_pwm_handle;
    TIM_HandleTypeDef g_pwm3_cplm_pwm_handle;
    TIM_HandleTypeDef g_pwm4_cplm_pwm_handle;

		//死区配置，公用
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
		friend class Monitor_Controller;
};

extern Pwm_ pwm;
#endif

#endif