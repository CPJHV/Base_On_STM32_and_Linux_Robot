// encoder.h
#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"
#include "pid.h"
#include "motor.h"
#include "config.h"
/*
1：A:A6 B:A7 TIM3 TIM6
2: A:B6 B:B7 TIM4 TIM7
3: A:A0 B:A1 TIM2 TIM5
4: A:A2 B:A3 TIM5 TIM12

*/
//第一台电机的编码器配置
#define PWM1_A_GPIO_PORT         GPIOA
#define PWM1_A_GPIO_PIN          GPIO_PIN_6
#define PWM1_A_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */

#define PWM1_B_GPIO_PORT         GPIOA
#define PWM1_B_GPIO_PIN          GPIO_PIN_7
#define PWM1_B_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */


#define PWM1_A_GPIO_AF            GPIO_AF2_TIM3                                /* 端口复用到TIM3 */
#define PWM1_B_GPIO_AF            GPIO_AF2_TIM3                                /* 端口复用到TIM3 */

#define PWM1_ENCODER_TIME_SOURCE                       TIM3                                         /* TIM3 */
#define PWM1_ENCODER_TIME_IRQn              TIM3_IRQn
#define PWM1_ENCODER_TIME_IRQHandler        TIM3_IRQHandler

#define PWM1_A_CHANNEL                 TIM_CHANNEL_1                                /* 通道1 */
#define PWM1_A_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3 时钟使能 */

#define PWM1_B_CHANNEL                   TIM_CHANNEL_2                                /* 通道2 */
#define PWM1_B_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3 时钟使能 */

/* 基本定时器：PWM1 专用速度采样 */
#define PWM1_BTIM_INT                           TIM6
#define PWM1_BTIM_INT_IRQn                      TIM6_DAC_IRQn
#define PWM1_BTIM_INT_IRQHandler                TIM6_DAC_IRQHandler
#define PWM1_BTIM_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)

//第二台
/******************************* 电机编码器测速 **************************************/
/* 通用定时器 定义 */
#define PWM2_A_GPIO_PORT         GPIOB
#define PWM2_A_GPIO_PIN          GPIO_PIN_6
#define PWM2_A_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)  /* PB口时钟使能 */

#define PWM2_B_GPIO_PORT         GPIOB
#define PWM2_B_GPIO_PIN          GPIO_PIN_7
#define PWM2_B_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)  /* PB口时钟使能 */

#define PWM2_A_GPIO_AF            GPIO_AF2_TIM4                                /* 端口复用到TIM4 */
#define PWM2_B_GPIO_AF            GPIO_AF2_TIM4                                /* 端口复用到TIM4 */

#define PWM2_ENCODER_TIME_SOURCE                       TIM4                                         /* TIM4 */
#define PWM2_ENCODER_TIME_IRQn              TIM4_IRQn
#define PWM2_ENCODER_TIME_IRQHandler        TIM4_IRQHandler

#define PWM2_A_CHANNEL                 TIM_CHANNEL_1                                /* 通道1 */
#define PWM2_A_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)   /* TIM4 时钟使能 */

#define PWM2_B_CHANNEL                   TIM_CHANNEL_2                                /* 通道2 */
#define PWM2_B_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM4_CLK_ENABLE(); }while(0)   /* TIM4 时钟使能 */

/* 基本定时器：PWM2 专用速度采样 */
#define PWM2_BTIM_INT                           TIM7
#define PWM2_BTIM_INT_IRQn                      TIM7_IRQn
#define PWM2_BTIM_INT_IRQHandler                TIM7_IRQHandler
#define PWM2_BTIM_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM7_CLK_ENABLE(); }while(0)
//第三台
/******************************* 电机编码器测速 **************************************/
/* 通用定时器 定义 */
#define PWM3_A_GPIO_PORT         GPIOA
#define PWM3_A_GPIO_PIN          GPIO_PIN_0
#define PWM3_A_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  /* PA口时钟使能 */

#define PWM3_B_GPIO_PORT         GPIOA
#define PWM3_B_GPIO_PIN          GPIO_PIN_1
#define PWM3_B_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)  /* PA口时钟使能 */

#define PWM3_A_GPIO_AF            GPIO_AF1_TIM2                                /* 端口复用到TIM2 */
#define PWM3_B_GPIO_AF            GPIO_AF1_TIM2                                /* 端口复用到TIM2 */

#define PWM3_ENCODER_TIME_SOURCE                       TIM2                                         /* TIM2 */
#define PWM3_ENCODER_TIME_IRQn              TIM2_IRQn
#define PWM3_ENCODER_TIME_IRQHandler        TIM2_IRQHandler

#define PWM3_A_CHANNEL                 TIM_CHANNEL_1                                /* 通道1 */
#define PWM3_A_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM2_CLK_ENABLE(); }while(0)   /* TIM2 时钟使能 */

#define PWM3_B_CHANNEL                   TIM_CHANNEL_2                                /* 通道2 */
#define PWM3_B_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM2_CLK_ENABLE(); }while(0)   /* TIM2 时钟使能 */

/* 基本定时器：PWM3 专用速度采样 */
#define PWM3_BTIM_INT                           TIM5
#define PWM3_BTIM_INT_IRQn                      TIM5_IRQn
#define PWM3_BTIM_INT_IRQHandler                TIM5_IRQHandler
#define PWM3_BTIM_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM5_CLK_ENABLE(); }while(0)
//第四台
/******************************* 电机编码器测速 **************************************/
#define PWM4_A_GPIO_PORT         GPIOA
#define PWM4_A_GPIO_PIN          GPIO_PIN_2
#define PWM4_A_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define PWM4_B_GPIO_PORT         GPIOA
#define PWM4_B_GPIO_PIN          GPIO_PIN_3
#define PWM4_B_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define PWM4_A_GPIO_AF            GPIO_AF2_TIM5
#define PWM4_B_GPIO_AF            GPIO_AF2_TIM5

#define PWM4_ENCODER_TIME_SOURCE        TIM5
#define PWM4_ENCODER_TIME_IRQn          TIM5_IRQn
#define PWM4_ENCODER_TIME_IRQHandler    TIM5_IRQHandler

#define PWM4_A_CHANNEL                 TIM_CHANNEL_3
#define PWM4_A_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM5_CLK_ENABLE(); }while(0)

#define PWM4_B_CHANNEL                 TIM_CHANNEL_4
#define PWM4_B_TIME_CLK_ENABLE()      do{ __HAL_RCC_TIM5_CLK_ENABLE(); }while(0)


/* 基本定时器：PWM4 专用速度采样 */
#define PWM4_BTIM_INT                           TIM12
#define PWM4_BTIM_INT_IRQn                      TIM8_BRK_TIM12_IRQn
#define PWM4_BTIM_INT_IRQHandler                TIM8_BRK_TIM12_IRQHandler
#define PWM4_BTIM_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM12_CLK_ENABLE(); }while(0)

#define REDUCTION_RATIO 30
#define ROTO_RATIO 1200

typedef struct 
{
    int encode_old;     /* 上一次计数值 */
    int encode_now;     /* 当前计数值 */
    float speed;        /* 编码器速度（RPM） */
} ENCODE_TypeDef;

class Encoder_ {
public:
    Encoder_(Pid_* pid);   // 构造函数，传入 PID 控制器指针（可选）
    
    // 编码器定时器初始化（成员函数）
    void pwm1_tim_encoder_init(uint16_t arr, uint16_t psc);
    void pwm2_tim_encoder_init(uint16_t arr, uint16_t psc);
    void pwm3_tim_encoder_init(uint16_t arr, uint16_t psc);
    void pwm4_tim_encoder_init(uint16_t arr, uint16_t psc);
    
    // 基本定时器初始化（用于周期性测速）
    void pwm1_btim_int_init(uint16_t arr, uint16_t psc);
    void pwm2_btim_int_init(uint16_t arr, uint16_t psc);
    void pwm3_btim_int_init(uint16_t arr, uint16_t psc);
    void pwm4_btim_int_init(uint16_t arr, uint16_t psc);
    
    // 获取编码器计数值（32位，带溢出计数）
    int pwm1_get_encode(void);
    int pwm2_get_encode(void);
    int pwm3_get_encode(void);
    int pwm4_get_encode(void);
    
    // 速度计算函数（在基本定时器中断中调用）
    void speed_computer(int32_t encode_now,uint8_t ms,ENCODE_TypeDef*g_encode,Motor_TypeDef*g_motor_data);
    
    // 电机运行标志
    uint8_t g1_run_flag, g2_run_flag, g3_run_flag, g4_run_flag;
    
    // 编码器数据结构（每个电机独立）
    ENCODE_TypeDef g_encode1, g_encode2, g_encode3, g_encode4;
    // 编码器句柄
		
		TIM_HandleTypeDef g1_tim_encode_handle;
    TIM_HandleTypeDef g2_tim_encode_handle;
    TIM_HandleTypeDef g3_tim_encode_handle;
    TIM_HandleTypeDef g4_tim_encode_handle;
		// 基本定时器句柄
		TIM_HandleTypeDef g_pwm1_btim_handle;
    TIM_HandleTypeDef g_pwm2_btim_handle;
    TIM_HandleTypeDef g_pwm3_btim_handle;
    TIM_HandleTypeDef g_pwm4_btim_handle;
		
		// 溢出计数变量
		volatile int g_encode1_count;
    volatile int g_encode2_count;
    volatile int g_encode3_count;
    volatile int g_encode4_count; 
		Pid_* pidptr;
private:
      
    
    TIM_Encoder_InitTypeDef sEncoderConfig; // 配置结构（可不用每次都创建）
};

// 外部声明全局对象
extern Encoder_ encoder;

#endif