#ifndef __SHOULDER_STEP_
#define __SHOULDER_STEP_

#include "main.h"
#include "MotorCommon.h"  // 公共定义

//========================= 引脚定义 =========================
#define PUL_TIMX_PWM_CH1_GPIO_PORT        GPIOE
#define PUL_TIMX_PWM_CH1_GPIO_PIN         GPIO_PIN_9
#define PUL_TIMX_PWM_CH1_GPIO_CLK_ENABLE()  do{  __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)
#define PUL_TIMX_PWM_CHY_GPIO_AF          GPIO_AF1_TIM1
#define PUL_TIMX_PWM                      TIM1
#define PUL_TIMX_INT_IRQn                 TIM1_CC_IRQn
#define PUL_TIMX_INT_IRQHandler           TIM1_CC_IRQHandler
#define PUL_TIMX_PWM_CH1                  TIM_CHANNEL_1
#define PUL_TIMX_PWM_CHY_CCRX             TIM1->CCR1
#define PUL_TIMX_PWM_CHY_CLK_ENABLE()     do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)

#define ENCODER_TIMX_PWM_CHEA_GPIO        GPIOA
#define ENCODER_TIMX_PWM_CHEA_PIN	      GPIO_PIN_0
#define ENCODER_TIMX_PWM_CHEB_GPIO        GPIOA
#define ENCODER_TIMX_PWM_CHEB_PIN	      GPIO_PIN_1
#define ENCODER_GPIO_CLK_ENABLE()	      do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define ENCODER_GPIO_AF                   GPIO_AF2_TIM5
#define ENCODER_TIMX_PWM                  TIM5
#define ENCODER_TIMX_INT_IRQn             TIM5_IRQn
#define ENCODER_TIMX_INT_IRQHandler       TIM5_IRQHandler
#define ENCODER_TIMX_PWM_CH1              TIM_CHANNEL_1
#define ENCODER_TIMX_PWM_CH2              TIM_CHANNEL_2
#define ENCODER_TIMX_PWM_CHY_CCRX         TIM5->CCR1
#define ENCODER_TIMX_PWM_CHY_CLK_ENABLE() do{ __HAL_RCC_TIM5_CLK_ENABLE(); }while(0)

#define DIR_GPIO_PORT                     GPIOE
#define DIR_GPIO_PIN                      GPIO_PIN_10
#define DIR_GPIO_CLK_ENABLE()	          do{  __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define EN_GPIO_PORT                      GPIOE
#define EN_GPIO_PIN                       GPIO_PIN_11
#define EN_GPIO_CLK_ENABLE()	          do{  __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define PEND_GPIO_PORT                    GPIOE
#define PEND_GPIO_PIN                     GPIO_PIN_14
#define PEND_GPIO_CLK_ENABLE()	          do{  __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define ALM_GPIO_PORT                     GPIOE
#define ALM_GPIO_PIN                      GPIO_PIN_15
#define ALM_GPIO_CLK_ENABLE()	          do{  __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

#define EZ_GPIO_PORT                      GPIOA
#define EZ_GPIO_PIN                       GPIO_PIN_2
#define EZ_GPIO_CLK_ENABLE()	          do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

//========================= 闭环参数 =========================
#define FREQ_UINT                        (T1_FREQ/(1000/10))
#define ENCODER_SPR                      (float)(2000*4)
#define MPR                              5
#define PPM                              (ENCODER_SPR/MPR)
#define MPP                              ((float)(MPR)/ENCODER_SPR)
#define FEEDBACK_CONST                   (float)(PULSE_REV/ENCODER_SPR)
#define SMAPLSE_PID_SPEED                20

//===================== 类定义 =====================
class Shoulder_Step
{
public:
    TIM_HandleTypeDef        g_atimx_handle;
    TIM_OC_InitTypeDef       g_atimx_oc_chy_handle;
    DIR_STATE                Dir_State;
    uint32_t                 step_frequent_val;
    int                      angle;
    volatile uint32_t        pulse_cout;
    volatile int             add_pulse_count;
    speed_calc_t             s_calc_speed;
    motor_state_typedef      motor_data;
    uint32_t                 g_step_pos;
    uint16_t                 g_toggle_pulse;
    volatile uint8_t         i;

    void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc);
    void change_StarorStop_Monitor(uint8_t para);
    void set_steper_angle(uint16_t angle, DIR_STATE dir);
    void Gpio_Init_other(void);
    void Set_Dir(uint8_t para);
    void Set_ENABLE(uint8_t para);
    void Set_EZ(uint8_t para);
    uint8_t calc_speed(int32_t vo, int32_t vt, float time);
    void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step);

    TIM_HandleTypeDef        g_timx_encode_chy_handle;
    TIM_Encoder_InitTypeDef  g_timx_encoder_chy_handle;
    void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc);
};

Shoulder_Step sh_step;

#endif