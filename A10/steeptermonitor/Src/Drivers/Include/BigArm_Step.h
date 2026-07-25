#ifndef __BIG_ARM_STEP_
#define __BIG_ARM_STEP_

#include "main.h"
#include "MotorCommon.h"  // 公共定义

//========================= 大臂 TMC2209 引脚定义 =========================
#define BIGARM_TIMX_PWM_CH1_GPIO_PORT        GPIOA
#define BIGARM_TIMX_PWM_CH1_GPIO_PIN         GPIO_PIN_5
#define BIGARM_TIMX_PWM_CH1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define BIGARM_TIMX_PWM_CHY_GPIO_AF          GPIO_AF1_TIM2
#define BIGARM_TIMX_PWM                      TIM2
#define BIGARM_TIMX_INT_IRQn                 TIM2_IRQn
#define BIGARM_TIMX_INT_IRQHandler           TIM2_IRQHandler
#define BIGARM_TIMX_PWM_CH1                  TIM_CHANNEL_1
#define BIGARM_TIMX_PWM_CHY_CLK_ENABLE()     __HAL_RCC_TIM2_CLK_ENABLE()

#define BIGARM_DIR_GPIO_PORT                GPIOC
#define BIGARM_DIR_GPIO_PIN                 GPIO_PIN_0
#define BIGARM_DIR_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()

#define BIGARM_EN_GPIO_PORT                 GPIOC
#define BIGARM_EN_GPIO_PIN                  GPIO_PIN_1
#define BIGARM_EN_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()

#define BIGARM_SPREAD_GPIO_PORT             GPIOC
#define BIGARM_SPREAD_GPIO_PIN              GPIO_PIN_2
#define BIGARM_SPREAD_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()

#define BIGARM_INDEX_GPIO_PORT              GPIOC
#define BIGARM_INDEX_GPIO_PIN               GPIO_PIN_3
#define BIGARM_INDEX_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()

#define BIGARM_DIAG_GPIO_PORT               GPIOC
#define BIGARM_DIAG_GPIO_PIN                GPIO_PIN_4
#define BIGARM_DIAG_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()

//===================== 大臂电机类 =====================
class BigArm_Step
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
    void Set_SPREAD(uint8_t para);
    uint8_t Get_DIAG(void);
    uint8_t Get_INDEX(void);
    uint8_t calc_speed(int32_t vo, int32_t vt, float time);
    void stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step);
};

extern BigArm_Step big_arm;

#endif