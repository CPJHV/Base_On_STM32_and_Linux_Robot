#ifndef __FOREARM_STEP_
#define __FOREARM_STEP_

#include "main.h"
#include "MotorCommon.h"

//========================= 小臂 TMC2209 引脚定义 =========================
// STEP -> PC6
#define FOREARM_TIMX_PWM_CH1_GPIO_PORT        GPIOC
#define FOREARM_TIMX_PWM_CH1_GPIO_PIN         GPIO_PIN_6
#define FOREARM_TIMX_PWM_CH1_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define FOREARM_TIMX_PWM_CHY_GPIO_AF          GPIO_AF2_TIM3

// 定时器 TIM3 通道1
#define FOREARM_TIMX_PWM                      TIM3
#define FOREARM_TIMX_INT_IRQn                 TIM3_IRQn
#define FOREARM_TIMX_INT_IRQHandler           TIM3_IRQHandler
#define FOREARM_TIMX_PWM_CH1                  TIM_CHANNEL_1
#define FOREARM_TIMX_PWM_CHY_CLK_ENABLE()     __HAL_RCC_TIM3_CLK_ENABLE()

// DIR -> PC7
#define FOREARM_DIR_GPIO_PORT                GPIOC
#define FOREARM_DIR_GPIO_PIN                 GPIO_PIN_7
#define FOREARM_DIR_CLK_ENABLE()             __HAL_RCC_GPIOC_CLK_ENABLE()

// EN -> PC8
#define FOREARM_EN_GPIO_PORT                 GPIOC
#define FOREARM_EN_GPIO_PIN                  GPIO_PIN_8
#define FOREARM_EN_CLK_ENABLE()              __HAL_RCC_GPIOC_CLK_ENABLE()

// SPREAD -> PC9
#define FOREARM_SPREAD_GPIO_PORT             GPIOC
#define FOREARM_SPREAD_GPIO_PIN              GPIO_PIN_9
#define FOREARM_SPREAD_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()

// INDEX -> PD0
#define FOREARM_INDEX_GPIO_PORT              GPIOD
#define FOREARM_INDEX_GPIO_PIN               GPIO_PIN_0
#define FOREARM_INDEX_CLK_ENABLE()           __HAL_RCC_GPIOD_CLK_ENABLE()

// DIAG -> PD1
#define FOREARM_DIAG_GPIO_PORT               GPIOD
#define FOREARM_DIAG_GPIO_PIN                GPIO_PIN_1
#define FOREARM_DIAG_CLK_ENABLE()            __HAL_RCC_GPIOD_CLK_ENABLE()

// UART：TX PB10  RX PB11（备用）
#define FOREARM_UART_TX_GPIO_PORT            GPIOB
#define FOREARM_UART_TX_GPIO_PIN             GPIO_PIN_10
#define FOREARM_UART_RX_GPIO_PORT            GPIOB
#define FOREARM_UART_RX_GPIO_PIN             GPIO_PIN_11

//===================== 小臂电机类 =====================
class Forearm_Step{
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

extern Forearm_Step forearm;

#endif