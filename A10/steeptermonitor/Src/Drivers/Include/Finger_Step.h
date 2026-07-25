#ifndef __FINGER_STEP_H
#define __FINGER_STEP_H

#include "main.h"
#include "MotorCommon.h"
#include "stm32f4xx_hal_uart.h"

//========================= 引脚定义 =========================
// 舵机 PWM  PD12 PD13
#define FINGER1_PWM_PIN        GPIO_PIN_12
#define FINGER1_PWM_PORT       GPIOD
#define FINGER2_PWM_PIN        GPIO_PIN_13
#define FINGER2_PWM_PORT       GPIOD

#define FINGER_PWM_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOD_CLK_ENABLE()

// 串口 UART4  TX=PC10 RX=PC11
#define FINGER_UART_TX_PIN      GPIO_PIN_10
#define FINGER_UART_RX_PIN      GPIO_PIN_11
#define FINGER_UART_PORT        GPIOC
#define FINGER_UART             UART4
#define FINGER_UART_CLK_ENABLE() __HAL_RCC_UART4_CLK_ENABLE()
#define FINGER_UART_AF          GPIO_AF8_UART4

// 定时器 TIM4
#define FINGER_TIM              TIM4
#define FINGER_TIM_CLK_ENABLE()  __HAL_RCC_TIM4_CLK_ENABLE()
#define FINGER_TIM_CH1          TIM_CHANNEL_1
#define FINGER_TIM_CH2          TIM_CHANNEL_2

//========================= 舵机参数 =========================
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_MIN_PULSE      500
#define SERVO_MAX_PULSE     2500

//========================= 类定义 =========================
class Finger_Step
{
public:
    TIM_HandleTypeDef  htim;
    UART_HandleTypeDef huart;

    void Finger_Init(void);
    void Finger_SetAngle(uint8_t servo_id, uint8_t angle);
    void Finger_UART_Send(uint8_t *data, uint16_t len);
    void Finger_Select_Servo(uint8_t id);

private:
    void PWM_Init(void);
    void UART_Init(void);
    uint16_t angle2pulse(uint8_t angle);
};

extern Finger_Step finger;

#endif