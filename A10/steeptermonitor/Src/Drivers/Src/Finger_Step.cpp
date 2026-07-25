#include "Finger_Step.h"

Finger_Step finger;

uint16_t Finger_Step::angle2pulse(uint8_t angle)
{
    if(angle > 180) angle = 180;
    return SERVO_MIN_PULSE + (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle / 180;
}

void Finger_Step::PWM_Init(void)
{
    FINGER_PWM_GPIO_CLK_ENABLE();
    FINGER_TIM_CLK_ENABLE();

    htim.Instance = FINGER_TIM;
    htim.Init.Prescaler = 167;
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 19999;
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim);

    TIM_OC_InitTypeDef sConfig = {0};
    sConfig.OCMode = TIM_OCMODE_PWM1;
    sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;

    sConfig.Pulse = angle2pulse(90);
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfig, FINGER_TIM_CH1);
    HAL_TIM_PWM_Start(&htim, FINGER_TIM_CH1);

    sConfig.Pulse = angle2pulse(90);
    HAL_TIM_PWM_ConfigChannel(&htim, &sConfig, FINGER_TIM_CH2);
    HAL_TIM_PWM_Start(&htim, FINGER_TIM_CH2);
}

void Finger_Step::UART_Init(void)
{
    FINGER_UART_CLK_ENABLE();

    huart.Instance = FINGER_UART;
    huart.Init.BaudRate = 9600;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits = UART_STOPBITS_1;
    huart.Init.Parity = UART_PARITY_NONE;
    huart.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart);
}

void Finger_Step::Finger_Init(void)
{
    PWM_Init();
    UART_Init();
}

void Finger_Step::Finger_UART_Send(uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart, data, len, 100);
}

void Finger_Step::Finger_Select_Servo(uint8_t id)
{
    uint8_t cmd[4] = {0x55, 0x01, id, 0xAA};
    Finger_UART_Send(cmd, 4);
}

void Finger_Step::Finger_SetAngle(uint8_t servo_id, uint8_t angle)
{
    uint16_t pulse = angle2pulse(angle);

    if(servo_id == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim, FINGER_TIM_CH1, pulse);
    }
    else if(servo_id == 2)
    {
        __HAL_TIM_SET_COMPARE(&htim, FINGER_TIM_CH2, pulse);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance == FINGER_TIM)
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = FINGER1_PWM_PIN | FINGER2_PWM_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOD, &gpio);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    if(huart->Instance == FINGER_UART)
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = FINGER_UART_TX_PIN | FINGER_UART_RX_PIN;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Alternate = FINGER_UART_AF;
        HAL_GPIO_Init(GPIOC, &gpio);
    }
}