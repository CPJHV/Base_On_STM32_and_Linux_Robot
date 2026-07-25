#include "gpio.h"

//使用软件缓存，不用频繁读取硬件io脚
static uint8_t fan_state = 0;
static uint8_t buzzer_state = 0;

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能时钟 */
  FAN_GPIO_CLK;
  BUZZER_GPIO_CLK;

  

  /* 配置风扇 + 蜂鸣器为推挽输出 */
  GPIO_InitStruct.Pin   = FAN_PIN | BUZZER_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;//推挽输出
  GPIO_InitStruct.Pull  = GPIO_NOPULL;//无上拉
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;//低速
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* 默认输出低电平 */
  HAL_GPIO_WritePin(FAN_GPIO_PORT,   FAN_PIN,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

void Fan_On(void)
{
  HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_PIN, GPIO_PIN_SET);
	fan_state = 1;
}

void Fan_Off(void)
{
  HAL_GPIO_WritePin(FAN_GPIO_PORT, FAN_PIN, GPIO_PIN_RESET);
	fan_state = 0;
}

void Buzzer_On(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_SET);
	buzzer_state = 1;
}

void Buzzer_Off(void)
{
  HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);
	buzzer_state = 0;
}

uint8_t get_FAN_State(void)
{
  return fan_state;
}

uint8_t get_Buzzer_State(void)
{
  return buzzer_state;
}