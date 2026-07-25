#ifndef __WRIST_STEP_H
#define __WRIST_STEP_H

#include "main.h"
#include "MotorCommon.h"

// 28BYJ-48 ULN2003 引脚定义
#define ULN_IN1_PIN     GPIO_PIN_12
#define ULN_IN2_PIN     GPIO_PIN_13
#define ULN_IN3_PIN     GPIO_PIN_14
#define ULN_IN4_PIN     GPIO_PIN_15
#define ULN_GPIO_PORT   GPIOB

#define ULN_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

// 28BYJ-48 参数
#define WRIST_FULL_STEP        512    // 减速后一圈总步数
#define WRIST_ANGLE_PER_STEP   (360.0f / WRIST_FULL_STEP)

// 6步通电时序表
extern const uint8_t wrist_seq_6step[6][4];

// 腕关节结构体
typedef struct
{
    int16_t cur_step;       // 当前步数
    int16_t target_step;    // 目标步数
    uint8_t dir;            // 0正转 1反转
    uint16_t delay_ms;      // 每步延时 ms
    uint8_t is_run;         // 运行标志
}Wrist_Step_t;

extern Wrist_Step_t wrist;

// 函数声明
void Wrist_GPIO_Init(void);
void Wrist_SetAllLow(void);
void Wrist_Move_Step(int16_t step_num, uint8_t dir, uint16_t speed_delay);
void Wrist_Task_Handler(void);
void Wrist_Angle_Move(float angle, uint16_t speed_delay);

#endif