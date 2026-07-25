#include "Wrist_Step.h"

// 6步换相时序 IN1 IN2 IN3 IN4
const uint8_t wrist_seq_6step[6][4] =
{
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,0,1}
};

Wrist_Step_t wrist;

// IO初始化
void Wrist_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_conf = {0};
    ULN_GPIO_CLK_ENABLE();

    gpio_conf.Pin = ULN_IN1_PIN|ULN_IN2_PIN|ULN_IN3_PIN|ULN_IN4_PIN;
    gpio_conf.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_conf.Pull = GPIO_NOPULL;
    gpio_conf.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ULN_GPIO_PORT, &gpio_conf);

    Wrist_SetAllLow();
    wrist.cur_step = 0;
    wrist.target_step = 0;
    wrist.is_run = 0;
}

// 全部引脚拉低断电
void Wrist_SetAllLow(void)
{
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN1_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN2_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN3_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN4_PIN,GPIO_PIN_RESET);
}

// 单步执行
static void Wrist_One_Step(uint8_t seq_idx)
{
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN1_PIN,wrist_seq_6step[seq_idx][0]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN2_PIN,wrist_seq_6step[seq_idx][1]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN3_PIN,wrist_seq_6step[seq_idx][2]?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ULN_GPIO_PORT,ULN_IN4_PIN,wrist_seq_6step[seq_idx][3]?GPIO_PIN_SET:GPIO_PIN_RESET);
}

// 设置移动步数、方向、速度
void Wrist_Move_Step(int16_t step_num, uint8_t dir, uint16_t speed_delay)
{
    if(step_num == 0) return;
    wrist.target_step = step_num;
    wrist.dir = dir;
    wrist.delay_ms = speed_delay;
    wrist.is_run = 1;
}

// 角度移动 正负代表方向
void Wrist_Angle_Move(float angle, uint16_t speed_delay)
{
    int16_t step = angle / WRIST_ANGLE_PER_STEP;
    if(step >= 0)
    {
        Wrist_Move_Step(step,0,speed_delay);
    }
    else
    {
        Wrist_Move_Step(-step,1,speed_delay);
    }
}

// 放在1ms/5ms定时器里轮询执行
void Wrist_Task_Handler(void)
{
    static uint8_t seq_cnt = 0;
    static uint16_t time_cnt = 0;

    if(!wrist.is_run) return;

    time_cnt++;
    if(time_cnt < wrist.delay_ms) return;
    time_cnt = 0;

    // 正转时序递增 反转递减
    if(wrist.dir == 0)
    {
        seq_cnt++;
        if(seq_cnt >= 6) seq_cnt = 0;
    }
    else
    {
        seq_cnt--;
        if(seq_cnt >= 6) seq_cnt = 5;
    }

    Wrist_One_Step(seq_cnt);
    wrist.cur_step ++;

    // 走完停止
    if(wrist.cur_step >= wrist.target_step)
    {
        wrist.is_run = 0;
        wrist.cur_step = 0;
        wrist.target_step = 0;
        Wrist_SetAllLow();
    }
}