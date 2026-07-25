#include "gpio.h"
 Monitor_Gpio gpio;
void Monitor_Gpio::Init()
{
    /* 初始化 SHUTDOWN 引脚时钟 */
    SHUTDOWN1_GPIO_CLK_ENABLE();
    SHUTDOWN2_GPIO_CLK_ENABLE();
    SHUTDOWN3_GPIO_CLK_ENABLE();
    SHUTDOWN4_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;

    /* SHUTDOWN1  PC8 */
    gpio_init_struct.Pin = SHUTDOWN1_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN1_GPIO_Port, &gpio_init_struct);

    /* SHUTDOWN2 (PB5), SHUTDOWN3 (PB4), SHUTDOWN4 (PB3) */
    gpio_init_struct.Pin = SHUTDOWN2_Pin | SHUTDOWN3_Pin | SHUTDOWN4_Pin;
    HAL_GPIO_Init(SHUTDOWN2_GPIO_Port, &gpio_init_struct);

    /* 默认全部拉低，关闭电机输出 */
    DISABLE_MOTOR1;
    DISABLE_MOTOR2;
    DISABLE_MOTOR3;
    DISABLE_MOTOR4;
}

// ==================== 电机1控制 ====================
void Monitor_Gpio::Start_Monitor1()
{
    ENABLE_MOTOR1;   /* 拉高SD引脚，开启电机1 */
}

void Monitor_Gpio::Stop_Monitor1()
{
    DISABLE_MOTOR1;  /* 拉低SD引脚，关闭电机1 */
}

// ==================== 电机2控制 ====================
void Monitor_Gpio::Start_Monitor2()
{
    ENABLE_MOTOR2;   /* 拉高SD引脚，开启电机2 */
}

void Monitor_Gpio::Stop_Monitor2()
{
    DISABLE_MOTOR2;  /* 拉低SD引脚，关闭电机2 */
}

// ==================== 电机3控制 ====================
void Monitor_Gpio::Start_Monitor3()
{
    ENABLE_MOTOR3;   /* 拉高SD引脚，开启电机3 */
}

void Monitor_Gpio::Stop_Monitor3()
{
    DISABLE_MOTOR3;  /* 拉低SD引脚，关闭电机3 */
}

// ==================== 电机4控制 ====================
void Monitor_Gpio::Start_Monitor4()
{
    ENABLE_MOTOR4;   /* 拉高SD引脚，开启电机4 */
}

void Monitor_Gpio::Stop_Monitor4()
{
    DISABLE_MOTOR4;  /* 拉低SD引脚，关闭电机4 */
}