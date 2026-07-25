#ifndef __CONFIG_H
#define __CONFIG_H

//motor.h的配置内容
#define MONITOR1_SPEED_LIMIT_VALUE 0X0F
#include "main.h"
typedef struct 
{
    uint8_t state;          /* 电机状态 */
    float current;          /* 电机电流 */
    float voltage;          /* 电机电压 */
    float power;            /* 电机功率 */
    float speed;            /* 电机实际速度 */
    float location;         /* 电机位置 */
    int32_t motor_pwm;      /* 设置比较值大小 */
} Motor_TypeDef;
#endif