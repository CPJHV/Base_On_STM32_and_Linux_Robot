#ifndef __PID_H
#define __PID_H

#include "main.h"

#ifdef __cplusplus
extern "C" {  // 新增：告诉编译器这是C函数
#endif

typedef struct
{
    float target;
    float feedback;
    float err;
    float err_last;
    float kp, ki, kd;
    float out;
} PID_TypeDef;

void pid_init(PID_TypeDef *pid, float kp, float ki, float kd);
float increment_pid_ctrl(PID_TypeDef *pid, float feedback);

#ifdef __cplusplus
}  // 新增
#endif

#endif