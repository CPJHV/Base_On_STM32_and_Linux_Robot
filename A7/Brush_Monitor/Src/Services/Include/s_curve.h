#ifndef __S_CURVE_H
#define __S_CURVE_H

#include "stm32f4xx_hal.h"

typedef struct
{
    float target;
    float now;
    float dt;
    float max_jerk;
}S_Curve;

// 小车运动模式
typedef enum
{
    CAR_STOP,            // 停止
    CAR_STRAIGHT_HOLD,   // 直线行走 + 航向保持
    CAR_TURN_ANGLE,      // 定点转向
}Car_Mode;

// 全局车体状态
typedef struct
{
    Car_Mode mode;       // 当前模式
    float target_speed;  // 目标前进速度
    float target_yaw;    // 目标航向角
    float current_yaw;   // 当前航向角
}Car_State;

extern Car_State car_state;

void S_Curve_Init(S_Curve *sc, float dt, float max_jerk);
float S_Curve_Calc(S_Curve *sc, float target);
void Car_Kinematics_Calc(float v, float omega);
void Angle_PID_Param_Init(void);
void Car_Move_To_Target_Angle(float base_speed, float target_angle);
void Car_Straight_Hold_Head(float speed);    // 直线+航向保持
void Car_Set_Mode(Car_Mode mode);           // 设置模式
void Car_Stop(void);                         // 停止
void Car_Run_10ms(void);                     // 10ms调用

#endif