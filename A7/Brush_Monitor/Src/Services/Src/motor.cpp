// motor.cpp
#include "motor.h"
#include "gpio.h"
#include "pwm.h"

extern Monitor_Gpio gpio;
extern Pwm_ pwm;

// 全局对象定义
Monitor_Controller monitor_contr(&gpio, &pwm);

Monitor_Controller::Monitor_Controller(Monitor_Gpio* gpio, Pwm_* pwm)
    : gpioptr(gpio), pwmptr(pwm)
{
    // 初始化电机1
    g_motor1_data.state = 0;
    g_motor1_data.current = 0.0f;
    g_motor1_data.voltage = 0.0f;
    g_motor1_data.power = 0.0f;
    g_motor1_data.speed = 0.0f;
    g_motor1_data.location = 0.0f;
    g_motor1_data.motor_pwm = 0;

    // 初始化电机2
    g_motor2_data.state = 0;
    g_motor2_data.current = 0.0f;
    g_motor2_data.voltage = 0.0f;
    g_motor2_data.power = 0.0f;
    g_motor2_data.speed = 0.0f;
    g_motor2_data.location = 0.0f;
    g_motor2_data.motor_pwm = 0;

    // 初始化电机3
    g_motor3_data.state = 0;
    g_motor3_data.current = 0.0f;
    g_motor3_data.voltage = 0.0f;
    g_motor3_data.power = 0.0f;
    g_motor3_data.speed = 0.0f;
    g_motor3_data.location = 0.0f;
    g_motor3_data.motor_pwm = 0;

    // 初始化电机4
    g_motor4_data.state = 0;
    g_motor4_data.current = 0.0f;
    g_motor4_data.voltage = 0.0f;
    g_motor4_data.power = 0.0f;
    g_motor4_data.speed = 0.0f;
    g_motor4_data.location = 0.0f;
    g_motor4_data.motor_pwm = 0;
}

void Monitor_Controller::Monitor_Gpio_Init()
{
    if (gpioptr) gpioptr->Init();
}

// ==================== 电机1 实现 ====================
void Monitor_Controller::dcmotor1_start(void)
{
    if (gpioptr) gpioptr->Start_Monitor1();
}

void Monitor_Controller::dcmotor1_stop(void)
{
    if (pwmptr) pwmptr->PWM1_STOP();
    if (gpioptr) gpioptr->Stop_Monitor1();
}

void Monitor_Controller::dcmotor1_dir(uint8_t para)
{
    if (pwmptr) {
        pwmptr->PWM1_STOP();
        pwmptr->PWM1_HorL_START(para);
    }
}

void Monitor_Controller::dcmotor1_set_speed(uint16_t para)
{
    if (pwmptr) {
        // 修正：传递整个句柄的指针，而不是 Instance
        uint16_t arr = __HAL_TIM_GetAutoreload(&pwmptr->g_pwm1_cplm_pwm_handle);
        #ifdef MONITOR1_SPEED_LIMIT_VALUE
        if (para > arr - MONITOR1_SPEED_LIMIT_VALUE) para = arr - MONITOR1_SPEED_LIMIT_VALUE;
        #endif
        __HAL_TIM_SetCompare(&pwmptr->g_pwm1_cplm_pwm_handle, PWM1_TIME_CHANNEL, para);
    }
}

void Monitor_Controller::motor1_set_pwm(float para)
{
    int val = (int)para;
    if (val >= 0) {
        dcmotor1_dir(0);
        dcmotor1_set_speed((uint16_t)val);
    } else {
        dcmotor1_dir(1);
        dcmotor1_set_speed((uint16_t)(-val));
    }
}

// ==================== 电机2 实现 ====================
void Monitor_Controller::dcmotor2_start(void)
{
    if (gpioptr) gpioptr->Start_Monitor2();
}

void Monitor_Controller::dcmotor2_stop(void)
{
    if (pwmptr) pwmptr->PWM2_STOP();
    if (gpioptr) gpioptr->Stop_Monitor2();
}

void Monitor_Controller::dcmotor2_dir(uint8_t para)
{
    if (pwmptr) {
        pwmptr->PWM2_STOP();
        pwmptr->PWM2_HorL_START(para);
    }
}

void Monitor_Controller::dcmotor2_set_speed(uint16_t para)
{
    if (pwmptr) {
        // 修正：传递整个句柄的指针
        uint16_t arr = __HAL_TIM_GetAutoreload(&pwmptr->g_pwm2_cplm_pwm_handle);
        #ifdef MONITOR2_SPEED_LIMIT_VALUE
        if (para > arr - MONITOR2_SPEED_LIMIT_VALUE) para = arr - MONITOR2_SPEED_LIMIT_VALUE;
        #endif
        __HAL_TIM_SetCompare(&pwmptr->g_pwm2_cplm_pwm_handle, PWM2_TIME_CHANNEL, para);
    }
}

void Monitor_Controller::motor2_set_pwm(float para)
{
    int val = (int)para;
    if (val >= 0) {
        dcmotor2_dir(0);
        dcmotor2_set_speed((uint16_t)val);
    } else {
        dcmotor2_dir(1);
        dcmotor2_set_speed((uint16_t)(-val));
    }
}

// ==================== 电机3 实现 ====================
void Monitor_Controller::dcmotor3_start(void)
{
    if (gpioptr) gpioptr->Start_Monitor3();
}

void Monitor_Controller::dcmotor3_stop(void)
{
    if (pwmptr) pwmptr->PWM3_STOP();
    if (gpioptr) gpioptr->Stop_Monitor3();
}

void Monitor_Controller::dcmotor3_dir(uint8_t para)
{
    if (pwmptr) {
        pwmptr->PWM3_STOP();
        pwmptr->PWM3_HorL_START(para);
    }
}

void Monitor_Controller::dcmotor3_set_speed(uint16_t para)
{
    if (pwmptr) {
        // 修正：传递整个句柄的指针
        uint16_t arr = __HAL_TIM_GetAutoreload(&pwmptr->g_pwm3_cplm_pwm_handle);
        #ifdef MONITOR3_SPEED_LIMIT_VALUE
        if (para > arr - MONITOR3_SPEED_LIMIT_VALUE) para = arr - MONITOR3_SPEED_LIMIT_VALUE;
        #endif
        __HAL_TIM_SetCompare(&pwmptr->g_pwm3_cplm_pwm_handle, PWM3_TIME_CHANNEL, para);
    }
}

void Monitor_Controller::motor3_set_pwm(float para)
{
    int val = (int)para;
    if (val >= 0) {
        dcmotor3_dir(0);
        dcmotor3_set_speed((uint16_t)val);
    } else {
        dcmotor3_dir(1);
        dcmotor3_set_speed((uint16_t)(-val));
    }
}

// ==================== 电机4 实现 ====================
void Monitor_Controller::dcmotor4_start(void)
{
    if (gpioptr) gpioptr->Start_Monitor4();
}

void Monitor_Controller::dcmotor4_stop(void)
{
    if (pwmptr) pwmptr->PWM4_STOP();
    if (gpioptr) gpioptr->Stop_Monitor4();
}

void Monitor_Controller::dcmotor4_dir(uint8_t para)
{
    if (pwmptr) {
        pwmptr->PWM4_STOP();
        pwmptr->PWM4_HorL_START(para);
    }
}

void Monitor_Controller::dcmotor4_set_speed(uint16_t para)
{
    if (pwmptr) {
        // 修正：传递整个句柄的指针
        uint16_t arr = __HAL_TIM_GetAutoreload(&pwmptr->g_pwm4_cplm_pwm_handle);
        #ifdef MONITOR4_SPEED_LIMIT_VALUE
        if (para > arr - MONITOR4_SPEED_LIMIT_VALUE) para = arr - MONITOR4_SPEED_LIMIT_VALUE;
        #endif
        __HAL_TIM_SetCompare(&pwmptr->g_pwm4_cplm_pwm_handle, PWM4_TIME_CHANNEL, para);
    }
}

void Monitor_Controller::motor4_set_pwm(float para)
{
    int val = (int)para;
    if (val >= 0) {
        dcmotor4_dir(0);
        dcmotor4_set_speed((uint16_t)val);
    } else {
        dcmotor4_dir(1);
        dcmotor4_set_speed((uint16_t)(-val));
    }
}