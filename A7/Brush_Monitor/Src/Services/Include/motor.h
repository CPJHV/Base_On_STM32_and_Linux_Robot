// motor.h
#ifndef __MOTOR_H
#define __MOTOR_H

#include "pwm.h"
#include "encoder.h"
#include "adc.h"
#include "config.h"
#include "gpio.h"

// 前置声明，避免循环包含
class Monitor_Gpio;
class Pwm_;



class Monitor_Controller {
public:
    Monitor_Controller(Monitor_Gpio* gpio, Pwm_* pwm);
    
    void Monitor_Gpio_Init();
    
    // 电机1 接口
    void dcmotor1_start(void);
    void dcmotor1_stop(void);
    void dcmotor1_dir(uint8_t para);
    void dcmotor1_set_speed(uint16_t para);
    void motor1_set_pwm(float para);
    
    // 电机2 接口
    void dcmotor2_start(void);
    void dcmotor2_stop(void);
    void dcmotor2_dir(uint8_t para);
    void dcmotor2_set_speed(uint16_t para);
    void motor2_set_pwm(float para);
    
    // 电机3 接口
    void dcmotor3_start(void);
    void dcmotor3_stop(void);
    void dcmotor3_dir(uint8_t para);
    void dcmotor3_set_speed(uint16_t para);
    void motor3_set_pwm(float para);
    
    // 电机4 接口
    void dcmotor4_start(void);
    void dcmotor4_stop(void);
    void dcmotor4_dir(uint8_t para);
    void dcmotor4_set_speed(uint16_t para);
    void motor4_set_pwm(float para);
    
    // 四台电机独立数据
    Motor_TypeDef g_motor1_data;             
    Motor_TypeDef g_motor2_data; 
    Motor_TypeDef g_motor3_data; 
    Motor_TypeDef g_motor4_data; 
    
		
		
private:
    Monitor_Gpio* gpioptr;
    Pwm_* pwmptr;
};

// 外部声明全局对象（在 motor.cpp 中定义）
extern Monitor_Controller monitor_contr;

#endif