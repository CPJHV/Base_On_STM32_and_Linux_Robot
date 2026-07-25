#ifndef _Monitor_GPIO_
#define _Monitor_GPIO_

#include "stm32f4xx_hal.h"

/*************************************    第一部分    基本驱动    *****************************************************/

/* 停止引脚操作宏定义 
 * 此引脚控制H桥是否生效以达到开启和关闭电机的效果
 */
#define SHUTDOWN1_Pin                 GPIO_PIN_8
#define SHUTDOWN1_GPIO_Port           GPIOC
#define SHUTDOWN1_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE();}while(0) 

#define SHUTDOWN2_Pin                 GPIO_PIN_5
#define SHUTDOWN2_GPIO_Port           GPIOB
#define SHUTDOWN2_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define SHUTDOWN3_Pin                 GPIO_PIN_4
#define SHUTDOWN3_GPIO_Port           GPIOB
#define SHUTDOWN3_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define SHUTDOWN4_Pin                 GPIO_PIN_3
#define SHUTDOWN4_GPIO_Port           GPIOB
#define SHUTDOWN4_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* 电机1开关 */
#define ENABLE_MOTOR1    HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port, SHUTDOWN1_Pin, GPIO_PIN_SET)
#define DISABLE_MOTOR1   HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port, SHUTDOWN1_Pin, GPIO_PIN_RESET)

/* 电机2开关 */
#define ENABLE_MOTOR2    HAL_GPIO_WritePin(SHUTDOWN2_GPIO_Port, SHUTDOWN2_Pin, GPIO_PIN_SET)
#define DISABLE_MOTOR2   HAL_GPIO_WritePin(SHUTDOWN2_GPIO_Port, SHUTDOWN2_Pin, GPIO_PIN_RESET)

/* 电机3开关 */
#define ENABLE_MOTOR3    HAL_GPIO_WritePin(SHUTDOWN3_GPIO_Port, SHUTDOWN3_Pin, GPIO_PIN_SET)
#define DISABLE_MOTOR3   HAL_GPIO_WritePin(SHUTDOWN3_GPIO_Port, SHUTDOWN3_Pin, GPIO_PIN_RESET)

/* 电机4开关 */
#define ENABLE_MOTOR4    HAL_GPIO_WritePin(SHUTDOWN4_GPIO_Port, SHUTDOWN4_Pin, GPIO_PIN_SET)
#define DISABLE_MOTOR4   HAL_GPIO_WritePin(SHUTDOWN4_GPIO_Port, SHUTDOWN4_Pin, GPIO_PIN_RESET)

class Monitor_Gpio {
public:
    void Init();
    
    // 电机1控制
    void Start_Monitor1();
    void Stop_Monitor1();
    
    // 电机2控制
    void Start_Monitor2();
    void Stop_Monitor2();
    
    // 电机3控制
    void Start_Monitor3();
    void Stop_Monitor3();
    
    // 电机4控制
    void Start_Monitor4();
    void Stop_Monitor4();
};

extern Monitor_Gpio gpio;

#endif