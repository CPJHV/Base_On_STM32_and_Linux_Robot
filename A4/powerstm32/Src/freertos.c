/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS 任务创建（CMSIS-RTOSv2）
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

#include "usercontroller.h"   // 只包含 C 接口部分


#define configUSE_FPU 1

/* 任务函数声明 */
void OLED_Task(void *argument);
void CAN_Tx_Task(void *argument);
void ADC_Control_Task(void *argument);

/* 任务句柄 */
osThreadId_t OLED_TaskHandle;
osThreadId_t CAN_Tx_TaskHandle;
osThreadId_t ADC_Control_TaskHandle;

/**
  * @brief  FreeRTOS 初始化 → 动态创建任务
  */
void MX_FREERTOS_Init(void)
{
    osThreadAttr_t attr = {0};
    attr.stack_size = 512;

    // 任务1：OLED 显示（优先级最低，会等待任务通知）
    attr.name = "OLED_Task";
    attr.priority = osPriorityLow;
    OLED_TaskHandle = osThreadNew(OLED_Task, NULL, &attr);

    // 任务2：CAN 定时发送数据（中等优先级）
    attr.name = "CAN_Tx_Task";
    attr.priority = osPriorityNormal;
    CAN_Tx_TaskHandle = osThreadNew(CAN_Tx_Task, NULL, &attr);

    // 任务3：ADC + 电源管理 + 自动控制（高优先级，实时采集）
    attr.name = "ADC_Control_Task";
    attr.priority = osPriorityHigh;
    ADC_Control_TaskHandle = osThreadNew(ADC_Control_Task, NULL, &attr);

    // 启动 RTOS 相关的队列和命令处理任务
    User_Start_RTOS_Tasks();
}

// ========================== 任务实现 ==========================
void OLED_Task(void *argument)
{
    for (;;)
    {
        User_Display_Update();   // 内部会调用 ulTaskNotifyTake 等待/超时
        // 注意：User_Display_Update 已经含有延时（任务通知超时 200ms）
        // 这里不再需要 osDelay，否则会导致刷新周期变长。
        // 但为了稳定，可以加一个短延时：
        osDelay(10);
    }
}

void CAN_Tx_Task(void *argument)
{
    for (;;)
    {
        User_CAN_Send_Data();
        osDelay(500);   // 每 500ms 发送一次 CAN 数据
    }
}

void ADC_Control_Task(void *argument)
{
    for (;;)
    {
        User_ADC_Control_Task();
        osDelay(50);    // 50ms 采集控制一次
    }
}