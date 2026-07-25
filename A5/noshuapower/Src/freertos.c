/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS tasks creation (CMSIS-RTOS v2)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"           // 若用 v2 建议改为 cmsis_os2.h
#include "user_controller.h"     // 用户 C 接口

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* 任务句柄 */
osThreadId_t OLED_TaskHandle;
osThreadId_t ADC_Control_TaskHandle;
osThreadId_t WiFi_TaskHandle;

/* 任务属性 */
const osThreadAttr_t OLED_attributes = {
  .name = "OLED_Task",
  .stack_size = 512,
  .priority = osPriorityLow,
};

const osThreadAttr_t ADC_attributes = {
  .name = "ADC_Control_Task",
  .stack_size = 512,
  .priority = osPriorityHigh,
};

const osThreadAttr_t WiFi_attributes = {
  .name = "WiFi_Task",
  .stack_size = 768,            // WiFi 可能需要更大栈
  .priority = osPriorityNormal,
};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void OLED_Task(void *argument);
void ADC_Control_Task(void *argument);
void WiFi_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* 创建任务 */
  OLED_TaskHandle = osThreadNew(OLED_Task, NULL, &OLED_attributes);
  ADC_Control_TaskHandle = osThreadNew(ADC_Control_Task, NULL, &ADC_attributes);
  WiFi_TaskHandle = osThreadNew(WiFi_Task, NULL, &WiFi_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* 可以在此处添加其他初始化 */
  /* USER CODE END RTOS_THREADS */
}

/* 任务实现 ---------------------------------------------------------------- */
void OLED_Task(void *argument)
{
  for (;;)
  {
    User_Display_Update();    // 内部有任务通知等待，无需额外延时
    osDelay(10);              // 防止过于频繁，也允许其他任务运行
  }
}

void ADC_Control_Task(void *argument)
{
  for (;;)
  {
    User_ADC_Control_Task();
    osDelay(50);              // 50ms 周期
  }
}

void WiFi_Task(void *argument)
{
  for (;;)
  {
    User_WiFi_Task();
    osDelay(100);             // 100ms 轮询，实际任务内部有定时上传逻辑，但这里加延时减少 CPU 占用
  }
}

/* 空闲钩子：进入低功耗 (需要在 FreeRTOSConfig.h 中定义 configUSE_IDLE_HOOK = 1) */
void vApplicationIdleHook(void)
{
  User_Enter_LowPower();
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */