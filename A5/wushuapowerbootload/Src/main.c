/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 地址定义 ----------------------------------------------------------------*/
#define APP_A_ADDR          0x08003000      /* APP A 起始地址 (24KB) */
#define APP_B_ADDR          0x08009000      /* APP B 起始地址 (24KB) */
#define STATUS_FLAG_ADDR    0x0800F000      /* 状态标志区 (4KB, 实际使用第一页512字节) */

typedef struct {
    uint32_t magic;          /* 固定为 0x5A5A5A5A */
    uint32_t active_app;     /* 0: APP_A, 1: APP_B */
    uint32_t upgrade_state;  /* 0: 空闲, 1: 接收中, 2: 完成待切换 */
    uint32_t upgrade_crc;    /* 新固件的 CRC32 值 */
    uint32_t upgrade_size;   /* 新固件大小（字节） */
    uint32_t reserved[2];
} AppStatus_t;
void Error_Handler(void);

static void JumpToApp(uint32_t app_addr);
static uint32_t CalculateCRC32(uint32_t start_addr, uint32_t size);
static void WriteStatus(const AppStatus_t *p);
static void Flash_Write_HalfWord(uint32_t addr, uint16_t data);
static void Flash_ErasePage(uint32_t page_addr);
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    AppStatus_t *pStatus = (AppStatus_t*)STATUS_FLAG_ADDR;

    /* 1. 初始化状态区（如果 magic 不正确） */
    if (pStatus->magic != 0x5A5A5A5A) {
        AppStatus_t init = {
            .magic = 0x5A5A5A5A,
            .active_app = 0,
            .upgrade_state = 0,
            .upgrade_crc = 0,
            .upgrade_size = 0,
            .reserved = {0}
        };
        WriteStatus(&init);
        pStatus = (AppStatus_t*)STATUS_FLAG_ADDR;
    }

    /* 2. 如果存在待切换的升级（upgrade_state == 2） */
    if (pStatus->upgrade_state == 2) {
        uint32_t new_app_addr = (pStatus->active_app == 0) ? APP_B_ADDR : APP_A_ADDR;
        uint32_t calc_crc = CalculateCRC32(new_app_addr, pStatus->upgrade_size);

        if (calc_crc == pStatus->upgrade_crc) {
            /* CRC 正确，切换 active_app */
            pStatus->active_app = (pStatus->active_app == 0) ? 1 : 0;
        }
        /* 无论 CRC 是否正确，都清除 upgrade_state */
        pStatus->upgrade_state = 0;
        WriteStatus(pStatus);
    }

    /* 3. 跳转到当前激活的 APP */
    uint32_t app_addr = (pStatus->active_app == 0) ? APP_A_ADDR : APP_B_ADDR;
    JumpToApp(app_addr);

    /* 正常情况下不会执行到这里 */
    while (1);
}
static void JumpToApp(uint32_t app_addr)
{
    /* 检查栈顶指针和复位向量是否合理 */
    uint32_t sp = *(__IO uint32_t*)app_addr;
    uint32_t reset = *(__IO uint32_t*)(app_addr + 4);
    if ((sp < 0x20000000 || sp >= 0x20005000) || (reset < 0x08000000)) {
        /* 无效 APP，死循环等待外部干预（如 CAN 升级） */
        while (1);
    }

    __disable_irq();
    __set_MSP(sp);
    typedef void (*pFunction)(void);
    pFunction jump = (pFunction)reset;
    jump();
    while (1);
}

/* 计算指定 Flash 区域的 CRC32（软件实现） */
static uint32_t CalculateCRC32(uint32_t start_addr, uint32_t size)
{
    uint32_t crc = 0xFFFFFFFF;
    uint8_t *p = (uint8_t*)start_addr;
    for (uint32_t i = 0; i < size; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

/* 向 Flash 写入一个半字 */
static void Flash_Write_HalfWord(uint32_t addr, uint16_t data)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data);
    HAL_FLASH_Lock();
}

/* 擦除一页 (512 字节) */
static void Flash_ErasePage(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_addr,
        .NbPages = 1
    };
    uint32_t page_err;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &page_err);
    HAL_FLASH_Lock();
}

/* 将状态结构体写入 Flash（擦除整个状态页，然后逐半字写入） */
static void WriteStatus(const AppStatus_t *p)
{
    Flash_ErasePage(STATUS_FLAG_ADDR);
    uint32_t addr = STATUS_FLAG_ADDR;
    const uint8_t *byte_ptr = (const uint8_t*)p;
    for (uint32_t i = 0; i < sizeof(AppStatus_t); i += 2) {
        uint16_t halfword = (byte_ptr[i+1] << 8) | byte_ptr[i];
        Flash_Write_HalfWord(addr + i, halfword);
    }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
