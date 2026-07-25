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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "mbedtls_config.h"
#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
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
// RSA公钥DER二进制数组，使用bin2c工具转换pub.der得到，自行填充
const uint8_t pub_der[] = {
    // 在此粘贴你的公钥DER字节数组
};
const uint32_t pub_der_len = sizeof(pub_der);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ReadStatus(AppStatus_t *out);
static void ReadStatusSingle(uint32_t addr, AppStatus_t *out);
static HAL_StatusTypeDef WriteStatusSingle(uint32_t addr, AppStatus_t *p);
static int SHA256_CalcRegion(uint32_t start_addr, uint32_t size, uint8_t *out_hash);
static int RSA_VerifyFirmware(const uint8_t *hash, const uint8_t *sign);
static void JumpToApp(uint32_t app_addr);
static uint32_t CalculateCRC32_Region(uint32_t start_addr, uint32_t size);
static void WriteStatus(AppStatus_t *p);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
CPU永远会先从第一条0X0000开始的，但是取出的虚拟地址，会被硬件映射转成实际物理地址
但是这个是属于PC带MMU设备的才可以有这个虚拟映射，而我们单片机是不存在有MMU的，因此
我们这里是采用的是基地址+偏移来映射，在单片机里叫内存重映射。


而如何映射，是BOOT0和BOOT1决定的
如果boot0=0则正常模式，0x0000会被映射到0x08000区
如果boot0=1，boot1=0，则进入系统的bootload模式
0x0000被映射到0x1fff0000系统存储区
而我们平时用的是原厂的ISP Bootloader就是在那的;
Cotex-M向量表规范：
基地址+0X00：SP初始化栈指针
基地址+0x04：复位程序入口地址PC  Reset_Handler 函数地址

【适配STM32F4分区】
bootload：
IROM1: Start=0x08000000, Size=0x8000 (32KB Flash)
IRAM1: Start=0x20000000, Size=0x2000 (8KB RAM)
APP_A：
IROM1: Start=0x08008000, Size=0x10000 (64KB Flash)
IRAM1: Start=0x20000000, Size=0x18000 (96KB RAM)
APP_B:
IROM1: Start=0x08018000, Size=0x10000 (64KB Flash)
IRAM1: Start=0x20000000, Size=0x18000 (96KB RAM)

RAM相当于内存，FLASH相当于ssd卡，分别作为两块独立物理物品
Flash 区块：地址段 0x08000000 ~ 0x080FFFFF
SRAM 区块：地址段 0x20000000 ~ 0x2001FFFF

RAM中空间排布是：
高地址：
	stack（栈）
	heap（堆）
	zi（0初始化静态数据区）
	rw（有初始值的静态变量）
低地址：

ROM存储的是
：code代码段
：RO-data只读常量段
注意一个点就是const char *msg="test"；这个字符串在ROM中，而msg在RAM中，因此会复制一个字符串给RAM；
但是如果是const char msy[]="test"则不会复制过去，而是都在ROM中；

*/

/* USER CODE END 0 */
/* 地址定义【STM32F4 分区】 */
#define STATUS_PAGE0    0x08028000U
#define STATUS_PAGE1    0x08028800U
//这里使用双分表来保证中途断电，也有一张合法的表
#define APP_A_ADDR          0x08008000U//APP1
#define APP_B_ADDR          0x08018000U//APP2
/*
0x0800 0000 到0x0800 7FFF为bootload程序
0x0800 8000 到0x0801 7FFF为APP1程序
0x0801 8000 到0x0802 7FFF为APP2程序
0x0802 8000 / 0x08028800 双状态备份页
*/
/* 状态结构体（必须与 APP 中定义一致） */
typedef struct {
    uint32_t magic;         // 固定 0x5A5A5A5A
    uint32_t active_app;    // 0 = APP_A, 1 = APP_B
    uint32_t upgrade_state; // 0:空闲, 1:接收中, 2:完成待切换
    uint32_t upgrade_crc;   // 接收固件的CRC32
    uint32_t upgrade_size;  // 固件大小
    uint32_t reserved[2];
	uint32_t write_cnt;//目前flash写入的次数
} AppStatus_t;

// 固件头部，存放在备用分区起始地址
typedef struct
{
    uint32_t firmware_size;    // APP有效二进制长度
    uint32_t firmware_crc32;    // CRC32（原有CRC算法）
    uint8_t  rsa_sign[256];    // RSA2048签名
} FirmwareHead_t;
//为了支持更好的检验效果，因此使用mbedtls实现RSA验签
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();
    SystemClock_Config();

//    AppStatus_t *p = (AppStatus_t*)STATUS_FLAG_ADDR;//读取状态标志，看看要跑A还是B
	//上面这种方式有隐患的，因为CPU是4字节访问
	/*
	如果一个结构体成员都是uint32_t，那么编译器会默认它为4字节对齐
	A指针指向只能是指向4字节对齐的地址，否则会认为是未定义行为，0x0800F000这里只是恰好是4字节地址而已
	深层原因是，如果我的地址不是4字节对齐，那么编译器会分成两步16bit来拼凑32bit
	不仅慢，M0内核还会触发硬件错误
	*/
	//推荐使用memcpy，因为它是一个字节一个字节的搬运，不需要字节对齐
	AppStatus_t tmp;
	ReadStatus(&tmp);

	if (tmp.magic != 0x5A5A5A5AU)
	{//说明是第一次下载程序或更新bootload程序
    AppStatus_t init = {
        .magic = 0x5A5A5A5AU,
        .active_app = 0U,
        .upgrade_state = 0U,
        .upgrade_crc = 0U,
        .upgrade_size = 0U,
        .reserved = {0U},
		.write_cnt=0U
    };
    WriteStatus(&init);
    // 写完重新读取最新状态到tmp
    ReadStatus(&tmp);
	}

// 后续全部操作 tmp，不使用裸指针直接访问Flash！
	if(tmp.upgrade_state == 2U)
	{
		uint32_t new_app_addr = (tmp.active_app == 0U) ? APP_B_ADDR : APP_A_ADDR;
		FirmwareHead_t head_buf;
		// 禁止裸结构体指针直接读取Flash，使用memcpy拷贝至RAM
		memcpy(&head_buf, (void *)new_app_addr, sizeof(FirmwareHead_t));

		uint32_t bin_start = new_app_addr + sizeof(FirmwareHead_t);
		uint32_t bin_len = head_buf.firmware_size;

		// 第一层：CRC快速完整性校验
		uint32_t calc_crc = CalculateCRC32_Region(bin_start, bin_len);
		if(calc_crc != head_buf.firmware_crc32)
		{
			tmp.upgrade_state = 0U;
			WriteStatus(&tmp);
			goto UPGRADE_END;
		}

		// 第二层：SHA256 + RSA验签
		uint8_t hash_buf[32U];
		if(SHA256_CalcRegion(bin_start, bin_len, hash_buf) != 0)
		{
			tmp.upgrade_state = 0U;
			WriteStatus(&tmp);
			goto UPGRADE_END;
		}
		if(RSA_VerifyFirmware(hash_buf, head_buf.rsa_sign) != 0)
		{
			// 签名非法，拒绝切换分区
			tmp.upgrade_state = 0U;
			WriteStatus(&tmp);
			goto UPGRADE_END;
		}

		// 两层校验通过，额外校验向量表防止固件损坏
		uint32_t test_sp    = *(__IO uint32_t *)bin_start;
		uint32_t test_reset = *(__IO uint32_t *)(bin_start + 4U);
		// F4 RAM范围 0x20000000 ~ 0x2001FFFF
		if((test_sp >= 0x20000000U && test_sp <= 0x2001FFFFU)
			&& (test_reset >= APP_A_ADDR && test_reset < STATUS_PAGE0))
		{
			tmp.active_app = (tmp.active_app == 0U) ? 1U : 0U;
		}
		tmp.upgrade_state = 0U;
		WriteStatus(&tmp);

UPGRADE_END:
		;
	}

	uint32_t app_addr = (tmp.active_app == 0U) ? APP_A_ADDR : APP_B_ADDR;
	JumpToApp(app_addr);

    while(1) {
        // 理论上不会到这里
    }
}

/* 系统时钟配置（F4 标准HSE 8M -> 168MHz，根据硬件调整） */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8U;
    RCC_OscInitStruct.PLL.PLLN = 336U;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7U;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/* 跳转到指定 APP 地址 */
static void JumpToApp(uint32_t app_addr)
{
    // 检查栈顶指针和复位向量是否合理
    uint32_t sp = *(__IO uint32_t*)app_addr;//栈顶，它里面存的值是RAM里面的栈顶指针位置
    uint32_t reset = *(__IO uint32_t*)(app_addr + 4U);//程序入口

    if ((sp < 0x20000000U || sp > 0x2001FFFFU) || (reset < APP_A_ADDR || reset >= STATUS_PAGE0)) {
        // 无效的 APP，死循环
        while(1);
    }

    typedef void (*pFunction)(void);
    pFunction jump = (pFunction)reset;//强转成函数指针

    __disable_irq();
		
	HAL_SuspendTick();    // 先关闭systick，再操作时钟
	HAL_RCC_DeInit();     // RCC复位，会关闭SysTick时钟源
	// 清除所有中断挂起标志
    NVIC->ICER[0] = 0xFFFFFFFFU;
	NVIC->ICER[1] = 0xFFFFFFFFU;
	NVIC->ICER[2] = 0xFFFFFFFFU;
	NVIC->ICER[3] = 0xFFFFFFFFU;
	NVIC->ICPR[0] = 0xFFFFFFFFU;
	NVIC->ICPR[1] = 0xFFFFFFFFU;
	NVIC->ICPR[2] = 0xFFFFFFFFU;
	NVIC->ICPR[3] = 0xFFFFFFFFU;
		SCB->VTOR=app_addr;//设置中断向量表
    __set_MSP(sp);//设置栈顶指针
    jump();
    while(1);
}

/* 计算 Flash 区域 CRC32（软件实现） */
static uint32_t CalculateCRC32_Region(uint32_t start_addr, uint32_t size)
{
	// F4 APP最大64KB
	if(size > 64*1024U)
	{
		return 0U; // 非法长度，直接返回无效CRC
	}
	if(start_addr < APP_A_ADDR || start_addr >= STATUS_PAGE0)
	{//避免非法地址越界读取
		return 0U;
	}
    uint32_t crc = 0xFFFFFFFFU;
    uint8_t *p = (uint8_t*)start_addr;
    for (uint32_t i = 0; i < size; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1U)
                crc = (crc >> 1) ^ 0xEDB88320U;
            else
                crc >>= 1U;
        }
    }
    return ~crc;
	//优化方向：可以使用F4硬件CRC外设加速
}

// 对外统一读取接口（自动挑选最新合法状态）
static void ReadStatus(AppStatus_t *out)
{
    AppStatus_t s0, s1;
    ReadStatusSingle(STATUS_PAGE0, &s0);
    ReadStatusSingle(STATUS_PAGE1, &s1);

    uint8_t valid0 = (s0.magic == 0x5A5A5A5AU);
    uint8_t valid1 = (s1.magic == 0x5A5A5A5AU);

    // 两份全部损坏，返回初始化默认配置
    if(!valid0 && !valid1)
    {
        AppStatus_t init = {
            .magic = 0x5A5A5A5AU,
            .active_app = 0U,
            .upgrade_state = 0U,
            .upgrade_crc = 0U,
            .upgrade_size = 0U,
            .reserved = {0U},
            .write_cnt = 0U
        };
        *out = init;
        return;
    }
    // 只有一页有效，选用有效页
    else if(valid0 && !valid1)
    {
        *out = s0;
    }
    else if(!valid0 && valid1)
    {
        *out = s1;
    }
    else
    {
        // 两份都合法，write_cnt更大 = 更新版本
        if(s0.write_cnt >= s1.write_cnt)
            *out = s0;
        else
            *out = s1;
    }
}

// 从指定Flash地址读取一份状态
static void ReadStatusSingle(uint32_t addr, AppStatus_t *out)
{
    memcpy(out, (void *)addr, sizeof(AppStatus_t));
}

// 写入单页状态（擦除+WORD编程）
static HAL_StatusTypeDef WriteStatusSingle(uint32_t addr, AppStatus_t *p)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = FLASH_SECTOR_10,
        .NbSectors = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t page_err;
    if(HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    uint32_t dest_addr = addr;
    uint32_t *src = (uint32_t *)p;
    uint32_t word_num = sizeof(AppStatus_t) / sizeof(uint32_t);

    for(uint32_t i = 0; i < word_num; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest_addr, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        dest_addr += 4U;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

/* 将状态结构体写入 Flash，自动轮流写入双备份页面 */
static void WriteStatus(AppStatus_t *p)
{
    if(p == NULL)
        return;

    AppStatus_t old_status;
    ReadStatus(&old_status);

    // 数据没有改动，跳过擦写，保护Flash寿命
    if(memcmp(&old_status, p, sizeof(AppStatus_t)) == 0)
    {
        return;
    }

    // 写入计数自增
    p->write_cnt = old_status.write_cnt + 1U;

    // 判断当前最新状态在哪一页，写入对面页面
    AppStatus_t s0, s1;
    ReadStatusSingle(STATUS_PAGE0, &s0);
    ReadStatusSingle(STATUS_PAGE1, &s1);
    uint8_t valid0 = (s0.magic == 0x5A5A5A5AU);
    uint8_t valid1 = (s1.magic == 0x5A5A5A5AU);

    uint32_t target_page;
    if(valid0 && valid1)
    {
        // 两份都有效，最新在哪一页，就写另一页
        if(s0.write_cnt >= s1.write_cnt)
            target_page = STATUS_PAGE1;
        else
            target_page = STATUS_PAGE0;
    }
    else if(valid0)
    {
        target_page = STATUS_PAGE1;
    }
    else if(valid1)
    {
        target_page = STATUS_PAGE0;
    }
    else
    {
        // 全部损坏，优先写入page0
        target_page = STATUS_PAGE0;
    }

    // 执行写入
    WriteStatusSingle(target_page, p);
}
static uint32_t GetFlashSector(uint32_t addr)
{
    if(addr < 0x08004000U) return FLASH_SECTOR_0;
    if(addr < 0x08008000U) return FLASH_SECTOR_1;
    if(addr < 0x0800C000U) return FLASH_SECTOR_2;
    if(addr < 0x08010000U) return FLASH_SECTOR_3;
    if(addr < 0x08020000U) return FLASH_SECTOR_4;
    if(addr < 0x08040000U) return FLASH_SECTOR_5;
    if(addr < 0x08080000U) return FLASH_SECTOR_6;
    if(addr < 0x08100000U) return FLASH_SECTOR_7;
    return FLASH_SECTOR_0;
}
// 写入单页状态（擦除+WORD编程）
static HAL_StatusTypeDef WriteStatusSingle(uint32_t addr, AppStatus_t *p)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = GetFlashSector(addr),   // 根据地址自动获取扇区
        .NbSectors = 1U,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    uint32_t page_err;
    if(HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    uint32_t dest_addr = addr;
    uint32_t *src = (uint32_t *)p;
    uint32_t word_num = sizeof(AppStatus_t) / sizeof(uint32_t);

    for(uint32_t i = 0; i < word_num; i++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest_addr, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        dest_addr += 4U;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}
static int SHA256_CalcRegion(uint32_t start_addr, uint32_t size, uint8_t *out_hash)
{
    if(out_hash == NULL)
        return -1;
    if(size > 64*1024U)
        return -1;
    if(start_addr < APP_A_ADDR || start_addr >= STATUS_PAGE0)
        return -1;

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0U); // 0 = SHA256

    uint8_t *p = (uint8_t *)start_addr;
    for(uint32_t i = 0; i < size; i++)
    {
        mbedtls_sha256_update(&ctx, p + i, 1U);
    }
    mbedtls_sha256_finish(&ctx, out_hash);
    mbedtls_sha256_free(&ctx);
    return 0;
}

static int RSA_VerifyFirmware(const uint8_t *hash, const uint8_t *sign)
{
    mbedtls_pk_context pk;
    int ret;

    mbedtls_pk_init(&pk);
    // 载入DER格式公钥
    ret = mbedtls_pk_parse_public_key(&pk, pub_der, pub_der_len);
    if(ret != 0)
    {
        mbedtls_pk_free(&pk);
        return -1;
    }
    // PKCS1-v1_5 SHA256 验签
    ret = mbedtls_pk_verify(
        &pk,
        MBEDTLS_MD_SHA256,
        hash, 32U,
        sign, 256U
    );
    mbedtls_pk_free(&pk);
    // ret == 0 → 签名合法
    return ret;
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