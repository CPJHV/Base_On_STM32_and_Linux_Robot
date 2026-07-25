#include "security.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"

// UID 基地址（STM32F103）
#define UID_BASE_ADDR          0x1FFFF7E8
#define FLASH_RDP_ADDR         0x1FFFF800   // 选项字节中RDP位置

// RDP 值定义
#define RDP_LEVEL_0            0xAA         // 无保护
#define RDP_LEVEL_1            0xBB         // 读保护
#define RDP_LEVEL_2            0xCC         // 无调试接口

// 自定义完整性校验区（可指定Flash地址范围）
#define APP_START_ADDR         0x08000000
#define APP_END_ADDR           0x0800FFFF   // 64KB 应用区
#define CRC_STORAGE_ADDR       0x0800FFFC   // 最后4字节存储CRC

void Security_::Init(void)
{
    ReadUID();
    GenerateEncryptionKey();
    InitCRC16Table();
    
    // 检查RDP等级并决定是否执行完整性校验
    if (GetRDPLevel() >= RDP_LEVEL_1)
    {
        SelfCheck();   // 如果校验失败，可触发异常处理
    }
}

void Security_::ReadUID(void)
{	//每个stm芯片都有自己的唯一的UID
    uid[0] = *(__IO uint32_t *)(UID_BASE_ADDR);
    uid[1] = *(__IO uint32_t *)(UID_BASE_ADDR + 4);
    uid[2] = *(__IO uint32_t *)(UID_BASE_ADDR + 8);
}

void Security_::GenerateEncryptionKey(void)
{
    // 简单地将UID异或混合作为密钥（实际可增加更复杂的算法）
    encryption_key = uid[0] ^ uid[1] ^ uid[2];
    encryption_key ^= 0xDEADBEEF;
}

bool Security_::IsAuthorized(void)
{
    // 示例：检查UID是否在预先设置的白名单中
    // 实际可将允许的UID哈希值存储在Flash末尾
    uint32_t expected_uid[] = {0x12345678, 0x9ABCDEF0, 0x11223344}; // 示例
    return (uid[0] == expected_uid[0] && uid[1] == expected_uid[1] && uid[2] == expected_uid[2]);
}

void Security_::EnableReadProtection(void)
{/*
	flash中有一组特殊安全配置区
	里面是
	读保护等级（RDP）
看门狗配置
引脚复位配置
等安全相关参数
	*/
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();//解除flash的选项字节配置
    
    FLASH_OBProgramInitTypeDef ob = {0};
    ob.OptionType = OPTIONBYTE_RDP;//读保护
    ob.RDPLevel = OB_RDP_LEVEL_1;//设置为Level 1
		/*
		Level 0 
		无保护
谁插个 ST-Link 都能读走你全部程序
产品绝对不能用
		RDP Level 1（0xBB）【最常用】
		读保护开启
		调试器不能读 Flash
		产品正常运行
		可退回 Level0（会自动清空 Flash）
		RDP Level 2（0xCC）【锁死】
		完全锁死
		永远不能调试
		永远不能降级
		一旦开启，废了！
		*/
    HAL_FLASHEx_OBProgram(&ob);//把配置写入硬件中
    
    HAL_FLASH_OB_Launch();//配置生效
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

void Security_::DisableReadProtection(void)
{
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    
    FLASH_OBProgramInitTypeDef ob = {0};
    ob.OptionType = OPTIONBYTE_RDP;
    ob.RDPLevel = OB_RDP_LEVEL_0;
    HAL_FLASHEx_OBProgram(&ob);
    
    HAL_FLASH_OB_Launch();
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
}

uint8_t Security_::GetRDPLevel(void)
{
    FLASH_OBProgramInitTypeDef ob;
    HAL_FLASHEx_OBGetConfig(&ob);
    return ob.RDPLevel;
}

void Security_::DecryptCodeSegment(uint32_t addr, uint32_t size, uint32_t key)
{//要解密的代码的地址起始地址，解密长度，密钥
    // 运行时解密一段代码（例如压缩或加密过的函数）
    // 注意：需要确保这段RAM或Flash可写
    uint8_t *ptr = (uint8_t *)addr;
    for (uint32_t i = 0; i < size; i++)
    {
			//一个个字节解密
        ptr[i] ^= (key >> ((i % 4) * 8)) & 0xFF;
    }
    // 刷新指令缓存（如果使用）
    __DSB();
    __ISB();//告诉CPU，哪些加密的片段已经破解，重新读取加密的片段代码，防止乱飞
}

bool Security_::SelfCheck(void)
{
    return CheckFlashIntegrity();
}

void Security_::InitCRC16Table(void)
{
    uint16_t poly = 0x8005;
    for (uint16_t i = 0; i < 256; i++)
    {
        uint16_t crc = i << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
        crc_table[i] = crc;
    }
}

uint16_t Security_::CalculateCRC16(uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++)
    {
        crc = (crc << 8) ^ crc_table[((crc >> 8) ^ buf[i]) & 0xFF];
    }
    return crc;
}

bool Security_::CheckFlashIntegrity(void)
{
    // 读取预先存储的CRC值
    uint16_t stored_crc = *(__IO uint16_t *)CRC_STORAGE_ADDR;
    // 计算应用区CRC（不包括存储CRC本身的位置）
    uint32_t len = APP_END_ADDR - APP_START_ADDR + 1 - 4;//计算整个APP程序
    uint16_t calc_crc = CalculateCRC16((uint8_t *)APP_START_ADDR, len);//进行与之前的进行验算
    return (calc_crc == stored_crc);//与预先的进行比对
}