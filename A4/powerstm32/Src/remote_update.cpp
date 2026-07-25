#include "remote_update.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
#include <string.h>
/*
修改点：
1、要加上应答
2、对写入的区域加上锁或者临界区
3、升级过程要及时喂狗
4、会话超时处理
5、升级，开机自检查
*/
typedef struct {
    uint32_t magic;         // 固定 0x5A5A5A5A
    uint32_t active_app;    // 0 = APP_A, 1 = APP_B
    uint32_t upgrade_state; // 0:空闲, 1:接收中, 2:完成待切换
    uint32_t upgrade_crc;   // 接收固件的CRC32
    uint32_t upgrade_size;  // 固件大小
    uint32_t reserved[2];
} AppStatus_t;
static AppStatus_t* pStatus = (AppStatus_t*)STATUS_FLAG_ADDR;

void RemoteUpdate_::Init(void)
{
    state = IDLE;
    // 检查是否有待切换的升级（Bootloader 未处理的情况）
    if (pStatus->upgrade_state == 2) {
        // 手动触发复位，让 Bootloader 完成切换
        NVIC_SystemReset();
    }
}

uint32_t RemoteUpdate_::GetInactiveAppAddr(void)//获取APP分区地址
{
    return (GetActiveAppId() == 0) ? APP_B_ADDR : APP_A_ADDR;
}

uint8_t RemoteUpdate_::GetActiveAppId(void)
{
    if (pStatus->magic != 0x5A5A5A5A) return 0; // 默认使用 APP_A
    return pStatus->active_app;
}

void RemoteUpdate_::SetActiveApp(uint8_t app_id)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = STATUS_FLAG_ADDR;
    erase.NbPages = 1;
    uint32_t page_err;
    HAL_FLASHEx_Erase(&erase, &page_err);
    pStatus->magic = 0x5A5A5A5A;
    pStatus->active_app = app_id;
    pStatus->upgrade_state = 0;
    HAL_FLASH_Lock();
}
/*
data[0]密码
data[1]cmd命令
*/
//升级任务建议不要放在中断里处理，因为升级速度慢，你这样搞会导致其他任务长时间被停止的，应该是中断发送升级包片段到缓冲区
//唤醒这里的独立任务去完成固件升级
void RemoteUpdate_::ProcessCanCommand(uint8_t* data, uint8_t len)
{
    if (len < 2) return;
    if (data[0] != UPDATE_PASSWORD) return;//密码验证

    uint8_t cmd = data[1];
    switch (cmd) {
        case UPDATE_CMD_START: {
            if (len >= 10) {
                target_size = (data[2]<<24)|(data[3]<<16)|(data[4]<<8)|data[5];//拿到固件包大小
                target_crc  = (data[6]<<24)|(data[7]<<16)|(data[8]<<8)|data[9];//crc值
                uint32_t inactive_addr = GetInactiveAppAddr();//获取备份分区地址
							
                if (EraseApp(inactive_addr)) {//擦除备份区
									
                    state = RECEIVING_DATA;//切换状态
                    flash_write_addr = inactive_addr;//写入地址
                    packet_index = 0;
                    calc_crc_accum = 0xFFFFFFFF; // CRC32 初始值
                    // 更新状态标志：接收中
									
										//更新存储的状态
                    HAL_FLASH_Unlock();
                    FLASH_EraseInitTypeDef erase;
                    erase.TypeErase = FLASH_TYPEERASE_PAGES;
                    erase.PageAddress = STATUS_FLAG_ADDR;
                    erase.NbPages = 1;
									
                    uint32_t err;
                    HAL_FLASHEx_Erase(&erase, &err);//擦除状态分区
                    pStatus->magic = 0x5A5A5A5A;
                    pStatus->active_app = GetActiveAppId(); // 保持不变
                    pStatus->upgrade_state = 1;
                    pStatus->upgrade_crc = target_crc;
                    pStatus->upgrade_size = target_size;
                    HAL_FLASH_Lock();
                } else {
                    state = IDLE;
                }
            }
            break;
        }
        case UPDATE_CMD_DATA: {
					//传输数据
            if (state == RECEIVING_DATA && len >= UPDATE_PACKET_SIZE+2) {
							
                uint8_t* packet = &data[2];
                if (WriteFlash(flash_write_addr, packet, UPDATE_PACKET_SIZE)) {
									//写入数据
                    flash_write_addr += UPDATE_PACKET_SIZE;
                    packet_index++;
                    // 累加 CRC
                    calc_crc_accum = CalculateCRC32(calc_crc_accum,packet, UPDATE_PACKET_SIZE);
                } else {
                    state = IDLE;
                }
            }
            break;
        }
        case UPDATE_CMD_END: {
            if (state == RECEIVING_DATA && len >= 6) {
                uint32_t final_crc = (data[2]<<24)|(data[3]<<16)|(data[4]<<8)|data[5];
                if (final_crc == calc_crc_accum) {
                    // 升级数据接收完成，设置完成标志
                    HAL_FLASH_Unlock();
                    FLASH_EraseInitTypeDef erase;
                    erase.TypeErase = FLASH_TYPEERASE_PAGES;
                    erase.PageAddress = STATUS_FLAG_ADDR;
                    erase.NbPages = 1;
                    uint32_t err;
                    HAL_FLASHEx_Erase(&erase, &err);
                    pStatus->magic = 0x5A5A5A5A;
                    pStatus->active_app = GetActiveAppId(); // 不变
                    pStatus->upgrade_state = 2;   // 完成，等待 Bootloader 切换
                    pStatus->upgrade_crc = target_crc;
                    pStatus->upgrade_size = target_size;
                    HAL_FLASH_Lock();
                    // 复位进入 Bootloader
                    ResetSystem();
                } else {
                    state = IDLE;
                }
            }
            break;
        }
        case UPDATE_CMD_JUMP: {
            // 强制切换分区（用于测试）
            uint8_t new_app = (GetActiveAppId() == 0) ? 1 : 0;
            SetActiveApp(new_app);
            ResetSystem();
            break;
        }
    }
}

bool RemoteUpdate_::EraseApp(uint32_t app_addr)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = app_addr;
    erase.NbPages = 48; // 24KB / 512B = 48页
    uint32_t page_err;
    if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    HAL_FLASH_Lock();
    return true;
}

bool RemoteUpdate_::WriteFlash(uint32_t addr, uint8_t* data, uint32_t len)
{
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len; i += 2) {
        uint16_t halfword = (data[i+1] << 8) | data[i];
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, halfword) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    HAL_FLASH_Lock();
    return true;
}

uint32_t RemoteUpdate_::CalculateCRC32(uint32_t crc_in,uint8_t* buf, uint32_t len)
{
    uint32_t crc = crc_in;
    for(uint32_t i=0;i<len;i++)
    {
        crc ^= buf[i];
        for(int j=0;j<8;j++)
        {
            if(crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

void RemoteUpdate_::ResetSystem(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void RemoteUpdate_::JumpToApp(void)
{
    uint32_t app_addr = GetActiveAppAddr();
    if (!IsAppValid(app_addr)) return;
    typedef void (*pFunction)(void);
    uint32_t jump_addr = *(__IO uint32_t*)(app_addr + 4);
    pFunction jump = (pFunction)jump_addr;
    __disable_irq();
    __set_MSP(*(__IO uint32_t*)app_addr);
    jump();
    while(1);
}

uint32_t RemoteUpdate_::GetActiveAppAddr(void)
{
    uint8_t active = GetActiveAppId();
    return (active == 0) ? APP_A_ADDR : APP_B_ADDR;
}

bool RemoteUpdate_::IsAppValid(uint32_t app_addr)
{
    // 检查栈顶指针和中断向量
    uint32_t sp = *(__IO uint32_t*)app_addr;
    uint32_t reset = *(__IO uint32_t*)(app_addr + 4);
    if ((sp < 0x20000000 || sp > 0x20004FFF) || reset < 0x08000000) return false;
    return true;
}