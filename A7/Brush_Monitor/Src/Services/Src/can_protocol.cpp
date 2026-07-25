#include "can_protocol.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"
#include <string.h>

// STM32F405 扇区大小（1MB Flash，每扇区128KB？不对，F4是混合扇区）
// 实际F405: 扇区0-3=16KB, 扇区4=64KB, 扇区5-11=128KB
// 简化处理：用扇区号

#define FLASH_SECTOR_0      0   // 0x08000000 - 0x08003FFF (16KB)
#define FLASH_SECTOR_1      1   // 0x08004000 - 0x08007FFF (16KB)
#define FLASH_SECTOR_2      2   // 0x08008000 - 0x0800BFFF (16KB)
#define FLASH_SECTOR_3      3   // 0x0800C000 - 0x0800FFFF (16KB)
#define FLASH_SECTOR_4      4   // 0x08010000 - 0x0801FFFF (64KB)
#define FLASH_SECTOR_5      5   // 0x08020000 - 0x0803FFFF (128KB)
// ... 一直到扇区11

typedef struct {
    uint32_t magic;         // 固定 0x5A5A5A5A
    uint32_t active_app;    // 0 = APP_A, 1 = APP_B
    uint32_t upgrade_state; // 0:空闲, 1:接收中, 2:完成待切换
    uint32_t upgrade_crc;   // 接收固件的CRC32
    uint32_t upgrade_size;  // 固件大小
    uint32_t reserved[2];
} AppStatus_t;

static AppStatus_t* pStatus = (AppStatus_t*)STATUS_FLAG_ADDR;

// 根据地址获取扇区号
static uint32_t GetSectorNumber(uint32_t addr) {
    if (addr < 0x08010000) {
        // 前256KB，扇区0-3 (16KB each)
        return (addr - 0x08000000) / 0x4000;  // 0x4000 = 16KB
    } else if (addr < 0x08020000) {
        return 4;  // 64KB sector
    } else {
        // 128KB sectors
        return 5 + (addr - 0x08020000) / 0x20000;
    }
}

void RemoteUpdate_::Init(void)
{
    state = IDLE;
    if (pStatus->magic == 0x5A5A5A5A && pStatus->upgrade_state == 2) {
        NVIC_SystemReset();
    }
}

uint32_t RemoteUpdate_::GetInactiveAppAddr(void)
{
    return (GetActiveAppId() == 0) ? APP_B_ADDR : APP_A_ADDR;
}

uint8_t RemoteUpdate_::GetActiveAppId(void)
{
    if (pStatus->magic != 0x5A5A5A5A) return 0;
    return pStatus->active_app;
}

void RemoteUpdate_::SetActiveApp(uint8_t app_id)
{
    HAL_FLASH_Unlock();
    
    // 擦除状态扇区（扇区3，因为 STATUS_FLAG_ADDR = 0x0800F000 在扇区3）
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;  // F4用这个
    erase.Sector = FLASH_SECTOR_3;               // 扇区号
    erase.NbSectors = 1;                         // 数量
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7-3.6V
    
    uint32_t sector_err;
    if (HAL_FLASHEx_Erase(&erase, &sector_err) == HAL_OK) {
        // 写入新数据
        uint32_t flash_addr = STATUS_FLAG_ADDR;
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, 0x5A5A5A5A);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 4, app_id);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 8, 0);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 12, 0);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 16, 0);
    }
    
    HAL_FLASH_Lock();
}

void RemoteUpdate_::ProcessCanCommand(uint8_t* data, uint8_t len)
{
    if (len < 2) return;
    if (data[0] != UPDATE_PASSWORD) return;

    uint8_t cmd = data[1];
    switch (cmd) {
        case UPDATE_CMD_START: {
            if (len >= 10) {
                target_size = (data[2]<<24)|(data[3]<<16)|(data[4]<<8)|data[5];
                target_crc  = (data[6]<<24)|(data[7]<<16)|(data[8]<<8)|data[9];
                uint32_t inactive_addr = GetInactiveAppAddr();
                
                if (EraseApp(inactive_addr)) {
                    state = RECEIVING_DATA;
                    flash_write_addr = inactive_addr;
                    packet_index = 0;
                    calc_crc_accum = 0xFFFFFFFF;
                    
                    // 保存升级状态
                    HAL_FLASH_Unlock();
                    FLASH_EraseInitTypeDef erase;
                    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
                    erase.Sector = FLASH_SECTOR_3;
                    erase.NbSectors = 1;
                    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
                    uint32_t err;
                    HAL_FLASHEx_Erase(&erase, &err);
                    
                    uint32_t flash_addr = STATUS_FLAG_ADDR;
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, 0x5A5A5A5A);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 4, GetActiveAppId());
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 8, 1);  // upgrade_state = 1
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 12, target_crc);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 16, target_size);
                    HAL_FLASH_Lock();
                } else {
                    state = IDLE;
                }
            }
            break;
        }
        case UPDATE_CMD_DATA: {
            if (state == RECEIVING_DATA && len >= UPDATE_PACKET_SIZE + 2) {
                uint8_t* packet = &data[2];
                if (WriteFlash(flash_write_addr, packet, UPDATE_PACKET_SIZE)) {
                    flash_write_addr += UPDATE_PACKET_SIZE;
                    packet_index++;
                    calc_crc_accum = CalculateCRC32(packet, UPDATE_PACKET_SIZE);
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
                    HAL_FLASH_Unlock();
                    FLASH_EraseInitTypeDef erase;
                    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
                    erase.Sector = FLASH_SECTOR_3;
                    erase.NbSectors = 1;
                    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
                    uint32_t err;
                    HAL_FLASHEx_Erase(&erase, &err);
                    
                    uint32_t flash_addr = STATUS_FLAG_ADDR;
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, 0x5A5A5A5A);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 4, GetActiveAppId());
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 8, 2);  // 完成
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 12, target_crc);
                    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr + 16, target_size);
                    HAL_FLASH_Lock();
                    
                    ResetSystem();
                } else {
                    state = IDLE;
                }
            }
            break;
        }
        case UPDATE_CMD_JUMP: {
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
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    
    // 计算需要擦除的扇区范围
    uint32_t start_sector = GetSectorNumber(app_addr);
    uint32_t end_sector = GetSectorNumber(app_addr + 24*1024);  // 24KB
    erase.Sector = start_sector;
    erase.NbSectors = end_sector - start_sector + 1;
    
    uint32_t sector_err;
    if (HAL_FLASHEx_Erase(&erase, &sector_err) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    
    HAL_FLASH_Lock();
    return true;
}

bool RemoteUpdate_::WriteFlash(uint32_t addr, uint8_t* data, uint32_t len)
{
    HAL_FLASH_Unlock();
    
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = 0;
        for (int j = 0; j < 4 && (i+j) < len; j++) {
            word |= (data[i+j] << (8*j));
        }
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    
    HAL_FLASH_Lock();
    return true;
}

uint32_t RemoteUpdate_::CalculateCRC32(uint8_t* buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
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
    uint32_t sp = *(__IO uint32_t*)app_addr;
    uint32_t reset = *(__IO uint32_t*)(app_addr + 4);
    if ((sp < 0x20000000 || sp > 0x2001FFFF) || reset < 0x08000000) return false;
    return true;
}