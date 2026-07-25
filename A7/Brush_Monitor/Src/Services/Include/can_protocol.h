#ifndef __REMOTE_UPDATE_H
#define __REMOTE_UPDATE_H

#include "main.h"
#include <stdint.h>

#define UPDATE_PASSWORD    0x4A //升级密码

// 双分区地址定义
#define BOOTLOADER_ADDR     0x08000000
#define APP_A_ADDR          0x08003000
#define APP_B_ADDR          0x08009000
#define STATUS_FLAG_ADDR    0x0800F000

#define UPDATE_PACKET_SIZE  32//每次升级包最大32字节

// 命令码
#define UPDATE_CMD_START    0x10 //开始升级
#define UPDATE_CMD_DATA     0x11//穿数据
#define UPDATE_CMD_END      0x12	//结束传输
#define UPDATE_CMD_JUMP     0x13   // 立即重启切换分区

class RemoteUpdate_
{
public:
    void Init(void);
    void ProcessCanCommand(uint8_t* data, uint8_t len);
    void JumpToApp(void);

private:
    enum UpdateState { IDLE, RECEIVING_DATA };
    UpdateState state;
    uint32_t target_size;      // 固件总大小
    uint32_t target_crc;       // 固件期望CRC
    uint32_t flash_write_addr; // 当前写入地址（指向备用分区）
    uint32_t packet_index;

    uint32_t calc_crc_accum;   // 实时累加CRC

    bool IsBootloaderRequest(void);
    uint32_t GetActiveAppAddr(void);
    uint32_t GetInactiveAppAddr(void);
    void SetActiveApp(uint8_t app_id);
    uint8_t GetActiveAppId(void);
    bool IsAppValid(uint32_t app_addr);
    bool EraseApp(uint32_t app_addr);
    bool WriteFlash(uint32_t addr, uint8_t* data, uint32_t len);
    uint32_t CalculateCRC32(uint8_t* buf, uint32_t len);
    void ResetSystem(void);
};

#endif