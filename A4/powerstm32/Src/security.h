#ifndef __SECURITY_H
#define __SECURITY_H

#include "main.h"
#include <stdint.h>

class Security_
{
public:
    // 初始化安全模块：读取UID、检查RDP等级、验证应用程序完整性
    void Init(void);
    
    // 检查当前设备是否已授权（基于UID和密钥）
    bool IsAuthorized(void);
    
    // 使能Flash读保护（Level 1），重启后生效
    void EnableReadProtection(void);
    
    // 禁用Flash读保护（会擦除全部Flash）
    void DisableReadProtection(void);
    
    // 获取当前RDP等级
    uint8_t GetRDPLevel(void);
    
    // 运行时对关键代码段进行解密（需要配合链接脚本使用）
    void DecryptCodeSegment(uint32_t addr, uint32_t size, uint32_t key);
    
    // 系统完整性自检（CRC校验）
    bool SelfCheck(void);

private:
    uint32_t uid[3];            // 96位唯一ID
    uint16_t crc_table[256];    // CRC16表
    uint32_t encryption_key;    // 加密密钥（可从UID派生）
    
    void ReadUID(void);
    void GenerateEncryptionKey(void);
    void InitCRC16Table(void);
    uint16_t CalculateCRC16(uint8_t *buf, uint32_t len);
    bool CheckFlashIntegrity(void);
};

#endif