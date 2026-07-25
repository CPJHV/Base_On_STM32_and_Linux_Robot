#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "main.h"   // 提供 uint32_t 等类型定义

class Watchdog_
{
public:
    void Init(uint32_t timeout_ms = 2000);   // 初始化并启动看门狗
    void Feed(void);                         // 喂狗（重载计数器）
    bool IsResetByWatchdog(void);            // 检查上次复位是否由看门狗引起

private:
    void WriteRegister(uint32_t reg, uint32_t value);
    void ReloadCounter(void);
};

#endif