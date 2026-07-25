#ifndef __WATCHDOG_H
#define __WATCHDOG_H
#include "main.h"
#ifdef __cplusplus

/* 独立看门狗 IWDG（LSI时钟，不受主时钟崩溃影响，推荐做整机最后防线） */
class Iwdg_
{
public:
    bool Init(uint32_t timeout_ms);
    void Feed(void);
    bool IsResetSource(void);

private:
    IWDG_HandleTypeDef hiwdg;
    uint32_t CalculateReload(uint32_t timeout_ms);
};

/* 窗口看门狗 WWDG（PCLK1时钟；用于检测任务执行时序异常，不能替代IWDG） */
typedef void (*WwdgEarlyWakeupCallback)(void);
class Wwdg_
{
public:
    static Wwdg_* instance;
    bool Init(uint32_t timeout_ms, bool early_wakeup_irq);
    void Feed(void);
    void RegisterEarlyWakeupCallback(WwdgEarlyWakeupCallback cb);
    // 静态回调入口，放在 stm32f1xx_it.c 的 HAL_WWDG_EarlyWakeupCallback 调用
    static void IRQHandlerStatic(WWDG_HandleTypeDef* hwwdg);

private:
    WWDG_HandleTypeDef hwwdg;
    WwdgEarlyWakeupCallback early_wakeup_callback;
};

#endif
#endif