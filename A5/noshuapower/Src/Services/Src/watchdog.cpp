#include "watchdog.h"
#include <cmath>

/* ===================== IWDG 独立看门狗 ===================== */
bool Iwdg_::Init(uint32_t timeout_ms)
{
    __HAL_RCC_LSI_ENABLE();
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY));

    hiwdg.Instance = IWDG;
    uint32_t reload = CalculateReload(timeout_ms);
    if (reload == 0 || reload > 0xFFFU)
        return false;

    // HAL_IWDG_Init内部会开启写保护、设置PR/RLR
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = reload;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        return false;
    return true;
}

void Iwdg_::Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

bool Iwdg_::IsResetSource(void)
{
    bool flag = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return flag;
}

uint32_t Iwdg_::CalculateReload(uint32_t timeout_ms)
{
    // LSI典型40kHz，预分频64
    const uint32_t lsi_freq = 40000U;
    const uint32_t prescaler = 64U;
    float freq = (float)lsi_freq / prescaler;
    float sec = timeout_ms / 1000.0f;
    // Tout = (reload + 1) / freq
    float reload_f = sec * freq - 1.0f;
    uint32_t reload = (uint32_t)roundf(reload_f);
    if (reload > 0xFFFU) reload = 0xFFFU;
    if (reload < 1U) reload = 1U;
    return reload;
}

/* ===================== WWDG 窗口看门狗（已修复时钟公式） ===================== */
Wwdg_* Wwdg_::instance = nullptr;

bool Wwdg_::Init(uint32_t timeout_ms, bool early_wakeup_irq)
{
    // 必须开启APB1时钟
    __HAL_RCC_WWDG_CLK_ENABLE();
    instance = this;
    early_wakeup_callback = nullptr;

    // 系统配置：PCLK1 = 36MHz（72M主频标准配置）
    const uint32_t pclk1 = 36000000U;
    // WWDG公式：f_cnt = PCLK1 / (4096 * prescale_factor)
    // WWDG_PRESCALER_8 → WDGTB=3 → factor=8
    const uint32_t wwdg_prescaler = WWDG_PRESCALER_8;
    const uint32_t factor = 8U;
    float f_cnt = (float)pclk1 / (4096.0f * factor);
    float tick_ms = 1000.0f / f_cnt; // 单个tick耗时 ms

    // WWDG最大有效区间：0x7F ~ 0x40 → 最多63 tick
    uint32_t max_possible_ms = (uint32_t)(63.0f * tick_ms);
    if (timeout_ms > max_possible_ms)
        timeout_ms = max_possible_ms;

    // 计算总tick数
    uint32_t total_tick = (uint32_t)roundf((float)timeout_ms / tick_ms);
    if (total_tick > 63U) total_tick = 63U;
    if (total_tick < 2U) total_tick = 2U;

    uint8_t counter_base = 0x40U + total_tick; // 初始计数器
    uint8_t window_val = counter_base - (total_tick / 2U); // 窗口取一半区间
    if (window_val <= 0x40U) window_val = 0x41U;

    hwwdg.Instance = WWDG;
    hwwdg.Init.Prescaler = wwdg_prescaler;
    hwwdg.Init.Window = window_val;
    hwwdg.Init.Counter = counter_base;
    hwwdg.Init.EWIMode = early_wakeup_irq ? WWDG_EWI_ENABLE : WWDG_EWI_DISABLE;

    if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
        return false;
    return true;
}

void Wwdg_::Feed(void)
{
    HAL_WWDG_Refresh(&hwwdg);
}

void Wwdg_::RegisterEarlyWakeupCallback(WwdgEarlyWakeupCallback cb)
{
    early_wakeup_callback = cb;
}

void Wwdg_::IRQHandlerStatic(WWDG_HandleTypeDef* hwwdg)
{
    (void)hwwdg;
    if (instance && instance->early_wakeup_callback)
        instance->early_wakeup_callback();
}