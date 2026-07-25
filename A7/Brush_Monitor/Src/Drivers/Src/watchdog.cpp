#include "watchdog.h"

// IWDG 寄存器基地址 (STM32F103)
#define IWDG_BASE_ADDR       0x40003000UL
#define IWDG_KR              (*(volatile uint32_t*)(IWDG_BASE_ADDR + 0x00))//键寄存器(IWDG_KR)
#define IWDG_PR              (*(volatile uint32_t*)(IWDG_BASE_ADDR + 0x04))//预分频寄存器(IWDG_PR)
#define IWDG_RLR             (*(volatile uint32_t*)(IWDG_BASE_ADDR + 0x08))//重装载寄存器(IWDG_RLR)
#define IWDG_SR              (*(volatile uint32_t*)(IWDG_BASE_ADDR + 0x0C))//状态寄存器(IWDG_SR)

// 密钥寄存器值
#define IWDG_KR_ENABLE       0xCCCC   // 启动看门狗
#define IWDG_KR_RELOAD       0xAAAA   // 重载计数器
#define IWDG_KR_UNLOCK       0x5555   // 允许修改 PR 和 RLR

// 预分频器值 (对应 PR[2:0])
#define IWDG_PR_DIV4         0x00     // /4
#define IWDG_PR_DIV8         0x01     // /8
#define IWDG_PR_DIV16        0x02     // /16
#define IWDG_PR_DIV32        0x03     // /32
#define IWDG_PR_DIV64        0x04     // /64
#define IWDG_PR_DIV128       0x05     // /128
#define IWDG_PR_DIV256       0x06     // /256

void Watchdog_::WriteRegister(uint32_t reg, uint32_t value)
{
    *(volatile uint32_t*)reg = value;
}

void Watchdog_::ReloadCounter(void)
{
    IWDG_KR = IWDG_KR_RELOAD;
}

void Watchdog_::Init(uint32_t timeout_ms)
{
    // 计算分频系数和重载值
    // LSI 频率约 40kHz，实际 32~40kHz，取 40kHz 计算最安全
    uint32_t lsi_freq = 40000;   // Hz
    uint32_t prescaler_reg = IWDG_PR_DIV64;  // 默认 /64
    uint32_t prescaler_value = 64;
    uint32_t reload = 0;
    
    // 根据超时时间选择合适的预分频器，使重载值不超过 4095
    if (timeout_ms <= 800) {
        prescaler_reg = IWDG_PR_DIV4;
        prescaler_value = 4;
    } else if (timeout_ms <= 1600) {
        prescaler_reg = IWDG_PR_DIV8;
        prescaler_value = 8;
    } else if (timeout_ms <= 3200) {
        prescaler_reg = IWDG_PR_DIV16;
        prescaler_value = 16;
    } else if (timeout_ms <= 6400) {
        prescaler_reg = IWDG_PR_DIV32;
        prescaler_value = 32;
    } else if (timeout_ms <= 12800) {
        prescaler_reg = IWDG_PR_DIV64;
        prescaler_value = 64;
    } else if (timeout_ms <= 25600) {
        prescaler_reg = IWDG_PR_DIV128;
        prescaler_value = 128;
    } else {
        prescaler_reg = IWDG_PR_DIV256;
        prescaler_value = 256;
    }
    
    // 计算重载值: 超时 = (reload + 1) * (prescaler / LSI_freq) 秒
    // reload = (timeout_ms * LSI_freq / (prescaler * 1000)) - 1
    reload = (timeout_ms * lsi_freq) / (prescaler_value * 1000);
    if (reload > 0xFFF) reload = 0xFFF;
    if (reload < 1) reload = 1;
    reload--;   // 因为重载值从 0 开始
    
    // 解锁 PR 和 RLR 寄存器
    IWDG_KR = IWDG_KR_UNLOCK;
    
    // 等待 PVU 和 RVU 清零 (可选的等待，简单延时也可以)
    while (IWDG_SR & 0x01);   // PVU (bit0)看门狗计数器重装载值更新
    IWDG_PR = prescaler_reg;
    while (IWDG_SR & 0x02);   // RVU (bit1)看门狗预分频值更新
    IWDG_RLR = reload;
    
    // 启动看门狗
    IWDG_KR = IWDG_KR_ENABLE;
    
    // 等待寄存器更新完成
    while (IWDG_SR & 0x03);
}

void Watchdog_::Feed(void)
{
    IWDG_KR = IWDG_KR_RELOAD;
}

bool Watchdog_::IsResetByWatchdog(void)
{
    // 检查 RCC 状态寄存器中的 IWDG 复位标志
    if (RCC->CSR & RCC_CSR_IWDGRSTF)
    {
        // 清除标志
        RCC->CSR |= RCC_CSR_RMVF;
        return true;
    }
    return false;
}