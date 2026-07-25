#include "lowpower.h"

extern void SystemClock_Config(void); // 在 main.c 中实现

void LowPower_::Init(Mode mode)
{
    current_mode = mode;
    EnableTickless();
}

void LowPower_::EnableTickless(void)
{
#if (configUSE_TICKLESS_IDLE == 1)
    __HAL_RCC_LSI_ENABLE();
    while(!__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY));
    NVIC_SetPriority(SysTick_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
#endif
}

void LowPower_::RegisterWakeupPin(uint16_t pin)
{
    ConfigureWakeupPin(pin);
}

void LowPower_::ConfigureWakeupPin(uint16_t pin)//WKUP引脚
{
    __HAL_RCC_PWR_CLK_ENABLE();
    if (pin == GPIO_PIN_0)
    {
        HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    }
}

void LowPower_::Enter(void)
{
    switch (current_mode)
    {
        case SLEEP_MODE:
            EnterSleep();
            break;
        case STOP_MODE:
            EnterStop();
            break;
        case STANDBY_MODE:
            EnterStandby();
            break;
        default:
            break;
    }
}

void LowPower_::EnterSleep(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_SuspendTick();
    __WFI();
	//直到有中断到来，就执行唤醒系统滴答
    HAL_ResumeTick();
}

void LowPower_::EnterStop(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    // F1 不支持 deepsleep 下的 ultra low power 和 fast wakeup，直接进入 STOP 模式
    // 参数：电压调节器低功耗模式，进入 WFI
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	//来了中断，再进入睡眠模式
   this->Init(LowPower_::SLEEP_MODE);  // 改为 SLEEP 模式
}

void LowPower_::EnterStandby(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_PWR_EnterSTANDBYMode();
}