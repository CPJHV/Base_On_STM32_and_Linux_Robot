#include "main.h"

// --------------- C++ 代码开始 ---------------
class MyControl
{
public:
    // 点灯
    void ledOn()  { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); }
    void ledOff() { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); }
    
    // 翻转LED
    void toggle() { HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); }
};

// 创建一个 C++ 对象
MyControl g_device;

// 供 FreeRTOS (C语言) 调用的接口
// 必须加 extern "C" !!!
extern "C" void cpp_run_task(void)
{
    // 在这里写你的 C++ 逻辑
    g_device.toggle();
}