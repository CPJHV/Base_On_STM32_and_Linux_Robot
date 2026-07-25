#ifndef __USER_CONTROLLER_H
#define __USER_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// -------------------- C 接口函数（供 freertos.c / main.c 调用）--------------------
void User_Init_All(void);
void User_Start_RTOS_Tasks(void);
void User_Display_Update(void);//显示屏幕任务
void User_ADC_Control_Task(void);//adc任务
void User_WiFi_Task(void);//wifi任务
void User_Enter_LowPower(void);

#ifdef __cplusplus
}
#endif

// -------------------- C++ 部分（仅 C++ 编译器可见）--------------------
#ifdef __cplusplus

#include "gpio.h"
#include "adc.h"
#include "oled.h"
#include "esp8266.h"
#include "battery_monitor.h"
#include "watchdog.h"
#include "lowpower.h"
#include "security.h"
#include "wifi_updater.h"

class User_ {
public:
    Gpio_          gpio;
    Adc_           adc;
    Oled_          oled;
    Esp8266_       wifi;
    BatteryMonitor_ battery;
    Iwdg_          watchdog;
    LowPower_      lowpower;
    Security_      security;
    WifiUpdater_   updater;

    void InitAll(void);
    void StartRTOS(void);
    void ADControlTask(void);
    void DisplayUpdate(void);
    void WiFiTask(void);
    void EnterLowPower(void);
};

extern User_ user;

#endif /* __cplusplus */

#endif /* __USER_CONTROLLER_H */