#include "user_controller.h"
#include <stdio.h>      // 如果需要 sprintf
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

User_ user;

void User_::InitAll(void)
{
    gpio.Init();
    adc.Init();
    oled.Init();
    wifi.Init(115200);
    battery.Init(2000.0f, 3.7f);
    watchdog.Init(3000);
    lowpower.Init();
    security.Init();
    updater.Init(&wifi);
}

void User_::ADControlTask(void)
{
    adc.Filter();//滤波
		//获取数据
    float v = adc.GetVoltage();
    float i = adc.GetCurrent();
    float t = adc.GetTemperature();
    //更新电池
		battery.Update(v, i);
		
    // 电源开关逻辑
    if (v < 6.0f) gpio.PowerOff();
    else          gpio.PowerOn();

    watchdog.Feed();//喂狗
}

void User_::DisplayUpdate(void)
{
    static uint32_t last_up = 0;
    if (HAL_GetTick() - last_up > 200) {//200次滴答刷新屏幕
				//更新时间
			last_up = HAL_GetTick();
			
			//获取数据
        float v = adc.GetVoltage();
        float i = adc.GetCurrent();
        float t = adc.GetTemperature();
        int bat = (int)battery.GetPercent();
        //显示
				char buf[32];
        sprintf(buf, "T:%.1fC", t);
        oled.ShowString(1, 1, buf);
        sprintf(buf, "V:%.2fV", v);
        oled.ShowString(2, 1, buf);
        sprintf(buf, "I:%.3fA", i);
        oled.ShowString(3, 1, buf);
        sprintf(buf, "Bat:%d%%", bat);
        oled.ShowString(4, 1, buf);
    }
}

void User_::WiFiTask(void)
{
    static uint32_t last_send = 0;
    if (HAL_GetTick() - last_send > 5000) {
        last_send = HAL_GetTick();
        float v = adc.GetVoltage();
        float i = adc.GetCurrent();
        float t = adc.GetTemperature();
        int bat = (int)battery.GetPercent();
        updater.UploadData(t, v, i, bat);   // UploadData 现在存在
    }

    static uint32_t last_check = 0;
    if (HAL_GetTick() - last_check > 60000) {
        last_check = HAL_GetTick();
        if (updater.CheckForUpdate()) {     // CheckForUpdate 存在
            // 有新版本，启动下载升级
            updater.DownloadAndUpdate();
        }
    }
}

void User_::EnterLowPower(void)
{
    this->lowpower.EnterSleep();
}

// C 接口实现
extern "C" {

void User_Init_All(void) { user.InitAll(); }
void User_Start_RTOS_Tasks(void) { /* 可选：创建队列等 */ }
void User_Display_Update(void) { user.DisplayUpdate(); }
void User_ADC_Control_Task(void) { user.ADControlTask(); }
void User_Enter_LowPower(void) { user.EnterLowPower(); }
void User_WiFi_Task(void) { user.WiFiTask(); }

} // extern "C"