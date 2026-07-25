#ifndef __USER_CONTROLLER_H
#define __USER_CONTROLLER_H
//这个宏只有c++编译器有定义，而且extern表示以c语言格式链接这些东西
#ifdef __cplusplus
extern "C" {
#endif
	
// 包含所有硬件驱动（C语言文件）
#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "can.h"
#include "OLED.h"
#include "lowpower.h"
#include "watchdog.h"
#include "remote_update.h"
#include <stdlib.h>
// FreeRTOS 头文件
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "security.h"
#include "battery_monitor.h"
#define CAN_PASSWORD    0xA5 //密码

// C 接口函数（供 FreeRTOS 任务调用）
void User_Display_Update(void);
void User_CAN_Send_Data(float temp,float volt,float curr);
void User_ADC_Control_Task(void);
void User_Init_All(void);          // 统一初始化所有硬件
void User_Start_RTOS_Tasks(void);  // 启动 RTOS 任务（队列创建 + 任务创建）
// 在 User_ADC_Control_Task 等声明的附近添加
void User_Enter_LowPower(void);
void Main_Init_EEOR(void);
#ifdef __cplusplus
}
#endif

// ========================== 前向声明 ==========================
class User_;
class show_;

// ========================== GPIO 类 ==========================
class gpio_
{
public:
    void Init();
    void drive_FAN_Switch(uint8_t fl);    // 风扇：1开 0关
    void drive_Buzzer_Switch(uint8_t fl); // 蜂鸣器：1开 0关
    uint8_t get_FAN_State();   // 获取风扇状态
    uint8_t get_Buzzer_State();// 获取蜂鸣器状态
	//建议加上这个，避免后续有人开发时候写了构造，你这里就避免有人写构造
		gpio_(const gpio_&) = delete;
		gpio_& operator=(const gpio_&) = delete;
};

// ========================== ADC 类 ==========================
class adc_
{
private:
    float tolerance_voltage;   // 容忍电压
    float tolerance_temper;    // 容忍温度
    float tolerance_electri;   // 容忍电流
public:
    void Init();
    void Filter();                // 滤波任务
    float Get_Temperature_();     // 读取温度
    float Get_Power_Voltage();    // 读取电压
    float get_electri();          // 读取电流
    uint16_t* get_dma();          // 获取DMA原始数据
    uint16_t get_max_temper();
    uint16_t get_min_temper();
    float get_max_voltage();
    float get_min_voltage();
    float get_max_electri();
    float get_min_electri();
    void set_torlerance_electri(uint8_t param);
    void set_torlerance_volt(uint8_t param);
    void set_torlerance_temper(uint8_t param);
    
    // 公有 getter，供外部 C 函数访问
    float get_tolerance_voltage() const { return tolerance_voltage; }
    float get_tolerance_temper() const { return tolerance_temper; }
    float get_tolerance_electri() const { return tolerance_electri; }
    
    friend class User_;
    friend class show_;
};

// ========================== CAN 类 ==========================
class can_
{
public:
    void Init();
    void SendMsg(uint32_t id, uint8_t* data, uint8_t len);
};

// ========================== OLED 显示类 ==========================
class show_
{
	private:
    // 弹窗相关
		static show_* instance_ptr;
    uint8_t can_msg_flag = 0;
    uint8_t message_show[2];
    osTimerId_t popup_timer_handle;    // 弹窗定时器句柄
public:
    void Init();
    void ShowPage(float temp, float volt, uint8_t fan_sta, uint8_t beep_sta);
		void Show_Main_EEOR();
		static void StaticPopupCallback(osTimerId_t timer);
    void PopupTimerCallback();
		void PopupStart(uint8_t ch0, uint8_t ch1);
};

// ========================== FreeRTOS 任务类 ==========================
class Task_
{
public:
    void Init(); // 创建任务（已废弃，改为 User_Start_RTOS_Tasks）
};

// ========================== 用户总控类（核心） ==========================
class User_
{
public:
    gpio_   gpio;
    adc_    adc;
    can_    can;
    show_   show;
    Task_   task;

		LowPower_    lowpower;     // 低功耗类
    Watchdog_    watchdog;     // 看门狗类
    RemoteUpdate_ remote_update; // OTA升级类
		Security_ security;					//安全类
		BatteryMonitor_ battery;		//电量类

    void InitAll(void);          // 一键初始化所有硬件
    void StartRTOS(void);        // 启动 RTOS 相关组件（队列、任务）
};

// 全局 user 对象声明（定义在 cpp 中）
extern User_ user;

#endif