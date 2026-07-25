#ifndef __BATTERY_MONITOR_H
#define __BATTERY_MONITOR_H

#include "main.h"
#include <stdint.h>

class BatteryMonitor_
{
public:
    // 初始化：总容量(mAh)，标称电压(V)（用于功率估算和显示，不影响积分）
    void Init(float total_capacity_mAh, float nominal_voltage = 3.7f);
    
    // 周期调用：传入当前电池电压(V)和电池输出总电流(A)（正值放电，负值充电）
    // dt_ms 为两次调用之间的时间间隔(ms)，若不传则使用内部系统时间差
    void Update(float voltage, float current, uint32_t dt_ms = 0);
    
    // 获取剩余电量百分比 (0.0 ~ 100.0)
    float GetPercent(void) const;
    
    // 获取剩余容量 (mAh)
    float GetRemaining_mAh(void) const;
    
    // 获取已消耗容量 (mAh)
    float GetConsumed_mAh(void) const;
    
    // 获取当前电流积分累积的电荷量 (mAh)（本次Update中的增量，可用于调试）
    float GetIncrement_mAh(void) const;
    
    // 获取当前电池电压 (最后更新值)
    float GetVoltage(void) const;
    
    // 获取当前电池电流 (最后更新值)
    float GetCurrent(void) const;
    
    // 重置电量状态（更换电池后调用）
    void Reset(void);
    
    // 手动设置剩余容量 (mAh) – 用于校准或外部电压法修正
    void SetRemaining_mAh(float mAh);

private:
    float total_capacity_mAh;    // 总容量 (mAh)
    float remaining_mAh;         // 剩余容量 (mAh)
    float consumed_mAh;          // 已消耗容量 (mAh) = total - remaining
    float voltage_last;          // 最近电压 (V)
    float current_last;          // 最近电流 (A)
    float delta_mAh;             // 本次积分增加的消耗量 (mAh)
    
    uint32_t last_tick_ms;       // 上一次调用时的时间点 (ms)
    bool first_update;           // 首次调用标志
    
    // 低通滤波器：平滑电流值
    float filtered_current;      // 滤波后的电流 (A)
    float alpha;                 // 一阶低通系数 (0~1)
    
    // 累积误差校正相关（简单防错）
    void ApplyCorrection(void);
    float LimitRange(float value, float min_val, float max_val) const;
    
    // 积分计算：电流(A) * 时间(s) -> 电荷量(mAh)
    float CurrentTo_mAh(float current_amp, float seconds) const;
};

#endif