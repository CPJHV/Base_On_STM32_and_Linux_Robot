#include "battery_monitor.h"
#include "FreeRTOS.h"
#include "task.h"

// 辅助函数：限制数值范围
static inline float clamp(float val, float minv, float maxv)
{
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}

float BatteryMonitor_::CurrentTo_mAh(float current_amp, float seconds) const
{
    // 1 A * 1 小时 = 1000 mAh
    // 所以 mAh = 电流(A) * 时间(秒) * (1000/3600)
    return current_amp * seconds * (1000.0f / 3600.0f);
}

void BatteryMonitor_::Init(float total_capacity_mAh, float nominal_voltage)
{
    (void)nominal_voltage; // 本模块不使用标称电压，仅为接口兼容
    total_capacity_mAh = total_capacity_mAh;
    remaining_mAh = total_capacity_mAh;
    consumed_mAh = 0.0f;
    voltage_last = 0.0f;
    current_last = 0.0f;
    filtered_current = 0.0f;
    alpha = 0.8f;           // 低通滤波系数，可根据需要调整(0.7~0.95)
    delta_mAh = 0.0f;
    first_update = true;
    last_tick_ms = 0;
}

void BatteryMonitor_::Update(float voltage, float current, uint32_t dt_ms)
{
    // 获取时间差
    uint32_t now = HAL_GetTick();
    if (first_update) {
        last_tick_ms = now;
        voltage_last = voltage;
        current_last = current;
        filtered_current = current;
        first_update = false;
        delta_mAh = 0.0f;
        return;
    }
    
    uint32_t dt = (dt_ms == 0) ? (now - last_tick_ms) : dt_ms;
    if (dt == 0) return;
    if (dt >= 5000) dt = 5000;   // 防止长时间未调用导致突变
    
    // 低通滤波电流值
    filtered_current = alpha * filtered_current + (1.0f - alpha) * current;
    float used_current = filtered_current;
    if (used_current < 0.0f) used_current = 0.0f; // 只积分放电电流，充电时（负值）不减少容量
    
    // 计算本次积分消耗的 mAh
    float seconds = dt / 1000.0f;
    delta_mAh = CurrentTo_mAh(used_current, seconds);
    
    // 更新剩余容量
    if (remaining_mAh >= delta_mAh)
        remaining_mAh -= delta_mAh;
    else
        remaining_mAh = 0.0f;
    
    // 更新已消耗容量（累计）
    consumed_mAh += delta_mAh;
    if (consumed_mAh > total_capacity_mAh)
        consumed_mAh = total_capacity_mAh;
    
    // 电压和电流记录
    voltage_last = voltage;
    current_last = current;
    last_tick_ms = now;
    
    // 简单校正：剩余容量不应小于0
    if (remaining_mAh < 0.0f) remaining_mAh = 0.0f;
    if (remaining_mAh > total_capacity_mAh) remaining_mAh = total_capacity_mAh;
}

float BatteryMonitor_::GetPercent(void) const
{
    if (total_capacity_mAh <= 0.0f) return 0.0f;
    float percent = (remaining_mAh / total_capacity_mAh) * 100.0f;
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return percent;
}

float BatteryMonitor_::GetRemaining_mAh(void) const
{
    return remaining_mAh;
}

float BatteryMonitor_::GetConsumed_mAh(void) const
{
    return consumed_mAh;
}

float BatteryMonitor_::GetIncrement_mAh(void) const
{
    return delta_mAh;
}

float BatteryMonitor_::GetVoltage(void) const
{
    return voltage_last;
}

float BatteryMonitor_::GetCurrent(void) const
{
    return current_last;
}

void BatteryMonitor_::Reset(void)
{
    remaining_mAh = total_capacity_mAh;
    consumed_mAh = 0.0f;
    last_tick_ms = HAL_GetTick();
    first_update = false;   // 重置后不能再变为 true，否则会丢失一次积分
    delta_mAh = 0.0f;
}

void BatteryMonitor_::SetRemaining_mAh(float mAh)
{
    if (mAh > total_capacity_mAh) mAh = total_capacity_mAh;
    if (mAh < 0.0f) mAh = 0.0f;
    remaining_mAh = mAh;
    consumed_mAh = total_capacity_mAh - remaining_mAh;
}