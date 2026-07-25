#include "battery_monitor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>
/*
OCV-SOC校准：
原理是上电后不要启动负载，测到的电压对照手册的表，
根据表推算电池容量值，然后替换掉动态积分的值即可
*/
static inline float clamp(float val, float minv, float maxv)
{
    if (val < minv) return minv;
    if (val > maxv) return maxv;
    return val;
}
//安时计算公式Q=I*t；而mAh=1000mA/3600s
float BatteryMonitor_::CurrentTo_mAh(float current_amp, float seconds) const
{
    return current_amp * seconds * (1000.0f / 3600.0f);
}

void BatteryMonitor_::Init(float total_capacity_mAh, float nominal_voltage)
{
    (void)nominal_voltage;
    this->total_capacity_mAh = total_capacity_mAh;

    remaining_mAh = total_capacity_mAh;
    consumed_mAh = 0.0f;
    voltage_last = 0.0f;
    current_last = 0.0f;
    filtered_current = 0.0f;
    alpha = 0.8f;
    delta_mAh = 0.0f;
    first_update = true;
    last_tick_ms = HAL_GetTick();

    // 配置默认参数
    enable_charge_support = false;    // 默认：一次性电池，关闭充电积分
    current_dead_zone = 0.02f;        // 电流死区 ±0.02A
    eta_discharge = 1.0f;             // 放电库仑效率
    eta_charge = 0.97f;               // 充电库仑效率
}

void BatteryMonitor_::SetFilterAlpha(float val)
{
    alpha = clamp(val, 0.1f, 0.98f);
}
//是否启动为二次充电电池
void BatteryMonitor_::EnableChargeSupport(bool en)
{
    enable_charge_support = en;
}

void BatteryMonitor_::Update(float voltage, float current, uint32_t dt_ms)
{
    uint32_t now = HAL_GetTick();
    if (first_update)
    {//第一次更新的话
        last_tick_ms = now;
        voltage_last = voltage;
        current_last = current;
        filtered_current = current;
        first_update = false;
        delta_mAh = 0.0f;
        return;
    }
		//获取变化时间
    uint32_t dt = (dt_ms == 0) ? (now - last_tick_ms) : dt_ms;
    if (dt == 0) return;
    dt = clamp((float)dt, 1.0f, 5000.0f);

    // 一阶低通滤波
    filtered_current = alpha * filtered_current + (1.0f - alpha) * current;
    float sec = dt / 1000.0f;
    delta_mAh = 0.0f;

    // 零点死区，微小电流不积分，消除ADC漂移
    if (fabsf(filtered_current) > current_dead_zone)
    {//ADC即便你不入电，也会因为漂移有微弱电流
			//而我们只是需要当检测到的电流比这个小，说明是噪音或者漂移，不要拿去积分
        if (filtered_current > 0.0f)
        {
            // 放电，因为并不是所有电荷都能做功
					//因此这个真实流出电流1A时候，有效损耗只有98%。
            float eff_current = filtered_current * eta_discharge;
					
            delta_mAh = CurrentTo_mAh(eff_current, sec);//转成安时
            remaining_mAh -= delta_mAh;//剩余电量
            consumed_mAh += delta_mAh;//累计消耗量
        }
        else if (enable_charge_support)
        {
            // 充电（仅开启充电支持才执行）
            float charge_amp = -filtered_current;
            float eff_current = charge_amp * eta_charge;
            delta_mAh = CurrentTo_mAh(eff_current, sec);
            remaining_mAh += delta_mAh;
            // 充电不叠加到 consumed_mAh
        }
    }

    // 边界钳位
    remaining_mAh = clamp(remaining_mAh, 0.0f, total_capacity_mAh);
    consumed_mAh = clamp(consumed_mAh, 0.0f, total_capacity_mAh);

    voltage_last = voltage;
    current_last = current;
    last_tick_ms = now;
}

float BatteryMonitor_::GetPercent(void) const
{
    if (total_capacity_mAh <= 1e-6f) return 0.0f;
    float pct = (remaining_mAh / total_capacity_mAh) * 100.0f;
    return clamp(pct, 0.0f, 100.0f);
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

// 预估剩余运行时间（分钟）
float BatteryMonitor_::EstimateRemainingMinute(void) const
{
    if (fabsf(filtered_current) < current_dead_zone)
        return -1.0f; // 空载无法预估
    float hour = remaining_mAh / (filtered_current * 1000.0f);
    return hour * 60.0f;
}

void BatteryMonitor_::Reset(void)
{
    remaining_mAh = total_capacity_mAh;
    consumed_mAh = 0.0f;
    last_tick_ms = HAL_GetTick();
    first_update = true;   // 修复：重置后重新对齐时间戳
    delta_mAh = 0.0f;
}

void BatteryMonitor_::SetRemaining_mAh(float mAh)
{
    remaining_mAh = clamp(mAh, 0.0f, total_capacity_mAh);
    consumed_mAh = total_capacity_mAh - remaining_mAh;
}

float BatteryMonitor_::LookupOCVToSOC(float ocv, const OCV_SOC_Point_t *table, uint16_t table_len)
{
    // 越界判断
    if (ocv <= table[0].voltage)
        return table[0].soc;
    if (ocv >= table[table_len-1].voltage)
        return table[table_len-1].soc;

    // 查找区间
    uint16_t idx;
    for(idx = 0; idx < table_len-1; idx++)
    {
        if(ocv >= table[idx].voltage && ocv <= table[idx+1].voltage)
            break;
    }

    // 线性插值
    float v0 = table[idx].voltage;
    float s0 = table[idx].soc;
    float v1 = table[idx+1].voltage;
    float s1 = table[idx+1].soc;

    float ratio = (ocv - v0) / (v1 - v0);
    return s0 + ratio * (s1 - s0);
}

bool BatteryMonitor_::CalibrateByOCV(float open_circuit_voltage)
{
    if(total_capacity_mAh < 1.0f)
        return false;

    float soc = LookupOCVToSOC(open_circuit_voltage, li_poly_table, OCV_TABLE_LEN);
    float real_remaining = soc / 100.0f * total_capacity_mAh;
    SetRemaining_mAh(real_remaining);
    return true;
}
