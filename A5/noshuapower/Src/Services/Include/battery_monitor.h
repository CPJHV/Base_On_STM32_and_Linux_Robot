#ifndef __BATTERY_MONITOR_H
#define __BATTERY_MONITOR_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus

const OCV_SOC_Point_t li_poly_table[] =
{
    {3.00f, 0.0f},
    {3.10f, 5.0f},
    {3.20f, 10.0f},
    {3.30f, 16.0f},
    {3.40f, 23.0f},
    {3.50f, 31.0f},
    {3.60f, 40.0f},
    {3.70f, 50.0f},
    {3.80f, 62.0f},
    {3.90f, 75.0f},
    {4.00f, 87.0f},
    {4.10f, 96.0f},
    {4.20f, 100.0f}
};
#define OCV_TABLE_LEN (sizeof(li_poly_table)/sizeof(OCV_SOC_Point_t))
// 锂电池OCV-SOC查找表结构
typedef struct
{
    float voltage;
    float soc;
} OCV_SOC_Point_t;

class BatteryMonitor_
{
public:
    /**
     * @brief 初始化电量监测
     * @param total_capacity_mAh 电池额定容量
     * @param ocv_table 开路电压SOC查表表
     * @param table_len 表长度
     */
    void Init(float total_capacity_mAh, const OCV_SOC_Point_t *ocv_table, uint16_t table_len);

    /**
     * @brief 周期更新
     * @param voltage 电池端电压(V)
     * @param current 电流(A) >0放电，<0充电
     * @param dt_ms 间隔ms，传入0自动内部计算
     */
    void Update(float voltage, float current, uint32_t dt_ms = 0);

    // ========== 对外读取接口 ==========
    float GetSOC(void) const;                         // 0~100.0
    float GetRemainingmAh(void) const;                // 剩余容量 mAh
    float GetConsumedmAh(void) const;                 // 累计消耗容量
    float GetLastVoltage(void) const;                 // 最新采样电压
    float GetFilteredCurrent(void) const;             // 滤波后电流
    float GetDeltamAh(void) const;                    // 本次增量
    float EstimateRunTimeMin(void) const;             // 预估剩余运行分钟

    // ========== 配置接口 ==========
    void SetFilterAlpha(float alpha);                 // 电流低通系数 0~1
    void SetCoulombEfficiency(float charge_eta, float discharge_eta); // 充放电效率
    void SetSelfDischargeRate(float mAh_per_hour);    // 自放电速率 mAh/h

    // ========== SOC校正接口 ==========
    // 根据当前开路电压强制校正SOC（外部确认电池静置无电流后调用）
    bool CalibrateByOCV(float open_circuit_voltage);
    void SetRemainingmAh(float mAh);                  // 手动设置剩余容量

    // ========== 状态复位 ==========
    void ResetFullCharge(void);                       // 充满复位为100%
    void Reset(void);

    // ========== 持久化：获取需要保存的数据（存入Flash） ==========
    void GetSaveData(float *rem_mah, uint32_t *tick_stamp);
    bool LoadSaveData(float rem_mah, uint32_t tick_stamp);
		
		bool CalibrateByOCV(float open_circuit_voltage);
		
private:
    // 内部计算函数
    float CurrentToMAh(float amp, float sec) const;
    float LinearInterpSOC(float ocv) const;
    float Clamp(float val, float minv, float maxv) const;

    // 积分核心
    float total_cap_mah;
    float remaining_mah;
    float consumed_mah;
    float delta_mah;

    // 采样
    float voltage_raw;
    float current_raw;
    float filtered_current;

    // 滤波
    float alpha_filter;

    // 效率参数
    float eta_charge;
    float eta_discharge;
    float self_discharge_mah_h;

    // 时间管理
    uint32_t last_tick_ms;
    bool first_update;

    // OCV查表
    const OCV_SOC_Point_t *ocv_table;
    uint16_t ocv_table_size;

    // 静置判定计时器：用于识别电池空载、允许OCV校正
    uint32_t idle_tick_ms;
    const float CURRENT_IDLE_THRESHOLD = 0.02f;       // 电流绝对值小于此判定为静置
    const uint32_t IDLE_CALIB_TIME_MS = 120000U;      // 静置2分钟允许校正

		float LookupOCVToSOC(float ocv, const OCV_SOC_Point_t *table, uint16_t table_len);
		
};

#endif
#endif