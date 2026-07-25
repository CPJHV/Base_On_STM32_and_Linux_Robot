#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stm32f1xx_hal_adc.h"
/* 宏定义 ------------------------------------------------------------------*/
#define R_PULL_UP           10.0f
#define NTC_TABLE_LEN       166
#define MAX_TEMPR 125
#define MIN_TEMPR -40

#define VOLT_UP_RES         20.0f
#define VOLT_DOWN_RES       3.3f
#define CACULATE_VOLTAGE(x)  x * (VOLT_UP_RES + VOLT_DOWN_RES) / VOLT_DOWN_RES * ADC_REF_VOLT / ADC_12BIT_RES

// 电流采样 –– 请根据您的实际硬件修改以下宏和函数实现
#define CACULATE_ELECTRI(x) (0.0f)  // 占位，请替换为实际公式

#define MAX_VOLT 23.3
#define MIN_VOLT 0
#define MAX_ELECTRI 1
#define MIN_ELECTRI 0

#define ADC_SOURCE ADC1
#define ADC_CLOCK_ENABLE __HAL_RCC_ADC1_CLK_ENABLE()
#define ADC_REF_VOLT        3.3f
#define ADC_12BIT_RES       4095.0f

#define ADC_TEMPER_Channel ADC_CHANNEL_1
#define ADC_TEMPER_GPIO_CLOCK __HAL_RCC_GPIOA_CLK_ENABLE();
#define ADC_TEMPER_PORT GPIOA
#define ADC_TEMPER_PIN GPIO_PIN_1

#define ADC_VOLITE_Channel ADC_CHANNEL_4
#define ADC_VOLT_GPIO_CLOCK __HAL_RCC_GPIOA_CLK_ENABLE();
#define ADC_VOLT_PORT GPIOA
#define ADC_VOLT_PIN GPIO_PIN_4

#define DMA_SOURCE DMA1_Channel1
#define ADC_BUFFER_SIZE     2
#define FILTER_COUNT        10

/* 外部变量声明 --------------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern uint16_t adc_dma_buf[ADC_BUFFER_SIZE];

/* C接口函数声明（完全保留原样）---------------------------------------------*/
void MX_ADC1_Init(void);
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle);
void ADC_Filter_Task(void);
uint16_t ADC_Get_Temp_Filtered(void);
uint16_t ADC_Get_Volt_Filtered(void);
float Get_Temperature_FromTable(void);
float Get_Power_Voltage(void);
float Get_electri(void);
uint16_t Get_max_temper(void);
uint16_t Get_min_temper(void);
float Get_max_voltage(void);
float Get_min_voltage(void);
float Get_max_electri(void);
float Get_min_electri(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Adc_ {
public:
    void Init(void);
    void Filter(void) { ADC_Filter_Task(); }
    float GetTemperature(void) { return Get_Temperature_FromTable(); }
    float GetVoltage(void)     { return Get_Power_Voltage(); }
    float GetCurrent(void)     { return Get_electri(); }
    uint16_t GetMaxTemp(void)  { return Get_max_temper(); }
    uint16_t GetMinTemp(void)  { return Get_min_temper(); }
    float GetMaxVoltage(void)  { return Get_max_voltage(); }
    float GetMinVoltage(void)  { return Get_min_voltage(); }
    float GetMaxCurrent(void)  { return Get_max_electri(); }
    float GetMinCurrent(void)  { return Get_min_electri(); }
};
#endif

#endif /* __ADC_H */