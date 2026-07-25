#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx_hal.h"

/* 4路电流采集 ADC 通道定义 (PC0, PC1, PC2, PC3) */
#define ADC1_CURRENT_SOURCE                ADC1
#define ADC1_CURRENT_CH0                   ADC_CHANNEL_10    /* PC0 */
#define ADC1_CURRENT_CH1                   ADC_CHANNEL_11    /* PC1 */
#define ADC1_CURRENT_CH2                   ADC_CHANNEL_12    /* PC2 */
#define ADC1_CURRENT_CH3                   ADC_CHANNEL_13    /* PC3 */

#define ADC1_CLK_ENABLE()                  do{ __HAL_RCC_ADC1_CLK_ENABLE(); }while(0)

#define ADC1_CH_NUM                        4       // 通道数量
#define ADC1_COLL                          120     // 平均值时一个通道采集次数（建议改为120，太大可能溢出）
#define ADC1_SUM                           (ADC1_CH_NUM * ADC1_COLL)

/* ADC GPIO 配置 PC0~PC3 */
#define ADC1_GPIO_PORT                     GPIOC
#define ADC1_GPIO_PIN                      (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
#define ADC1_GPIO_CLK_ENABLE()             do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/* DMA 配置 */
#define ADC1_ADCX_DMASx                    DMA2_Stream4
#define ADC1_ADCX_DMASx_Chanel             DMA_CHANNEL_0
#define ADC1_ADCX_DMASx_IRQn               DMA2_Stream4_IRQn
#define ADC1_ADCX_DMASx_IRQHandler         DMA2_Stream4_IRQHandler

#define ADC_ADCX_DMASx_IS_TC()             __HAL_DMA_GET_FLAG(&g_dma_nch_adc_handle, DMA_FLAG_TCIF0_4)
#define ADC_ADCX_DMASx_CLR_TC()            __HAL_DMA_CLEAR_FLAG(&g_dma_nch_adc_handle, DMA_FLAG_TCIF0_4)

/* 电流转换系数 根据你的硬件自行修改 */
#define ADC2CURT                           0.01f

#ifdef __cplusplus
class Adc_ {
public:
    Adc_();  // 构造函数
    void Init();
    float GetCurrent(uint8_t index);
    
    // 中断处理接口（公有，供外部调用）
    void DMA_IRQHandler(void);
    void ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
    
    // DMA句柄改为公有（供中断使用）
    DMA_HandleTypeDef g_dma_nch_adc_handle;

private:
    void calc_adc_val(void);
    void Adc_Init(void);
    void Adc_DMAandADC_Init(void);
    
    ADC_HandleTypeDef g_adc_nch_dma_handle;
    float g_current_value[ADC1_CH_NUM];     // 存放均值后的adc值（原始ADC值）
    uint16_t g_adc_val[ADC1_SUM];           // adc原始值数组
    
    // 四电机独立电流值（滤波后）
    float g_motor_current[4];
    
    // 零点校准相关
    float init_adc_value[4];
    uint8_t init_done;
    uint8_t init_cnt;
};



#endif

#endif