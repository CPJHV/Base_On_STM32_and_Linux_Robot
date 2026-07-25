#include "adc.h"
#include <string.h>
#include "motor.h"  // 如果需要访问 g_motor_current 等

extern Monitor_Controller monitor_contr;
// 全局对象定义
Adc_ adc;
// 构造函数
Adc_::Adc_() 
    : init_done(0), init_cnt(0)
{
    for (int i = 0; i < ADC1_CH_NUM; i++) {
        g_current_value[i] = 0.0f;
        g_motor_current[i] = 0.0f;
        init_adc_value[i] = 0.0f;
    }
    
    for (int i = 0; i < ADC1_SUM; i++) {
        g_adc_val[i] = 0;
    }
    
    memset(&g_adc_nch_dma_handle, 0, sizeof(ADC_HandleTypeDef));
    memset(&g_dma_nch_adc_handle, 0, sizeof(DMA_HandleTypeDef));
}

void Adc_::Init()
{		Adc_Init();
    Adc_DMAandADC_Init();
}

float Adc_::GetCurrent(uint8_t index)
{
    if (index >= ADC1_CH_NUM) return 0.0f;
    return g_motor_current[index];
}

// 计算ADC平均值（私有方法）
void Adc_::calc_adc_val(void)
{
    // 此函数可以用于平均计算，但在HAL_ADC_ConvCpltCallback中已实现
    // 留空或根据需要实现
}

/* ADC 初始化 */
void Adc_::Adc_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    g_adc_nch_dma_handle.Instance = ADC1_CURRENT_SOURCE;
    g_adc_nch_dma_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    g_adc_nch_dma_handle.Init.Resolution = ADC_RESOLUTION_12B;
    g_adc_nch_dma_handle.Init.ScanConvMode = ENABLE;
    g_adc_nch_dma_handle.Init.ContinuousConvMode = ENABLE;
    g_adc_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;
    g_adc_nch_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    g_adc_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    g_adc_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    g_adc_nch_dma_handle.Init.NbrOfConversion = ADC1_CH_NUM;
    g_adc_nch_dma_handle.Init.DMAContinuousRequests = ENABLE;
    g_adc_nch_dma_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&g_adc_nch_dma_handle);

    /* 4路电流通道配置 */
    sConfig.Channel = ADC1_CURRENT_CH0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC1_CURRENT_CH1;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC1_CURRENT_CH2;
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC1_CURRENT_CH3;
    sConfig.Rank = 4;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
}

/* ADC DMA 初始化 */
void Adc_::Adc_DMAandADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    ADC1_CLK_ENABLE();
    ADC1_GPIO_CLK_ENABLE();
    
    // 配置GPIO
    GPIO_InitStruct.Pin = ADC1_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC1_GPIO_PORT, &GPIO_InitStruct);
    
    // 配置ADC
    Adc_Init();

    __HAL_RCC_DMA2_CLK_ENABLE();

    g_dma_nch_adc_handle.Instance = ADC1_ADCX_DMASx;
    g_dma_nch_adc_handle.Init.Channel = ADC1_ADCX_DMASx_Chanel;
    g_dma_nch_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_dma_nch_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_nch_adc_handle.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_nch_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    g_dma_nch_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    g_dma_nch_adc_handle.Init.Mode = DMA_CIRCULAR;
    g_dma_nch_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&g_dma_nch_adc_handle);

    __HAL_LINKDMA(&g_adc_nch_dma_handle, DMA_Handle, g_dma_nch_adc_handle);

    HAL_NVIC_SetPriority(ADC1_ADCX_DMASx_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(ADC1_ADCX_DMASx_IRQn);

    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle, (uint32_t *)g_adc_val, ADC1_SUM);
}

/* DMA 中断处理（公有接口） */
void Adc_::DMA_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc_handle);
}

/* ADC 转换完成回调（公有接口） */
void Adc_::ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    float temp_c[4] = {0};
    static float add_adc[4] = {0};
    static uint8_t adc_count1 = 0, adc_count2 = 0;
    int j = 0;
    uint32_t temp[4] = {0};
    
    if (hadc->Instance == ADC1_CURRENT_SOURCE)
    {
        HAL_ADC_Stop_DMA(&g_adc_nch_dma_handle);
        
        // 对电流进行均值
        for (j = 0; j < ADC1_COLL; j++) {
            temp[0] += g_adc_val[0 + j * ADC1_CH_NUM];
            temp[1] += g_adc_val[1 + j * ADC1_CH_NUM];  // 修正：每个通道独立
            temp[2] += g_adc_val[2 + j * ADC1_CH_NUM];
            temp[3] += g_adc_val[3 + j * ADC1_CH_NUM];
        }
        
        for (j = 0; j < ADC1_CH_NUM; j++) {
            temp[j] /= ADC1_COLL;
            g_current_value[j] = (float)temp[j];  // 原始ADC值
            add_adc[j] += g_current_value[j];
        }
        
        if (adc_count1 >= 14)  // 累计15次（0-14）
        {
            // 再一次取均值
            for (j = 0; j < ADC1_CH_NUM; j++) {
                add_adc[j] = add_adc[j] / 15.0f;
            }

            if (adc_count2 <= 15)  // 采集16次
            {
                adc_count2++;
                for (j = 0; j < ADC1_CH_NUM; j++) {
                    init_adc_value[j] += add_adc[j];
                }

                if (adc_count2 == 16) {
                    adc_count2 = 17;
                    for (j = 0; j < ADC1_CH_NUM; j++) {
                        init_adc_value[j] = init_adc_value[j] / 16.0f;
                    }
                }
            }

            if (adc_count2 >= 17)  // 采集完参考ADC值后
            {
                for (j = 0; j < ADC1_CH_NUM; j++) {
                    temp_c[j] = (add_adc[j] - init_adc_value[j]) * ADC2CURT;
                    // 一阶低通滤波
                    g_motor_current[j] = 0.6f * g_motor_current[j] + 0.4f * temp_c[j];
                    
                    // 过滤微弱电流
                    if (g_motor_current[j] <= 0.02f) {
                        g_motor_current[j] = 0.0f;
                    }
                    
                    // 更新到对应电机的数据结构（如果有）
                    switch(j) {
                        case 0: monitor_contr.g_motor1_data.current = g_motor_current[j]; break;
                        case 1: monitor_contr.g_motor2_data.current = g_motor_current[j]; break;
                        case 2: monitor_contr.g_motor3_data.current = g_motor_current[j]; break;
                        case 3: monitor_contr.g_motor4_data.current = g_motor_current[j]; break;
                    }
                }
            }
            
            // 重置累加器
            for (j = 0; j < ADC1_CH_NUM; j++) {
                add_adc[j] = 0;
            }
            adc_count1 = 0;
        }
        adc_count1++;
        
        HAL_ADC_Start_DMA(&g_adc_nch_dma_handle, (uint32_t *)g_adc_val, ADC1_SUM);
    }
}
void ADC1_ADCX_DMASx_IRQHandler(void)
{
    adc.DMA_IRQHandler();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    adc.ADC_ConvCpltCallback(hadc);
}

extern "C"{

uint8_t get_current_val(uint8_t index){
	
	return adc.g_motor_current[index-1];
}

}