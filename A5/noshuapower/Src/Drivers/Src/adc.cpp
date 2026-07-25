#include "adc.h"

/* 全局变量定义 -------------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
uint16_t adc_dma_buf[ADC_BUFFER_SIZE];
uint16_t filter_temp[FILTER_COUNT];
uint16_t filter_volt[FILTER_COUNT];
uint8_t  filter_index = 0;

/* NTC 温度表 ℃ (-40℃ ~ 125℃) */
const int16_t NTC_Temp_Table[NTC_TABLE_LEN] = {
    -40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,
    -29,-28,-27,-26,-25,-24,-23,-22,-21,-20,
    -19,-18,-17,-16,-15,-14,-13,-12,-11,-10,
    -9,-8,-7,-6,-5,-4,-3,-2,-1,0,
    1,2,3,4,5,6,7,8,9,10,
    11,12,13,14,15,16,17,18,19,20,
    21,22,23,24,25,26,27,28,29,30,
    31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,
    51,52,53,54,55,56,57,58,59,60,
    61,62,63,64,65,66,67,68,69,70,
    71,72,73,74,75,76,77,78,79,80,
    81,82,83,84,85,86,87,88,89,90,
    91,92,93,94,95,96,97,98,99,100,
    101,102,103,104,105,106,107,108,109,110,
    111,112,113,114,115,116,117,118,119,120,
    121,122,123,124,125
};

const float NTC_Res_Table[NTC_TABLE_LEN] = {
    197.390,186.540,176.350,166.800,157.820,149.390,141.510,134.090,127.110,120.530,114.340,
    108.530,103.040,97.870,92.989,88.381,84.036,79.931,76.052,72.384,68.915,
    65.634,62.529,59.589,56.804,54.166,51.665,49.294,47.046,44.913,42.889,
    40.967,39.142,37.408,35.761,34.196,32.707,31.291,29.945,28.664,27.445,
    26.283,25.177,24.124,23.121,22.165,21.253,20.384,19.555,18.764,18.010,
    17.290,16.602,15.946,15.319,14.720,14.148,13.601,13.078,12.578,12.099,
    11.642,11.204,10.785,10.384,10.000,9.632,9.280,8.943,8.619,8.309,
    8.012,7.727,7.453,7.191,6.939,6.698,6.466,6.243,6.029,5.824,
    5.627,5.437,5.255,5.080,4.911,4.749,4.593,4.443,4.299,4.160,
    4.027,3.898,3.774,3.654,3.539,3.429,3.322,3.219,3.119,3.024,
    2.931,2.842,2.756,2.673,2.593,2.516,2.441,2.369,2.300,2.233,
    2.168,2.105,2.044,1.986,1.929,1.874,1.821,1.770,1.720,1.673,
    1.626,1.581,1.538,1.496,1.455,1.416,1.377,1.340,1.304,1.270,
    1.236,1.204,1.172,1.141,1.112,1.083,1.055,1.028,1.002,0.976,
    0.951,0.927,0.904,0.882,0.860,0.838,0.818,0.798,0.778,0.759,
    0.741,0.723,0.706,0.689,0.673,0.657,0.641,0.626,0.612,0.598,
    0.584,0.570,0.557,0.545,0.532
};

/* ADC初始化函数（不变）------------------------------------------------------*/
void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC_SOURCE;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_TEMPER_Channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

    sConfig.Channel = ADC_VOLITE_Channel;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(adcHandle->Instance==ADC1) {
        ADC_CLOCK_ENABLE;
        ADC_TEMPER_GPIO_CLOCK;
        __HAL_RCC_DMA1_CLK_ENABLE();

        GPIO_InitStruct.Pin = ADC_TEMPER_PIN | ADC_VOLT_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        HAL_GPIO_Init(ADC_TEMPER_PORT, &GPIO_InitStruct);

        hdma_adc1.Instance = DMA_SOURCE;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;
        HAL_DMA_Init(&hdma_adc1);
        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {
    if(adcHandle->Instance==ADC_SOURCE) {
        __HAL_RCC_ADC1_CLK_DISABLE();
        HAL_GPIO_DeInit(ADC_TEMPER_PORT, ADC_TEMPER_PIN|ADC_VOLT_PIN);
        HAL_DMA_DeInit(adcHandle->DMA_Handle);
    }
}

void ADC_Filter_Task(void) {
    filter_temp[filter_index] = adc_dma_buf[0];
    filter_volt[filter_index] = adc_dma_buf[1];
    filter_index++;
    if(filter_index >= FILTER_COUNT) filter_index = 0;
}

uint16_t ADC_Get_Temp_Filtered(void) {
    uint32_t sum = 0;
    for(uint8_t i=0; i<FILTER_COUNT; i++) sum += filter_temp[i];
    return sum / FILTER_COUNT;
}

uint16_t ADC_Get_Volt_Filtered(void) {
    uint32_t sum = 0;
    for(uint8_t i=0; i<FILTER_COUNT; i++) sum += filter_volt[i];
    return sum / FILTER_COUNT;
}

float Get_Temperature_FromTable(void) {
    uint16_t adc = ADC_Get_Temp_Filtered();
    if(adc >= ADC_12BIT_RES) return 125.0f;
    if(adc == 0) return -40.0f;
    float Rntc = R_PULL_UP * (float)adc / (ADC_12BIT_RES - (float)adc);
    for(int i = 0; i < NTC_TABLE_LEN - 1; i++) {
        float R1 = NTC_Res_Table[i];
        float R2 = NTC_Res_Table[i+1];
        if(Rntc <= R1 && Rntc >= R2) {
            float T1 = NTC_Temp_Table[i];
            float T2 = NTC_Temp_Table[i+1];
            float temp = T1 + (T2 - T1) * (R1 - Rntc) / (R1 - R2);
            return temp;
        }
    }
    if(Rntc > NTC_Res_Table[0]) return -40.0f;
    return 125.0f;
}

float Get_Power_Voltage(void) {
    uint16_t adc = ADC_Get_Volt_Filtered();
    return CACULATE_VOLTAGE(adc);
}

float Get_electri(void) {
    // 请根据实际电流采样硬件修改此函数
    // 示例：通过电压和已知负载估算，或使用专用电流通道
    // 目前返回0，需要您自行实现
    return 0.0f;
}

uint16_t Get_max_temper(void) { return MAX_TEMPR; }
uint16_t Get_min_temper(void) { return MIN_TEMPR; }
float Get_max_voltage(void)   { return MAX_VOLT; }
float Get_min_voltage(void)   { return MIN_VOLT; }
float Get_max_electri(void)   { return MAX_ELECTRI; }
float Get_min_electri(void)   { return MIN_ELECTRI; }

/* C++ 类方法实现 -----------------------------------------------------------*/
void Adc_::Init(void) {
    MX_ADC1_Init();
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_BUFFER_SIZE);
}