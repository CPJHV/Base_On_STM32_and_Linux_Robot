#include "adc.h"

/* 全局变量定义 -------------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

//因为DMA是异步自己写入内存的，但是在代码里编译器认为数组没有赋值行为
//固然就优化了拿第一次值放入寄存器，后续循环读取就复用为寄存器版本的，而需要加入volatile
//来避免这个问题，禁止编译器缓存在内存到寄存器中
volatile uint16_t adc_dma_buf[ADC_BUFFER_SIZE];

uint16_t filter_temp[FILTER_COUNT];//滤波的缓冲器
uint16_t filter_volt[FILTER_COUNT];
static uint8_t  index = 0;
static uint8_t  valid_cnt = 0;   // 当前缓冲区有效样本数量
static uint32_t sum_temp = 0;
static uint32_t sum_volt = 0;
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

/* NTC 对应电阻值 KΩ */
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

/* ADC 初始化
12位ADC采集固定转换周期为12.5个ADC时钟
时间=（采样周期+12.5）*ADC外设时钟

*/
void MX_ADC1_Init(void)
{
  // 定义ADC通道配置结构体变量，并初始化为0（清空所有配置）
ADC_ChannelConfTypeDef sConfig = {0};

// 绑定ADC句柄到对应的硬件外设（ADC1/ADC2等，由宏ADC_SOURCE指定）
hadc1.Instance = ADC_SOURCE;

// 开启扫描模式：用于同时采集多个ADC通道（这里配置了2个通道）
hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;

// 开启连续转换模式：ADC转换完成后自动重新开始下一轮转换，无需手动触发
hadc1.Init.ContinuousConvMode = ENABLE;

// 关闭间断模式：不使用分段采集，一次性完成所有通道转换
hadc1.Init.DiscontinuousConvMode = DISABLE;

// ADC触发源：软件触发（代码调用启动转换，不使用外部硬件触发）
hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;

// ADC数据对齐方式：右对齐（STM32默认常用方式，结果存放在低12位）
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;

// 设置规则组需要转换的通道总数：这里配置了2个通道
hadc1.Init.NbrOfConversion = 2;

// 初始化ADC外设，如果初始化失败则进入错误处理函数
if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

// ===================== 配置第一个ADC通道 =====================
// 设置要配置的ADC通道：温度传感器通道（由宏ADC_TEMPER_Channel指定）
sConfig.Channel = ADC_TEMPER_Channel;
// 设置通道转换顺序：第1个转换
sConfig.Rank = ADC_REGULAR_RANK_1;
// 设置通道采样时间：7.5个时钟周期
sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
// 将配置写入ADC寄存器，失败则进入错误处理
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();

// ===================== 配置第二个ADC通道 =====================
// 复用同一个结构体，修改通道为：电压采样通道
sConfig.Channel = ADC_VOLITE_Channel;
// 设置通道转换顺序：第2个转换
sConfig.Rank = ADC_REGULAR_RANK_2;
// 采样时间保持和上一个通道一致：7.5个时钟周期
sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
// 配置第二个通道
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}
void ADC_Start_DMA(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, ADC_BUFFER_SIZE);
}
void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance == ADC1)
  {
    ADC_CLOCK_ENABLE;
    ADC_TEMPER_GPIO_CLOCK;
    __HAL_RCC_DMA1_CLK_ENABLE();

    GPIO_InitStruct.Pin = ADC_TEMPER_PIN | ADC_VOLT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(ADC_TEMPER_PORT, &GPIO_InitStruct);

		//DMA配置
   // 选择DMA硬件外设（如DMA1_Channel1等，由宏DMA_SOURCE指定）
hdma_adc1.Instance = DMA_SOURCE;

// DMA传输方向：外设（ADC） -> 内存（数组）
hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;

// 外设地址不自增：ADC数据寄存器只有一个，固定地址读取
hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;

// 内存地址自增：ADC采完一个值，自动存到数组下一个位置
hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;

// 外设数据位宽：半字（16位），ADC是12位，用16位刚好存放
hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;

// 内存数据位宽：半字（16位），与ADC数据格式匹配
hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;

// DMA模式：循环模式。ADC不断采集，DMA循环覆盖数组
hdma_adc1.Init.Mode = DMA_CIRCULAR;

// DMA优先级：中等（不跟其他外设抢总线时用这个即可）
hdma_adc1.Init.Priority = DMA_PRIORITY_MEDIUM;

// 初始化DMA通道，失败则自动进入错误处理
if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

// 把DMA句柄 绑定 到ADC句柄，让ADC自动触发DMA传输
__HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{
  if(adcHandle->Instance == ADC1)
  {
    __HAL_RCC_ADC1_CLK_DISABLE();
    HAL_GPIO_DeInit(ADC_TEMPER_PORT, ADC_TEMPER_PIN | ADC_VOLT_PIN);
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  }
}

/* 滑动滤波 */
void ADC_Filter_Task(void)
{
  uint16_t new_t = adc_dma_buf[0];
    uint16_t new_v = adc_dma_buf[1];
	osEnterCriticalSection();
  if(valid_cnt >= FILTER_COUNT)
    {
        //窗口已满，减去被覆盖的旧数据
        sum_temp -= filter_temp[index];
        sum_volt -= filter_volt[index];
    }
    else
    {
        valid_cnt++;
    }
		filter_temp[index] = new_t;
    filter_volt[index] = new_v;

    sum_temp += new_t;
    sum_volt += new_v;

    index++;
    if(index >= FILTER_COUNT)
        index = 0;
		osExitCriticalSection();
}
//获取adc检测的温度的值滤波后的
uint16_t ADC_Get_Temp_Filtered(void)
{
  if(valid_cnt == 0) return 0;
	//使用临界区，来处理时间短的操作
	//还要注意的点就是中断会让CPU处于ISR的，也就是不受任务调度器控制，此时不建议在中断放入互斥锁这些
	osEnterCriticalSection();
    if(valid_cnt < FILTER_COUNT)
    {
        //可选：窗口未填满，不输出 / 除以当前数量
        return sum_temp / valid_cnt;
    }
		osExitCriticalSection();
    return sum_temp / FILTER_COUNT;
}
//获取adc的电压的值滤波后的
uint16_t ADC_Get_Volt_Filtered(void)
{
 if(valid_cnt == 0) return 0;
	osEnterCriticalSection();
    if(valid_cnt < FILTER_COUNT)
    {
        //可选：窗口未填满，不输出 / 除以当前数量
        return sum_volt / valid_cnt;
    }
		osExitCriticalSection();
    return sum_volt / FILTER_COUNT;
}

float Get_Temperature_FromTable(void)
{
    uint16_t adc = ADC_Get_Temp_Filtered();
    if(adc >= ADC_12BIT_RES) return 125.0f;
    if(adc == 0) return -40.0f;

    float Rntc = R_PULL_UP * (float)adc / (ADC_12BIT_RES - (float)adc);

    int left  = 0;
    int right = NTC_TABLE_LEN - 1;
    int mid;

    // NTC_Res_Table：从大 → 小（-40℃电阻最大，125℃最小）
    while(left < right)
    {
        mid = (left + right + 1) / 2;
        if(Rntc <= NTC_Res_Table[mid])
        {
            left = mid;
        }
        else
        {
            right = mid - 1;
        }
    }

    int i = left;
    if(i >= NTC_TABLE_LEN - 1)
    {
        return 125.0f;
    }
    float R1 = NTC_Res_Table[i];
    float R2 = NTC_Res_Table[i+1];
    float T1 = NTC_Temp_Table[i];
    float T2 = NTC_Temp_Table[i+1];

    float temp = T1 + (T2 - T1) * (R1 - Rntc) / (R1 - R2);
    return temp;
}
//获取计算公式转换后的电压
float Get_Power_Voltage(void)
{
    uint16_t adc = ADC_Get_Volt_Filtered();
    return CACULATE_VOLTAGE(adc);
}
//获取转换后的电流
float Get_electri(void)
{
    return CACULATE_ELECTRI(Get_Power_Voltage());
}

uint16_t Get_max_temper(void) { return MAX_TEMPR; }
uint16_t Get_min_temper(void) { return MIN_TEMPR; }
float Get_max_voltage(void)   { return MAX_VOLT; }
float Get_min_voltage(void)   { return MIN_VOLT; }
float Get_max_electri(void)   { return MAX_ELECTRI; }
float Get_min_electri(void)   { return MIN_ELECTRI; }