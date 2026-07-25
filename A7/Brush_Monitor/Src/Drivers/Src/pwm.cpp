#include "pwm.h"

/******************************* 第一部分  4路电机基本驱动 互补输出带死区控制 **************************************/

/* 电机1 互补PWM初始化 */
void Pwm_::pwm1_tim_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;
		//定时器初始化
    g_pwm1_cplm_pwm_handle.Instance = PWM1_TIME_SOURCE;
    g_pwm1_cplm_pwm_handle.Init.Prescaler = psc;
    g_pwm1_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm1_cplm_pwm_handle.Init.Period = arr;
    g_pwm1_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_pwm1_cplm_pwm_handle.Init.RepetitionCounter = 0;
    g_pwm1_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&pwm.g_pwm1_cplm_pwm_handle);
		//定时器输出带初始化
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;//低电平有效（LOW） = 定时器输出的电平会被 “取反” 后送到引脚！
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;//如果使能快速模式，那么新写入的ccr将马上写入而不是等待下一个周期
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&pwm.g_pwm1_cplm_pwm_handle, &sConfigOC, PWM1_TIME_CHANNEL);

	//死区配置
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;/* OSSR设置为1运行时，没发PWM时，空闲状态下时时通道输出什么电平，避免短路炸mos管子 */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;/* OSSI设置为0 运行时，芯片进入停机或者休眠时导通的输出电平*/
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;/* 上电只能写一次，需要更新死区时间时只能用此值 
		BDTR死区刹车配置寄存器保护等级 3级：
		1：不锁定，程序随时可以修改死区，OSSR,BKE这些
		2：只能修改部分，死区时间锁死
		3：硬件锁死，一旦设置，无法软件修改
		测试阶段，用不锁定，方便调试，一但成品，就上锁
		*/
    sBreakDeadTimeConfig.DeadTime = 0X0F;/* 死区时间 */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;/* BKE = 0, 
		关闭外部刹车引脚BKIN，或者开启BKIN引脚检测，一但触发立马关闭所有PWM，进入空闲保护
		关闭BKIN检测 */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW; /* BKP = 1, BKIN低电平触发 */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;/* 使能AOE位，
		刹车撤销后，定时器自动恢复PWM输出
		刹车触发后，锁住不再自动恢复，需要软件重新启动PWM
		允许刹车后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&pwm.g_pwm1_cplm_pwm_handle, &sBreakDeadTimeConfig); /* 设置BDTR寄存器 */
}

/* 电机2 互补PWM初始化 */
void Pwm_::pwm2_tim_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;

    g_pwm2_cplm_pwm_handle.Instance = PWM2_TIME_SOURCE;
    g_pwm2_cplm_pwm_handle.Init.Prescaler = psc;
    g_pwm2_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm2_cplm_pwm_handle.Init.Period = arr;
    g_pwm2_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_pwm2_cplm_pwm_handle.Init.RepetitionCounter = 0;
    g_pwm2_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&pwm.g_pwm2_cplm_pwm_handle);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&pwm.g_pwm2_cplm_pwm_handle, &sConfigOC, PWM2_TIME_CHANNEL);

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0X0F;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&pwm.g_pwm2_cplm_pwm_handle, &sBreakDeadTimeConfig);
}

/* 电机3 互补PWM初始化 */
void Pwm_::pwm3_tim_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;

    g_pwm3_cplm_pwm_handle.Instance = PWM3_TIME_SOURCE;
    g_pwm3_cplm_pwm_handle.Init.Prescaler = psc;
    g_pwm3_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm3_cplm_pwm_handle.Init.Period = arr;
    g_pwm3_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_pwm3_cplm_pwm_handle.Init.RepetitionCounter = 0;
    g_pwm3_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&pwm.g_pwm3_cplm_pwm_handle);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&pwm.g_pwm3_cplm_pwm_handle, &sConfigOC, PWM3_TIME_CHANNEL);

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0X0F;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&pwm.g_pwm3_cplm_pwm_handle, &sBreakDeadTimeConfig);
}

/* 电机4 互补PWM初始化 */
void Pwm_::pwm4_tim_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;

    g_pwm4_cplm_pwm_handle.Instance = PWM4_TIME_SOURCE;
    g_pwm4_cplm_pwm_handle.Init.Prescaler = psc;
    g_pwm4_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm4_cplm_pwm_handle.Init.Period = arr;
    g_pwm4_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_pwm4_cplm_pwm_handle.Init.RepetitionCounter = 0;
    g_pwm4_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&pwm.g_pwm4_cplm_pwm_handle);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&pwm.g_pwm4_cplm_pwm_handle, &sConfigOC, PWM4_TIME_CHANNEL);
}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init_struct;

    if(htim->Instance == TIM1)
    {
        /* PWM1 / PWM2 / PWM3 时钟 */
        PWM1_TIM_CLK_ENABLE();
        PWM1_H_GPIO_CLK_ENABLE();
        PWM1_L_GPIO_CLK_ENABLE();

        PWM2_H_GPIO_CLK_ENABLE();
        PWM2_L_GPIO_CLK_ENABLE();

        PWM3_H_GPIO_CLK_ENABLE();
        PWM3_L_GPIO_CLK_ENABLE();

        /* 主通道 PA8 PA9 PA10 */
        gpio_init_struct.Pin = PWM1_H_GPIO_PIN | PWM2_H_GPIO_PIN | PWM3_H_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = PWM1_TIM_GPIO_AF;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        /* 互补通道 PB13 PB14 PB15 */
        gpio_init_struct.Pin = PWM1_L_GPIO_PIN | PWM2_L_GPIO_PIN | PWM3_L_GPIO_PIN;
        HAL_GPIO_Init(GPIOB, &gpio_init_struct);
    }

    if(htim->Instance == TIM3)
    {	//PWM4的
        PWM4_TIM_CLK_ENABLE();
        PWM4_H_GPIO_CLK_ENABLE();
        PWM4_L_GPIO_CLK_ENABLE();

        /* PC6 */
        gpio_init_struct.Pin = PWM4_H_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = PWM4_TIM_GPIO_AF;
        HAL_GPIO_Init(GPIOC, &gpio_init_struct);

        /* PA5 */
        gpio_init_struct.Pin = PWM4_L_GPIO_PIN;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    }
}
//停止和开始PWM的输出控制接口
void Pwm_::PWM1_HorL_START(uint8_t param){
    if(param == 0) HAL_TIM_PWM_Start(&g_pwm1_cplm_pwm_handle, PWM1_TIME_CHANNEL);
    else           HAL_TIMEx_PWMN_Start(&g_pwm1_cplm_pwm_handle, PWM1_TIME_CHANNEL);
}

void Pwm_::PWM2_HorL_START(uint8_t param){
    if(param == 0) HAL_TIM_PWM_Start(&g_pwm2_cplm_pwm_handle, PWM2_TIME_CHANNEL);
    else           HAL_TIMEx_PWMN_Start(&g_pwm2_cplm_pwm_handle, PWM2_TIME_CHANNEL);
}

void Pwm_::PWM3_HorL_START(uint8_t param){
    if(param == 0) HAL_TIM_PWM_Start(&g_pwm3_cplm_pwm_handle, PWM3_TIME_CHANNEL);
    else           HAL_TIMEx_PWMN_Start(&g_pwm3_cplm_pwm_handle, PWM3_TIME_CHANNEL);
}

void Pwm_::PWM4_HorL_START(uint8_t param){
    if(param == 0) HAL_TIM_PWM_Start(&g_pwm4_cplm_pwm_handle, PWM4_TIME_CHANNEL);
    else           HAL_TIMEx_PWMN_Start(&g_pwm4_cplm_pwm_handle, PWM4_TIME_CHANNEL);
}

/******************************************************************
 * 函数功能：停止该电机所有PWM输出
 ******************************************************************/
void Pwm_::PWM1_STOP(void){
    HAL_TIM_PWM_Stop(&g_pwm1_cplm_pwm_handle, PWM1_TIME_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&g_pwm1_cplm_pwm_handle, PWM1_TIME_CHANNEL);
}

void Pwm_::PWM2_STOP(void){
    HAL_TIM_PWM_Stop(&g_pwm2_cplm_pwm_handle, PWM2_TIME_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&g_pwm2_cplm_pwm_handle, PWM2_TIME_CHANNEL);
}

void Pwm_::PWM3_STOP(void){
    HAL_TIM_PWM_Stop(&g_pwm3_cplm_pwm_handle, PWM3_TIME_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&g_pwm3_cplm_pwm_handle, PWM3_TIME_CHANNEL);
}

void Pwm_::PWM4_STOP(void){
    HAL_TIM_PWM_Stop(&g_pwm4_cplm_pwm_handle, PWM4_TIME_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&g_pwm4_cplm_pwm_handle, PWM4_TIME_CHANNEL);
}

// 类实例化
Pwm_ pwm;

