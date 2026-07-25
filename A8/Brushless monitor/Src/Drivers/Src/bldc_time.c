#ifdef _BLDC_WU_
#include "bldc_time.h"
#include "bldc.h"
#include "pid.h"
#include "adc.h"
#include "zero_ctr.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/******************************************************************************************/
/* 定时器配置句柄 定义 */

/* 高级定时器PWM */
TIM_HandleTypeDef g_atimx_handle;           /* 定时器x句柄 */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;   /* 定时器输出句柄 */
extern _bldc_obj g_bldc_motor1;

/******************************************************************************************/

/**
 * @brief       高级定时器TIMX PWM 初始化函数
 * @note
 *              高级定时器的时钟来自APB2, 而PCLK2 = 168Mhz, 我们设置PPRE2不分频, 因此
 *              高级定时器时钟 = 168Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();        /* TIMX 时钟使能 */


    g_atimx_handle.Instance = ATIM_TIMX_PWM;                    /* 定时器x */
    g_atimx_handle.Init.Prescaler = psc;                        /* 定时器分频 */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
    g_atimx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* 分频因子 */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*使能TIMx_ARR进行缓冲*/
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* 开始时不计数*/
    HAL_TIM_PWM_Init(&g_atimx_handle);                          /* 初始化PWM */

    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* 模式选择PWM1 */
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* 输出比较极性为高 */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH3); /* 配置TIMx通道y */
    
    /* 开启定时器通道1输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);

    /* 开启定时器通道2输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);

    /* 开启定时器通道3输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);

    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

    HAL_TIM_Base_Start_IT(&g_atimx_handle);                         /* 启动高级定时器1 */
}


/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* 开启通道y的IO时钟 */
        /* 三个上桥臂对应IO时钟使能 */
        ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE();                        /* IO时钟使能 */
        ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE();                        /* IO时钟使能 */
        ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE();                        /* IO时钟使能 */
        /* 三个下桥臂对应IO时钟使能 */
        M1_LOW_SIDE_U_GPIO_CLK_ENABLE();                            /* IO时钟使能 */
        M1_LOW_SIDE_V_GPIO_CLK_ENABLE();                            /* IO时钟使能 */
        M1_LOW_SIDE_W_GPIO_CLK_ENABLE();                            /* IO时钟使能 */

        /* UVW_LOW的IO初始化 */
        gpio_init_struct.Pin = M1_LOW_SIDE_U_PIN;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* 推挽输出模式 */
        HAL_GPIO_Init(M1_LOW_SIDE_U_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_V_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_V_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_W_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_W_PORT, &gpio_init_struct);


        /*定时器IO初始化*/
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH1_GPIO_PIN;          /* 通道y的IO口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* 不上拉 不下拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* 端口复用 */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH2_GPIO_PIN;      
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH3_GPIO_PIN;        
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);
    }
}
/*****************************************************************************************/
/*基本定时器初始化*/
TIM_HandleTypeDef timx_handler;                                     /* 定时器参数句柄 */


/**
 * @brief       基本定时器TIMX定时中断初始化函数
 * @note
 *              基本定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              基本定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;                      /* 通用定时器X */
    timx_handler.Init.Prescaler = psc;                          /* 设置预分频器  */
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 向上计数器 */
    timx_handler.Init.Period = arr;                             /* 自动装载值 */
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* 时钟分频因子 */
    HAL_TIM_Base_Init(&timx_handler);

    HAL_TIM_Base_Start_IT(&timx_handler);                       /* 使能通用定时器x和及其更新中断：TIM_IT_UPDATE */
    __HAL_TIM_CLEAR_IT(&timx_handler,TIM_IT_UPDATE);            /* 清除更新中断标志位 */
}

/**
 * @brief       定时器底册驱动，开启时钟，设置中断优先级
                此函数会被HAL_TIM_Base_Init()函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* 使能TIM时钟 */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3); /* 抢占1，子优先级3，组2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* 开启ITM3中断 */
    }
}

/**
 * @brief       基本定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);                  /* 定时器回调函数 */
} 
/**
 * @brief       高级定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void ATIM_TIMX_PWM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx_handle);
}
/***********************************************定时器中断回调函数***********************************************/
static uint8_t cf_count = 0;        /* 尖峰电流计数 */
int32_t temp_pwm1 = 0;              /* 存放速度环的PID计算结果 */
int32_t temp_pwm2 = 0;              /* 存放电流环的PID计算结果 */
int32_t motor_pwm_s = 0;            /* 速度环一阶滤波后的结果 */
int32_t motor_pwm_c = 0;            /* 电流环一阶滤波后的结果 */
int32_t motor_pwm_sl= 0;
  
extern  Hallless g_hallless_three;                      /* 反电势结构体 */

uint8_t clc = 0;                    /* 等待时间进入闭环控制时间 */
static uint8_t pid_c_count=0;       /* 定时器时间记录 */
static uint8_t pid_s_count=0;       /* 定时器时间记录 */
/* debug使用 */
static uint8_t debug_switch=0;
float debug_data_temp=0.0;
float *user_setpoint = (float*)(&g_speed_pid.SetPoint);
/* 停机状态下电流采集使用 */
#define ADC_AMP_OFFSET_TIMES 50
uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1];
uint8_t adc_amp_offset_p = 0;
int16_t adc_amp[3];
int16_t adc_amp_un[3];
float  adc_amp_bus=0.0f;

volatile uint16_t adc_val_m1[ADC_CH_NUM];
/**
 * @brief       定时器中断回调
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t i;
    static uint8_t times_count=0;       /* 定时器时间记录 */
    if(htim->Instance == ATIM_TIMX_PWM) /* 55us */
    {
        if(g_bldc_motor1.run_flag == RUN)
        {
            for(i=0; i<3; i++)
            {
                adc_val_m1[i] = g_adc_val[i+2];//获取uvw三个相的电流，0和1是温度和电压值
							
                adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];
							//减去零点漂移
							
							/*
							这是因为电机两相通电，一相悬空，可是转子还在切割磁场，这导致悬空的那个出现反向电流
							所以叫反电动势
							*/
                if(adc_amp[i] < 0)  
                    adc_amp_un[i] = 0;                  /* 反电动势电压为悬空绕组直接清0 */
                else if(adc_amp[i] >= 0)                /* 去除反电动势引起的负电流数据 */
                    adc_amp_un[i] = adc_amp[i];
            }
            /*运算母线电流（母线电流为任意两个有开关动作的相电流之和）*/
            adc_amp_bus = (adc_amp_un[0] + adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;
        }
#ifdef H_PWM_L_ON
        /* 过零控制 */
        zero_ctr_loop();//执行无感启动，过零换相
				
        /******************************* PID控制 *******************************/
        if(g_bldc_motor1.run_flag == RUN && g_zero_ctr_status == 3) /* 具有一定速度（有速度测量说明已经进入过零闭环状态）后才能进入PID闭环控制 */
        {//电机处于运转状态且进入了闭环状态
					
            /* 速度+电流双环控制 */
            pid_s_count++;
            pid_c_count++;
					
					
            if(pid_s_count > 2)
            {//它这是隔开几次才算一次PID
#if DEBUG_ENABLE
							/*
							要停下来需要等待降速，一但全部悬空，电机依旧在转，切割3个相，产生的反向电流会很大
							*/
                /* 对上位机 -> 开发板 数据进判断及操作 */
                debug_set_point_range(3200,-3200,6400);         /* 控制目标调节范围(3200~-3200)并且最大步进值不超过3200 RPM */
                if(*user_setpoint == 0)
                {//上位机发送数据是0就是电机停止
                    debug_data_temp = *user_setpoint;
										//当电机速度小于230才停止
                    if(abs((int)g_bldc_motor1.speed) <= 230)
                    {
                        pid_init();                             /* 初始化PID*/
                        bldc_ctrl(MOTOR_1,g_bldc_motor1.dir,0);
                        debug_switch = 1;                       /* 标记此状态 后面将重新启动*/
                    }
                }
                else if(*user_setpoint < 0)                     /* 上位机指令欲切换旋转方向*/
                {//电机需要反转
                    debug_data_temp = *user_setpoint;
										//如果电机正处于正转，则不能马上停
                    if(g_bldc_motor1.dir == CW&&g_bldc_motor1.speed != 0)/* 当前状态不能立刻切换 */
                    {
                        /* 过零方式启动后PID调节保持有速度，所以先降速至PID默认的最低速后再停止 */
                        *user_setpoint = 210;//先减速
												//当速度小于230时才反转电机
                        if(abs((int)g_bldc_motor1.speed) <= 230)
                        {
                            pid_init();                         /* 初始化PID */
                            bldc_ctrl(MOTOR_1,CCW,0);
                            *user_setpoint = debug_data_temp;   /* 重新保持上位机指令要求 */
                            debug_switch = 1;                   /* 标记此状态 后面将重新启动 */
                        }
                    }
                }
                else if(*user_setpoint > 0)
                {//同理
                    debug_data_temp = *user_setpoint;
                    if(g_bldc_motor1.dir == CCW && g_bldc_motor1.speed != 0)/* 当前状态不能立刻切换 */
                    {
                        /* 过零方式启动后PID调节保持有速度，所以先降速至PID默认的最低速后再停止 */
                        *user_setpoint = -210;
                        if(abs((int)g_bldc_motor1.speed) <= 230)
                        {
                            pid_init();                         /* 初始化PID */
                            bldc_ctrl(MOTOR_1,CW,0);
                            *user_setpoint = debug_data_temp;   /* 重新保持上位机指令要求 */
                            debug_switch = 1;                   /* 标记此状态 后面将重新启动 */
                        }
                    }
                }
#endif                               
        /******************************* PID计算 *******************************/
                if(debug_switch == 0)
                {
                    /* 速度环 */
                    temp_pwm1 = increment_pid_ctrl(&g_speed_pid,g_bldc_motor1.speed);   /* 速度环的PID计算 */
                    FirstOrderRC_LPF(motor_pwm_s,temp_pwm1,0.25);//一阶低通滤波
									
                    if(motor_pwm_s < 0)
                    {
                        /*加速启动速度*/
											//因为电机需要一个最小速度，否则根本动不了
                        if(motor_pwm_s >= -600)
                            motor_pwm_s = -600;
                            motor_pwm_sl = -motor_pwm_s;
                    }
                    else
                    {
                        if(motor_pwm_s <= 600)
                            motor_pwm_s = 600;
                            motor_pwm_sl = motor_pwm_s;
                    }
                    *user_setpoint = debug_data_temp;                   /* 重新保持上位机指令要求 */
                    pid_s_count = 0;
                }
            }
            if(debug_switch == 0)
            {
                /* 电流环 */
                if(pid_c_count > 1)
                {
                    /* 换向尖峰电流大于设定的电流值将导致PID调节转至电流环调节 速度环无法起作用，转速无法调节 */
                    if(adc_amp_bus > (g_current_pid.SetPoint - 20)) /* 如果电流值接近目标值，有可能是尖峰电流 */
                    {
                        cf_count++;                                 
                        if(cf_count > 4)                            /* 滤除换向尖峰电流的影响 */
                        {
                            cf_count = 0;
                            temp_pwm2 = increment_pid_ctrl(&g_current_pid,adc_amp_bus); /* 滤波后的电流进行PID运算 */
                            FirstOrderRC_LPF(motor_pwm_c,temp_pwm2,0.085);              /* 一阶数字滤波 滤波系数0.085 */
                        }
                    }
                    else                                            /* 正常电流进行电流环PID运算 */
                    {
                        cf_count = 0;
                        temp_pwm2 = increment_pid_ctrl(&g_current_pid,adc_amp_bus);
                        FirstOrderRC_LPF(motor_pwm_c,temp_pwm2,0.085);
                    }
                    pid_c_count = 0;
                }
								//记住它这里用的是并联的双环控制
								
                /* 电流环输出值大于速度环输出则使用速度环调节 */
                if(motor_pwm_c > motor_pwm_sl)
                {//电流环大于速度环，则表明速度小，按照速度来走
                    g_bldc_motor1.pwm_duty = motor_pwm_sl;
                    if(motor_pwm_s < 0)                             /* 正反转积分控制，方向 */
                        g_current_pid.Ui = -g_speed_pid.Ui;//这是为了同步两者，
										//如果不这样的话，速度环到达了，但电流环还没达到，就会打架，所以同步两者
                    else
                        g_current_pid.Ui = g_speed_pid.Ui;
                }
                else  /* 速度环输出值大于电流环输出则使用电流环调节 */
                {
                    g_bldc_motor1.pwm_duty = motor_pwm_c;
                    if(g_bldc_motor1.dir == CCW)
                        g_speed_pid.Ui = -g_current_pid.Ui;
                    else
                        g_speed_pid.Ui = g_current_pid.Ui;
                }
            }                   
        }
        /* 上位机 -> 开发板 方向变化处理 */
        if(debug_switch == 1 && abs((int)(*user_setpoint)) >= 400)
        {//debug_switch=1表示准备好重新启动了，而目标转速要大于400转才启动
					
            if(*user_setpoint > 0)
                g_bldc_motor1.dir = CW;
            else
                g_bldc_motor1.dir = CCW;
						//方向选择
						
            start_motor1();                 /* 开启运行 */
            g_zero_ctr_status = 0;//重新定位
            g_bldc_motor1.run_flag = RUN;   /* 标记运行 */
            debug_switch = 0;//重启完成
        }

#endif
    }
		
		
		
		
		
    if(htim->Instance == TIM6)
    {
        /******************************* 采集电机停机状态下的偏置电压 *******************************/
        times_count++;
        if(g_bldc_motor1.run_flag == STOP)
        {//电机处于静止状态
					
            uint8_t i;
            uint32_t avg[3] = {0,0,0};
            adc_amp_offset[0][adc_amp_offset_p] = g_adc_val[2];         /* 得到还未开始运动时三相的基准电压 */
            adc_amp_offset[1][adc_amp_offset_p] = g_adc_val[3];         
            adc_amp_offset[2][adc_amp_offset_p] = g_adc_val[4];         
						adc_amp_offset_p ++;
						
						//如果偏移大于最大值，则清零
            NUM_CLEAR(adc_amp_offset_p,ADC_AMP_OFFSET_TIMES);           /* 最大采集ADC_AMP_OFFSET_TIMES次，超过即从0开始继续采集 */
            
						for(i=0; i<ADC_AMP_OFFSET_TIMES; i++)                       /* 将采集的每个通道值累加 */
            {
                avg[0] += adc_amp_offset[0][i];
                avg[1] += adc_amp_offset[1][i];
                avg[2] += adc_amp_offset[2][i];
            }
            for(i=0; i<3; i++)
            {
                avg[i] /= ADC_AMP_OFFSET_TIMES;                         /* 取平均即软件滤波 */
                adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];       /* 得到还未开始运动时的基准电压 */
            }
						//会在adc_amp_offset的最后一位放入滤波过的值
        }
				
				
        /******************************* 定时判断电机是否发生堵塞 *******************************/
        if(times_count == SMAPLSE_PID_SPEED)//超时了
        {
#if (LOCK_TAC == 2)//2表示开启堵转自动重启功能
					
            if(g_bldc_motor1.locked_rotor == 1)   /* 到达一定速度后可进入闭环控制 */
            {//如果电机堵转了
							
                clc++;
                if(clc > 50)                      /* 延迟2s后重新启动 */
                {
                    clc = 0;
                    pid_init();//PID清零
                    stop_motor1();//停止电机
                    g_speed_pid.SetPoint = 400.0; /* 400PRM */
                    g_bldc_motor1.dir = CW;
									
                    g_bldc_motor1.pwm_duty = 600; /* 加速启动速度 */
                  
										g_bldc_motor1.run_flag = RUN; /* 开启运行 */
                    start_motor1();               /* 运行电机 */
                    g_bldc_motor1.locked_rotor = 0;//设置为不堵转
                    g_zero_ctr_status = 0;        /* 堵塞状态需要重新定位初始位置，重新定位 */
                }
            }
#endif
            times_count = 0;
        }
    }
}

#endif


