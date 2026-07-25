#include "bldc_timedm.h"
#include "bldcbm.h"
#include "pid.h"
#include "adc.h"


/******************************** 定时器配置句柄 定义 ***********************************/

TIM_HandleTypeDef g_atimx_handle;           /* 定时器x句柄 */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;   /* 定时器输出句柄 */

/******************************** 定义全局变量 ************************************/

extern _bldc_obj g_bldc_motor1;
extern PID_TypeDef  g_speed_pid;            /* 位置PID参数结构体 */

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
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* TIMX 时钟使能 */


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
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* 端口复用 */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH2_GPIO_PIN;    
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH3_GPIO_PIN;       
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);
    }
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

/******************************************基本定时器初始化**********************************************************/
TIM_HandleTypeDef timx_handler;         /* 定时器参数句柄 */


/**
 * @brief       基本定时器TIMX定时中断初始化函数
 * @note
 *              基本定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              基本定时器的时钟为APB1时钟的2倍, 而APB1为42 所以定时器时钟 = 84MHZ
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;                      /* 基本定时器X */
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
        BTIM_TIMX_INT_CLK_ENABLE();                     /* 使能TIM时钟*/
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
    HAL_TIM_IRQHandler(&timx_handler);  /*定时器回调函数*/
}


/***********************************************定时器中断回调函数***********************************************/
static uint8_t pid_s_count = 0;         /* 速度环运行标志 */
static uint8_t pid_c_count = 0;         /* 电流环运行标志 */
static uint8_t cf_count = 0;            /* 定时器时间记录 */
int32_t temp_pwm1 = 0;                  /* 存放速度环的PID计算结果 */
int32_t temp_pwm2 = 0;                  /* 存放电流环的PID计算结果 */

/* 将PID期望值进行一阶滤波后存放至以下变量 */
int32_t motor_pwm_s = 0;
int32_t motor_pwm_c = 0;
int32_t motor_pwm_sl= 0;

/* 停机状态下电流采集使用 */
#define ADC_AMP_OFFSET_TIMES 50    
uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1];
uint8_t adc_amp_offset_p = 0;
int16_t adc_amp[3];
volatile uint16_t adc_val_m1[ADC_CH_NUM];
int16_t adc_amp_un[3];
float adc_amp_bus = 0.0f;

float debug_data_temp = 0.0f;
float *user_setpoint = (float*)(&g_speed_pid.SetPoint); /* 目标值赋值 */
uint8_t clc = 0;                                        /* 等待时间进入堵塞控制时间 */
/**
 * @brief       定时器中断回调
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t i;
    uint8_t bldc_dir=0;
    static uint8_t times_count=0;                       /* 定时器时间记录 */
    int16_t temp_speed=0;                               /* 临时速度存储 */
    if(htim->Instance == ATIM_TIMX_PWM)                 /* 55us */
    {
        if(g_bldc_motor1.run_flag == RUN)
        {
            /******************************* 三相电流计算 *******************************/
            for(i=0; i<3; i++)
            {
                adc_val_m1[i] = g_adc_val[i+2];         /* UVW三相ADC通道 */
                adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];

               if(adc_amp[i] >= 0)                      /* 去除反电动势引起的负电流数据 */
               adc_amp_un[i] = adc_amp[i];
            }
            /* 运算母线电流（母线电流为任意两个有开关动作的相电流之和）*/
            if(g_bldc_motor1.step_sta == 0x05)
            {
                adc_amp_bus = (adc_amp_un[0] + adc_amp_un[1])*ADC2CURT; /* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x01)
            {
                adc_amp_bus = (adc_amp_un[0] + adc_amp_un[2])*ADC2CURT; /* UW */
            }
            else if(g_bldc_motor1.step_sta == 0x03)
            {
                adc_amp_bus = (adc_amp_un[1]+ adc_amp_un[2])*ADC2CURT;  /* VW */
            }
            else if(g_bldc_motor1.step_sta == 0x02)
            {
                adc_amp_bus = (adc_amp_un[0]+ adc_amp_un[1])*ADC2CURT;  /* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x06)
            {
                adc_amp_bus= (adc_amp_un[0]+ adc_amp_un[2])*ADC2CURT;   /* WU */
            }
            else if(g_bldc_motor1.step_sta == 0x04)
            {
                adc_amp_bus= (adc_amp_un[2]+ adc_amp_un[1])*ADC2CURT;   /* WV */
            }
            
            /******************************* 六步换向 *******************************/
            if(g_bldc_motor1.dir == CW)
            {//正向的话，获取用霍尔编码器读取状态1到6
                g_bldc_motor1.step_sta = hallsensor_get_state(MOTOR_1);
            }
            else
            {//反转就是7-霍尔值，倒过来
                g_bldc_motor1.step_sta = 7 - hallsensor_get_state(MOTOR_1);
            }
						
            if((g_bldc_motor1.step_sta <= 6)&&(g_bldc_motor1.step_sta >= 1))     
            {
                pfunclist_m1[g_bldc_motor1.step_sta-1]();
            }
            else    /* 编码器错误、接触不良、断开等情况 */
            {
                stop_motor1();
                g_bldc_motor1.run_flag = STOP;
            }
						
            /******************************* 速度计算 *******************************/
            g_bldc_motor1.count_j++;                /* 计算速度专用计数值 */
            g_bldc_motor1.hall_sta_edge = uemf_edge(g_bldc_motor1.hall_single_sta);
						//检测霍尔引脚是否有跳变
						
            if(g_bldc_motor1.hall_sta_edge == 0)    /* 统计单个霍尔信号的高电平时间 */
            {//高电平结束就计算速度
							
                /* 计算速度 */
                if(g_bldc_motor1.dir == CW)
                    temp_speed = (SPEED_COEFF/g_bldc_motor1.count_j);
                else
                    temp_speed = -(SPEED_COEFF/g_bldc_motor1.count_j);
								
                FirstOrderRC_LPF(g_bldc_motor1.speed,temp_speed,0.2379);
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 1)//表名N级开始进入霍尔元件，也就是高电平开始
            {
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 2)            /* 霍尔值一直不变代表未换向 */
            {
                g_bldc_motor1.no_single++;                  /* 不换相时间累计 超时则判定速度为0 */
                if(g_bldc_motor1.no_single > 15000)
                {
                    g_bldc_motor1.no_single = 0;
                    g_bldc_motor1.speed = 0;                /* 超时换向 判定为停止 速度为0 */
                }
            }
            /******************************* 位置记录以及堵转标记 *******************************/
            if(g_bldc_motor1.step_last != g_bldc_motor1.step_sta)
            {
                g_bldc_motor1.hall_keep_t = 0;
                bldc_dir = check_hall_dir(&g_bldc_motor1);//检测旋转方向
							
                if(bldc_dir == CCW)
                {
                    g_bldc_motor1.pos -= 1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor1.pos += 1;
                }
                g_bldc_motor1.step_last = g_bldc_motor1.step_sta;
            }
            else if(g_bldc_motor1.run_flag == RUN)          /* 运行且霍尔保持时 */
            {
                g_bldc_motor1.hall_keep_t++;                /* 换向一次所需计数值（时间） 单位1/18k */
                if(g_bldc_motor1.hall_keep_t > 15000)       /* 堵转 */
                {
                    g_bldc_motor1.hall_keep_t = 0;
#if LOCK_TAC
                    stop_motor1();
                    g_bldc_motor1.run_flag = STOP;;         /* 标记停机 */
                    g_bldc_motor1.pwm_duty = 0;
#endif
                    g_bldc_motor1.locked_rotor = 1;         /* 标记堵转 */
                }
            }
        /******************************* PID控制 *******************************/
        if(g_bldc_motor1.run_flag == RUN)                   /* 进入PID闭环控制 */
        {
            pid_c_count++;
            pid_s_count++;

            if(pid_s_count > 2)
            {
/* 开启上位机调试 在PID执行之前对调节数值进行预先判断 */
#if DEBUG_ENABLE 
                debug_set_point_range(3000,-3000,6000);     /* 控制目标调节范围(3000~-3000)并且最大步进值不超过6000 RPM */
                debug_data_temp = *user_setpoint;
                if(*user_setpoint < 0)                      /* 上位机指令欲切换旋转方向 */
                {
                    /* 为简化控制逻辑 只以电机状态作为判依据（同步）*/
                    if(g_bldc_motor1.speed == 0)
                    {
                        g_bldc_motor1.dir = CCW;
                    }
                    else if(g_bldc_motor1.dir == CW && g_bldc_motor1.speed != 0)/* 当前状态不能立刻切换 */
                    {
                        *user_setpoint = 0;
                    }
                }
                else if(*user_setpoint > 0)
                {
                    if(g_bldc_motor1.speed == 0)
                    {
                        g_bldc_motor1.dir = CW;
                    }
                    else if(g_bldc_motor1.dir == CCW && g_bldc_motor1.speed != 0)/* 当前状态不能立刻切换 */
                    {
                        *user_setpoint = 0;
                    }
                }
#endif
         /******************************* PID计算 *******************************/
                temp_pwm1 = increment_pid_ctrl(&g_speed_pid,g_bldc_motor1.speed);
                FirstOrderRC_LPF(motor_pwm_s,temp_pwm1,0.085);
                if(motor_pwm_s < 0)
                {
                    motor_pwm_sl = -motor_pwm_s;
                }
                else
                {
                    motor_pwm_sl = motor_pwm_s;
                }

                *user_setpoint = debug_data_temp;   /* 重新保持上位机指令要求 */
                pid_s_count = 0;
            }

            if(pid_c_count > 1)                     /* 电流环 */
            {
                /* 换向尖峰电流大于设定的电流值将导致PID调节转至电流环调节 速度环无法起作用，转速无法调节 */
                if(adc_amp_bus > (g_current_pid.SetPoint - 20))
                {
                    cf_count++;                     /* 滤除换向尖峰电流的影响 */
                    if(cf_count > 4)
                    {
                        cf_count = 0;
                        temp_pwm2 = increment_pid_ctrl(&g_current_pid,adc_amp_bus);
                        FirstOrderRC_LPF(motor_pwm_c,temp_pwm2,0.085);/* 一阶数字滤波 滤波系数0.08 */
                    }
                }
                else
                {
                    cf_count = 0;
                    temp_pwm2 = increment_pid_ctrl(&g_current_pid,adc_amp_bus);
                    FirstOrderRC_LPF(motor_pwm_c,temp_pwm2,0.085);
                }
                pid_c_count = 0;
            }
            /* 电流环输出值大于速度环输出则使用速度环调节 */
            if(motor_pwm_c > motor_pwm_sl)
            {
                g_bldc_motor1.pwm_duty = motor_pwm_sl;
                if(motor_pwm_s < 0)                     /* 正反转积分控制 */
                    g_current_pid.Ui = -g_speed_pid.Ui;
                else
                    g_current_pid.Ui = g_speed_pid.Ui;
            }
            else  /* 速度环输出值大于电流环输出则使用电流环调节 */
            {
                g_bldc_motor1.pwm_duty = motor_pwm_c;
                if(motor_pwm_s < 0)
                    g_speed_pid.Ui = -g_current_pid.Ui;
                else
                    g_speed_pid.Ui = g_current_pid.Ui;
            }
        }
        }
    }
    if(htim->Instance == TIM6)
    {
        /******************************* 采集电机停机状态下的偏置电压 *******************************/
        times_count++;
        if(g_bldc_motor1.run_flag == STOP)
        {
            uint8_t i;
            uint32_t avg[3] = {0,0,0};
            adc_amp_offset[0][adc_amp_offset_p] = g_adc_val[2];     /* 得到还未开始运动时三相的基准电压 */
            adc_amp_offset[1][adc_amp_offset_p] = g_adc_val[3];
            adc_amp_offset[2][adc_amp_offset_p] = g_adc_val[4];

            adc_amp_offset_p++;
            NUM_CLEAR(adc_amp_offset_p,ADC_AMP_OFFSET_TIMES);       /* 最大采集ADC_AMP_OFFSET_TIMES次，超过即从0开始继续采集 */
            for(i = 0; i < ADC_AMP_OFFSET_TIMES; i++)               /* 将采集的每个通道值累加 */
            {
                avg[0] += adc_amp_offset[0][i];
                avg[1] += adc_amp_offset[1][i];
                avg[2] += adc_amp_offset[2][i];
            }
            for(i = 0; i < 3; i++)                                  /* 取平均即软件滤波 */
            {
                avg[i] /= ADC_AMP_OFFSET_TIMES;
                adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];   /* 得到还未开始运动时的基准电压 */
            }
        }
        /******************************* 定时判断电机是否发生堵塞 *******************************/
        if(times_count == SMAPLSE_PID_SPEED)
        {
#if (LOCK_TAC == 2)
            if(g_bldc_motor1.locked_rotor == 1)         /* 堵转处理，当到达一定速度后可进入闭环控制 */
            {
                clc++;
                if(clc > 50)                            /* 延迟2s后重新启动 */
                {
#if DEBUG_ENABLE /*开启调试*/
                    debug_send_motorstate(RUN_STATE);   /* 电机运行*/
#endif
                    clc = 0;
                    pid_init();
                    stop_motor1();
                    g_speed_pid.SetPoint = 400.0;       /* 400PRM */
                    g_bldc_motor1.pwm_duty = 500;       /* 加速启动速度 */
                    g_bldc_motor1.run_flag = RUN;       /* 开启运行 */
                    start_motor1();                     /* 运行电机 */
                    g_bldc_motor1.locked_rotor = 0;
                }
            }
#endif
            times_count = 0;
        }

    }
}


