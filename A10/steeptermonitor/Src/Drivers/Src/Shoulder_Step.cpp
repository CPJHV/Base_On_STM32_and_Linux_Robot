#include "Shoulder_Step.h"
#include "math.h"
#include "pid.h"   /* 必须包含PID头文件 */
#include "BigArm_Step.h"
#include "Forearm_Step.h"
extern Shoulder_Step sh_step;

uint32_t g_count_val = 0;

/**
 * @brief 高级定时器OC翻转模式初始化，用于产生步进电机PUL脉冲
 * @param arr 自动重装载值
 * @param psc 预分频系数
 * @note 模式：TIM_OCMODE_TOGGLE，比较匹配时硬件自动翻转IO电平
 */
void Shoulder_Step::atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    PUL_TIMX_PWM_CHY_CLK_ENABLE();  // 开启定时器时钟

    g_atimx_handle.Instance = PUL_TIMX_PWM;
    g_atimx_handle.Init.Prescaler = psc;
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;   // 向上计数
    g_atimx_handle.Init.Period = arr;
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // 关闭ARR预装载
    g_atimx_handle.Init.RepetitionCounter = 0; // 高级定时器重复计数器关闭
    HAL_TIM_OC_Init(&this->g_atimx_handle);

    // 通道配置：比较输出翻转模式
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_TOGGLE; // 匹配时电平翻转
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCPOLARITY_LOW;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_OC_ConfigChannel(&this->g_atimx_handle, &this->g_atimx_oc_chy_handle, PUL_TIMX_PWM_CH1);

    HAL_TIM_Base_Start(&this->g_atimx_handle); // 启动定时器计数器（CNT开始计数）
}

/**
 * @brief 定时器OC底层硬件初始化（GPIO、中断），HAL库标准Msp回调
 */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == PUL_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        PUL_TIMX_PWM_CHY_CLK_ENABLE();
        PUL_TIMX_PWM_CH1_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = PUL_TIMX_PWM_CH1_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;    // 复用推挽输出
        gpio_init_struct.Pull = GPIO_PULLUP;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = PUL_TIMX_PWM_CHY_GPIO_AF;
        HAL_GPIO_Init(PUL_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);

        // 设置定时器中断优先级并使能
        HAL_NVIC_SetPriority(PUL_TIMX_INT_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(PUL_TIMX_INT_IRQn);
    }
}

/**
 * @brief 定时器中断入口函数，交给HAL库统一分发
 */
void PUL_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&sh_step.g_atimx_handle);
}

/**
 * @brief 启动/停止步进脉冲输出
 * @param para 1=启动脉冲；0=停止脉冲
 */
void Shoulder_Step::change_StarorStop_Monitor(uint8_t para)
{
    if(para == 1)
        HAL_TIM_OC_Start_IT(&this->g_atimx_handle, PUL_TIMX_PWM_CH1);  // 开启通道+比较中断，开始翻转
    else
        HAL_TIM_OC_Stop_IT(&this->g_atimx_handle, PUL_TIMX_PWM_CH1);   // 关闭脉冲输出
}

/**
 * @brief 角度换算脉冲，简易定角度运动（旧接口，不支持S曲线）
 * @param angle 目标转动角度
 * @param dir 转向
 * @note MAX_STEP_ANGLE：单脉冲对应机械角度
 */
void Shoulder_Step::set_steper_angle(uint16_t angle, DIR_STATE dir)
{
    this->pulse_cout = angle / MAX_STEP_ANGLE;
    if(this->pulse_cout == 0){
        this->change_StarorStop_Monitor(0);
    }else{
        this->change_StarorStop_Monitor(1);
        this->Dir_State = dir;
    }
}

/**
 * @brief 步进外设GPIO初始化：DIR方向、EN使能、限位、报警、EZ信号
 */
void Shoulder_Step::Gpio_Init_other(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    DIR_GPIO_CLK_ENABLE();
    EN_GPIO_CLK_ENABLE();
    PEND_GPIO_CLK_ENABLE();
    ALM_GPIO_CLK_ENABLE();
    EZ_GPIO_CLK_ENABLE();

    // DIR、EN：推挽输出
    gpio_init_struct.Pin = DIR_GPIO_PIN | EN_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DIR_GPIO_PORT, &gpio_init_struct);

    // PEND限位、ALM报警：上拉输入
    gpio_init_struct.Pin = PEND_GPIO_PIN | ALM_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PEND_GPIO_PORT, &gpio_init_struct);

    // EZ零位信号输出
    gpio_init_struct.Pin = EZ_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(EZ_GPIO_PORT, &gpio_init_struct);

    this->Dir_State = CWN;  // 默认方向
}

/**
 * @brief 设置步进电机转向
 * @param para 0/CW正向，1/CWN反向
 */
void Shoulder_Step::Set_Dir(uint8_t para)
{
    if(para == 0){
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_RESET);
        this->Dir_State = CW;
    }else{
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_SET);
        this->Dir_State = CWN;
    }
}

/**
 * @brief 步进驱动器使能控制
 * @param para 0=脱机失能，1=使能锁定
 */
void Shoulder_Step::Set_ENABLE(uint8_t para)
{
    if(para == 0)
        HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, GPIO_PIN_SET);
}

/**
 * @brief EZ零位信号输出控制
 */
void Shoulder_Step::Set_EZ(uint8_t para)
{
    if(para == 0)
        HAL_GPIO_WritePin(EZ_GPIO_PORT, EZ_GPIO_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(EZ_GPIO_PORT, EZ_GPIO_PIN, GPIO_PIN_SET);
}

//=============================================================================
//====================== S形算法 无 malloc 版 ========================
//=============================================================================
#define MAX_SPEED_TABLE_SIZE 1024
static float s_speed_buffer[MAX_SPEED_TABLE_SIZE]; // 静态速度表缓冲区，不使用堆内存

/**
 * @brief S曲线速度表计算函数，生成加速/减速速度序列
 * @param vo 起始速度
 * @param vt 目标速度
 * @param time 加速/减速总时间
 * @return TRUE成功；FALSE超出表格长度失败
 * @note 输出速度表存入全局静态数组s_speed_buffer
 */
uint8_t Shoulder_Step::calc_speed(int32_t vo, int32_t vt, float time)
{
    uint8_t is_dec = FALSE;
    int32_t i = 0;
    int32_t vm =0;
    int32_t inc_acc_stp = 0;
    int32_t dec_acc_stp = 0;
    int32_t accel_step = 0;
    float jerk = 0;
    float ti = 0;
    float sum_t = 0;
    float delta_v = 0;
    float ti_cube = 0;
    float *velocity_tab = s_speed_buffer;

    // 判断是否为减速运动，交换起始终末速度
    if(vo > vt ){
        is_dec = TRUE;
        this->s_calc_speed.vo = ROUNDPS_2_STEPPS(vt);
        this->s_calc_speed.vt = ROUNDPS_2_STEPPS(vo);
    }else{
        is_dec = FALSE;
        this->s_calc_speed.vo = ROUNDPS_2_STEPPS(vo);
        this->s_calc_speed.vt = ROUNDPS_2_STEPPS(vt);
    }

    time = ACCEL_TIME(time);//1/2时间
    vm =  (this->s_calc_speed.vo + this->s_calc_speed.vt) / 2 ;
    jerk = fabs(2.0f * (vm - this->s_calc_speed.vo) /  (time * time)); // 加加速度jerk
    inc_acc_stp = (int32_t)(this->s_calc_speed.vo * time + INCACCELSTEP(jerk,time));
    dec_acc_stp = (int32_t)((this->s_calc_speed.vt + this->s_calc_speed.vo) * time - inc_acc_stp);
    accel_step = dec_acc_stp + inc_acc_stp; // 本段加速总脉冲步数

    if(accel_step >= MAX_SPEED_TABLE_SIZE) return FALSE; // 超出缓冲区，失败

    ti_cube  = 6.0f * 1.0f / jerk;//这里求单位脉冲的时间，所以为1f
    ti = pow(ti_cube,(1 / 3.0f));
    sum_t = ti;
    delta_v = 0.5f * jerk * pow(sum_t,2);
    velocity_tab[0] = this->s_calc_speed.vo + delta_v;

    if( velocity_tab[0] <= SPEED_MIN )
        velocity_tab[0] = SPEED_MIN; // 限制最低速度，防止丢步

    // 逐点生成S曲线速度表
    for(i = 1; i < accel_step; i++)
    {
        ti = 1.0f / velocity_tab[i-1];//v是表示单位时间内输出的脉冲
			//1/v是输出这个脉冲要多长时间
        if( i < inc_acc_stp)
        {//处于加加速
            sum_t += ti;
            delta_v = 0.5f * jerk * pow(sum_t,2);
            velocity_tab[i] = this->s_calc_speed.vo + delta_v;
            if(i == inc_acc_stp - 1)
                sum_t  = fabs(sum_t - time );
        }
        else
        {//处于减减速
            sum_t += ti;
            delta_v = 0.5f * jerk * pow(fabs( time - sum_t),2);
            velocity_tab[i] = this->s_calc_speed.vt - delta_v;
            if(velocity_tab[i] >= this->s_calc_speed.vt)
            {
                accel_step = i;
                break;
            }
        }
    }

    // 如果是减速段：反转速度表（加速表倒序=减速表）
    if(is_dec == TRUE)
    {
        float tmp_Speed = 0;
        for(i = 0; i< (accel_step / 2); i++)
        {
            tmp_Speed = velocity_tab[i];
            velocity_tab[i] = velocity_tab[accel_step-1 - i];
            velocity_tab[accel_step-1 - i] = tmp_Speed;
        }
        this->s_calc_speed.decel_tab = velocity_tab;
        this->s_calc_speed.decel_step = accel_step;
    }
    else
    {
        this->s_calc_speed.accel_tab = velocity_tab;
        this->s_calc_speed.accel_step = accel_step;
    }
    return TRUE;
}

/**
 * @brief 相对位移运动接口，带S曲线加减速
 * @param vo 初始速度
 * @param vt 最大匀速速度
 * @param AcTime 加速时间
 * @param DeTime 减速时间
 * @param step 目标脉冲，正数正向、负数反向
 * @note 运动距离必须大于加速+减速总步数，否则直接返回不运行
 */
void Shoulder_Step::stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step)
{
    // 生成加速表、减速表
    if(this->calc_speed(vo,vt,AcTime) == FALSE) return;
    if(this->calc_speed(vt,vo,DeTime) == FALSE) return;

    // 判断方向
    if(step < 0)
    {
        step = -step;
        this->Set_Dir(CW);
    }
    else
    {
        this->Set_Dir(CWN);
    }

    // 判断行程是否足够容纳加速段+减速段；不足则拒绝启动
    if(step >= (this->s_calc_speed.decel_step + this->s_calc_speed.accel_step) )
    {
        this->s_calc_speed.step = step;
        this->s_calc_speed.dec_point = this->s_calc_speed.step - this->s_calc_speed.decel_step; // 到达该脉冲位置开始减速
    }
    else
    {
        return;
    }

    // 初始化运动状态机参数
    this->s_calc_speed.step_pos = 0;
    this->motor_data = STATE_ACCEL; // 初始进入加速状态
    this->s_calc_speed.ptr = this->s_calc_speed.accel_tab;
    this->g_toggle_pulse  = (uint32_t)(T1_FREQ/(*this->s_calc_speed.ptr));
		//f是每秒cnt递增多少，而速度表里的是每秒输出多少脉冲，相除以就是每个脉冲需要cnt增长多少；
    this->s_calc_speed.ptr++;

    __HAL_TIM_SET_COUNTER(&this->g_atimx_handle,0);
    __HAL_TIM_SET_COMPARE(&this->g_atimx_handle,TIM_CHANNEL_1,(uint16_t)(this->g_toggle_pulse/2));
		//因为一个脉冲需要cnt递增到n，而这个输出口是翻转的，因此要/2，来n/2就要去翻转
    HAL_TIM_OC_Start_IT(&this->g_atimx_handle,TIM_CHANNEL_1); // 启动脉冲输出
    this->Set_ENABLE(1); // 驱动器使能
}

/**
 * @brief 定时器比较匹配回调函数【脉冲生成核心】
 * @note 每次IO电平翻转触发一次回调；翻转2次=1个完整步进脉冲
 *       同时支持肩关节、大臂、小臂三路独立定时器脉冲控制
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t Tim_Count = 0;
    uint32_t tmp = 0;
    float Tim_Pulse = 0;

    // ======================== 肩关节 sh_step ========================
    if(htim->Instance == PUL_TIMX_PWM )
    {
        sh_step.i++;
        // i==2：完成两次电平翻转，产生1个完整PUL脉冲
        if(sh_step.i == 2)
        {
            sh_step.i = 0;
            sh_step.g_step_pos ++; // 全局绝对脉冲位置计数

            if((sh_step.motor_data != STATE_IDLE) && (sh_step.motor_data != STATE_STOP))
            {
                sh_step.s_calc_speed.step_pos ++; // 本次运动相对脉冲计数
            }

            // 运动状态机：加速/匀速/减速/停止
            switch(sh_step.motor_data)
            {
                case STATE_ACCEL:
                    sh_step.add_pulse_count++;
                    Tim_Pulse = T1_FREQ / (*sh_step.s_calc_speed.ptr);
                    sh_step.s_calc_speed.ptr++;
                    sh_step.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    // 加速脉冲走完 → 切换匀速
                    if(sh_step.s_calc_speed.step_pos >= sh_step.s_calc_speed.accel_step)
                    {
                        sh_step.motor_data = STATE_AVESPEED;
                    }
                    break;

                case STATE_DECEL:
                    sh_step.add_pulse_count++;
                    Tim_Pulse = T1_FREQ / (*sh_step.s_calc_speed.ptr);
                    sh_step.s_calc_speed.ptr++;
                    sh_step.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    // 全部脉冲发送完成 → 进入停止状态
                    if(sh_step.s_calc_speed.step_pos >= sh_step.s_calc_speed.step )
                    {
                        sh_step.motor_data = STATE_STOP;
                    }
                    break;

                case STATE_AVESPEED:
                    sh_step.add_pulse_count++;
                    Tim_Pulse  = T1_FREQ / sh_step.s_calc_speed.vt;
                    sh_step.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    // 到达减速点，切换减速状态
                    if(sh_step.s_calc_speed.step_pos >= sh_step.s_calc_speed.dec_point )
                    {
                        sh_step.s_calc_speed.ptr = sh_step.s_calc_speed.decel_tab;
                        sh_step.motor_data = STATE_DECEL;
                    }
                    break;

                case STATE_STOP:
                    HAL_TIM_OC_Stop_IT(&sh_step.g_atimx_handle, PUL_TIMX_PWM_CH1);
                    sh_step.motor_data = STATE_IDLE;
                    break;

                case STATE_IDLE:
                    break;
            }
        }

        // 读取当前定时器计数值，更新下一次比较匹配点（相对延时模式）
        Tim_Count = __HAL_TIM_GET_COUNTER(&sh_step.g_atimx_handle);
        tmp = 0xFFFF & (Tim_Count + sh_step.g_toggle_pulse); //16位截断防溢出
        __HAL_TIM_SET_COMPARE(&sh_step.g_atimx_handle,PUL_TIMX_PWM_CH1,tmp);
    }

    // ======================== 大臂 big_arm ========================
    else if(htim->Instance == BIGARM_TIMX_PWM )
    {
        big_arm.i++;
        if(big_arm.i == 2)
        {
            big_arm.i = 0;
            big_arm.g_step_pos ++;

            if((big_arm.motor_data != STATE_IDLE) && (big_arm.motor_data != STATE_STOP))
            {
                big_arm.s_calc_speed.step_pos ++;
            }

            switch(big_arm.motor_data)
            {
                case STATE_ACCEL:
                    big_arm.add_pulse_count++;
                    Tim_Pulse = T1_FREQ / (*big_arm.s_calc_speed.ptr);
                    big_arm.s_calc_speed.ptr++;
                    big_arm.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    if(big_arm.s_calc_speed.step_pos >= big_arm.s_calc_speed.accel_step)
                    {
                        big_arm.motor_data = STATE_AVESPEED;
                    }
                    break;

                case STATE_DECEL:
                    big_arm.add_pulse_count++;
                    Tim_Pulse = T1_FREQ / (*big_arm.s_calc_speed.ptr);
                    big_arm.s_calc_speed.ptr++;
                    big_arm.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    if(big_arm.s_calc_speed.step_pos >= big_arm.s_calc_speed.step )
                    {
                        big_arm.motor_data = STATE_STOP;
                    }
                    break;

                case STATE_AVESPEED:
                    big_arm.add_pulse_count++;
                    Tim_Pulse  = T1_FREQ / big_arm.s_calc_speed.vt;
                    big_arm.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    if(big_arm.s_calc_speed.step_pos >= big_arm.s_calc_speed.dec_point )
                    {
                        big_arm.s_calc_speed.ptr = big_arm.s_calc_speed.decel_tab;
                        big_arm.motor_data = STATE_DECEL;
                    }
                    break;

                case STATE_STOP:
                    HAL_TIM_OC_Stop_IT(&big_arm.g_atimx_handle, BIGARM_TIMX_PWM_CH1);
                    big_arm.motor_data = STATE_IDLE;
                    break;

                case STATE_IDLE:
                    break;
            }
        }

        Tim_Count = __HAL_TIM_GET_COUNTER(&big_arm.g_atimx_handle);
        tmp = 0xFFFF & (Tim_Count + big_arm.g_toggle_pulse);
        __HAL_TIM_SET_COMPARE(&big_arm.g_atimx_handle,BIGARM_TIMX_PWM_CH1,tmp);
    }

    // ======================== 小臂 forearm ========================
    else if(htim->Instance == FOREARM_TIMX_PWM )
    {
        forearm.i++;
        if(forearm.i == 2)
        {
            forearm.i = 0;
            forearm.g_step_pos ++;

            if((forearm.motor_data != STATE_IDLE) && (forearm.motor_data != STATE_STOP))
            {
                forearm.s_calc_speed.step_pos ++;
            }

            switch(forearm.motor_data)
            {
                case STATE_ACCEL:
                    forearm.add_pulse_count++;
                    Tim_Pulse = T1_FREQ / (*forearm.s_calc_speed.ptr);
                    forearm.s_calc_speed.ptr++;
                    forearm.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    if(forearm.s_calc_speed.step_pos >= forearm.s_calc_speed.accel_step)
                    {
                        forearm.motor_data = STATE_AVESPEED;
                    }
                    break;

                case STATE_DECEL:
                    forearm.add_pulse_count++;
                    Tim_Pulse = T1_FREQ / (*forearm.s_calc_speed.ptr);
                    forearm.s_calc_speed.ptr++;
                    forearm.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    if(forearm.s_calc_speed.step_pos >= forearm.s_calc_speed.step )
                    {
                        forearm.motor_data = STATE_STOP;
                    }
                    break;

                case STATE_AVESPEED:
                    forearm.add_pulse_count++;
                    Tim_Pulse  = T1_FREQ / forearm.s_calc_speed.vt;
                    forearm.g_toggle_pulse = (uint16_t) (Tim_Pulse / 2);

                    if(forearm.s_calc_speed.step_pos >= forearm.s_calc_speed.dec_point )
                    {
                        forearm.s_calc_speed.ptr = forearm.s_calc_speed.decel_tab;
                        forearm.motor_data = STATE_DECEL;
                    }
                    break;

                case STATE_STOP:
                    HAL_TIM_OC_Stop_IT(&forearm.g_atimx_handle, FOREARM_TIMX_PWM_CH1);
                    forearm.motor_data = STATE_IDLE;
                    break;

                case STATE_IDLE:
                    break;
            }
        }

        Tim_Count = __HAL_TIM_GET_COUNTER(&forearm.g_atimx_handle);
        tmp = 0xFFFF & (Tim_Count + forearm.g_toggle_pulse);
        __HAL_TIM_SET_COMPARE(&forearm.g_atimx_handle,FOREARM_TIMX_PWM_CH1,tmp);
    }
}

//=============================================================================
//======================== 编码器部分 【追加内容，不改动原有代码】 ========================
//=============================================================================


/* 全局闭环参数 */
Motor_TypeDef  g_step_motor;
PID_TypeDef    g_location_pid;

volatile int32_t g_timx_encode_count = 0;    /* 编码器16位定时器溢出累计值 */
volatile int g_encode_now = 0, g_encode_old = 0, g_speed = 0;
volatile int32_t g_motor_pwm = 0;
uint8_t g_run_flag = 0;
uint8_t g_send_flag = 0;


//=============================================================================
// 编码器定时器初始化（正交编码器模式）
//=============================================================================
void Shoulder_Step::gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc)
{
    ENCODER_TIMX_PWM_CHY_CLK_ENABLE();
    ENCODER_GPIO_CLK_ENABLE();

    g_timx_encode_chy_handle.Instance = ENCODER_TIMX_PWM;
    g_timx_encode_chy_handle.Init.Prescaler = psc;
    g_timx_encode_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_timx_encode_chy_handle.Init.Period = 65535; //16位最大值
    g_timx_encode_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    // 正交编码器模式，TI1+TI2双边计数
    g_timx_encoder_chy_handle.EncoderMode = TIM_ENCODERMODE_TI12;
    g_timx_encoder_chy_handle.IC1Polarity = TIM_ICPOLARITY_RISING;
    g_timx_encoder_chy_handle.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    g_timx_encoder_chy_handle.IC1Prescaler = TIM_ICPSC_DIV1;
    g_timx_encoder_chy_handle.IC1Filter = 10; //滤波，抑制毛刺

    g_timx_encoder_chy_handle.IC2Polarity = TIM_ICPOLARITY_RISING;
    g_timx_encoder_chy_handle.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    g_timx_encoder_chy_handle.IC2Prescaler = TIM_ICPSC_DIV1;
    g_timx_encoder_chy_handle.IC2Filter = 10;

    HAL_TIM_Encoder_Init(&g_timx_encode_chy_handle, &g_timx_encoder_chy_handle);

    // 启动两路正交输入
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle, ENCODER_TIMX_PWM_CH1);
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle, ENCODER_TIMX_PWM_CH2);
    __HAL_TIM_CLEAR_FLAG(&g_timx_encode_chy_handle, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&g_timx_encode_chy_handle, TIM_IT_UPDATE); // 开启溢出中断
}

//=============================================================================
// 编码器GPIO底层初始化 Msp回调
//=============================================================================
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *tim_encoderHandle)
{
    if (tim_encoderHandle->Instance == ENCODER_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ENCODER_GPIO_CLK_ENABLE();

        gpio_init_struct.Pin = ENCODER_TIMX_PWM_CHEA_PIN | ENCODER_TIMX_PWM_CHEB_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init_struct.Alternate = ENCODER_GPIO_AF;

        HAL_GPIO_Init(ENCODER_TIMX_PWM_CHEA_GPIO, &gpio_init_struct);

        // 编码器溢出中断优先级配置
        HAL_NVIC_SetPriority(ENCODER_TIMX_INT_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(ENCODER_TIMX_INT_IRQn);
    }
}

//=============================================================================
// 读取编码器总绝对位置（处理16位溢出）
// @retval 带溢出补偿的总脉冲位置
//=============================================================================
int32_t gtim_get_encode(void)
{
    return (int32_t)(__HAL_TIM_GET_COUNTER(&sh_step.g_timx_encode_chy_handle) + g_timx_encode_count * 65536);
}

//=============================================================================
// 编码器定时器中断入口
//=============================================================================
void ENCODER_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&sh_step.g_timx_encode_chy_handle);
}

//=============================================================================
// 定时器更新中断回调：编码器溢出检测 + 周期位置PID闭环任务
// @note TIM6/TIM7作为20ms系统周期调度
//=============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint16_t val = 0;
    static uint16_t sum_pulse = 0;

    // 编码器定时器溢出中断：判断向上溢出/向下溢出，累加溢出计数
    if (htim->Instance == ENCODER_TIMX_PWM)
    {
        if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&sh_step.g_timx_encode_chy_handle))
            g_timx_encode_count--;
        else
            g_timx_encode_count++;
    }

    /* 20ms定时闭环调度任务 */
    if (htim->Instance == TIM6 || htim->Instance == TIM7) /* 基本定时器20ms */
    {
        g_encode_now = gtim_get_encode();

        if (val == SMAPLSE_PID_SPEED)
        {
            g_speed = g_encode_now - g_encode_old; // 20ms内编码器脉冲变化量
            sum_pulse +=(uint16_t)(g_speed >= 0 ? g_speed : -g_speed);
            g_encode_old = g_encode_now;

            /* 根据编码器实际脉冲计算真实转速 RPM */
            g_step_motor.speed = g_speed * ((1000.0f / SMAPLSE_PID_SPEED) * 60.0f) / ENCODER_SPR;

            /* 根据当前脉冲周期计算理论设定转速（脉冲发生器目标转速） */
            g_step_motor.setspeed = ((1000000.0f / (sh_step.g_toggle_pulse * 2)) / PULSE_REV) * 60.0f;

            if (g_run_flag)
            {
                g_step_motor.location = gtim_get_encode();

                /* 位置环增量PID运算，输出目标脉冲数量 */
                g_motor_pwm = increment_pid_ctrl(&g_location_pid, g_step_motor.location);

                /* 根据PID输出正负自动设置转向 */
                if (g_motor_pwm > 0)
                    g_step_motor.dir = CW;
                else
                    g_step_motor.dir = CWN;

                g_motor_pwm = (g_motor_pwm >= 0 ? g_motor_pwm : -g_motor_pwm);

                /* 调用S曲线运动函数，实现编码器位置闭环 */
                sh_step.stepmotor_move_rel(50, 300, 0.5f, 0.5f, g_motor_pwm);
                g_send_flag = 1;
            }
            val = 0;
        }
        val++;
    }
}