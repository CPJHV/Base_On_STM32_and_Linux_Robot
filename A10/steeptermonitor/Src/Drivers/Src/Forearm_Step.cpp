#include "Forearm_Step.h"
#include "math.h"

Forearm_Step forearm;

void Forearm_Step::atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    FOREARM_TIMX_PWM_CHY_CLK_ENABLE();

    g_atimx_handle.Instance = FOREARM_TIMX_PWM;
    g_atimx_handle.Init.Prescaler = psc;
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_atimx_handle.Init.Period = arr;
    g_atimx_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    g_atimx_handle.Init.RepetitionCounter = 0;
    HAL_TIM_OC_Init(&this->g_atimx_handle);

    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_TOGGLE;
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(&this->g_atimx_handle, &this->g_atimx_oc_chy_handle, FOREARM_TIMX_PWM_CH1);

    HAL_TIM_Base_Start(&this->g_atimx_handle);
}

void FOREARM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&forearm.g_atimx_handle);
}

void Forearm_Step::change_StarorStop_Monitor(uint8_t para)
{
    if(para == 1)
        HAL_TIM_OC_Start_IT(&this->g_atimx_handle, FOREARM_TIMX_PWM_CH1);
    else
        HAL_TIM_OC_Stop_IT(&this->g_atimx_handle, FOREARM_TIMX_PWM_CH1);
}

void Forearm_Step::set_steper_angle(uint16_t angle, DIR_STATE dir)
{
    this->pulse_cout = angle / MAX_STEP_ANGLE;
    if(this->pulse_cout == 0){
        this->change_StarorStop_Monitor(0);
    }else{
        this->change_StarorStop_Monitor(1);
        this->Dir_State = dir;
    }
}

void Forearm_Step::Gpio_Init_other(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FOREARM_DIR_CLK_ENABLE();
    FOREARM_EN_CLK_ENABLE();
    FOREARM_SPREAD_CLK_ENABLE();
    FOREARM_INDEX_CLK_ENABLE();
    FOREARM_DIAG_CLK_ENABLE();

    gpio_init_struct.Pin = FOREARM_DIR_GPIO_PIN | FOREARM_EN_GPIO_PIN | FOREARM_SPREAD_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FOREARM_DIR_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = FOREARM_DIAG_GPIO_PIN | FOREARM_INDEX_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(FOREARM_DIAG_GPIO_PORT, &gpio_init_struct);

    this->Dir_State = CWN;
}

void Forearm_Step::Set_Dir(uint8_t para)
{
    if(para == 0){
        HAL_GPIO_WritePin(FOREARM_DIR_GPIO_PORT, FOREARM_DIR_GPIO_PIN, GPIO_PIN_RESET);
        this->Dir_State = CW;
    }else{
        HAL_GPIO_WritePin(FOREARM_DIR_GPIO_PORT, FOREARM_DIR_GPIO_PIN, GPIO_PIN_SET);
        this->Dir_State = CWN;
    }
}

void Forearm_Step::Set_ENABLE(uint8_t para)
{
    if(para == 0)
        HAL_GPIO_WritePin(FOREARM_EN_GPIO_PORT, FOREARM_EN_GPIO_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(FOREARM_EN_GPIO_PORT, FOREARM_EN_GPIO_PIN, GPIO_PIN_SET);
}

void Forearm_Step::Set_SPREAD(uint8_t para)
{
    HAL_GPIO_WritePin(FOREARM_SPREAD_GPIO_PORT, FOREARM_SPREAD_GPIO_PIN, para ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t Forearm_Step::Get_DIAG(void)
{
    return HAL_GPIO_ReadPin(FOREARM_DIAG_GPIO_PORT, FOREARM_DIAG_GPIO_PIN);
}

uint8_t Forearm_Step::Get_INDEX(void)
{
    return HAL_GPIO_ReadPin(FOREARM_INDEX_GPIO_PORT, FOREARM_INDEX_GPIO_PIN);
}

#define MAX_SPEED_TABLE_SIZE 1024
static float s_speed_buffer[MAX_SPEED_TABLE_SIZE];

uint8_t Forearm_Step::calc_speed(int32_t vo, int32_t vt, float time)
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

    if(vo > vt ){
        is_dec = TRUE;
        this->s_calc_speed.vo = ROUNDPS_2_STEPPS(vt);
        this->s_calc_speed.vt = ROUNDPS_2_STEPPS(vo);
    }else{
        is_dec = FALSE;
        this->s_calc_speed.vo = ROUNDPS_2_STEPPS(vo);
        this->s_calc_speed.vt = ROUNDPS_2_STEPPS(vt);
    }

    time = ACCEL_TIME(time);
    vm =  (this->s_calc_speed.vo + this->s_calc_speed.vt) / 2 ;
    jerk = fabs(2.0f * (vm - this->s_calc_speed.vo) /  (time * time));
    inc_acc_stp = (int32_t)(this->s_calc_speed.vo * time + INCACCELSTEP(jerk,time));
    dec_acc_stp = (int32_t)((this->s_calc_speed.vt + this->s_calc_speed.vo) * time - inc_acc_stp);
    accel_step = dec_acc_stp + inc_acc_stp;

    if(accel_step >= MAX_SPEED_TABLE_SIZE) return FALSE;

    ti_cube  = 6.0f * 1.0f / jerk;
    ti = pow(ti_cube,(1 / 3.0f));
    sum_t = ti;
    delta_v = 0.5f * jerk * pow(sum_t,2);
    velocity_tab[0] = this->s_calc_speed.vo + delta_v;

    if( velocity_tab[0] <= SPEED_MIN )
        velocity_tab[0] = SPEED_MIN;

    for(i = 1; i < accel_step; i++)
    {
        ti = 1.0f / velocity_tab[i-1];
        if( i < inc_acc_stp)
        {
            sum_t += ti;
            delta_v = 0.5f * jerk * pow(sum_t,2);
            velocity_tab[i] = this->s_calc_speed.vo + delta_v;
            if(i == inc_acc_stp - 1)
                sum_t  = fabs(sum_t - time );
        }
        else
        {
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

void Forearm_Step::stepmotor_move_rel(int32_t vo, int32_t vt, float AcTime,float DeTime,int32_t step)
{
    if(this->calc_speed(vo,vt,AcTime) == FALSE) return;
    if(this->calc_speed(vt,vo,DeTime) == FALSE) return;

    if(step < 0)
    {
        step = -step;
        this->Set_Dir(CW);
    }
    else
    {
        this->Set_Dir(CWN);
    }

    if(step >= (this->s_calc_speed.decel_step + this->s_calc_speed.accel_step) )
    {
        this->s_calc_speed.step = step;
        this->s_calc_speed.dec_point = this->s_calc_speed.step - this->s_calc_speed.decel_step;
    }
    else
    {
        return;
    }

    this->s_calc_speed.step_pos = 0;
    this->motor_data = STATE_ACCEL;
    this->s_calc_speed.ptr = this->s_calc_speed.accel_tab;
    this->g_toggle_pulse  = (uint32_t)(T1_FREQ/(*this->s_calc_speed.ptr));
    this->s_calc_speed.ptr++;

    __HAL_TIM_SET_COUNTER(&this->g_atimx_handle,0);
    __HAL_TIM_SET_COMPARE(&this->g_atimx_handle,FOREARM_TIMX_PWM_CH1,(uint16_t)(this->g_toggle_pulse/2));
    HAL_TIM_OC_Start_IT(&this->g_atimx_handle,FOREARM_TIMX_PWM_CH1);
    this->Set_ENABLE(1);
}