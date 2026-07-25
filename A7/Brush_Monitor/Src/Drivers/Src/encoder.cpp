// encoder.cpp
#include "encoder.h"
#include "main.h"
#include "pid.h"
#include <string.h>
extern Pid_ pidx;
Encoder_ encoder(&pidx);
/******************************* 第二部分  4路电机编码器测速 ****************************************************/
// 构造初始化
Encoder_::Encoder_(Pid_* pid) : pidptr(pid)
{
    // 初始化编码器数据结构
    memset(&g_encode1, 0, sizeof(ENCODE_TypeDef));
    memset(&g_encode2, 0, sizeof(ENCODE_TypeDef));
    memset(&g_encode3, 0, sizeof(ENCODE_TypeDef));
    memset(&g_encode4, 0, sizeof(ENCODE_TypeDef));
    
    // 初始化溢出计数
    g_encode1_count = 0;
    g_encode2_count = 0;
    g_encode3_count = 0;
    g_encode4_count = 0;
    
    // 初始化运行标志
    g1_run_flag = 0;
    g2_run_flag = 0;
    g3_run_flag = 0;
    g4_run_flag = 0;
    
    // 初始化编码器句柄
    memset(&g1_tim_encode_handle, 0, sizeof(TIM_HandleTypeDef));
    memset(&g2_tim_encode_handle, 0, sizeof(TIM_HandleTypeDef));
    memset(&g3_tim_encode_handle, 0, sizeof(TIM_HandleTypeDef));
    memset(&g4_tim_encode_handle, 0, sizeof(TIM_HandleTypeDef));
    
    // 初始化基本定时器句柄
    memset(&g_pwm1_btim_handle, 0, sizeof(TIM_HandleTypeDef));
    memset(&g_pwm2_btim_handle, 0, sizeof(TIM_HandleTypeDef));
    memset(&g_pwm3_btim_handle, 0, sizeof(TIM_HandleTypeDef));
    memset(&g_pwm4_btim_handle, 0, sizeof(TIM_HandleTypeDef));
    
    // 初始化编码器配置结构
    memset(&sEncoderConfig, 0, sizeof(TIM_Encoder_InitTypeDef));
}

/* 电机1 编码器 TIM3 初始化 */
void Encoder_::pwm1_tim_encoder_init(uint16_t arr, uint16_t psc)
{
    // 初始化定时器
    g1_tim_encode_handle.Instance = PWM1_ENCODER_TIME_SOURCE;
    g1_tim_encode_handle.Init.Prescaler = psc;
    g1_tim_encode_handle.Init.Period = arr;
    g1_tim_encode_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    // 初始化捕获通道
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;//TI1+TI2模式
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;//上升沿捕获
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;//直接映射
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 10;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 10;
    
    // 初始化编码器
    HAL_TIM_Encoder_Init(&g1_tim_encode_handle, &sEncoderConfig);
    
    // 启动两个通道
    HAL_TIM_Encoder_Start(&g1_tim_encode_handle, PWM1_A_CHANNEL);
    HAL_TIM_Encoder_Start(&g1_tim_encode_handle, PWM1_B_CHANNEL);
    
    // 启动输入捕获溢出中断
    __HAL_TIM_ENABLE_IT(&g1_tim_encode_handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&g1_tim_encode_handle, TIM_IT_UPDATE);
}

/* 电机2 编码器 TIM4 初始化 */
void Encoder_::pwm2_tim_encoder_init(uint16_t arr, uint16_t psc)
{
    g2_tim_encode_handle.Instance = PWM2_ENCODER_TIME_SOURCE;
    g2_tim_encode_handle.Init.Prescaler = psc;
    g2_tim_encode_handle.Init.Period = arr;
    g2_tim_encode_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 10;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 10;
    
    HAL_TIM_Encoder_Init(&g2_tim_encode_handle, &sEncoderConfig);
    HAL_TIM_Encoder_Start(&g2_tim_encode_handle, PWM2_A_CHANNEL);
    HAL_TIM_Encoder_Start(&g2_tim_encode_handle, PWM2_B_CHANNEL);
    __HAL_TIM_ENABLE_IT(&g2_tim_encode_handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&g2_tim_encode_handle, TIM_IT_UPDATE);
}

/* 电机3 编码器 TIM2 初始化 */
void Encoder_::pwm3_tim_encoder_init(uint16_t arr, uint16_t psc)
{
    g3_tim_encode_handle.Instance = PWM3_ENCODER_TIME_SOURCE;
    g3_tim_encode_handle.Init.Prescaler = psc;
    g3_tim_encode_handle.Init.Period = arr;
    g3_tim_encode_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 10;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 10;
    
    HAL_TIM_Encoder_Init(&g3_tim_encode_handle, &sEncoderConfig);
    HAL_TIM_Encoder_Start(&g3_tim_encode_handle, PWM3_A_CHANNEL);
    HAL_TIM_Encoder_Start(&g3_tim_encode_handle, PWM3_B_CHANNEL);
    __HAL_TIM_ENABLE_IT(&g3_tim_encode_handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&g3_tim_encode_handle, TIM_IT_UPDATE);
}

/* 电机4 编码器 TIM5 初始化 */
void Encoder_::pwm4_tim_encoder_init(uint16_t arr, uint16_t psc)
{
    g4_tim_encode_handle.Instance = PWM4_ENCODER_TIME_SOURCE;
    g4_tim_encode_handle.Init.Prescaler = psc;
    g4_tim_encode_handle.Init.Period = arr;
    g4_tim_encode_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 10;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 10;
    
    HAL_TIM_Encoder_Init(&g4_tim_encode_handle, &sEncoderConfig);
    HAL_TIM_Encoder_Start(&g4_tim_encode_handle, PWM4_A_CHANNEL);
    HAL_TIM_Encoder_Start(&g4_tim_encode_handle, PWM4_B_CHANNEL);
    __HAL_TIM_ENABLE_IT(&g4_tim_encode_handle, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&g4_tim_encode_handle, TIM_IT_UPDATE);
}




/**
 * @brief       编码器 MSP 初始化
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio_init_struct;

    /* TIM3 电机1编码器 PA6 PA7 */
    if(htim->Instance == TIM3)
    {
        PWM1_A_GPIO_CLK_ENABLE();
        PWM1_B_GPIO_CLK_ENABLE();
        PWM1_A_TIME_CLK_ENABLE();

        gpio_init_struct.Pin = PWM1_A_GPIO_PIN | PWM1_B_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = PWM1_A_GPIO_AF;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        HAL_NVIC_SetPriority(PWM1_ENCODER_TIME_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(PWM1_ENCODER_TIME_IRQn);
    }

    /* TIM4 电机2编码器 PB6 PB7 */
    if(htim->Instance == TIM4)
    {
        PWM2_A_GPIO_CLK_ENABLE();
        PWM2_B_GPIO_CLK_ENABLE();
        PWM2_A_TIME_CLK_ENABLE();

        gpio_init_struct.Pin = PWM2_A_GPIO_PIN | PWM2_B_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = PWM2_A_GPIO_AF;
        HAL_GPIO_Init(GPIOB, &gpio_init_struct);

        HAL_NVIC_SetPriority(PWM2_ENCODER_TIME_IRQn, 2, 1);
        HAL_NVIC_EnableIRQ(PWM2_ENCODER_TIME_IRQn);
    }

    /* TIM2 电机3编码器 PA0/1/2/3 */
    if(htim->Instance == TIM2)
    {
        PWM3_A_GPIO_CLK_ENABLE();
				PWM3_B_GPIO_CLK_ENABLE();
				PWM3_A_TIME_CLK_ENABLE();
			
        gpio_init_struct.Pin = PWM3_A_GPIO_PIN | PWM3_B_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio_init_struct.Alternate = PWM3_A_GPIO_AF;
        HAL_GPIO_Init(GPIOA, &gpio_init_struct);

        HAL_NVIC_SetPriority(PWM3_ENCODER_TIME_IRQn, 2, 2);
        HAL_NVIC_EnableIRQ(PWM3_ENCODER_TIME_IRQn);
    }
/* TIM5 电机4编码器 PA2 PA3 */
if(htim->Instance == TIM5)
{
    PWM4_A_GPIO_CLK_ENABLE();
    PWM4_B_GPIO_CLK_ENABLE();
    PWM4_A_TIME_CLK_ENABLE();

    gpio_init_struct.Pin = PWM4_A_GPIO_PIN | PWM4_B_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = PWM4_A_GPIO_AF;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    HAL_NVIC_SetPriority(PWM4_ENCODER_TIME_IRQn, 2, 3);
    HAL_NVIC_EnableIRQ(PWM4_ENCODER_TIME_IRQn);
}
}



/**************************** 4路编码器中断服务函数 ****************************/
void PWM1_ENCODER_TIME_IRQHandler(void) { HAL_TIM_IRQHandler(&encoder.g1_tim_encode_handle); }
void PWM2_ENCODER_TIME_IRQHandler(void) { HAL_TIM_IRQHandler(&encoder.g2_tim_encode_handle); }
void PWM3_ENCODER_TIME_IRQHandler(void) { HAL_TIM_IRQHandler(&encoder.g3_tim_encode_handle); }
 

//中断共有处理问题：
void TIM5_IRQHandler(void){
//PWM4_ENCODER_TIME_IRQHandler
	HAL_TIM_IRQHandler(&encoder.g4_tim_encode_handle);
//PWM3_BTIM_INT_IRQHandler
	  HAL_TIM_IRQHandler(&encoder.g_pwm3_btim_handle); 

}

/******************************* 第三部分 4路独立基本定时器采样 ************************************/




void Encoder_::pwm1_btim_int_init(uint16_t arr, uint16_t psc)
{
    g_pwm1_btim_handle.Instance = PWM1_BTIM_INT;
    g_pwm1_btim_handle.Init.Prescaler = psc;
    g_pwm1_btim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm1_btim_handle.Init.Period = arr;
    g_pwm1_btim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&g_pwm1_btim_handle);
    HAL_TIM_Base_Start_IT(&g_pwm1_btim_handle);
}

void Encoder_::pwm2_btim_int_init(uint16_t arr, uint16_t psc)
{
    g_pwm2_btim_handle.Instance = PWM2_BTIM_INT;
    g_pwm2_btim_handle.Init.Prescaler = psc;
    g_pwm2_btim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm2_btim_handle.Init.Period = arr;
    g_pwm2_btim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&g_pwm2_btim_handle);
    HAL_TIM_Base_Start_IT(&g_pwm2_btim_handle);
}

void Encoder_::pwm3_btim_int_init(uint16_t arr, uint16_t psc)
{
    g_pwm3_btim_handle.Instance = PWM3_BTIM_INT;
    g_pwm3_btim_handle.Init.Prescaler = psc;
    g_pwm3_btim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm3_btim_handle.Init.Period = arr;
    g_pwm3_btim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&g_pwm3_btim_handle);
    HAL_TIM_Base_Start_IT(&g_pwm3_btim_handle);
}

void Encoder_::pwm4_btim_int_init(uint16_t arr, uint16_t psc)
{
    g_pwm4_btim_handle.Instance = PWM4_BTIM_INT;
    g_pwm4_btim_handle.Init.Prescaler = psc;
    g_pwm4_btim_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    g_pwm4_btim_handle.Init.Period = arr;
    g_pwm4_btim_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&g_pwm4_btim_handle);
    HAL_TIM_Base_Start_IT(&g_pwm4_btim_handle);
}



/**
 * @brief       基本定时器 MSP 初始化
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == PWM1_BTIM_INT)
    {
        PWM1_BTIM_INT_CLK_ENABLE();
        HAL_NVIC_SetPriority(PWM1_BTIM_INT_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(PWM1_BTIM_INT_IRQn);
    }
    if(htim->Instance == PWM2_BTIM_INT)
    {
        PWM2_BTIM_INT_CLK_ENABLE();
        HAL_NVIC_SetPriority(PWM2_BTIM_INT_IRQn, 1, 1);
        HAL_NVIC_EnableIRQ(PWM2_BTIM_INT_IRQn);
    }
    if(htim->Instance == PWM3_BTIM_INT)
    {
        PWM3_BTIM_INT_CLK_ENABLE();
        HAL_NVIC_SetPriority(PWM3_BTIM_INT_IRQn, 1, 2);
        HAL_NVIC_EnableIRQ(PWM3_BTIM_INT_IRQn);
    }
    if(htim->Instance == PWM4_BTIM_INT)
    {
        PWM4_BTIM_INT_CLK_ENABLE();
        HAL_NVIC_SetPriority(PWM4_BTIM_INT_IRQn, 1, 3);
        HAL_NVIC_EnableIRQ(PWM4_BTIM_INT_IRQn);
    }
}

/**************************** 基本定时器中断服务函数 ****************************/
void PWM1_BTIM_INT_IRQHandler(void) { HAL_TIM_IRQHandler(&encoder.g_pwm1_btim_handle); }
void PWM2_BTIM_INT_IRQHandler(void) { HAL_TIM_IRQHandler(&encoder.g_pwm2_btim_handle); }

void PWM4_BTIM_INT_IRQHandler(void) { HAL_TIM_IRQHandler(&encoder.g_pwm4_btim_handle); }



/******************************* 第四部分 公用：编码器计数 + 4路PID控制 ************************************/

extern Monitor_Controller monitor_contr;

extern osMessageQueueId_t canDataQueueHandle;

extern struct CAN_Data_t;

extern osThreadId_t canSendTaskHandle;

/**
 * @brief       定时器更新中断回调
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		CAN_Data_t tex_can;
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		BaseType_t xWakeFlag = pdFALSE;
    static uint8_t val1 = 0, val2 = 0, val3 = 0, val4 = 0;
    
    // ========== 编码器溢出中断处理（保持不变）==========
    if (htim->Instance == PWM1_ENCODER_TIME_SOURCE) {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder.g1_tim_encode_handle)) encoder.g_encode1_count--;
        else encoder.g_encode1_count++;
    }
    else if (htim->Instance == PWM2_ENCODER_TIME_SOURCE) {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder.g2_tim_encode_handle))encoder. g_encode2_count--;
        else encoder.g_encode2_count++;
    }
    else if (htim->Instance == PWM3_ENCODER_TIME_SOURCE) {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder.g3_tim_encode_handle)) encoder.g_encode3_count--;
        else encoder.g_encode3_count++;
    }
    else if (htim->Instance == PWM4_ENCODER_TIME_SOURCE) {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&encoder.g4_tim_encode_handle))encoder. g_encode4_count--;
        else encoder.g_encode4_count++;
    }
    
    // ========== 电机1 速度采样和 PID 控制 ==========
    if (htim->Instance == PWM1_BTIM_INT) {
        int Encode_now = encoder.pwm1_get_encode();
      //计算当前速度  
			encoder.speed_computer(Encode_now, 5, &encoder.g_encode1,&monitor_contr.g_motor1_data);   // 需要修改 speed_computer 支持电机ID，见下方说明
      //发送数据给队列 
			
			tex_can.motorid=1;
			tex_can.motorspeed=monitor_contr.g_motor1_data.speed;
			//发送给这个队列
			xQueueSendFromISR(canDataQueueHandle,&tex_can,&xHigherPriorityTaskWoken);
			//唤醒任务
			xTaskNotifyFromISR(
        canSendTaskHandle,
        0,               // 传给任务的通知值
        eNoAction,  
        &xWakeFlag);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
//这里出现错误，这里应该3个值都是同一个，因为如果入队没有唤醒高优先级的任务的话，则xH标志为false
//而xT唤醒后，xW为true，port进行任务切换时发现标志的xH为false则不切换；			
        if (val1 % SMAPLSE_PID_SPEED1 == 0) {
            if (encoder.g1_run_flag) {
                encoder.pidptr->integral_limit(&encoder.pidptr->g1_speed_pid, 7000, -7000);
                encoder.pidptr->integral_limit(&encoder.pidptr->g1_current_pid, 2500, -2500);
                
                // 速度环计算，使用电机1的速度数据
                monitor_contr.g_motor1_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g1_speed_pid, monitor_contr.g_motor1_data.speed);
                //这里有个隐患，如果采用增量式PID，因为没有对输入进行限制，如果电机物理上只能跑500，而输入1000
							//那么PID运算一直存在500误差，就是死活无法加速了，但是Actull值一直在+=。而位置式因为有误差累计限额，所以不会出现这个问题
                if (monitor_contr.g_motor1_data.motor_pwm > 0) {
                    monitor_contr.dcmotor1_dir(0);
                } else {
                    monitor_contr.g_motor1_data.motor_pwm = -monitor_contr.g_motor1_data.motor_pwm;
                    monitor_contr.dcmotor1_dir(1);
                }
                
                if (monitor_contr.g_motor1_data.motor_pwm >= 200)
                    monitor_contr.g_motor1_data.motor_pwm = 200;
                
                // 电流环计算，使用电机1的电流数据
                encoder.pidptr->g1_current_pid.SetPoint = monitor_contr.g_motor1_data.motor_pwm;
                monitor_contr.g_motor1_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g1_current_pid, monitor_contr.g_motor1_data.current);
                
                if (monitor_contr.g_motor1_data.motor_pwm >= 8200)
                    monitor_contr.g_motor1_data.motor_pwm = 8200;
                else if (monitor_contr.g_motor1_data.motor_pwm <= 0)
                    monitor_contr.g_motor1_data.motor_pwm = 0;
                
                // 设置PWM占空比（自动处理方向）
                monitor_contr.motor1_set_pwm(monitor_contr.g_motor1_data.motor_pwm);
            }
            val1 = 0;
        }
        val1++;
    }
    
    // ========== 电机2 速度采样和 PID 控制 ==========
    if (htim->Instance == PWM2_BTIM_INT) {
        int Encode_now = encoder.pwm2_get_encode();
        encoder.speed_computer(Encode_now, 5, &encoder.g_encode2,&monitor_contr.g_motor2_data);
        
			//发送数据给队列 
		
			tex_can.motorid=2;
			tex_can.motorspeed=monitor_contr.g_motor2_data.speed;
			//发送给这个队列
			xQueueSendFromISR(canDataQueueHandle,&tex_can,&xHigherPriorityTaskWoken);
			//唤醒任务
			xTaskNotifyFromISR(
        canSendTaskHandle,
        0,               // 传给任务的通知值
        eNoAction,  
        &xWakeFlag);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
			
        if (val2 % SMAPLSE_PID_SPEED2 == 0) {
            if (encoder.g2_run_flag) {
                encoder.pidptr->integral_limit(&encoder.pidptr->g2_speed_pid, 7000, -7000);
                encoder.pidptr->integral_limit(&encoder.pidptr->g2_current_pid, 2500, -2500);
                
                monitor_contr.g_motor2_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g2_speed_pid, monitor_contr.g_motor2_data.speed);
                
                if (monitor_contr.g_motor2_data.motor_pwm > 0) {
                    monitor_contr.dcmotor2_dir(0);
                } else {
                    monitor_contr.g_motor2_data.motor_pwm = -monitor_contr.g_motor2_data.motor_pwm;
                    monitor_contr.dcmotor2_dir(1);
                }
                
                if (monitor_contr.g_motor2_data.motor_pwm >= 200)
                    monitor_contr.g_motor2_data.motor_pwm = 200;
                
                encoder.pidptr->g2_current_pid.SetPoint = monitor_contr.g_motor2_data.motor_pwm;
                monitor_contr.g_motor2_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g2_current_pid, monitor_contr.g_motor2_data.current);
                
                if (monitor_contr.g_motor2_data.motor_pwm >= 8200)
                    monitor_contr.g_motor2_data.motor_pwm = 8200;
                else if (monitor_contr.g_motor2_data.motor_pwm <= 0)
                    monitor_contr.g_motor2_data.motor_pwm = 0;
                
                monitor_contr.motor2_set_pwm(monitor_contr.g_motor2_data.motor_pwm);
            }
            val2 = 0;
        }
        val2++;
    }
    
    // ========== 电机3 速度采样和 PID 控制 ==========
    if (htim->Instance == PWM3_BTIM_INT) {
        int Encode_now = encoder.pwm3_get_encode();
        encoder.speed_computer(Encode_now, 5, &encoder.g_encode3,&monitor_contr.g_motor3_data);
        
			//发送数据给队列 
			
			tex_can.motorid=3;
			tex_can.motorspeed=monitor_contr.g_motor3_data.speed;
			//发送给这个队列
			xQueueSendFromISR(canDataQueueHandle,&tex_can,&xHigherPriorityTaskWoken);
			//唤醒任务
			xTaskNotifyFromISR(
        canSendTaskHandle,
        0,               // 传给任务的通知值
        eNoAction,  
        &xWakeFlag);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
			
        if (val3 % SMAPLSE_PID_SPEED3 == 0) {
            if (encoder.g3_run_flag) {
                encoder.pidptr->integral_limit(&encoder.pidptr->g3_speed_pid, 7000, -7000);
                encoder.pidptr->integral_limit(&encoder.pidptr->g3_current_pid, 2500, -2500);
                
                monitor_contr.g_motor3_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g3_speed_pid, monitor_contr.g_motor3_data.speed);
                
                if (monitor_contr.g_motor3_data.motor_pwm > 0) {
                    monitor_contr.dcmotor3_dir(0);
                } else {
                    monitor_contr.g_motor3_data.motor_pwm = -monitor_contr.g_motor3_data.motor_pwm;
                    monitor_contr.dcmotor3_dir(1);
                }
                
                if (monitor_contr.g_motor3_data.motor_pwm >= 200)
                    monitor_contr.g_motor3_data.motor_pwm = 200;
                
                encoder.pidptr->g3_current_pid.SetPoint = monitor_contr.g_motor3_data.motor_pwm;
                monitor_contr.g_motor3_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g3_current_pid, monitor_contr.g_motor3_data.current);
                
                if (monitor_contr.g_motor3_data.motor_pwm >= 8200)
                    monitor_contr.g_motor3_data.motor_pwm = 8200;
                else if (monitor_contr.g_motor3_data.motor_pwm <= 0)
                    monitor_contr.g_motor3_data.motor_pwm = 0;
                
                monitor_contr.motor3_set_pwm(monitor_contr.g_motor3_data.motor_pwm);
            }
            val3 = 0;
        }
        val3++;
    }
    
    // ========== 电机4 速度采样和 PID 控制 ==========
    if (htim->Instance == PWM4_BTIM_INT) {
        int Encode_now = encoder.pwm4_get_encode();
        encoder.speed_computer(Encode_now, 5, &encoder.g_encode4,&monitor_contr.g_motor4_data);
        
				//发送数据给队列 
			
			tex_can.motorid=4;
			tex_can.motorspeed=monitor_contr.g_motor4_data.speed;
			//发送给这个队列
			xQueueSendFromISR(canDataQueueHandle,&tex_can,&xHigherPriorityTaskWoken);
			//唤醒任务
			xTaskNotifyFromISR(
        canSendTaskHandle,
        0,               // 传给任务的通知值
        eNoAction,  
        &xWakeFlag);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);	
			
        if (val4 % SMAPLSE_PID_SPEED4 == 0) {
            if (encoder.g4_run_flag) {
                encoder.pidptr->integral_limit(&encoder.pidptr->g4_speed_pid, 7000, -7000);
                encoder.pidptr->integral_limit(&encoder.pidptr->g4_current_pid, 2500, -2500);
                
                monitor_contr.g_motor4_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g4_speed_pid, monitor_contr.g_motor4_data.speed);
                
                if (monitor_contr.g_motor4_data.motor_pwm > 0) {
                    monitor_contr.dcmotor4_dir(0);
                } else {
                    monitor_contr.g_motor4_data.motor_pwm = -monitor_contr.g_motor4_data.motor_pwm;
                    monitor_contr.dcmotor4_dir(1);
                }
                
                if (monitor_contr.g_motor4_data.motor_pwm >= 200)
                    monitor_contr.g_motor4_data.motor_pwm = 200;
                
                encoder.pidptr->g4_current_pid.SetPoint = monitor_contr.g_motor4_data.motor_pwm;
                monitor_contr.g_motor4_data.motor_pwm = encoder.pidptr->increment_pid_ctrl(&encoder.pidptr->g4_current_pid, monitor_contr.g_motor4_data.current);
                
                if (monitor_contr.g_motor4_data.motor_pwm >= 8200)
                    monitor_contr.g_motor4_data.motor_pwm = 8200;
                else if (monitor_contr.g_motor4_data.motor_pwm <= 0)
                    monitor_contr.g_motor4_data.motor_pwm = 0;
                
                monitor_contr.motor4_set_pwm(monitor_contr.g_motor4_data.motor_pwm);
            }
            val4 = 0;
        }
        val4++;
    }
}

//因为g_encode1_cout存在并发，也就是一个读，一个写，应该要加入临时边界进行保护
/* 获取4路编码器值 */
/* 获取4路编码器值 */
int Encoder_::pwm1_get_encode(void) 
{ 
    return (int32_t)__HAL_TIM_GET_COUNTER(&g1_tim_encode_handle) + g_encode1_count * 65536; 
}

int Encoder_::pwm2_get_encode(void) 
{ 
    return (int32_t)__HAL_TIM_GET_COUNTER(&g2_tim_encode_handle) + g_encode2_count * 65536; 
}

int Encoder_::pwm3_get_encode(void) 
{ 
    return (int32_t)__HAL_TIM_GET_COUNTER(&g3_tim_encode_handle) + g_encode3_count * 65536; 
}

int Encoder_::pwm4_get_encode(void) 
{ 
    return (int32_t)__HAL_TIM_GET_COUNTER(&g4_tim_encode_handle) + g_encode4_count * 65536; 
}
//改进提议，中断应该只做标志处理，然后因为存在4台电机，为了保证时序正确，这些大量运算应该放在freertos里
//计算速度函数
//ms，因为中断是1ms进1次，那么我这里填50，则是50ms计算一次速度
void Encoder_::speed_computer(int32_t encode_now,uint8_t ms,ENCODE_TypeDef*g_encode,Motor_TypeDef*g_motor_data){
		uint8_t i = 0, j = 0;
    float temp = 0.0;
    static uint8_t sp_count = 0, k = 0;
    static float speed_arr[10] = {0.0};                     /* 存储速度进行滤波运算 */
		int sm=0;
    if (sp_count == ms)                                     /* 计算一次速度 */
    {
        /* 计算电机转速 
           第一步 ：计算ms毫秒内计数变化量
           第二步 ；计算1min内计数变化量：g_encode.speed * ((1000 / ms) * 60 ，
           第三步 ：除以编码器旋转一圈的计数次数（倍频倍数 * 编码器分辨率）
           第四步 ：除以减速比即可得出电机转速
        */
				
        (*g_encode).encode_now = encode_now;                                /* 取出编码器当前计数值 */
        (*g_encode).speed = ((*g_encode).encode_now - (*g_encode).encode_old);    /* 计算编码器计数值的变化量，也就是这段50ms内转了多少脉冲 */
        
			//REDUCTION_RATIO：减速比(电机输出轴转速/负载轴转速)
			//ROTO_RATIO：编码器每转一圈产生的脉冲数(经过倍频后)
			
			//1000=1秒，1000/50=20个50ms
			//50ms进入计算时*乘的是脉冲变化量，所以可以理解为50ms变化内speed脉冲量
			//然后*60就是计算1分钟的，最后除以脉冲数/圈得到你实际转了多少圈，
			//除以一个减速比就知道最后的转的圈数，而RMP用的就是圈速表示速度
        speed_arr[k++] = (float)((*g_encode).speed * ((1000 / ms) * 60.0) / REDUCTION_RATIO / ROTO_RATIO );    /* 保存电机转速 */
			
			
        (*g_encode).encode_old = (*g_encode).encode_now;          /* 保存当前编码器的值 */

        /* 累计10次速度值，后续进行滤波*/
        if (k == 10)
        {
            for (i = 10; i >= 1; i--)                       /* 冒泡排序*/
            {
                for (j = 0; j < (i - 1); j++) 
                {
                    if (speed_arr[j] > speed_arr[j + 1])    /* 数值比较 */
                    { 
                        temp = speed_arr[j];                /* 数值换位 */
                        speed_arr[j] = speed_arr[j + 1];
                        speed_arr[j + 1] = temp;
                    }
                }
            }
            
            temp = 0.0;
            
            for (i = 2; i < 8; i++)                         /* 去除两边高低数据 */
            {
                temp += speed_arr[i];                       /* 将中间数值累加 */
            }
            
            temp = (float)(temp / 6);                       /*求速度平均值*/
            
            /* 一阶低通滤波
             * 公式为：Y(n)= qX(n) + (1-q)Y(n-1)
             * 其中X(n)为本次采样值；Y(n-1)为上次滤波输出值；Y(n)为本次滤波输出值，q为滤波系数
             * q值越小则上一次输出对本次输出影响越大，整体曲线越平稳，但是对于速度变化的响应也会越慢
             */
            (*g_motor_data).speed = (float)( ((float)0.48 * temp) + ((*g_motor_data).speed * (float)0.52) );
            k = 0;
        }
        sp_count = 0;
    }
    sp_count ++;

}