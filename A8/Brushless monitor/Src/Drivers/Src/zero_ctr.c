#ifdef _BLDC_WU_
#include "zero_ctr.h"
#include "bldc.h"
#include "bldc_time.h"

Hallless g_hallless_three;                      /* 反电势结构体 */
VvvF_start  g_vvvf_sp;                          /* 无感开环结构体 */
uint16_t g_delay_temp = 0;                      /* 延迟变量 */
uint8_t g_zero_ctr_status = 0;                  /* 无感状态标志位 */
static uint16_t count_i = 0;                    /* 过零信号稳定标志 */
static uint8_t switch_hallless = 0;             /* 获取当前过零信号是否稳定 */

static uint32_t count_j = 0;                    /* 运行时间 计算速度使用 */


/**
 * @brief       无感过零控制逻辑
 * @note        实现定位并以一定速度旋转并获取到稳定的过零信号之后，切换至过零控制
 * @param       无
 * @retval      无
 */
void zero_ctr_loop(void)
{
    if(g_bldc_motor1.run_flag == RUN)
    {
        switch(g_zero_ctr_status)
        {
            case 0:                                 /* 定位：固定一相U+V-*/
            {//强行让电机定位到某个位置，让电机知道是从哪里开始转的
							
                g_vvvf_sp.Voilage_Ref = (uint16_t)(MAX_PWM_DUTY/20);//给一个小电压
							
                g_bldc_motor1.pwm_duty = g_vvvf_sp.Voilage_Ref;//占空比
							
                m1_uhvl();//u+ v-换相通电
							
                zero_ctr_init();//无感初始化
                g_vvvf_sp.Freq_T_Ref = 300;         /* 初始延迟时间为：55*300us，不同电机此参数不同 */
                g_zero_ctr_status = 1;              /* 切换至延迟计数*/
                count_i = 0;
                switch_hallless = 0;
                g_vvvf_sp.Count = 0;
                g_vvvf_sp.VvvF_Count = 0; 
            }
            break;
            case 1:/*定时延迟每隔一段时间换一次相*/
            {
                g_delay_temp++;//开始计时
                if(g_delay_temp >= g_vvvf_sp.Freq_T_Ref)
                {//当计时达到延迟频率时
                    g_delay_temp = 0;
                    g_zero_ctr_status = 2;          /* 切换至换相操作时间一到马上切换下一相 */
                }
#if (DRIVE_MODE)                                    /* 过零闭环控制 为0开环控制 */
                switch_hallless = hallless_sw();    /* 检测过零信号 稳定后切换至闭环 */
                if(switch_hallless == 1)
                {
                    g_zero_ctr_status = 3;//如果信号稳定，直接跳进闭环
                }
#endif
            }
            break;
            case 2:/*加速换相（提高换向频率）*/
            {
                g_vvvf_sp.Freq_T_Ref -= g_vvvf_sp.Freq_T_Ref/15+1;/* 改变换向频率 */
                g_vvvf_sp.Count++;
                change_voltage();                   /* 改变换向所需电压 提高电压*/
                if(g_vvvf_sp.Freq_T_Ref < 180)
                {//最低延迟时间，低于就只能默认180最低了
                    g_vvvf_sp.Freq_T_Ref = 180;     /* 固定次频率（延迟时间），不同电机此参数有差异4对级的可设置150 */
                    g_zero_ctr_status = 1;          /* 切换至延迟计数*/
                }
                else
                {
                    g_zero_ctr_status = 1;          /* 切换至延迟计数 */
                }
								
								
                g_vvvf_sp.VvvF_Count++;             /* 换向计数 */

                if(g_vvvf_sp.VvvF_Count == 6)
                {//换完一次轮回重复
                    g_vvvf_sp.VvvF_Count = 0;
                }
                anwerfen_sw();//6步换相走一步

            }
            break;
            case 3:
            {//此时进入了真正的转动时刻
                hallless_sw();                      /*切换至闭环过零检测状态*/
            }
            break;
            default:  break;
        }
    }
    else if(g_bldc_motor1.run_flag == STOP)
    {
        zero_ctr_init();
    }
}
/**
 * @brief       无感控制初始化，将无感状态标志清0
 * @param       无
 * @retval      无
 */
void zero_ctr_init(void)
{
    g_zero_ctr_status = 0;
    g_delay_temp = 0;
    g_vvvf_sp.Count = 0;
    g_vvvf_sp.VvvF_Count = 0;
    g_hallless_three.Filter_Delay = 0;
}
/**
 * @brief       无感开环6步换相顺序
 * @param       无
 * @retval      无
 */
void anwerfen_sw(void)
{
    if(g_bldc_motor1.dir == CCW)
    {//方向为正
        switch(g_vvvf_sp.VvvF_Count)/*换向次数*/
        {
            /* 六步换向顺序:(U+V-)-> (U+W-)-> (V+W-)-> (V+U-)-> (W+U-)->(W+V-) */
            case  0x00:  m1_uhvl();break;       /* U+V-* 对应初始固定相 */
            case  0x01:  m1_uhwl();break;
            case  0x02:  m1_vhwl();break;
            case  0x03:  m1_vhul();break;
            case  0x04:  m1_whul();break;
            case  0x05:  m1_whvl();break;
            default:break;
        }
    }
    else
    {//方向为负
        switch(g_vvvf_sp.VvvF_Count)
        {
            case  0x00:  m1_uhvl();break;       /* U+V-* 对应初始固定相 */
            case  0x01:  m1_whvl();break;
            case  0x02:  m1_whul();break;
            case  0x03:  m1_vhul();break;
            case  0x04:  m1_vhwl();break;
            case  0x05:  m1_uhwl();break;
            default:    break;
        }
    }

}
#define FSCA 2
/**
 * @brief       换相电压修改
 * @param       无
 * @retval      无
 */
void change_voltage(void)
{
    switch(g_vvvf_sp.Count)
    {
        case  1:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 10;break;
        case  2:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 9;break;
        case  3:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 8;break;
        case  4:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 7;break;
        case  5:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 7;break;
        case  6:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 6;break;
        case  7:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 6;break;
        case  8:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 5;break;
        case  9:  g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 5;break;
        case  10: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  11: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  12: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  13: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  14: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        case  15: g_bldc_motor1.pwm_duty = MAX_PWM_DUTY/FSCA / 4;break;
        default:    break;   
    }
}


/**
  * @brief  过零闭环控制
  * @param  无
  * @retval 0 ： 不稳定的过零信号  1：稳定的过零信号
  */
uint8_t hallless_sw(void)
{
    static uint8_t edge_flag = 0;
    count_j++;/*运行过程时间计数 18K的中断频率为单位*/
	
    g_hallless_three.Queue_Status[0] = g_hallless_three.Queue_Status[0] << 1;//把旧数据移出去
    g_hallless_three.Queue_Status[1] = g_hallless_three.Queue_Status[1] << 1;
    g_hallless_three.Queue_Status[2] = g_hallless_three.Queue_Status[2] << 1;
    /*检测过零点*/
    g_hallless_three.Queue_Status[0] |= HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO,HALL1_TIM_CH1_PIN);/*U相过零点*/
		//读取U相引脚
    g_hallless_three.Queue_Status[1] |= HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO,HALL1_TIM_CH2_PIN);/*V相过零点*/
    g_hallless_three.Queue_Status[2] |= HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO,HALL1_TIM_CH3_PIN);/*W相过零点*/
    
	g_hallless_three.Filter_Math = g_hallless_three.Queue_Status[0] & FILTER_LONG;//滤波
	//这里的FILTER_LONG是0xffff=1111 1111 1111 1111，表示信号都为1，则信号稳定在1了
	//都为0，则信号稳定在0，否则就认为信号不稳定
    if(g_hallless_three.Filter_Math == FILTER_LONG)
    {
        g_hallless_three.QFilter_Status[0] = 1;

    }
    else if(g_hallless_three.Filter_Math == 0x00)
    {
        g_hallless_three.QFilter_Status[0] = 0;
    }
    else
    {//信号不稳定，不算数
        g_hallless_three.Filter_Count++;
        return 2;
    }
		
		
    g_hallless_three.Filter_Math = g_hallless_three.Queue_Status[1] & FILTER_LONG;
    if(g_hallless_three.Filter_Math == FILTER_LONG)
    {
        g_hallless_three.QFilter_Status[1] = 1;
    }
    else if(g_hallless_three.Filter_Math == 0x00)
    {
        g_hallless_three.QFilter_Status[1] = 0;
    }
    else
    {
        g_hallless_three.Filter_Count++;
        return 2;
    }
    g_hallless_three.Filter_Math = g_hallless_three.Queue_Status[2] & FILTER_LONG;
    if(g_hallless_three.Filter_Math == FILTER_LONG)
    {
        g_hallless_three.QFilter_Status[2] = 1;
    }
    else if(g_hallless_three.Filter_Math == 0x00)
    {
        g_hallless_three.QFilter_Status[2] = 0;
    }
    else
    {
        g_hallless_three.Filter_Count++;
        return 2;
    }
		/*
		这是因为启动时速度太慢，切割出的反向电动势这些和驱动电压太小，单片机信号检测时很容易被其他元件干扰
		当速度达到一定时，检测信号稳定
		*/
		
		
		
    /************************************** 速度计算 ***************************************/
    /*
		过零原理：转子经过悬空的相时，交替切割产生了正电压和反电压，
		当正到低，低到正时，就穿过了零点，这个穿过0电位的瞬间=电角度的过零点
		每60度电角度必然出现一次反电动势的过零点
		
		*/
		g_hallless_three.Filter_Edge = uemf_edge(g_hallless_three.QFilter_Status[0]);
		//边沿检测，看有没有发生变化
		
    if(g_hallless_three.Filter_Edge == 0)     
    {//表示检测到下降沿，过零了
			
        /* 延迟30°换相，因为硬件上低通滤波器和软件延迟的原因，实际延迟角度需小于30°，
        最优解可以通过示波器确定 */
			
        if(count_i >= 4)/*稳定检测到过零信号才对速度进行测量，检测到4次过零才开始计算*/
        {
					//速度=系数/两次过零时间
					/*
					计算电机RPM：多少转/分钟
					因为count_j每1就是1/18000秒，因此这个值越小，电机速度越快
					
					RPM是每分钟多少圈：
					18000*60表示一分钟多少中断，/count_j表示两个过零点之间触发多少次中断，/4表示转了几圈
					*/
            if(g_bldc_motor1.dir == CW)
                g_hallless_three.Speed_RPM = ((uint32_t)SPEED_COEFF/count_j);
            else
                g_hallless_three.Speed_RPM = -((uint32_t)SPEED_COEFF/count_j);
						
            FirstOrderRC_LPF(g_bldc_motor1.speed,g_hallless_three.Speed_RPM,0.2379);  /* 过滤尖峰带来的影响 */
        }
        g_hallless_three.Filter_Delay = count_j/ 10;  /* 实际延迟时间计算过零后延迟30度电角度需要几个中断周期
				而cout_j是两次过零之间的时间，/10就是1/12周期就是30度，
				
				而count是过零->上升沿->过零->上升沿的时间，所以实际上是过零（下降沿）->上生沿（180）->过零（360）
				实际是180度
					*/
        g_hallless_three.Filter_Count = 0;
        count_j = 0;
        count_i++;                              /* 捕捉过零点，累计次数 */
    }
    if(g_hallless_three.Filter_Edge == 1)
    {//检测到低到高上升沿，但不计算速度，只等下一个下降沿才认为过零
        g_hallless_three.Filter_Count = 0;
        count_j = 0;
    }
    if(g_hallless_three.Filter_Edge == 2)
    {//无信号发生
        g_hallless_three.Filter_Count++;        /* 不换相时间累计 超时则判定速度为0 */
        if(g_hallless_three.Filter_Count > 15000)
        {
            g_hallless_three.Filter_Count = 0;
            g_hallless_three.Speed_RPMF = 0;    /* 超时换向 判定为停止 速度为0 */
            g_bldc_motor1.locked_rotor = 1;
            stop_motor1();
            g_bldc_motor1.run_flag = STOP;      /* 标记停机 */
            g_bldc_motor1.pwm_duty = 0;
        }
    }
		
		
		
    /************************************** 过零控制 ***************************************/
    if(count_i >= 4)                         /* 稳定检测到过零信号 并旋转2圈之后进入*/
    {//稳定捕抓了4次过零，才进入闭环
			
        count_i = 4;
        edge_flag++;                         /* edge_flag==2 才正式进入过零闭环控制*/
        if(edge_flag >= 2)
        {//必须确保再转两圈才算稳定
					
            edge_flag = 2;
					
            g_hallless_three.Hallless_State = g_hallless_three.QFilter_Status[0] +(g_hallless_three.QFilter_Status[1]<<1) +(g_hallless_three.QFilter_Status[2]<<2);
						//把三个相电压平成一个数值，其实是三个的反向电平，这样就可以知道我们电机处于什么位置了
					
            if(g_hallless_three.Hallless_State <= 0x00 || g_hallless_three.Hallless_State > 0x06)
            {
                return 0;
            }
            if(g_hallless_three.Hallless_State != g_hallless_three.OldHallless_State)
            {//检测状态是否发生变化
                g_hallless_three.Filter_Count_All++;//当转子方向发生变化，说明准备换相了，开始计数
            }

						//延迟30度的电角度才换相
            if(g_hallless_three.Filter_Count_All >= (g_hallless_three.Filter_Delay))//延迟30度
            {
                g_hallless_three.Filter_Count_All = 0;
							
                if(g_hallless_three.Hallless_State != g_hallless_three.OldHallless_State)
                {//转子方向发生变化
									
                    g_bldc_motor1.sum_pos++;//记录转子换了多少次相
                    if(g_bldc_motor1.dir == CCW)
                    {
                        g_bldc_motor1.pos--;    /* 电机位置计数 */
												//根据状态完成自动换相
                        pfunclist_m1[g_hallless_three.Hallless_State-1]();
                    }
                    else
                    {
                        g_bldc_motor1.pos++;    /* 电机位置计数 */
                        pfunclist_m2[g_hallless_three.Hallless_State-1]();
                    }
                }
                g_hallless_three.OldHallless_State = g_hallless_three.Hallless_State ;
            }
        }

        return 1;
    }
    return 0;

}
#endif
