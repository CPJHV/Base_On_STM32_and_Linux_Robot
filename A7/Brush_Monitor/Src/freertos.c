#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
/* 任务句柄 ----------------------------------------------------------------*/
osThreadId_t canSendTaskHandle;
osThreadId_t motorControlTaskHandle;

osMessageQueueId_t canDataQueueHandle;      // CAN数据队列
osTaskNotification_t canSendNotification;    // CAN发送任务通知

extern Car_State car_state;

enum can_data_type{
	SPEED=0,
	STATE=1
};
/* CAN数据结构体 -----------------------------------------------------------*/
typedef struct {
    int8_t motorid;   // 电机号码
    int8_t motorspeed;   // 电机速度
} CAN_Data_t;
typdef struct{
	uint8_t cmd;        // 命令码
    int16_t param1;     // 参数1（速度值/角度值）
    int16_t param2;     // 参数2（备用）
    uint8_t mode;       // 模式
	
}MotorCmd_t;

/* 任务函数声明 ------------------------------------------------------------*/
void StartCANSendTask(void *argument);
void StartMotorControlTask(void *argument);

/* 空闲钩子函数声明 --------------------------------------------------------*/
void vApplicationIdleHook(void);

/* ============================ 任务创建 ============================ */
void MX_FREERTOS_Init(void)
{
		//采用队列+任务通知方式处理
		canDataQueueHandle = osMessageQueueNew(20, sizeof(CAN_Data_t), NULL);
	
    // 任务1：CAN发送任务
    osThreadAttr_t canSendAttr = {
        .name = "CAN_Send",
        .stack_size = 512,
        .priority = osPriorityLow,
    };
    canSendTaskHandle = osThreadNew(StartCANSendTask, NULL, &canSendAttr);
    
    // 任务2：电机控制任务
    osThreadAttr_t motorControlAttr = {
        .name = "MotorControl",
        .stack_size = 512,
        .priority = osPriorityNormal,
    };
    motorControlTaskHandle = osThreadNew(StartMotorControlTask, NULL, &motorControlAttr);
}

/* ============================ 任务1 ============================ */
/*
can要发啥？
基础的状态数据：车速度，电流检测值
*/
void StartCANSendTask(void *argument)
{
		CAN_Data_t tx_data;
    uint8_t can_buf[8];
    for(;;)
    {
			//等待任务通知
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			//从队列里获取数据
			while (osMessageQueueGet(canDataQueueHandle, &tx_data, NULL, 0) == osOK)
        {
				
						can_buf[0]=tx_data.motorid;
					  can_buf[1]=tx_data.motorspeed;
						can_buf[2]=car_state;
						can_buf[3]=get_current_val(tx_data.motorid);
						
					
				 CAN_Send_Msg(0x102,can_buf,4);
				
				
				}
       
       
        vTaskDelay(pdMS_TO_TICKS(20));  // 周期20ms
    }
}

/* ============================ 任务2 ============================ */
void StartMotorControlTask(void *argument)
{
    for(;;)
    {
         // ========== 处理CAN命令（非阻塞） ==========
        while (osMessageQueueGet(motorCmdQueueHandle, &cmd, NULL, 0) == osOK)
        {
            switch(cmd.cmd)
            {
                case 0x01:  // OTA升级命令
                    // 注意：OTA需要更多数据，在CAN中断中单独处理
                    break;
                    
                case 0x02:  // 修改PID参数
                    // cmd.param1 = P, cmd.param2 = I, cmd.mode = D
                    pidx.angle_pid.Proportion = cmd.param1 / 100.0f;
                    pidx.angle_pid.Integral = cmd.param2 / 100.0f;
                    pidx.angle_pid.Derivative = cmd.mode / 100.0f;
                    break;
                    
                case 0x03:  // 设置车辆速度
                    car_state.target_speed = cmd.param1;
                    car_state.mode = CAR_STRAIGHT_HOLD;
                    car_state.target_yaw = car_state.current_yaw;
                    break;
                    
                case 0x04:  // 停止
                    Car_Stop();
                    break;
                    
                case 0x05:  // 转向运动（目标角度）
                    car_state.target_yaw = cmd.param1 / 100.0f;  // 传入角度*100
                    car_state.mode = CAR_TURN_ANGLE;
                    break;
                    
                case 0x06:  // 差速控制（左轮速度，右轮速度）
                    {
                        float vl = cmd.param1;
                        float vr = cmd.param2;
                        float smooth_l = S_Curve_Calc(&motor1_smooth, LineSpeed_To_MotorSpeed(vl));
                        float smooth_r = S_Curve_Calc(&motor2_smooth, LineSpeed_To_MotorSpeed(vr));
                        
                        monitor_contr.g_motor1_data.speed = smooth_l;
                        monitor_contr.g_motor2_data.speed = smooth_r;
                        monitor_contr.g_motor3_data.speed = smooth_l;
                        monitor_contr.g_motor4_data.speed = smooth_r;
                        
                        pidx.g1_speed_pid.SetPoint = smooth_l;
                        pidx.g2_speed_pid.SetPoint = smooth_r;
                        pidx.g3_speed_pid.SetPoint = smooth_l;
                        pidx.g4_speed_pid.SetPoint = smooth_r;
                    }
                    break;
                    
                default:
                    break;
            }
        }
        
        // ========== 更新当前航向角 ==========
        car_state.current_yaw = 0;  // 从你的陀螺仪获取
        
        // ========== 执行运动控制（10ms周期） ==========
        Car_Run_10ms();
        vTaskDelay(pdMS_TO_TICKS(10));  // 周期10ms
    }
}

/* ============================ 空闲钩子（低功耗） ============================ */
void vApplicationIdleHook(void)
{
    // 低功耗代码
    __WFI();
}