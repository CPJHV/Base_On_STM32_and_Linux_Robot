#include "usercontroller.h"

#ifdef __cplusplus

// ==================== FreeRTOS 全局对象 ====================
static QueueHandle_t can_cmd_queue = NULL;      // 命令队列
static TaskHandle_t display_task_handle = NULL; // 显示任务句柄（用于发送通知）
static TaskHandle_t can_cmd_task_handle = NULL; // CAN 命令处理任务句柄
show_* show_::instance_ptr = nullptr;

// CAN 命令结构体（简化）
typedef struct {
    uint8_t cmd;       // 命令码
    uint8_t param;     // 参数
    uint8_t msg_char1; // 弹窗字符1
    uint8_t msg_char2; // 弹窗字符2
} can_cmd_t;
typedef enum
{
    CMD_FAN_SWITCH      = 0x02U,
    CMD_BUZZER_SWITCH   = 0x03U,
    CMD_SET_CURR_THR    = 0x04U,
    CMD_SET_VOLT_THR    = 0x05U,
    CMD_SET_TEMP_THR    = 0x06U,
} can_cmd_id_t;
// 全局标志（兼容原有逻辑）
uint8_t can_msg_flag = 0;
uint32_t can_msg_timer = 0;
uint8_t can_msg_displayed = 0;
char message_show[2] = {' ', ' '};

// 全局 User 对象
User_ user;

// CRC8 表（保持不变）
static const uint8_t CRC8_Table[256] = 
{
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,
    0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,
    0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,
    0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xC9,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,
    0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,
    0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,
    0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,
    0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,
    0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,
    0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,
    0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,
    0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,
    0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,
    0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,
    0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,
    0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,
    0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

uint8_t Calculate_CRC8(uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0x00;
    for(uint8_t i = 0; i < len; i++)
        crc = CRC8_Table[crc ^ buf[i]];
    return crc;
}



// ========================== GPIO 实现 ==========================
void gpio_::Init() { MX_GPIO_Init(); }
void gpio_::drive_FAN_Switch(uint8_t fl) { fl ? Fan_On() : Fan_Off(); }
void gpio_::drive_Buzzer_Switch(uint8_t fl) { fl ? Buzzer_On() : Buzzer_Off(); }
uint8_t gpio_::get_FAN_State(void) { return ::get_FAN_State(); }
uint8_t gpio_::get_Buzzer_State(void) { return ::get_Buzzer_State(); }



// ========================== ADC 实现 ==========================
void adc_::Init()
{
    MX_ADC1_Init();
    tolerance_temper = 55;
    tolerance_voltage = 3;
    tolerance_electri = tolerance_voltage / 3.3f;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_BUFFER_SIZE);
}
void adc_::Filter() { ADC_Filter_Task(); }
float adc_::Get_Temperature_() { return Get_Temperature_FromTable(); }
float adc_::Get_Power_Voltage() { return Get_Power_Voltage(); }
float adc_::get_electri() { return Get_electri(); }
uint16_t* adc_::get_dma() { return adc_dma_buf; }
uint16_t adc_::get_max_temper() { return Get_max_temper(); }
uint16_t adc_::get_min_temper() { return Get_min_temper(); }
float adc_::get_max_voltage() { return Get_max_voltage(); }
float adc_::get_min_voltage() { return Get_min_voltage(); }
float adc_::get_max_electri() { return Get_max_electri(); }
float adc_::get_min_electri() { return Get_min_electri(); }
void adc_::set_torlerance_electri(uint8_t param) { tolerance_electri = param; tolerance_voltage = tolerance_electri * 3.3f; }
void adc_::set_torlerance_volt(uint8_t param) { tolerance_voltage = param; tolerance_electri = tolerance_voltage / 3.3f; }
void adc_::set_torlerance_temper(uint8_t param) { tolerance_temper = param; }

// ========================== CAN 实现 ==========================
void can_::Init() { CAN_Filter_Init(); CAN_Start(); }
void can_::SendMsg(uint32_t id, uint8_t* data, uint8_t len) { CAN_Send_Msg(id, data, len); }

// ========================== OLED 显示实现 ==========================
void show_::Init() {
osTimerAttr_t timer_attr = {0};
    timer_attr.name = "PopupTimer";
instance_ptr = this;
popup_timer_handle = osTimerNew(&show_::PopupTimerCallback,
                                     osTimerOnce,
                                     nullptr,    // 把类实例指针传入回调
                                     &timer_attr);
    if(popup_timer_handle == NULL)
    {
        Error_Handler();
    }
		
OLED_Init(); OLED_Clear(); }
void show_::StaticPopupCallback(osTimerId_t timer)
{
    if(instance_ptr != nullptr)
    {
        instance_ptr->PopupTimerCallback();
    }
}
void show_::PopupTimerCallback()
{
    // 10s到，清除弹窗标志
    can_msg_flag = 0;
    
    // 主动唤醒OLED任务，立刻刷新界面，不用等到200ms超时
    vTaskNotifyGive(OLED_TaskHandle);
}
void show_::PopupStart(uint8_t ch0, uint8_t ch1)
{
    message_show[0] = ch0;
    message_show[1] = ch1;
    can_msg_flag = 1;
    // 重启定时器：新消息到来，重新倒计时10s
    osTimerStart(m_popup_timer, pdMS_TO_TICKS(10000U));
    // 唤醒OLED立刻绘制弹窗
    vTaskNotifyGive(OLED_TaskHandle);
}
// ========================== OLED 显示主页面（增强版：支持任务通知） ==========================
void show_::ShowPage(float temp, float volt, uint8_t fan_sta, uint8_t beep_sta)
{
    // 等待显示任务通知：超时 200ms 则刷新常规界面
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

    if (can_msg_flag != 0) // 收到了 CAN 弹窗通知
    {
        // 显示弹窗（不清屏，以免闪烁）
        OLED_ShowString(1, 1, "MESSAGE");
        OLED_ShowChar(2, 1, message_show[0]);
        OLED_ShowChar(3, 1, message_show[1]);
    }
    else // 超时 → 刷新常规界面
    {
        // ==== 第1行：温度 ====
			//这里有隐患的因为温度存在负数，而uint32是无符号整数，一但转换马上变成乱码
        OLED_ShowString(1, 1, "T:");
        OLED_ShowSignedNum(1, 3, (int32_t)temp, 2);
        OLED_ShowChar(1, 5, '.');
        uint32_t temp_dec = (uint32_t)((temp - (int32_t)temp) * 10);
        OLED_ShowNum(1, 6, temp_dec, 1);
        OLED_ShowChar(1, 7, 'C');

        uint16_t max_t = user.adc.get_max_temper();
        OLED_ShowString(1, 9, "Max:");
        OLED_ShowNum(1, 13, max_t, 2);
        uint16_t min_t = user.adc.get_min_temper();
        OLED_ShowString(1, 16, "Min:");
        OLED_ShowNum(1, 20, min_t, 2);

        // ==== 第2行：电压 ====
        OLED_ShowString(2, 1, "V:");
        OLED_ShowNum(2, 3, (uint32_t)volt, 2);
        OLED_ShowChar(2, 5, '.');
        uint32_t volt_dec = (uint32_t)((volt - (uint32_t)volt) * 100);
        OLED_ShowNum(2, 6, volt_dec, 2);
        OLED_ShowChar(2, 8, 'V');

        float max_v = user.adc.get_max_voltage();
        OLED_ShowString(2, 10, "Max:");
        OLED_ShowNum(2, 14, (uint32_t)max_v, 2);
        float min_v = user.adc.get_min_voltage();
        OLED_ShowString(2, 17, "Min:");
        OLED_ShowNum(2, 21, (uint32_t)min_v, 2);

        // ==== 第3行：电流 ====
        float current = user.adc.get_electri();
        OLED_ShowString(3, 1, "I:");
        OLED_ShowNum(3, 3, (uint32_t)current, 2);
        OLED_ShowChar(3, 5, '.');
        uint32_t cur_dec = (uint32_t)((current - (uint32_t)current) * 100);
        OLED_ShowNum(3, 6, cur_dec, 2);
        OLED_ShowChar(3, 8, 'A');

        float max_c = user.adc.get_max_electri();
        OLED_ShowString(3, 10, "Max:");
        OLED_ShowNum(3, 14, (uint32_t)max_c, 2);
        float min_c = user.adc.get_min_electri();
        OLED_ShowString(3, 17, "Min:");
        OLED_ShowNum(3, 21, (uint32_t)min_c, 2);

        // ==== 第4行：状态 ====
        OLED_ShowString(4, 1, "Fan:");
        OLED_ShowString(4, 5, fan_sta ? "ON " : "OFF");
        OLED_ShowString(4, 9, "Beep:");
        OLED_ShowString(4, 14, beep_sta ? "ON " : "OFF");
				//提示警告信息
        if (max_t - temp < user.adc.get_tolerance_temper())
            OLED_ShowString(4, 18, "T-Warn");
        else if (max_v - volt < user.adc.get_tolerance_voltage())
            OLED_ShowString(4, 18, "V-Warn");
        else
            OLED_ShowString(4, 18, "Sys OK");
    }
}

void show_::Show_Main_EEOR(){

	OLED_ShowString(1,1,"The Main Init is EEOR!!");

}
// ========================== CAN 命令处理任务（在 RTOS 线程中执行） ==========================
static void CAN_Command_Task(void *pvParameters)
{
    can_cmd_t cmd;
    for (;;)
    {
        if (xQueueReceive(can_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {//从队列里获取数据
            switch (cmd.cmd)
            {
                case CMD_FAN_SWITCH:
                    user.gpio.DriveFanSwitch(cmd.param);
                    break;
                case CMD_BUZZER_SWITCH:
                    user.gpio.DriveBuzzerSwitch(cmd.param);
                    break;
                case CMD_SET_CURR_THR:
                    user.adc.SetToleranceElectri((float)cmd.param);
                    break;
                case CMD_SET_VOLT_THR:
                    user.adc.SetToleranceVolt((float)cmd.param);
                    break;
                case CMD_SET_TEMP_THR:
                    user.adc.SetToleranceTemper((float)cmd.param);
                    break;
                default:
                    break;
            }

            // 弹窗消息交给show类内部处理（内部启动软件定时器）
            if (cmd.msg_char1 != 0U)
            {
                user.show.PopupStart(cmd.msg_char1, cmd.msg_char2);
            }
        }
    }
}

// ========================== 总初始化 ==========================
void User_::InitAll(void)
{
    this->gpio.Init();
    this->adc.Init();
    this->can.Init();
    this->show.Init();
		this->watchdog.Init(3000);          // 3秒看门狗
    this->lowpower.Init(LowPower_::STOP_MODE);
    this->remote_update.Init();
		this->security.Init();
		this->battery.Init(2000.0f, 3.7f);
}

// ========================== 启动 RTOS 组件 ==========================
void User_::StartRTOS(void)
{
    // 创建队列（可存放 5 条命令）
    can_cmd_queue = xQueueCreate(5, sizeof(can_cmd_t));
    if (can_cmd_queue == NULL)
        return; // 创建失败处理

    // 创建 CAN 命令处理任务（优先级稍高）
    xTaskCreate(CAN_Command_Task, "CAN_CmdTask", 512, NULL, 3, &can_cmd_task_handle);
}

#endif /* __cplusplus */

// ========================== C 接口（供 FreeRTOS 任务调用） ==========================
extern "C" {
 extern TaskHandle_t OLED_TaskHandle;
// 调度任务的显示函数
void User_Display_Update(void)
{	//获取数据
    float temp = user.adc.Get_Temperature_();
    float volt = user.adc.Get_Power_Voltage();
    //获取设备状态
		uint8_t fanstate = user.gpio.get_FAN_State();
    uint8_t buzzerstate = user.gpio.get_Buzzer_State();
	//显示页面调用
    user.show.ShowPage(temp, volt, fanstate, buzzerstate);
}

void User_CAN_Send_Data(float temp,float volt,float curr)
{		
    uint8_t tx_buf[8];
	//这里无符号不适合temp温度这个带负数的值，应该要带上abs绝对值或者fabsf浮点绝对值处理
		//合并数据
		tx_buf[0] = (uint8_t)temp;
    tx_buf[1] = (uint8_t)((temp - (uint8_t)temp) * 10);
    tx_buf[2] = (uint8_t)volt;
    tx_buf[3] = (uint8_t)((volt - (uint8_t)volt) * 100);
    tx_buf[4] = (uint8_t)curr;
    tx_buf[5] = (uint8_t)((curr - (uint8_t)curr) * 100);
		tx_buf[6]=0;
		//调用底层驱动发送can数据
    user.can.SendMsg(0x100, tx_buf, 7);
}

void User_ADC_Control_Task(void)
{
    user.adc.Filter();//滤波处理
	
	//缓存旧数据
		static float tempold =user.adc.Get_Temperature_();
		static float voltold = user.adc.Get_Power_Voltage();
    static float electriold = user.adc.get_electri();
		
	//获取新的电压，温度，电流值
    float temp = user.adc.Get_Temperature_();
    float volt = user.adc.Get_Power_Voltage();
    float electri = user.adc.get_electri();
	
		
		uint8_t alarm_buf[1];
		//以容忍度为准，设定电压电流保护和温度保护
	//不要重复读取，建议用临时值作为存储，因为你上面读取的一次，后面又读取一次，有可能两个值因为时间不同导致数值不一样
    if (volt > user.adc.get_tolerance_voltage() || electri > user.adc.get_tolerance_electri())
		{ user.gpio.drive_Buzzer_Switch(1);//启动蜂鸣器
				alarm_buf[0] = 1;
				user.can.SendMsg(0x203, alarm_buf, 1);}
    else
        user.gpio.drive_Buzzer_Switch(0);//关闭蜂鸣器

    if (temp > user.adc.get_tolerance_temper()){
        user.gpio.drive_FAN_Switch(1);//打开风扇
				alarm_buf[0] = 2;
				user.can.SendMsg(0x203, alarm_buf, 1);}
    else
        user.gpio.drive_FAN_Switch(0);//关闭风扇
		
		if(abs((int)(temp-tempold))>2||abs((int)(volt-voltold))>0.2||abs((int)(electri-electriold))>0.2){
			//如果数据发生改变，则can回报给上位机，并更新数据
		tempold=temp;
		voltold=volt;
		electriold=electri;
		User_CAN_Send_Data(temp,volt,electri);
		}
		//更新电池的数据
		user.battery.Update(volt, electri);
		
		user.watchdog.Feed();//喂看门狗 
}

void User_Init_All(void)
{
    user.InitAll();
}

// 启动 RTOS 任务（由 main 或 MX_FREERTOS_Init 调用）
void User_Start_RTOS_Tasks(void)
{
    user.StartRTOS();
    // 获取显示任务的句柄（假设显示任务名为 "OLED_Task"）
   display_task_handle = OLED_TaskHandle;
}

// ==================== CAN 中断回调（极简版：只入队，不干活） ====================
/*
格式是：
数据端位置8字节：
0：密码
1：cmd：
	1：0x01：升级程序
	2:0x02：
2：param
3：msg
4：msg
5：保留
6：保留
7：crc
*/
//推荐加上应答上位机处理，避免消息因为队列过满抛弃后丢失信息
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    CAN_RxHeaderTypeDef rx_header;
    uint8_t buf[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, buf) != HAL_OK)
        return;

    // 1. 密码校验
    if (buf[0] != CAN_PASSWORD)
        return;
    // 2. CRC8 校验
    if (Calculate_CRC8(buf, 7) != buf[7])
        return;

    
	

// 先判断是否为升级命令
if (buf[1] ==0x01)
{
    user.remote_update.ProcessCanCommand(buf, 8);
    // 不入队列，直接处理
}
else
{
    // 3. 组装命令结构体
    can_cmd_t cmd;
    cmd.cmd       = buf[1];
    cmd.param     = buf[2];
    cmd.msg_char1 = buf[3];
    cmd.msg_char2 = buf[4];

    // 4. 发送到队列（ISR 安全版本）
	//任务中 xQueueSend：队列满时可以选择阻塞等待；
//中断专用 xQueueSendToBackFromISR：没有阻塞选项，满即丢弃。
    if (can_cmd_queue != NULL)
    {
       if( xQueueSendToBackFromISR(can_cmd_queue, &cmd, &xHigherPriorityTaskWoken) == pdPASS )
{
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
    }
}
}
void User_Enter_LowPower(void)
{
    user.lowpower.Enter();
}
void Main_Init_EEOR(void){

		user.show.Show_Main_EEOR();
}
} // extern "C"