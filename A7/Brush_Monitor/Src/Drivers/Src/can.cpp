#include "can.h"
#include "FreeRTOS.h"      // 添加：提供 BaseType_t 和 pdFALSE
#include "task.h"          // 添加：FreeRTOS 任务相关
#include <string.h>        // 添加：提供 memset
CAN_HandleTypeDef hcan;




// 添加 CRC8 计算函数（放在文件开头，include 之后）
static uint8_t Calculate_CRC8(uint8_t* data, uint8_t len)
{
    uint8_t crc = 0xFF;  // 初始值
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;  // 0x31 是 CRC8 常用多项式
            else
                crc <<= 1;
        }
    }
    return crc;
}

void MX_CAN_Init(void)
{
  // 选择CAN硬件外设：CAN1
hcan.Instance = CAN1;

// CAN 时钟预分频器
// 假设 APB1 时钟为 36MHz，36MHz / 6 = 6MHz（一个 Time Quantum 时钟）
hcan.Init.Prescaler = 6;

// CAN 工作模式：正常模式（收发都可用）
// 其他可选：环回模式（自测）、静默模式、静默环回模式
hcan.Init.Mode = CAN_MODE_NORMAL;

// 同步跳转宽度：1 个时间单位（TQ）
// 用于CAN总线时钟同步，值越小同步精度越高
hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;

// 时间段1：8 个时间单位（TQ）
// 包含同步段+缓冲段+采样点之前的时间段，决定采样点位置
hcan.Init.TimeSeg1 = CAN_BS1_8TQ;

// 时间段2：3 个时间单位（TQ）
// 采样点之后的时间段，用于处理信号传输延迟
hcan.Init.TimeSeg2 = CAN_BS2_3TQ;

// 关闭时间触发模式：不使用硬件定时器触发收发
hcan.Init.TimeTriggeredMode = DISABLE;

// 开启自动总线关闭管理：CAN总线异常关闭后，硬件自动恢复
hcan.Init.AutoBusOff = ENABLE;

// 关闭自动唤醒：CAN不主动从睡眠模式唤醒
hcan.Init.AutoWakeUp = DISABLE;

// 开启自动重传：发送失败时，硬件自动重复发送直到成功
hcan.Init.AutoRetransmission = ENABLE;

// 接收FIFO不锁定：新报文会覆盖旧报文（开启则锁定，溢出不覆盖）
hcan.Init.ReceiveFifoLocked = DISABLE;

// 发送FIFO优先级：按请求顺序发送，而非ID优先级
hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance == CAN1)
  {
    __HAL_RCC_CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
  }
}

/* 过滤器：接收标准ID 0x100 */
void CAN_Filter_Init(void)
{
  CAN_FilterTypeDef can_filter;

// 过滤器模式：掩码模式（ID + 掩码 组合过滤）
// 掩码模式：只有ID对应位=1的地方，才必须和报文ID一致；=0的地方为“不关心”
can_filter.FilterMode = CAN_FILTERMODE_IDMASK;

// 过滤器位宽：32位模式（整个过滤器用一个32位ID+掩码，支持标准/扩展帧）
// 32位 = 完整ID(11/29位) + 控制位，比16位更精确
can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

// 使用第0号过滤器组（STM32最多有14/28组，这里用第0组）
can_filter.FilterBank   = 0;

// 过滤器ID高16位：标准ID 0x100 左移5位
// 标准CAN ID只有11位，存放在32位过滤器的【高位28~18位】
// 0x100 <<5 就是把 11位标准ID 放到正确的寄存器位置
	//0x100=001 0000 0000，左移5位，就是0010 0000 0000 0000->只取前11bit就是0x200
	//但是这一步只是为了适配寄存器位置，放进去而已，实际还是只接受0x100的
can_filter.FilterIdHigh = 0x100 << 5;

// 过滤器ID低16位：32位模式下，这里填0即可（完整ID已放在高16位）
can_filter.FilterIdLow  = 0x0000;

// 过滤器掩码高16位：全11位掩码 0x7FF 左移5位
// 0x7FF 是11位全1，表示【所有ID位都必须严格匹配】
// 左移5位 = 把掩码对准ID位置
//0x7ff-》111 1111 1111 左移5bit=1111 1111 1110 0000->取前11bit 就是1111 1111 111，也就是全要对上
can_filter.FilterMaskIdHigh = 0x7FF << 5;

// 过滤器掩码低16位：32位模式下填0即可
can_filter.FilterMaskIdLow  = 0x0000;

// 匹配成功的报文，放入【RX FIFO0】接收
can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;

// 激活这个过滤器（使能生效）
can_filter.FilterActivation = ENABLE;

// 双CAN时使用：从第14组过滤器开始分配给CAN2
// 单CAN1时，这个值不影响功能
can_filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &can_filter) != HAL_OK)
  {
    Error_Handler();
  }
}

void CAN_Start(void)
{
  if (HAL_CAN_Start(&hcan) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}
/*
===============================
CAN 协议发送函数说明
===============================
功能：按照 CAN 2.0A 标准帧格式发送数据
帧类型：数据帧（Data Frame）
ID类型：标准ID（11位）
最大数据长度：8字节
符合：STM32 HAL CAN 驱动规范
*/
//当前这个设备是0x102
uint8_t CAN_Send_Msg(uint32_t id, uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header;  // CAN 发送帧头结构体（存储一整帧的协议信息）
  uint32_t tx_mailbox;            // 保存硬件发送邮箱编号（CAN 发送用 3 个邮箱队列）

  if(len > 8) len = 8;  // CAN 协议规定：一帧最多只能发 8 字节数据，超长则截断

  // ==================== 以下是 CAN 帧协议核心配置 ====================
  tx_header.StdId = id;           // 【标准ID】设置 11 位 CAN 标准帧 ID（对应过滤器识别的 ID：0x100）
  tx_header.ExtId = 0;            // 【扩展ID】不用扩展帧（29位），所以填 0
  tx_header.RTR = CAN_RTR_DATA;   // 【帧类型】= 数据帧（DATA）；另一种是远程帧（无数据）
  tx_header.IDE = CAN_ID_STD;     // 【ID模式】= 标准帧（11位）；另一种是扩展帧（29位）
  tx_header.DLC = len;            // 【数据长度】= 0~8，CAN 协议固定 DLC 表示数据字节数

  // 把帧头 + 数据写入 CAN 发送邮箱，启动硬件发送
  if(HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) != HAL_OK)
    return 1;  // 发送失败（邮箱满/总线错误）返回 1

  return 0;    // 发送成功返回 0
}



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


// 判断是否为升级命令（0x01）
    if (buf[1] == 0x01)
    {
        // OTA升级命令，直接处理
        RemoteUpdate.ProcessCanCommand(buf, 8);
    }
    else
    {
        // 普通命令，发送到队列给电机控制任务
        cmd.cmd = buf[1];
        cmd.param1 = (buf[2] << 8) | buf[3];
        cmd.param2 = (buf[4] << 8) | buf[5];
        cmd.mode = buf[6];
        
        osMessageQueuePut(motorCmdQueueHandle, &cmd, 0, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

