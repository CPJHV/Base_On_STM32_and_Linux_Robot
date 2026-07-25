#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#define CAN_PASSWORD 0x11
extern CAN_HandleTypeDef hcan;

void MX_CAN_Init(void);
void CAN_Filter_Init(void);
void CAN_Start(void);
uint8_t CAN_Send_Msg(uint32_t id, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */