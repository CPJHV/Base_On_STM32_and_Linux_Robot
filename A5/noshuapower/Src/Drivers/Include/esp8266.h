#ifndef __ESP8266_H
#define __ESP8266_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ESP8266_Init(uint32_t baud);
void ESP8266_SendString(const char* str);
int8_t ESP8266_ReceiveLine(char* buf, uint16_t len, uint32_t timeout_ms);
void ESP8266_AT_Reset(void);
uint8_t ESP8266_ConnectWiFi(const char* ssid, const char* pwd);
uint8_t ESP8266_StartTCPClient(const char* host, uint16_t port);
uint8_t ESP8266_SendData(const uint8_t* data, uint16_t len);
// 修改 ReceiveData 签名：返回成功与否，实际接收的数据长度通过指针返回
uint8_t ESP8266_ReceiveData(uint8_t* buf, uint16_t max_len, uint16_t* out_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Esp8266_ {
public:
    void Init(uint32_t baud = 115200);
    bool ConnectWiFi(const char* ssid, const char* pwd);
    bool StartTCPClient(const char* host, uint16_t port);
    bool SendData(const uint8_t* data, uint16_t len);
    bool ReceiveData(uint8_t* buf, uint16_t max_len, uint16_t* out_len, uint32_t timeout_ms);
    void Reset();
};
#endif

#endif