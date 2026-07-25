#include "esp8266.h"
#include <string.h>
#include <stdio.h>
#include "stm32f1xx_hal_uart.h"

UART_HandleTypeDef huart1;

void ESP8266_Init(uint32_t baud) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baud;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart1);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

void ESP8266_SendString(const char* str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 100);
}

int8_t ESP8266_ReceiveLine(char* buf, uint16_t len, uint32_t timeout_ms) {
    uint16_t i=0;
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
        if (HAL_UART_Receive(&huart1, (uint8_t*)&buf[i], 1, 50) == HAL_OK) {
            if (buf[i] == '\n') { buf[i] = '\0'; return 0; }
            i++; if(i>=len-1) break;
        }
    }
    return -1;
}

void ESP8266_AT_Reset(void) {
    ESP8266_SendString("AT+RST\r\n");
    HAL_Delay(3000);
}

uint8_t ESP8266_ConnectWiFi(const char* ssid, const char* pwd) {
    char cmd[100];
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    ESP8266_SendString(cmd);
    char resp[64];
    if (ESP8266_ReceiveLine(resp, 64, 5000) == 0) {
        return (strstr(resp, "OK") != NULL);
    }
    return 0;
}

uint8_t ESP8266_StartTCPClient(const char* host, uint16_t port) {
    char cmd[64];
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", host, port);
    ESP8266_SendString(cmd);
    char resp[64];
    if (ESP8266_ReceiveLine(resp, 64, 5000) == 0) {
        return (strstr(resp, "CONNECT") != NULL);
    }
    return 0;
}

uint8_t ESP8266_SendData(const uint8_t* data, uint16_t len) {
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%d\r\n", len);
    ESP8266_SendString(cmd);
    HAL_Delay(100);
    ESP8266_SendString((const char*)data);
    char resp[32];
    return (ESP8266_ReceiveLine(resp,32,2000)==0 && strstr(resp,"SEND OK"));
}

// 修正后的 ReceiveData：解析 +IPD 数据包
uint8_t ESP8266_ReceiveData(uint8_t* buf, uint16_t max_len, uint16_t* out_len, uint32_t timeout_ms) {
    char line[128];
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout_ms) {
        if (ESP8266_ReceiveLine(line, sizeof(line), 200) == 0) {
            if (strstr(line, "+IPD,")) {
                // 解析 +IPD,<len>:data
                int data_len = 0;
                char *colon = strchr(line, ':');
                if (colon) {
                    sscanf(line, "+IPD,%d:", &data_len);
                    if (data_len > 0 && data_len <= max_len) {
                        // 复制数据部分
                        char *data_start = colon + 1;
                        memcpy(buf, data_start, data_len);
                        *out_len = data_len;
                        return 1;
                    }
                }
            }
        }
    }
    return 0;
}

// C++ 类方法实现
void Esp8266_::Init(uint32_t baud) { ESP8266_Init(baud); }
bool Esp8266_::ConnectWiFi(const char* ssid, const char* pwd) { return ESP8266_ConnectWiFi(ssid, pwd) == 1; }
bool Esp8266_::StartTCPClient(const char* host, uint16_t port) { return ESP8266_StartTCPClient(host, port) == 1; }
bool Esp8266_::SendData(const uint8_t* data, uint16_t len) { return ESP8266_SendData(data, len) == 1; }
bool Esp8266_::ReceiveData(uint8_t* buf, uint16_t max_len, uint16_t* out_len, uint32_t timeout_ms) {
    return ESP8266_ReceiveData(buf, max_len, out_len, timeout_ms) == 1;
}
void Esp8266_::Reset() { ESP8266_AT_Reset(); }