#ifndef ESP8266_HPP
#define ESP8266_HPP

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "main.h"

class ESP8266 {
private:
    UART_HandleTypeDef* huart;  // 串口句柄
    GPIO_TypeDef* resetPort;    // 复位引脚端口
    uint16_t resetPin;          // 复位引脚
    GPIO_TypeDef* chPdPort;     // CH_PD引脚端口
    uint16_t chPdPin;           // CH_PD引脚
    
    char rxBuffer[512];         // 接收缓冲区
    uint16_t rxIndex;           // 接收索引
    bool dataReady;             // 数据就绪标志
    bool echoEnabled;           // 回显状态
    
public:
    // 构造函数
    ESP8266(UART_HandleTypeDef* uart, 
            GPIO_TypeDef* rstPort, uint16_t rstPin,
            GPIO_TypeDef* chPort, uint16_t chPin);
    
    // 析构函数
    ~ESP8266();
    
    // 初始化ESP8266
    bool begin(void);
    
    // 连接WiFi
    bool connectWiFi(const char* ssid, const char* password);
    
    // 断开WiFi
    bool disconnectWiFi(void);
    
    // 获取IP地址
    bool getIPAddress(char* ipBuffer, uint16_t bufferSize);
    
    // 建立TCP连接
    bool connectTCP(const char* host, uint16_t port);
    
    // 建立UDP连接
    bool connectUDP(const char* host, uint16_t port);
    
    // 发送数据
    bool sendData(const char* data, uint16_t len);
    
    // 发送字符串数据
    bool sendString(const char* data);
    
    // 发送HTTP GET请求
    bool httpGet(const char* url);
    
    // 发送HTTP POST请求
    bool httpPost(const char* url, const char* postData);
    
    // 接收数据
    bool receiveData(char* buffer, uint16_t* len, uint32_t timeout);
    
    // 断开TCP/UDP连接
    bool disconnect(void);
    
    // 检查WiFi连接状态
    bool isWiFiConnected(void);
    
    // 检查TCP连接状态
    bool isTCPConnected(void);
    
    // 获取模块版本信息
    bool getVersion(char* version, uint16_t bufferSize);
    
    // 设置工作模式 (1: station, 2: AP, 3: both)
    bool setMode(uint8_t mode);
    
    // 扫描可用WiFi
    bool scanWiFi(char* results, uint16_t bufferSize);
    
    // 处理串口接收（在中断中调用）
    void handleReceive(uint8_t data);
    
    // 清除接收缓冲区
    void clearRxBuffer(void);
    
    // 设置回显
    void setEcho(bool enable);
    
private:
    // 硬件复位
    void hardwareReset(void);
    
    // 软件复位
    bool softwareReset(void);
    
    // 发送AT命令并等待响应
    bool sendCommand(const char* cmd, const char* expected, uint32_t timeout);
    
    // 发送AT命令（无响应检查）
    bool sendCommandOnly(const char* cmd);
    
    // 发送原始数据
    bool sendRawData(const char* data, uint16_t len, const char* expected, uint32_t timeout);
    
    // 等待指定字符串
    bool waitForResponse(const char* expected, uint32_t timeout);
    
    // 从URL中提取主机名
    const char* getHostFromUrl(const char* url);
    
    // 等待数据接收完成
    bool waitForData(uint32_t timeout);
};

#endif