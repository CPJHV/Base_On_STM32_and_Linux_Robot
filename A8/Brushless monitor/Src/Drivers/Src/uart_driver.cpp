#include "uart_driver.h"

// 构造函数
ESP8266::ESP8266(UART_HandleTypeDef* uart, 
                GPIO_TypeDef* rstPort, uint16_t rstPin,
                GPIO_TypeDef* chPort, uint16_t chPin)
    : huart(uart), resetPort(rstPort), resetPin(rstPin),
      chPdPort(chPort), chPdPin(chPin), rxIndex(0), 
      dataReady(false), echoEnabled(true) {
    memset(rxBuffer, 0, sizeof(rxBuffer));
}

// 析构函数
ESP8266::~ESP8266() {
    // 断开连接
    disconnect();
    // 硬件复位
    hardwareReset();
}

// 初始化ESP8266
bool ESP8266::begin(void) {
    // 硬件复位
    hardwareReset();
    HAL_Delay(2000);
    
    // 测试AT命令
    if (!sendCommand("AT", "OK", 2000)) {
        return false;
    }
    
    // 关闭回显（可选）
    if (!echoEnabled) {
        sendCommand("ATE0", "OK", 1000);
    }
    
    // 设置工作模式为station模式
    if (!setMode(1)) {
        return false;
    }
    
    return true;
}

// 连接WiFi
bool ESP8266::connectWiFi(const char* ssid, const char* password) {
    char cmd[256];
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
    
    if (sendCommand(cmd, "OK", 15000)) {
        return true;
    }
    return false;
}

// 断开WiFi
bool ESP8266::disconnectWiFi(void) {
    return sendCommand("AT+CWQAP", "OK", 2000);
}

// 获取IP地址
bool ESP8266::getIPAddress(char* ipBuffer, uint16_t bufferSize) {
    if (sendCommand("AT+CIFSR", "OK", 3000)) {
        // 从响应中提取IP地址
        char* start = strstr(rxBuffer, "STAIP");
        if (start) {
            start = strstr(start, "\"");
            if (start) {
                start++;
                char* end = strstr(start, "\"");
                if (end) {
                    uint16_t len = end - start;
                    if (len < bufferSize) {
                        strncpy(ipBuffer, start, len);
                        ipBuffer[len] = '\0';
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

// 建立TCP连接
bool ESP8266::connectTCP(const char* host, uint16_t port) {
    char cmd[128];
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d", host, port);
    return sendCommand(cmd, "CONNECT", 10000);
}

// 建立UDP连接
bool ESP8266::connectUDP(const char* host, uint16_t port) {
    char cmd[128];
    sprintf(cmd, "AT+CIPSTART=\"UDP\",\"%s\",%d", host, port);
    return sendCommand(cmd, "OK", 5000);
}

// 发送数据
bool ESP8266::sendData(const char* data, uint16_t len) {
    char cmd[32];
    sprintf(cmd, "AT+CIPSEND=%d", len);
    
    if (!sendCommand(cmd, ">", 2000)) {
        return false;
    }
    
    return sendRawData(data, len, "SEND OK", 5000);
}

// 发送字符串数据
bool ESP8266::sendString(const char* data) {
    return sendData(data, strlen(data));
}

// 发送HTTP GET请求
bool ESP8266::httpGet(const char* url) {
    char request[512];
    const char* host = getHostFromUrl(url);
    sprintf(request, "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", 
            url, host);
    return sendData(request, strlen(request));
}

// 发送HTTP POST请求
bool ESP8266::httpPost(const char* url, const char* postData) {
    char request[512];
    const char* host = getHostFromUrl(url);
    sprintf(request, "POST %s HTTP/1.1\r\nHost: %s\r\n"
            "Content-Type: application/x-www-form-urlencoded\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n\r\n%s",
            url, host, strlen(postData), postData);
    return sendData(request, strlen(request));
}

// 接收数据
bool ESP8266::receiveData(char* buffer, uint16_t* len, uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    uint16_t index = 0;
    
    while (HAL_GetTick() - startTime < timeout) {
        if (dataReady) {
            // 解析 +IPD 格式的数据
            char* ipdStart = strstr(rxBuffer, "+IPD");
            if (ipdStart) {
                int dataLen = 0;
                sscanf(ipdStart, "+IPD,%d:", &dataLen);
                
                char* dataStart = strstr(ipdStart, ":");
                if (dataStart && dataLen > 0) {
                    dataStart++;
                    if (dataLen < *len) {
                        memcpy(buffer, dataStart, dataLen);
                        buffer[dataLen] = '\0';
                        *len = dataLen;
                        clearRxBuffer();
                        return true;
                    }
                }
            }
            clearRxBuffer();
        }
        HAL_Delay(10);
    }
    return false;
}

// 断开TCP/UDP连接
bool ESP8266::disconnect(void) {
    return sendCommand("AT+CIPCLOSE", "CLOSED", 3000);
}

// 检查WiFi连接状态
bool ESP8266::isWiFiConnected(void) {
    return sendCommand("AT+CWJAP?", "OK", 2000);
}

// 检查TCP连接状态
bool ESP8266::isTCPConnected(void) {
    return sendCommand("AT+CIPSTATUS", "STATUS:2", 1000);
}

// 获取模块版本信息
bool ESP8266::getVersion(char* version, uint16_t bufferSize) {
    if (sendCommand("AT+GMR", "OK", 2000)) {
        strncpy(version, rxBuffer, bufferSize - 1);
        version[bufferSize - 1] = '\0';
        return true;
    }
    return false;
}

// 设置工作模式
bool ESP8266::setMode(uint8_t mode) {
    char cmd[32];
    sprintf(cmd, "AT+CWMODE=%d", mode);
    return sendCommand(cmd, "OK", 1000);
}

// 扫描可用WiFi
bool ESP8266::scanWiFi(char* results, uint16_t bufferSize) {
    if (sendCommand("AT+CWLAP", "OK", 10000)) {
        strncpy(results, rxBuffer, bufferSize - 1);
        results[bufferSize - 1] = '\0';
        return true;
    }
    return false;
}

// 处理串口接收
void ESP8266::handleReceive(uint8_t data) {
    if (rxIndex < sizeof(rxBuffer) - 1) {
        rxBuffer[rxIndex++] = data;
        rxBuffer[rxIndex] = '\0';
        dataReady = true;
    }
}

// 清除接收缓冲区
void ESP8266::clearRxBuffer(void) {
    memset(rxBuffer, 0, sizeof(rxBuffer));
    rxIndex = 0;
    dataReady = false;
}

// 设置回显
void ESP8266::setEcho(bool enable) {
    echoEnabled = enable;
    if (enable) {
        sendCommand("ATE1", "OK", 1000);
    } else {
        sendCommand("ATE0", "OK", 1000);
    }
}

// 硬件复位
void ESP8266::hardwareReset(void) {
    HAL_GPIO_WritePin(resetPort, resetPin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(resetPort, resetPin, GPIO_PIN_SET);
    HAL_Delay(1000);
    
    // 使能CH_PD
    HAL_GPIO_WritePin(chPdPort, chPdPin, GPIO_PIN_SET);
    HAL_Delay(100);
}

// 软件复位
bool ESP8266::softwareReset(void) {
    return sendCommand("AT+RST", "ready", 3000);
}

// 发送AT命令并等待响应
bool ESP8266::sendCommand(const char* cmd, const char* expected, uint32_t timeout) {
    clearRxBuffer();
    
    // 发送命令
    char cmdWithCRLF[256];
    sprintf(cmdWithCRLF, "%s\r\n", cmd);
    HAL_UART_Transmit(huart, (uint8_t*)cmdWithCRLF, strlen(cmdWithCRLF), 1000);
    
    return waitForResponse(expected, timeout);
}

// 发送AT命令（无响应检查）
bool ESP8266::sendCommandOnly(const char* cmd) {
    char cmdWithCRLF[256];
    sprintf(cmdWithCRLF, "%s\r\n", cmd);
    return HAL_UART_Transmit(huart, (uint8_t*)cmdWithCRLF, strlen(cmdWithCRLF), 1000) == HAL_OK;
}

// 发送原始数据
bool ESP8266::sendRawData(const char* data, uint16_t len, const char* expected, uint32_t timeout) {
    HAL_UART_Transmit(huart, (uint8_t*)data, len, 1000);
    return waitForResponse(expected, timeout);
}

// 等待指定字符串
bool ESP8266::waitForResponse(const char* expected, uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    
    while (HAL_GetTick() - startTime < timeout) {
        if (dataReady) {
            if (strstr(rxBuffer, expected) != NULL) {
                return true;
            }
            if (strstr(rxBuffer, "ERROR") != NULL) {
                return false;
            }
            if (strstr(rxBuffer, "FAIL") != NULL) {
                return false;
            }
            clearRxBuffer();  // 继续等待
        }
        HAL_Delay(10);
    }
    return false;
}

// 从URL中提取主机名
const char* ESP8266::getHostFromUrl(const char* url) {
    // 跳过 http:// 或 https://
    const char* host = strstr(url, "://");
    if (host) host += 3;
    else host = url;
    
    // 找到第一个/结束
    static char hostBuffer[64];
    int i = 0;
    while (*host && *host != '/' && i < 63) {
        hostBuffer[i++] = *host++;
    }
    hostBuffer[i] = '\0';
    return hostBuffer;
}

// 等待数据接收完成
bool ESP8266::waitForData(uint32_t timeout) {
    uint32_t startTime = HAL_GetTick();
    while (HAL_GetTick() - startTime < timeout) {
        if (dataReady && strstr(rxBuffer, "OK") != NULL) {
            return true;
        }
        HAL_Delay(10);
    }
    return false;
}