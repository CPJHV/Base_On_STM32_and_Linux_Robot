#ifndef __WIFI_UPDATER_H
#define __WIFI_UPDATER_H
#include "esp8266.h"
#include <stdint.h>

#define APP_A_ADDR          0x08003000U
#define APP_B_ADDR          0x08009000U
#define STATUS_FLAG_ADDR    0x0800F000U
#define APP_PARTITION_SIZE  0x6000U    //24KB
#define BUFFER_SIZE         512U
#define MAGIC_WORD          0x5A5A5A5AU

typedef struct {
    uint32_t magic;         // 0x5A5A5A5A
    uint32_t active_app;    // 0:A,1:B
    uint32_t upgrade_state; //0:空闲 1:接收中 2:完成待切换
    uint32_t upgrade_crc32;
    uint32_t upgrade_size;
    uint32_t reserved[2];
} AppStatus_t;

class WifiUpdater_
{
public:
    void Init(Esp8266_* wifi);

    uint32_t GetInactiveAppAddr(void);
    bool CheckForUpdate(const char* current_ver, const char* server_ver_url);
    bool DownloadAndUpdate(const char* firmware_url);
    void SetUpgradeCompleted(uint32_t crc32, uint32_t size);

    bool EraseApp(uint32_t app_addr);
    bool WriteFlash(uint32_t addr, uint8_t* data, uint32_t len);
    uint32_t GetProgress(void) { return progress; }

    // 上传监测数据
    bool UploadMonitorData(float temp, float volt, float curr, int bat_percent);

private:
    Esp8266_* m_wifi;
    uint32_t progress;

    uint32_t CRC32_Feed(uint32_t crc, uint8_t* buf, uint32_t len);
    // HTTP辅助：寻找\r\n\r\n，分离header与body
    bool SkipHttpHeader(uint8_t* buf, uint16_t len, uint32_t* remain_cache,
                        uint8_t* cache, uint16_t cache_len, uint32_t* content_len_out);
};

extern WifiUpdater_ wifi_updater;
#endif