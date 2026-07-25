#include "wifi_updater.h"
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "cmsis_os.h"

WifiUpdater_ wifi_updater;

void WifiUpdater_::Init(Esp8266_* wifi)
{
    m_wifi = wifi;
    progress = 0;
}

uint32_t WifiUpdater_::GetInactiveAppAddr(void)
{
    AppStatus_t* pStatus = (AppStatus_t*)STATUS_FLAG_ADDR;
    if (pStatus->magic != MAGIC_WORD)
        return APP_A_ADDR;
    return (pStatus->active_app == 0U) ? APP_B_ADDR : APP_A_ADDR;
}

bool WifiUpdater_::EraseApp(uint32_t app_addr)
{
    if((app_addr != APP_A_ADDR && app_addr != APP_B_ADDR))
        return false;
    taskENTER_CRITICAL();//临界区
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase_cfg = {0};
    erase_cfg.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_cfg.PageAddress = app_addr;
    erase_cfg.NbPages = APP_PARTITION_SIZE / 2048U;
		//一页=2048字节，这里计算一共12页，几乎把整个app区擦除
    uint32_t page_err = 0;
    bool ret = (HAL_FLASHEx_Erase(&erase_cfg, &page_err) == HAL_OK);
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
    return ret;
}

bool WifiUpdater_::WriteFlash(uint32_t addr, uint8_t* data, uint32_t len)
{
    taskENTER_CRITICAL();
    HAL_FLASH_Unlock();
    bool ok = true;
    for(uint32_t i = 0; i < len; i += 2U)
    {
        uint16_t hw;
        if(i+1 < len)
            hw = (data[i+1] << 8U) | data[i];
				//拼接成半字节
        else
            hw = data[i]; // 奇数字节，末尾单字节
				//这是针对奇偶字节长度的一种解决方式
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, hw) != HAL_OK)
        {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
    return ok;
}

uint32_t WifiUpdater_::CRC32_Feed(uint32_t crc, uint8_t* buf, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for(int j = 0; j < 8; j++)
        {
            if(crc & 1U)
                crc = (crc >> 1U) ^ 0xEDB88320U;
            else
                crc >>= 1U;
        }
    }
    return crc;
}

bool WifiUpdater_::SkipHttpHeader(uint8_t* buf, uint16_t len, uint32_t* cache_len,
                                   uint8_t* cache, uint16_t cache_max, uint32_t* content_len_out)
{
    // 把新数据并入cache，查找"\r\n\r\n"
    if(*cache_len + len > cache_max)
    {//判断大小
        *cache_len = 0;
        return false;
    }
    memcpy(cache + *cache_len, buf, len);
    *cache_len += len;
			
    const char* header_end = strstr((char*)cache, "\r\n\r\n");
    if(header_end == nullptr)
        return false;

    // 尝试解析Content-Length
    char cl_key[] = "Content-Length:";
    char* cl_ptr = strstr((char*)cache, cl_key);
    if(cl_ptr != nullptr)
    {
        cl_ptr += strlen(cl_key);//跳过这个字段
        while(*cl_ptr == ' ') cl_ptr++;//跳过空白
        *content_len_out = atol(cl_ptr);//字符串转数字，也就是固件大小
    }

    uint32_t header_bytes = header_end - (char*)cache + 4U;
    // 把剩余body前移到缓冲区开头
    uint32_t body_len = *cache_len - header_bytes;
    memmove(cache, cache + header_bytes, body_len);
    *cache_len = body_len;
    return true;
}

bool WifiUpdater_::CheckForUpdate(const char* current_ver, const char* server_ver_url)
{
    char req[256];
    snprintf(req, sizeof(req), "GET %s HTTP/1.1\r\nHost: your-server.com\r\nConnection:close\r\n\r\n", server_ver_url);
    if(!m_wifi->SendData((uint8_t*)req, strlen(req)))
        return false;

    uint8_t resp[512];
    uint16_t recv_len = 0;
    if(!m_wifi->ReceiveData(resp, sizeof(resp), &recv_len, 5000U))
        return false;
    resp[recv_len] = '\0';

    // 简易逻辑：固件版本不相等判定有更新；量产改用版本号数值比较
    if(strstr((char*)resp, current_ver) != nullptr)
        return false;
    return true;
}

bool WifiUpdater_::DownloadAndUpdate(const char* firmware_url)
{
    uint32_t target_addr = GetInactiveAppAddr();
    if(!EraseApp(target_addr))
        return false;

    char req[256];
    snprintf(req, sizeof(req), "GET %s HTTP/1.1\r\nHost: your-server.com\r\nConnection:close\r\n\r\n", firmware_url);
    if(!m_wifi->SendData((uint8_t*)req, strlen(req)))
        return false;

    uint8_t buffer[BUFFER_SIZE];
    uint8_t header_cache[256];
    uint32_t cache_used = 0;
    bool header_finished = false;
    uint32_t content_length = 0U;

    uint32_t write_ptr = target_addr;
    uint32_t crc_acc = 0xFFFFFFFFU;
    uint32_t total_recv = 0U;

    while(1)
    {
        uint16_t rlen = 0;
        if(!m_wifi->ReceiveData(buffer, BUFFER_SIZE, &rlen, 3000U))
            break;
        if(rlen == 0U)
            break;

        if(!header_finished)
        {
            if(SkipHttpHeader(buffer, rlen, &cache_used, header_cache, sizeof(header_cache), &content_length))
            {//开始解析报文
                header_finished = true;
                // 缓存内残留body先写入
                if(cache_used > 0U)
                {
                    if(write_ptr + cache_used > target_addr + APP_PARTITION_SIZE)
                        return false;
                    WriteFlash(write_ptr, header_cache, cache_used);
                    crc_acc = CRC32_Feed(crc_acc, header_cache, cache_used);
										//整个固件包crc
                    write_ptr += cache_used;
                    total_recv += cache_used;
                    cache_used = 0U;
                }
            }
            continue;
        }

        // 接收固件二进制
        if(write_ptr + rlen > target_addr + APP_PARTITION_SIZE)
            return false;

        WriteFlash(write_ptr, buffer, rlen);
        crc_acc = CRC32_Feed(crc_acc, buffer, rlen);
        write_ptr += rlen;
        total_recv += rlen;

        if(content_length > 0U)
            progress = (total_recv * 100U) / content_length;
        else
            progress = (total_recv * 100U) / (APP_PARTITION_SIZE);

        if(content_length > 0U && total_recv >= content_length)
            break;

        // 喂狗，防止长时间阻塞复位
        // Iwdg::Feed();
    }

    if(total_recv < 256U)
        return false;

    uint32_t final_crc = ~crc_acc;
    SetUpgradeCompleted(final_crc, total_recv);
    osDelay(100); // 等待Flash写入稳定
    NVIC_SystemReset();
    return true;
}

void WifiUpdater_::SetUpgradeCompleted(uint32_t crc32, uint32_t size)
{
    AppStatus_t status_new;
    memset(&status_new, 0xFFU, sizeof(status_new));
    status_new.magic = MAGIC_WORD;
    status_new.upgrade_state = 2U;
    status_new.upgrade_crc32 = crc32;
    status_new.upgrade_size = size;

    // 单页方案；量产建议增加双状态页备份
    uint32_t flag_page = STATUS_FLAG_ADDR;
    taskENTER_CRITICAL();
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = flag_page;
    erase.NbPages = 1U;
    uint32_t err;
    HAL_FLASHEx_Erase(&erase, &err);

    // 半字依次写入
    uint16_t* src = (uint16_t*)&status_new;
    for(uint32_t i = 0; i < sizeof(AppStatus_t)/2U; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flag_page + i*2U, src[i]);
    }
    HAL_FLASH_Lock();
    taskEXIT_CRITICAL();
}

bool WifiUpdater_::UploadMonitorData(float temp, float volt, float curr, int bat_percent)
{
    char json_payload[128];
    char http_req[256];
    // 先组装负载
    snprintf(json_payload, sizeof(json_payload),
             "{\"temp\":%.1f,\"volt\":%.2f,\"curr\":%.3f,\"bat\":%d}",
             temp, volt, curr, bat_percent);
    uint32_t payload_len = strlen(json_payload);

    snprintf(http_req, sizeof(http_req),
             "POST /api/data HTTP/1.1\r\n"
             "Host: your-server.com\r\n"
             "Content-Length:%lu\r\n"
             "Content-Type:application/json\r\n\r\n%s",
             payload_len, json_payload);
    return m_wifi->SendData((uint8_t*)http_req, strlen(http_req));
}