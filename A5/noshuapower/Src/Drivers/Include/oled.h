#ifndef __OLED_H
#define __OLED_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 原始 C 函数声明（供底层调用，保留） ---------- */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#ifdef __cplusplus
}
#endif

/* ---------- C++ 封装类 ---------- */
#ifdef __cplusplus

class Oled_ {
public:
    void Init(void) { OLED_Init(); }
    void Clear(void) { OLED_Clear(); }
    void ShowChar(uint8_t line, uint8_t col, char ch) { OLED_ShowChar(line, col, ch); }
    void ShowString(uint8_t line, uint8_t col, const char* str) { OLED_ShowString(line, col, (char*)str); }
    void ShowNum(uint8_t line, uint8_t col, uint32_t num, uint8_t len) { OLED_ShowNum(line, col, num, len); }
    void ShowSignedNum(uint8_t line, uint8_t col, int32_t num, uint8_t len) { OLED_ShowSignedNum(line, col, num, len); }
    void ShowHexNum(uint8_t line, uint8_t col, uint32_t num, uint8_t len) { OLED_ShowHexNum(line, col, num, len); }
    void ShowBinNum(uint8_t line, uint8_t col, uint32_t num, uint8_t len) { OLED_ShowBinNum(line, col, num, len); }

    // 便捷方法：显示温/压/流/电量（可根据需要扩展）
    void ShowTemperature(float temp);
    void ShowVoltage(float volt);
    void ShowCurrent(float curr);
    void ShowBatteryPercent(uint8_t percent);
};

#endif /* __cplusplus */

#endif /* __OLED_H */