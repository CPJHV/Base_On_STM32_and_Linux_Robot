#ifndef __GPIO_H
#define __GPIO_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void MX_GPIO_Init(void);
void PowerSwitch_On(void);   // PA0 高电平
void PowerSwitch_Off(void);  // PA0 低电平

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class Gpio_ {
public:
    void Init() { MX_GPIO_Init(); }
    void PowerOn() { PowerSwitch_On(); }
    void PowerOff() { PowerSwitch_Off(); }
};
#endif

#endif