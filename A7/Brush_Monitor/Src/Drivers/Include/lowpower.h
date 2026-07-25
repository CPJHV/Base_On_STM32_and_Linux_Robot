#ifndef __LOWPOWER_H
#define __LOWPOWER_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

class LowPower_
{
public:
    enum Mode {
        SLEEP_MODE,
        STOP_MODE,
        STANDBY_MODE
    };

    void Init(Mode mode = STOP_MODE);
    void Enter(void);
    void RegisterWakeupPin(uint16_t pin);
    void EnableTickless(void);

private:
    Mode current_mode;
    void EnterSleep(void);
    void EnterStop(void);
    void EnterStandby(void);
    void ConfigureWakeupPin(uint16_t pin);
};

#endif