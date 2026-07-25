#ifndef __USER_CONTROLLER_H
#define __USER_CONTROLLER_H

#include "config.h"
#include "pwm.h"
#include "encoder.h"
#include "adc.h"
#include "can.h"      // 添加这一行
#include "motor.h"
#include "pid.h"
#include "s_curve.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif

class User_ {
public:
   void Init_all_drivers();
};

extern User_ user;

#endif