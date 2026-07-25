// RealMotorDriver.cpp
#include "RealMotorDriver.hpp"
#include <cmath>

// 你的底层API头文件
#include "Shoulder_Step.h"
#include "BigArm_Step.h"
#include "Forearm_Step.h"
// 假设手腕和舵机API也在其他地方定义
extern void Wrist_Angle_Move(float angle, uint16_t speed_delay);
extern void Finger_SetAngle(uint8_t servo_id, uint8_t angle);

// 方向枚举（根据你的定义）
// 假设 DIR_STATE 为 {CW, CWN}

void RealMotorDriver::setJointAngle(int joint_id, double angle_rad) {
    // 弧度转度
    double angle_deg = angle_rad * 180.0 / M_PI;
    // 根据关节号调用不同API
    switch (joint_id) {
        case 0: { // 肩关节
            DIR_STATE dir = (angle_deg >= 0) ? CW : CWN;
            uint16_t abs_angle = (uint16_t) fabs(angle_deg);
            sh_step.set_steper_angle(abs_angle, dir);
            break;
        }
        case 1: { // 大臂
            DIR_STATE dir = (angle_deg >= 0) ? CW : CWN;
            uint16_t abs_angle = (uint16_t) fabs(angle_deg);
            big_arm.set_steper_angle(abs_angle, dir);
            break;
        }
        case 2: { // 小臂
            DIR_STATE dir = (angle_deg >= 0) ? CW : CWN;
            uint16_t abs_angle = (uint16_t) fabs(angle_deg);
            forearm.set_steper_angle(abs_angle, dir);
            break;
        }
        case 3: { // 手腕（第4轴）
            // Wrist_Angle_Move 参数 float angle, uint16_t speed_delay
            // 这里速度固定，你也可以根据角度大小调整
            Wrist_Angle_Move((float)angle_deg, 100);
            break;
        }
        case 4: { // 第5轴（舵机 id=1）
            // 舵机角度范围 0~180，这里将弧度映射到 0~180
            // 假设角度范围为 [-90°, 90°] 对应 0~180°
            double servo_deg = angle_deg + 90.0; // 偏移
            if (servo_deg < 0) servo_deg = 0;
            if (servo_deg > 180) servo_deg = 180;
            Finger_SetAngle(1, (uint8_t)servo_deg);
            break;
        }
        case 5: { // 第6轴（舵机 id=0）
            double servo_deg = angle_deg + 90.0;
            if (servo_deg < 0) servo_deg = 0;
            if (servo_deg > 180) servo_deg = 180;
            Finger_SetAngle(0, (uint8_t)servo_deg);
            break;
        }
        default:
            break;
    }
}