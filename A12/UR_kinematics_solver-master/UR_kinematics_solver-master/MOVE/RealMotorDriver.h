// RealMotorDriver.hpp
#ifndef REAL_MOTOR_DRIVER_HPP
#define REAL_MOTOR_DRIVER_HPP

#include "MotorInterface.hpp"

// 假设你的全局对象已存在，或者通过外部引用
extern Shoulder_Step sh_step;
extern BigArm_Step big_arm;
extern Forearm_Step forearm;

class RealMotorDriver : public MotorInterface {
public:
    void setJointAngle(int joint_id, double angle_rad) override;
};

#endif