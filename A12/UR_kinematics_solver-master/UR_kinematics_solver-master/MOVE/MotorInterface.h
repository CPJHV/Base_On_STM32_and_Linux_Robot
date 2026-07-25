#ifndef MOTOR_INTERFACE_HPP
#define MOTOR_INTERFACE_HPP

#include <array>

class MotorInterface {
public:
    virtual ~MotorInterface() = default;
    // 设置关节角度（弧度）
    virtual void setJointAngle(int joint_id, double angle_rad) = 0;
};

#endif