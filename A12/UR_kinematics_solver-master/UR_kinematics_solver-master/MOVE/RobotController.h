// RobotController.hpp
#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "UR10Kinematics.hpp"
#include "MotorInterface.hpp"
#include <memory>
#include <array>

class RobotController {
public:
    RobotController(std::shared_ptr<MotorInterface> motor);
    void moveToPose(const std::array<double,6>& target_pose); // [x,y,z,rx,ry,rz] 弧度
    void setCurrentAngles(const std::array<double,6>& q); // 设置当前角度（可选）
    std::array<double,6> getCurrentAngles() const;

private:
    std::shared_ptr<MotorInterface> motor_;
    UR10Kinematics kinematics_;
    std::array<double,6> current_q_;
    int interpolation_steps_ = 20; // 插值步数，可调
};

#endif