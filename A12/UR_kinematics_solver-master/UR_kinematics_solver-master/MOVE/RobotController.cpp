// RobotController.cpp
#include "RobotController.hpp"
#include <thread>   // 如果需要延时，可包含
#include <chrono>

RobotController::RobotController(std::shared_ptr<MotorInterface> motor)
    : motor_(motor), current_q_{0,0,0,0,0,0} {}

void RobotController::moveToPose(const std::array<double,6>& target_pose) {

    Matrix4x4 T06 = UR10Kinematics::urPoseToMatrix(target_pose);
    //获取目标的姿态矩阵

    std::array<double,6> q_target = kinematics_.inverseKinematics(T06, current_q_);
    //开始逆运动学计算，获取目标角度

    // 线性插值
    //就像动画一样，一阵一阵的走
    for (int step = 0; step <= interpolation_steps_; ++step) {
        double t = (double)step / interpolation_steps_;
        std::array<double,6> q_interp;
        for (int j=0; j<6; ++j)
            q_interp[j] = current_q_[j] + t * (q_target[j] - current_q_[j]);
        //当前角度+t（进度）*(目标角度 - 当前角度)
        //也就是在当前角度上加，但是用t来表示这个动作完成度，防止直接就是1，电机瞬间完成
        // 发送给电机
        for (int j=0; j<6; ++j)
            motor_->setJointAngle(j, q_interp[j]);

        // 延时，使运动可见（根据实际情况调整或使用定时器）
        // 如果已有实时控制，可省略此延时
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    current_q_ = q_target;
}
//设置目前角度
void RobotController::setCurrentAngles(const std::array<double,6>& q) {
    current_q_ = q;
}

std::array<double,6> RobotController::getCurrentAngles() const {
    return current_q_;
}