#include "RobotController.hpp"
#include "RealMotorDriver.hpp"
#include <iostream>

int main() {
    // 创建电机驱动对象
    auto motor = std::make_shared<RealMotorDriver>();
    RobotController robot(motor);

    // 设置初始角度（根据你的机械臂初始位置，弧度）
    // 可根据需要调整
    robot.setCurrentAngles({0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0});

    // 目标点位列表（[x,y,z,rx,ry,rz] 弧度）
    std::vector<std::array<double,6>> target_poses = {
        {-0.4, -0.2, 0.6, 0, 0, 0},
        {-0.3, -0.5, 0.6, 0, 0, 0},
        {-0.1, -0.5, 0.7, 0, 0, 0},
        {0.1, -0.3, 0.7, 0, 0, 0},
        {-0.1, -0.2, 0.5, 0, 0, 0},
        {-0.4, -0.2, 0.6, 0, 0, 0}
    };

    // 循环运动
    while (true) {
        for (const auto& pose : target_poses) {
            std::cout << "Moving to pose: " << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;
            robot.moveToPose(pose);
            // 等待按键或其他条件继续
            // 可以增加延时或检查输入
        }
    }

    return 0;
}