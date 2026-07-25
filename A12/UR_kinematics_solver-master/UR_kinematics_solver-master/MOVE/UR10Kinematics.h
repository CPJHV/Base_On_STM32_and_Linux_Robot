#ifndef UR10_KINEMATICS_HPP
#define UR10_KINEMATICS_HPP

#include "Matrix4x4.hpp"
#include <array>
#include <vector>

class UR10Kinematics {
public:
    UR10Kinematics();

    // 正运动学：输入 6 个关节角（弧度），返回末端位姿矩阵
    Matrix4x4 forwardKinematics(const std::array<double,6>& theta) const;

    // 逆运动学：输入目标位姿矩阵和当前关节角（弧度），返回最优的一组角度
    std::array<double,6> inverseKinematics(const Matrix4x4& T06,
                                           const std::array<double,6>& q_current) const;

    // 工具：将 UR 格式 [x,y,z,rx,ry,rz] 转为矩阵（rx,ry,rz 为欧拉角，弧度）
    static Matrix4x4 urPoseToMatrix(const std::array<double,6>& ur_pose);

private:
    double d[6], a[6], alpha[6];

    Matrix4x4 HTM(int i, const std::array<double,6>& theta) const;
    std::array<double,6> selectBest(const std::array<std::array<double,6>, 8>& solutions,
                                    const std::array<double,6>& q_current) const;
};

#endif