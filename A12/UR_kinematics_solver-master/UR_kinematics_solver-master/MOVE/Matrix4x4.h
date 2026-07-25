#ifndef MATRIX4X4_HPP
#define MATRIX4X4_HPP

#include <array>
#include <cmath>

struct Matrix4x4 {
    double m[4][4];

    Matrix4x4();
    static Matrix4x4 identity();
    Matrix4x4 operator*(const Matrix4x4& other) const;
    Matrix4x4 inverse() const;        // 用高斯-约旦法求逆
    void print() const;               // 调试用
};

#endif