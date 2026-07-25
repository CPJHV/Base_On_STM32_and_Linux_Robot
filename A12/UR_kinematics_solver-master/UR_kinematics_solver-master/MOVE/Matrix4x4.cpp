#include "Matrix4x4.hpp"
#include <cstdio>
#include <algorithm>

Matrix4x4::Matrix4x4() {
    for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j)
            m[i][j] = 0.0;
}

Matrix4x4 Matrix4x4::identity() {
    Matrix4x4 I;
    for (int i=0; i<4; ++i) I.m[i][i] = 1.0;
    return I;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
    Matrix4x4 result;
    for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j)
            for (int k=0; k<4; ++k)
                result.m[i][j] += m[i][k] * other.m[k][j];
    return result;
}

Matrix4x4 Matrix4x4::inverse() const {
    // 高斯-约旦消元求逆（增广矩阵 [A|I]）
    double aug[4][8];
    for (int i=0; i<4; ++i) {
        for (int j=0; j<4; ++j) aug[i][j] = m[i][j];
        for (int j=4; j<8; ++j) aug[i][j] = (i == (j-4)) ? 1.0 : 0.0;
    }

    for (int col=0; col<4; ++col) {
        // 寻找主元
        int pivot = col;
        for (int i=col+1; i<4; ++i)
            if (fabs(aug[i][col]) > fabs(aug[pivot][col])) pivot = i;
        if (fabs(aug[pivot][col]) < 1e-12) {
            // 奇异矩阵，返回单位矩阵（或抛出异常）
            return identity();
        }
        // 交换行
        if (pivot != col) {
            for (int j=0; j<8; ++j) std::swap(aug[col][j], aug[pivot][j]);
        }
        // 主元归一
        double pivotVal = aug[col][col];
        for (int j=0; j<8; ++j) aug[col][j] /= pivotVal;
        // 消去其他行
        for (int i=0; i<4; ++i) {
            if (i == col) continue;
            double factor = aug[i][col];
            for (int j=0; j<8; ++j)
                aug[i][j] -= factor * aug[col][j];
        }
    }

    Matrix4x4 inv;
    for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j)
            inv.m[i][j] = aug[i][j+4];
    return inv;
}

void Matrix4x4::print() const {
    for (int i=0; i<4; ++i) {
        for (int j=0; j<4; ++j) printf("%8.4f ", m[i][j]);
        printf("\n");
    }
}