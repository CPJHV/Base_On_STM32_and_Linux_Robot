#include "UR10Kinematics.hpp"
#include <cmath>
#include <algorithm>
#include <cfloat>

// 辅助函数：裁剪到 [-1,1]
static double clip(double v) {
    if (v > 1.0) return 1.0;
    if (v < -1.0) return -1.0;
    return v;
}

UR10Kinematics::UR10Kinematics() {
    // UR10 参数
    d[0] = 0.1273;   d[1] = 0.0;     d[2] = 0.0;
    d[3] = 0.163941; d[4] = 0.1157;  d[5] = 0.0922;

    a[0] = 0.0;      a[1] = -0.612;  a[2] = -0.5723;
    a[3] = 0.0;      a[4] = 0.0;     a[5] = 0.0;

    alpha[0] = M_PI/2;  alpha[1] = 0.0;    alpha[2] = 0.0;
    alpha[3] = M_PI/2;  alpha[4] = -M_PI/2; alpha[5] = 0.0;
}

Matrix4x4 UR10Kinematics::HTM(int i, const std::array<double,6>& theta) const {
    double ct = cos(theta[i]);
    double st = sin(theta[i]);
    double ca = cos(alpha[i]);
    double sa = sin(alpha[i]);

    Matrix4x4 T;
    T.m[0][0] = ct;         T.m[0][1] = -st*ca;   T.m[0][2] = st*sa;    T.m[0][3] = a[i]*ct;
    T.m[1][0] = st;         T.m[1][1] = ct*ca;    T.m[1][2] = -ct*sa;   T.m[1][3] = a[i]*st;
    T.m[2][0] = 0.0;        T.m[2][1] = sa;       T.m[2][2] = ca;       T.m[2][3] = d[i];
    T.m[3][0] = 0.0;        T.m[3][1] = 0.0;      T.m[3][2] = 0.0;      T.m[3][3] = 1.0;
    return T;
}

Matrix4x4 UR10Kinematics::forwardKinematics(const std::array<double,6>& theta) const {
    Matrix4x4 T = Matrix4x4::identity();
    for (int i=0; i<6; ++i)
        T = T * HTM(i, theta);
    return T;
}

// 实现逆运动学（完全照搬 Python 逻辑）
std::array<double,6> UR10Kinematics::inverseKinematics(const Matrix4x4& T06,
                                                       const std::array<double,6>& q_current) const {
    // 存储 8 组解
    std::array<std::array<double,6>, 8> theta;//2*2*2组解法
    //这里的theta1是表示角度
    // ---------- 计算 theta1 ----------
    double P05_x = T06.m[0][3] - d[5] * T06.m[0][2];
    double P05_y = T06.m[1][3] - d[5] * T06.m[1][2];
    double P05_z = T06.m[2][3] - d[5] * T06.m[2][2]; 
    //因为电机5与电机6挨着，你得到的T06其实是第6个电机的位置，而我们想知道第5个电机的位置和姿态矩阵的话
    //所以根据结构
    /*
    |下面这个点就是T06了，而要得到5的就是沿着6的Z轴*距离就是沿着z轴移动d6长度，而这里d[5]实则是d6，那么位置就拿到了
    .
    | d6|
    ---[6]
        |
        |
       [5]
    */                                                    
    double phi1 = atan2(P05_y, P05_x);//这里是利用到运动控制学推导出来的atan2（y，x）就得到了角度1了
    //这里其实就是反切atan y/x得到角度而已
    /*
    因为第一个轴是基座，所以它要明确目标的下的第第5个轴位于第一个电机的角度
    这就好像从z轴俯视下去看
    */
    double r = sqrt(P05_x*P05_x + P05_y*P05_y);
    double val = d[3] / r;//这里是d4/r
    val = clip(val);//限制比值在-1到1之间
    double phi2 = acos(val);
    //算出来一个补角
    /*
    |d4  |
    5----4
         |
         3
         |
         2
         |
         1
     所以acos d4/r=角度序号154了；

    */
    /*
    这里我讲解一下原理：
    比如我的水瓶在我身体的右侧45度位置，也就是电机1到目标的phi1角度
    而我的手臂在我的身体右侧的50度位置，也就是电机1到电机5的phi2角度
    那我只需要身体转向50-45度即可抓到，否则你身体转向45度，你只能看着水平，但你够不着
    
    这里两个解是因为，你可以左手去抓，同理是因为这个机械结构不同
    有的是在左边，有的在右边
    */
    double t1_1 = M_PI/2 + phi1 + phi2;
    double t1_2 = M_PI/2 + phi1 - phi2;

    for (int i=0; i<4; ++i) theta[i][0] = t1_1;
    for (int i=4; i<8; ++i) theta[i][0] = t1_2;

    // ---------- 计算 theta5 ----------
    double P06_x = T06.m[0][3];
    double P06_y = T06.m[1][3];
    double P06_z = T06.m[2][3];

    double t5_arg1 = (P06_x * sin(t1_1) - P06_y * cos(t1_1) - d[3]) / d[5];
    t5_arg1 = clip(t5_arg1);
    double t5_1 = acos(t5_arg1);

    double t5_arg2 = (P06_x * sin(t1_2) - P06_y * cos(t1_2) - d[3]) / d[5];
    t5_arg2 = clip(t5_arg2);
    double t5_2 = acos(t5_arg2);

    for (int i=0; i<2; ++i) {
        theta[2*i][4] = t5_1;
        theta[2*i+1][4] = -t5_1;
        theta[2*i+4][4] = t5_2;
        theta[2*i+5][4] = -t5_2;
    }

    // ---------- 计算 theta6 ----------
    Matrix4x4 T60 = T06.inverse();
    double t6_list[4];
    int idx6 = 0;
    for (int i1=0; i1<2; ++i1) {
        double t1 = (i1==0) ? t1_1 : t1_2;
        for (int i5=0; i5<2; ++i5) {
            double t5 = (i5==0) ? t5_1 : -t5_1;
            double s1 = sin(t1);
            double c1 = cos(t1);
            double s5 = sin(t5);
            double num = (-T60.m[1][0] * s1 + T60.m[1][1] * c1) / s5;
            double den = (T60.m[0][0] * s1 - T60.m[0][1] * c1) / s5;
            t6_list[idx6++] = atan2(num, den);
        }
    }

    for (int i=0; i<2; ++i) {
        theta[i][5] = t6_list[0];
        theta[i+2][5] = t6_list[1];
        theta[i+4][5] = t6_list[2];
        theta[i+6][5] = t6_list[3];
    }

    // ---------- 计算 theta3, theta2, theta4 ----------
    for (int sol=0; sol<8; ++sol) {
        // T46
        Matrix4x4 T46 = HTM(4, theta[sol]) * HTM(5, theta[sol]);
        // T14
        Matrix4x4 T01 = HTM(0, theta[sol]);
        Matrix4x4 T14 = T01.inverse() * T06 * T46.inverse();

        // P13
        Matrix4x4 tmp = T14 * Matrix4x4::identity(); // 仅用于乘法，我们直接计算向量
        double P13_x = T14.m[0][0]*0 + T14.m[0][1]*(-d[3]) + T14.m[0][2]*0 + T14.m[0][3]*1;
        double P13_y = T14.m[1][0]*0 + T14.m[1][1]*(-d[3]) + T14.m[1][2]*0 + T14.m[1][3]*1;
        double P13_z = T14.m[2][0]*0 + T14.m[2][1]*(-d[3]) + T14.m[2][2]*0 + T14.m[2][3]*1;
        // 减去原点 [0,0,0,1] -> 实际上已经包含了，不需要额外减，因为上式计算的是从原点出发
        // 但原Python有减去 np.array([[0,0,0,1]]).T，由于我们计算的是 T14 * [0,-d4,0,1]^T，
        // 已经得到的是关节3到关节1的向量，不需要再减。
        double p13_x = P13_x;
        double p13_y = P13_y;
        double norm_p13 = sqrt(p13_x*p13_x + p13_y*p13_y);

        // theta3
        double cost3 = (norm_p13*norm_p13 - a[1]*a[1] - a[2]*a[2]) / (2 * a[1] * a[2]);
        cost3 = clip(cost3);
        double t3_pos = -acos(cost3);
        double t3_neg = -t3_pos;
        if (sol % 2 == 0) // sol in [0,2,4,6]
            theta[sol][2] = t3_pos;
        else
            theta[sol][2] = t3_neg;

        // theta2
        double t3_cur = theta[sol][2];
        double sin_t3 = sin(t3_cur);
        double t2 = -atan2(p13_y, -p13_x) + asin(a[2] * sin_t3 / norm_p13);
        theta[sol][1] = t2;

        // theta4
        Matrix4x4 T13 = HTM(1, theta[sol]) * HTM(2, theta[sol]);
        Matrix4x4 T34 = T13.inverse() * T14;
        double t4 = atan2(T34.m[1][0], T34.m[0][0]);
        theta[sol][3] = t4;
    }

    // 选择最优解
    return selectBest(theta, q_current);
}

std::array<double,6> UR10Kinematics::selectBest(
        const std::array<std::array<double,6>, 8>& solutions,
        const std::array<double,6>& q_current) const {
    double min_err = 1e30;
    int best_idx = 0;
    for (int i=0; i<8; ++i) {
        double err = 0.0;
        for (int j=0; j<6; ++j) {
            double diff = solutions[i][j] - q_current[j];
            //结果与当前角度相减
            err += diff * diff;//累加误差欧氏距离的平方
        }
        if (err < min_err) {
            min_err = err;
            best_idx = i;
        }
    }
    return solutions[best_idx];//选择最小的那个，也就是避免电机转动角度过大
}

// 转换 UR 格式 [x,y,z,rx,ry,rz] 到齐次矩阵
Matrix4x4 UR10Kinematics::urPoseToMatrix(const std::array<double,6>& ur_pose) {
    double x = ur_pose[0], y = ur_pose[1], z = ur_pose[2];
    double rx = ur_pose[3], ry = ur_pose[4], rz = ur_pose[5];
    //获取数据先

    Matrix4x4 R = Matrix4x4::identity();
    /*
    这里获取了一个4*4的单位矩阵
    */

    // 绕X
    if (fabs(rx) > 1e-12) {
        double cx = cos(rx), sx = sin(rx);

        Matrix4x4 Rx;//4*4的旋转矩阵
        Rx.m[0][0]=1; Rx.m[0][1]=0; Rx.m[0][2]=0; Rx.m[0][3]=0;
        Rx.m[1][0]=0; Rx.m[1][1]=cx; Rx.m[1][2]=-sx; Rx.m[1][3]=0;
        Rx.m[2][0]=0; Rx.m[2][1]=sx; Rx.m[2][2]=cx; Rx.m[2][3]=0;
        Rx.m[3][0]=0; Rx.m[3][1]=0; Rx.m[3][2]=0; Rx.m[3][3]=1;
        /*
        旋转矩阵是有固定格式的，这个是利用的是映射的方式计算旋转矩阵的
        */
        R = R * Rx;
        //T'=T*R
    }
    // 绕Y
    if (fabs(ry) > 1e-12) {
        double cy = cos(ry), sy = sin(ry);
        Matrix4x4 Ry;
        Ry.m[0][0]=cy; Ry.m[0][1]=0; Ry.m[0][2]=sy; Ry.m[0][3]=0;
        Ry.m[1][0]=0;  Ry.m[1][1]=1; Ry.m[1][2]=0;  Ry.m[1][3]=0;
        Ry.m[2][0]=-sy;Ry.m[2][1]=0; Ry.m[2][2]=cy; Ry.m[2][3]=0;
        Ry.m[3][0]=0;  Ry.m[3][1]=0; Ry.m[3][2]=0;  Ry.m[3][3]=1;
        R = R * Ry;
    }
    // 绕Z
    if (fabs(rz) > 1e-12) {
        double cz = cos(rz), sz = sin(rz);
        Matrix4x4 Rz;
        Rz.m[0][0]=cz; Rz.m[0][1]=-sz; Rz.m[0][2]=0; Rz.m[0][3]=0;
        Rz.m[1][0]=sz; Rz.m[1][1]=cz;  Rz.m[1][2]=0; Rz.m[1][3]=0;
        Rz.m[2][0]=0;  Rz.m[2][1]=0;   Rz.m[2][2]=1; Rz.m[2][3]=0;
        Rz.m[3][0]=0;  Rz.m[3][1]=0;   Rz.m[3][2]=0; Rz.m[3][3]=1;
        R = R * Rz;
    }

    Matrix4x4 T = Matrix4x4::identity();
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            T.m[i][j] = R.m[i][j];
    //读取左边3*3的旋转矩阵
    T.m[0][3] = x;
    T.m[1][3] = y;
    T.m[2][3] = z;
    //读取最后一列的位置矩阵
    return T;
}