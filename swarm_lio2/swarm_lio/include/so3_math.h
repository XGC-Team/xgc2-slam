/*
 * SO3_MATH.H
 * SO(3)旋转群数学库 - 提供旋转矩阵相关的数学运算
 * 主要功能：
 * - 反对称矩阵构造
 * - 指数映射（李代数到李群的转换）
 * - 对数映射（李群到李代数的转换）
 * - 欧拉角与旋转矩阵的相互转换
 */

#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

// 反对称矩阵宏定义，用于将3维向量v转换为3x3反对称矩阵的展开形式
// 反对称矩阵性质: A^T = -A, 用于表示叉乘运算 [v]×
// 对于向量v=[v0,v1,v2], 其反对称矩阵为: [0, -v2, v1; v2, 0, -v0; -v1, v0, 0]
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/**
 * @brief 构造反对称矩阵（skew-symmetric matrix）
 * @param v 输入的3维向量
 * @return 返回对应的3x3反对称矩阵
 *
 * 功能说明：将3维向量转换为其对应的反对称矩阵形式
 * 数学原理：对于向量v=[x,y,z], 其反对称矩阵[v]×满足: [v]×u = v×u (叉乘)
 * 应用场景：用于旋转运算中的李代数表示
 */
template<typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat<<0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0;
    return skew_sym_mat;
}

/**
 * @brief 指数映射 - 将旋转向量（李代数so(3)）转换为旋转矩阵（李群SO(3)）
 * @param ang 旋转向量（轴角表示），向量方向为旋转轴，模长为旋转角度
 * @return 返回对应的3x3旋转矩阵
 *
 * 数学原理：使用Rodrigues公式进行指数映射
 * 公式: R = I + sin(θ)*K + (1-cos(θ))*K^2
 * 其中：θ为旋转角度，K为单位旋转轴的反对称矩阵
 *
 * 特殊情况：当旋转角度接近0时，直接返回单位矩阵
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang)
{
    T ang_norm = ang.norm();  // 计算旋转角度（向量的模长）
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;  // 归一化得到旋转轴
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);  // 构造旋转轴的反对称矩阵
        /// Rodrigues变换公式
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;  // 角度接近0，返回单位矩阵（无旋转）
    }
}

/**
 * @brief 指数映射（带时间步长） - 将角速度转换为旋转矩阵
 * @param ang_vel 角速度向量（rad/s），向量方向为旋转轴，模长为角速度大小
 * @param dt 时间步长（秒）
 * @return 返回在dt时间内旋转的旋转矩阵
 *
 * 功能说明：根据角速度和时间间隔计算旋转矩阵
 * 计算过程：先计算旋转角度 θ = ||ω|| * dt，然后应用Rodrigues公式
 * 应用场景：IMU积分、运动预测等需要将角速度转换为姿态变化的场景
 */
template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();  // 计算角速度的模长
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;  // 归一化得到旋转轴
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);  // 构造旋转轴的反对称矩阵

        T r_ang = ang_vel_norm * dt;  // 计算旋转角度 = 角速度 × 时间

        /// Rodrigues变换公式
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;  // 角速度接近0，返回单位矩阵（无旋转）
    }
}

/**
 * @brief 指数映射（分量形式） - 将旋转向量的三个分量转换为旋转矩阵
 * @param v1 旋转向量的x分量
 * @param v2 旋转向量的y分量
 * @param v3 旋转向量的z分量
 * @return 返回对应的3x3旋转矩阵
 *
 * 功能说明：Exp函数的重载版本，直接接受三个标量参数
 * 适用场景：当旋转向量以分量形式给出时使用
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);  // 计算旋转角度
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};  // 归一化得到旋转轴
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);  // 构造反对称矩阵

        /// Rodrigues变换公式
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;  // 角度接近0，返回单位矩阵
    }
}

/**
 * @brief 计算A矩阵 - 用于误差状态卡尔曼滤波中的雅可比矩阵
 * @param ang_vel 角速度向量或旋转向量
 * @return 返回A矩阵（3x3）
 *
 * 数学原理：计算右雅可比矩阵的逆矩阵（或其近似）
 * 公式: A = I + ((1-cos(θ))/θ)*K + (1-sin(θ)/θ)*K^2
 * 其中：θ为旋转角度，K为单位旋转轴的反对称矩阵
 *
 * 应用场景：
 * - 在ESKF（误差状态卡尔曼滤波）中用于计算状态转移矩阵
 * - 用于旋转误差的线性化
 */
template<typename T>
Eigen::Matrix<T, 3, 3> A_cal(const Eigen::Matrix<T, 3, 1> & ang_vel)
{
    T norm = ang_vel.norm();  // 计算角度或角速度的模长
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        Eigen::Matrix<T, 3, 1> r_ang = ang_vel / norm;  // 归一化得到旋转轴
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);  // 构造反对称矩阵
        // 计算A矩阵：I + ((1-cos(θ))/θ)*K + (1-sin(θ)/θ)*K^2
        return Eye3  + ((1.0 - std::cos(norm))/norm) * K + (1.0 - std::sin(norm)/norm) * K * K ;
    }
    else
    {
        return Eye3;  // 角度接近0时返回单位矩阵
    }
}

/**
 * @brief 对数映射 - 将旋转矩阵（李群SO(3)）转换为旋转向量（李代数so(3)）
 * @param R 输入的3x3旋转矩阵
 * @return 返回对应的旋转向量（轴角表示）
 *
 * 数学原理：旋转矩阵的对数映射，是指数映射的逆运算
 * 计算步骤：
 * 1. 根据迹(trace)计算旋转角度：θ = arccos((trace(R)-1)/2)
 * 2. 从旋转矩阵的反对称部分提取旋转轴
 * 3. 组合得到旋转向量 = (θ/sin(θ)) * 反对称部分
 *
 * 特殊情况：
 * - 当θ接近0时，使用近似公式避免除零
 * - 当trace(R)接近3时，表示接近单位矩阵（无旋转）
 */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
    // 计算旋转角度：θ = arccos((trace(R)-1)/2)
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    // 提取反对称部分构造向量K
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    // 根据θ的大小选择不同的公式，避免数值不稳定
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/**
 * @brief 旋转矩阵转欧拉角 - 将旋转矩阵转换为ZYX欧拉角（roll-pitch-yaw）
 * @param rot 输入的3x3旋转矩阵
 * @return 返回欧拉角向量 [roll, pitch, yaw]
 *
 * 旋转顺序：ZYX (yaw-pitch-roll)，即先绕Z轴旋转yaw，再绕Y轴旋转pitch，最后绕X轴旋转roll
 *
 * 计算方法：
 * - roll (x轴旋转)  = atan2(R_21, R_22)
 * - pitch (y轴旋转) = atan2(-R_20, sqrt(R_00^2 + R_10^2))
 * - yaw (z轴旋转)   = atan2(R_10, R_00)
 *
 * 万向锁处理：当pitch接近±90度时，会出现万向锁（gimbal lock）奇异情况
 * 此时将yaw设为0，只计算roll和pitch的组合效果
 */
template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));  // 计算sqrt(R_00^2 + R_10^2)
    bool singular = sy < 1e-6;  // 判断是否接近万向锁奇异点
    T x, y, z;
    if(!singular)  // 非奇异情况，正常计算欧拉角
    {
        x = atan2(rot(2, 1), rot(2, 2));      // roll角
        y = atan2(-rot(2, 0), sy);            // pitch角
        z = atan2(rot(1, 0), rot(0, 0));      // yaw角，范围 -pi ~ pi
    }
    else  // 奇异情况（万向锁），设yaw=0
    {
        x = atan2(-rot(1, 2), rot(1, 1));     // roll角
        y = atan2(-rot(2, 0), sy);            // pitch角
        z = 0;                                 // yaw角设为0
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

/**
 * @brief 欧拉角转旋转矩阵 - 将ZYX欧拉角（roll-pitch-yaw）转换为旋转矩阵
 * @param theta 欧拉角向量 [roll, pitch, yaw]
 * @return 返回对应的3x3旋转矩阵
 *
 * 旋转顺序：ZYX (yaw-pitch-roll)
 * 计算方法：R = R_z(yaw) * R_y(pitch) * R_x(roll)
 * 即：先绕X轴旋转roll，再绕Y轴旋转pitch，最后绕Z轴旋转yaw
 *
 * 应用场景：
 * - 将欧拉角表示的姿态转换为旋转矩阵
 * - 姿态初始化
 * - 坐标系转换
 */
template<typename T>
Eigen::Matrix<T, 3, 3> EulerToRotM(const Eigen::Matrix<T, 3, 1> &theta)
{
    // 分别构造绕X、Y、Z轴的旋转矩阵
    Eigen::Matrix<T, 3, 3> R_x = Eigen::AngleAxis<T>(theta(0),Eigen::Matrix<T, 3, 1>(1,0,0)).toRotationMatrix();
    Eigen::Matrix<T, 3, 3> R_y = Eigen::AngleAxis<T>(theta(1),Eigen::Matrix<T, 3, 1>(0,1,0)).toRotationMatrix();
    Eigen::Matrix<T, 3, 3> R_z = Eigen::AngleAxis<T>(theta(2),Eigen::Matrix<T, 3, 1>(0,0,1)).toRotationMatrix();
    // 按ZYX顺序组合旋转
    return R_z*R_y*R_x;
}



#endif
