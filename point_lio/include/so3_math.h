/**
 * @file so3_math.h
 * @brief SO(3)特殊正交群数学库
 *
 * 本文件提供了SO(3)李群和李代数的常用数学运算函数，主要用于三维旋转的表示和计算。
 * 包括以下核心功能：
 * - 反对称矩阵（skew-symmetric matrix）的构造
 * - 李代数到李群的指数映射（Exponential map）- Rodrigues公式
 * - 李群到李代数的对数映射（Logarithm map）
 * - 旋转矩阵与欧拉角的相互转换
 * - 右雅可比矩阵的逆（Right Jacobian inverse）
 *
 * SO(3): Special Orthogonal Group in 3D - 三维空间中的特殊正交群，表示所有三维旋转矩阵的集合
 * so(3): Lie Algebra of SO(3) - SO(3)的李代数，表示为反对称矩阵或三维向量
 */

#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

// #include <common_lib.h>

/**
 * @brief 反对称矩阵宏定义
 * @param v 三维向量 [v0, v1, v2]
 * @return 展开为3x3反对称矩阵的元素序列（按行优先）
 *
 * 反对称矩阵形式：
 * [  0  -v2  v1 ]
 * [ v2   0  -v0 ]
 * [-v1  v0   0  ]
 *
 * 用于将三维向量映射到so(3)李代数空间
 */
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/**
 * @brief 构造反对称矩阵（Skew-symmetric matrix）
 * @tparam T 数据类型（如double, float）
 * @param v 输入的三维向量
 * @return 3x3反对称矩阵
 *
 * 将三维向量v转换为对应的反对称矩阵，满足性质：skew(v) * x = v × x（叉乘）
 * 这是李代数so(3)的矩阵表示形式
 */
template<typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat<<0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0;
    return skew_sym_mat;
}

/**
 * @brief SO(3)的指数映射 - 将旋转向量转换为旋转矩阵（Rodrigues公式）
 * @tparam T 数据类型
 * @param ang 旋转向量（轴角表示），其方向为旋转轴，模长为旋转角度
 * @return 对应的3x3旋转矩阵
 *
 * 使用Rodrigues公式：R = I + sin(θ)*K + (1-cos(θ))*K²
 * 其中：θ为旋转角度，K为单位旋转轴的反对称矩阵
 *
 * 当旋转角度接近0时，返回单位矩阵（无旋转）
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang)
{
    T ang_norm = ang.norm();  // 计算旋转角度（向量的模）
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
        return Eye3;  // 旋转角度趋近于0时返回单位矩阵
    }
}

/**
 * @brief SO(3)的指数映射（带时间步长）- 将角速度和时间间隔转换为旋转矩阵
 * @tparam T 数据类型
 * @tparam Ts 时间步长数据类型
 * @param ang_vel 角速度向量（rad/s），方向为旋转轴，模长为角速度大小
 * @param dt 时间间隔（秒）
 * @return 在时间dt内旋转ang_vel所对应的旋转矩阵
 *
 * 通过角速度和时间步长计算旋转矩阵：R = Exp(ω * dt)
 * 其中旋转角度 θ = ||ω|| * dt
 *
 * 常用于IMU角速度积分更新旋转状态
 */
template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();  // 计算角速度的模（角速度大小）
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;  // 归一化得到旋转轴
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);  // 构造旋转轴的反对称矩阵

        T r_ang = ang_vel_norm * dt;  // 计算总旋转角度 = 角速度 × 时间

        /// Rodrigues变换公式
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;  // 角速度趋近于0时返回单位矩阵（无旋转）
    }
}

/**
 * @brief SO(3)的指数映射（标量参数版本）- 将三个标量组成的旋转向量转换为旋转矩阵
 * @tparam T 数据类型
 * @param v1 旋转向量的x分量
 * @param v2 旋转向量的y分量
 * @param v3 旋转向量的z分量
 * @return 对应的3x3旋转矩阵
 *
 * 功能与第一个Exp函数相同，但接受三个标量参数而非向量
 * 方便在某些场景下直接传入分量值
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
        K << SKEW_SYM_MATRX(r_ang);  // 构造旋转轴的反对称矩阵

        /// Rodrigues变换公式
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;  // 旋转角度趋近于0时返回单位矩阵
    }
}

/**
 * @brief SO(3)的对数映射 - 将旋转矩阵转换为旋转向量
 * @tparam T 数据类型
 * @param R 输入的3x3旋转矩阵
 * @return 对应的旋转向量（轴角表示）
 *
 * 这是指数映射的逆运算，将旋转矩阵R映射回李代数so(3)
 *
 * 计算步骤：
 * 1. 通过trace(R)计算旋转角度：θ = arccos((trace(R)-1)/2)
 * 2. 通过反对称部分提取旋转轴：K = (R - R^T) / 2
 * 3. 返回旋转向量：ω = (θ / sin(θ)) * vex(K)
 *
 * 当旋转角度接近0时使用一阶近似
 */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> R)
{
    // 通过迹计算旋转角度，trace(R) = 1 + 2*cos(θ)
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    // 提取反对称部分：(R - R^T)/2 对应的向量形式
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    // 小角度近似：θ→0时，θ/sin(θ)→1，使用1/2作为近似值
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/**
 * @brief 将旋转矩阵转换为欧拉角（ZYX顺序，即RPY - Roll-Pitch-Yaw）
 * @tparam T 数据类型
 * @param rot 输入的3x3旋转矩阵
 * @return 欧拉角向量 [roll, pitch, yaw]（单位：弧度）
 *
 * 欧拉角定义（ZYX内旋顺序）：
 * - x (roll):  绕X轴旋转角度（横滚角）
 * - y (pitch): 绕Y轴旋转角度（俯仰角）
 * - z (yaw):   绕Z轴旋转角度（偏航角）
 *
 * 注意：
 * - 当pitch接近±90度时存在万向锁（Gimbal Lock）奇异性
 * - 奇异情况下yaw角被设置为0，roll角包含了yaw的信息
 */
template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    // 计算sy用于判断是否接近奇异点（pitch ≈ ±90°）
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;  // 判断是否处于万向锁奇异位置
    T x, y, z;
    if(!singular)  // 非奇异情况
    {
        x = atan2(rot(2, 1), rot(2, 2));   // roll  = atan2(r32, r33)
        y = atan2(-rot(2, 0), sy);         // pitch = atan2(-r31, sy)
        z = atan2(rot(1, 0), rot(0, 0));   // yaw   = atan2(r21, r11)
    }
    else  // 奇异情况处理（万向锁）
    {
        x = atan2(-rot(1, 2), rot(1, 1)); // roll  = atan2(-r23, r22)
        y = atan2(-rot(2, 0), sy);        // pitch = atan2(-r31, sy)
        z = 0;  // yaw设为0，其信息已融入roll中
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

/**
 * @brief 计算SO(3)右雅可比矩阵的逆
 * @tparam T 数据类型（模板参数，但实际返回double类型）
 * @param vec 旋转向量（轴角表示）
 * @return 右雅可比矩阵的逆 Jr^(-1)
 *
 * 右雅可比矩阵用于李群和李代数之间的线性化关系：
 * dR = R * Exp(Jr * dφ)
 *
 * 右雅可比逆的计算公式：
 * Jr^(-1) = I + (1/2)*[φ]× + (1/||φ||² - (1+cos(||φ||))/(2||φ||sin(||φ||))) * [φ]×²
 *
 * 应用场景：
 * - 扩展卡尔曼滤波（EKF）中的误差状态更新
 * - 优化问题中旋转的一阶近似
 * - 将李代数空间的扰动映射到李群空间
 *
 * 当旋转角度接近0时，返回单位矩阵
 */
template<typename T>
Eigen::Matrix3d Jacob_right_inv(Eigen::Vector3d &vec){
    Eigen::Matrix3d hat_v, res;
    hat_v << SKEW_SYM_MATRX(vec);  // 构造旋转向量的反对称矩阵
    if(vec.norm() > 1e-6)  // 非小角度情况
    {
        // 右雅可比逆的完整公式
        res = Eigen::Matrix<double, 3, 3>::Identity() + 0.5 * hat_v + (1 - vec.norm() * std::cos(vec.norm() / 2) / 2 / std::sin(vec.norm() / 2)) * hat_v * hat_v / vec.squaredNorm();
    }
    else  // 小角度近似
    {
        res = Eigen::Matrix<double, 3, 3>::Identity();  // 小角度时近似为单位矩阵
    }
    return res;
}

#endif
