/**
 * @file eigen_types.h
 * @brief Eigen类型别名定义文件
 * @author xiang
 * @date 2021/7/16
 *
 * @details 本文件为Point-LIO/FASTER-LIO系统提供常用Eigen类型的别名定义
 * 主要包括：
 * - 整数向量类型（Vec2i, Vec3i）
 * - 浮点数向量类型（Vec2d/f, Vec3d/f, Vec5d/f, Vec6d/f, Vec15d）
 * - 矩阵类型（Mat1d到Mat15d的各种维度）
 * - 四元数类型（Quatd, Quatf）
 * - 向量比较和哈希函数模板（用于STL容器）
 *
 * 这些类型别名简化了代码书写，提高了代码可读性
 */

#ifndef FASTER_LIO_EIGEN_TYPES_H
#define FASTER_LIO_EIGEN_TYPES_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/// Eigen类型别名定义

// ========== 整数向量类型 ==========
using Vec2i = Eigen::Vector2i;  ///< 2维整数向量，常用于图像坐标、网格索引
using Vec3i = Eigen::Vector3i;  ///< 3维整数向量，常用于体素网格索引

// ========== 浮点数向量类型 ==========
using Vec2d = Eigen::Vector2d;  ///< 2维双精度浮点向量
using Vec2f = Eigen::Vector2f;  ///< 2维单精度浮点向量
using Vec3d = Eigen::Vector3d;  ///< 3维双精度浮点向量，常用于3D位置、速度、加速度等
using Vec3f = Eigen::Vector3f;  ///< 3维单精度浮点向量
using Vec5d = Eigen::Matrix<double, 5, 1>;  ///< 5维双精度向量
using Vec5f = Eigen::Matrix<float, 5, 1>;   ///< 5维单精度向量
using Vec6d = Eigen::Matrix<double, 6, 1>;  ///< 6维双精度向量，常用于位姿（位置+姿态）
using Vec6f = Eigen::Matrix<float, 6, 1>;   ///< 6维单精度向量
using Vec15d = Eigen::Matrix<double, 15, 15>;  ///< 15维双精度向量，常用于ESKF状态向量

// ========== 矩阵类型 ==========
using Mat1d = Eigen::Matrix<double, 1, 1>;  ///< 1x1双精度矩阵（标量）
using Mat3d = Eigen::Matrix3d;  ///< 3x3双精度矩阵，常用于旋转矩阵、协方差矩阵
using Mat3f = Eigen::Matrix3f;  ///< 3x3单精度矩阵
using Mat4d = Eigen::Matrix4d;  ///< 4x4双精度矩阵，常用于齐次变换矩阵
using Mat4f = Eigen::Matrix4f;  ///< 4x4单精度矩阵
using Mat5d = Eigen::Matrix<double, 5, 5>;  ///< 5x5双精度矩阵
using Mat5f = Eigen::Matrix<float, 5, 5>;   ///< 5x5单精度矩阵
using Mat6d = Eigen::Matrix<double, 6, 6>;  ///< 6x6双精度矩阵，常用于6自由度协方差
using Mat6f = Eigen::Matrix<float, 6, 6>;   ///< 6x6单精度矩阵
using Mat15d = Eigen::Matrix<double, 15, 15>;  ///< 15x15双精度矩阵，常用于ESKF协方差矩阵

// ========== 四元数类型 ==========
using Quatd = Eigen::Quaterniond;  ///< 双精度四元数，用于表示3D旋转
using Quatf = Eigen::Quaternionf;  ///< 单精度四元数

namespace faster_lio {

/**
 * @brief 向量比较函数模板
 * @tparam N 向量维度
 *
 * 用于在STL容器（如std::map, std::set）中对整数向量进行排序
 * 实现字典序比较：先比较第一个元素，相等则比较第二个，以此类推
 */
template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

/**
 * @brief 向量哈希函数模板
 * @tparam N 向量维度
 *
 * 用于在哈希容器（如std::unordered_map, std::unordered_set）中存储整数向量
 * 实现高效的空间哈希算法
 */
template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

// ========== 模板函数实现 ==========

/**
 * @brief 2维整数向量的比较运算符特化
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 如果v1 < v2（字典序）则返回true
 *
 * 字典序比较：首先比较x分量，如果相等则比较y分量
 */
template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

/**
 * @brief 3维整数向量的比较运算符特化
 * @param v1 第一个向量
 * @param v2 第二个向量
 * @return 如果v1 < v2（字典序）则返回true
 *
 * 字典序比较：依次比较x、y、z分量
 */
template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) && (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

/**
 * @brief 2维整数向量的哈希函数特化
 * @param v 输入向量
 * @return 哈希值
 *
 * 使用优化的空间哈希算法，利用大质数进行异或运算
 * 该算法在空间数据结构中广泛应用，能够有效减少哈希冲突
 * @see Optimized Spatial Hashing for Collision Detection of Deformable Objects,
 *      Matthias Teschner et. al., VMV 2003
 */
template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
}

/**
 * @brief 3维整数向量的哈希函数特化
 * @param v 输入向量
 * @return 哈希值
 *
 * 使用优化的3D空间哈希算法，对三个坐标分量分别乘以不同大质数后进行异或
 * 73856093, 471943, 83492791是精心选择的质数，能够在3D空间中提供良好的哈希分布
 */
template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
}

/**
 * 注释掉的代码：使用C++11 lambda表达式实现的2D向量比较函数
 * 功能与less_vec<2>相同，但使用现代C++语法
 * 可能因为需要与旧版本编译器兼容而被注释
 */
// constexpr auto less_vec2i = [](const Vec2i& v1, const Vec2i& v2) {
//     return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
// };

}  // namespace faster_lio

#endif
