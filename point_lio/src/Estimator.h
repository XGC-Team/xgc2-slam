/**
 * @file Estimator.h
 * @brief Point-LIO状态估计器头文件
 *
 * 本文件定义了Point-LIO算法中状态估计相关的核心函数和全局变量。
 * 主要功能包括：
 * - 基于误差卡尔曼滤波器(ESKF)的状态估计
 * - 点云特征处理和最近邻搜索
 * - IMU-LiDAR融合的观测模型
 * - 输入/输出状态的动力学模型
 * - 过程噪声协方差矩阵计算
 */

#ifndef Estimator_H
#define Estimator_H

#include "common_lib.h"
#include "parameters.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <ikd-Tree/ikd_Tree.h>
#include <pcl/io/pcd_io.h>
#include <unordered_set>

// ==================== 全局变量声明 ====================

extern PointCloudXYZI::Ptr normvec; // 存储平面法向量的点云，用于表示特征点所在平面的法向量
extern std::vector<int> time_seq; // 时间序列，记录点云帧的时间戳顺序
extern PointCloudXYZI::Ptr feats_down_body; // 降采样后的特征点云（在机体坐标系下）
extern PointCloudXYZI::Ptr feats_down_world; // 降采样后的特征点云（在世界坐标系下）
extern std::vector<V3D> pbody_list; // 机体坐标系下的点列表
extern std::vector<PointVector> Nearest_Points; // 最近邻点集合，用于构建平面约束
extern std::shared_ptr<IVoxType> ivox_; // iVox数据结构，用于高效存储和查询局部地图
extern std::vector<float> pointSearchSqDis; // 点搜索的平方距离，用于最近邻搜索
extern bool point_selected_surf[100000]; // 标记点是否被选为平面特征点的数组
extern std::vector<M3D> crossmat_list; // 反对称矩阵列表，用于点到平面距离的雅可比计算
extern int effct_feat_num; // 有效特征点数量
extern int k; // 迭代索引或计数器
extern int idx; // 通用索引变量
extern V3D angvel_avr, acc_avr, acc_avr_norm; // 角速度平均值、加速度平均值、归一化加速度平均值
extern int feats_down_size; // 降采样后的特征点数量
// extern std::vector<Eigen::Vector3d> normvec_holder;
extern V3D Lidar_T_wrt_IMU; // LiDAR相对于IMU的平移外参
extern M3D Lidar_R_wrt_IMU; // LiDAR相对于IMU的旋转外参
extern double G_m_s2; // 重力加速度常量（单位：m/s²）
extern input_ikfom input_in; // ESKF的输入状态（IMU测量值）

// ==================== 过程噪声协方差矩阵函数 ====================

/**
 * @brief 计算输入状态的过程噪声协方差矩阵
 * @return 24x24的过程噪声协方差矩阵，对应输入状态向量的维度
 * @note 输入状态包括：位置(3)、旋转(3)、速度(3)、IMU偏差等
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_input();

/**
 * @brief 计算输出状态的过程噪声协方差矩阵
 * @return 30x30的过程噪声协方差矩阵，对应输出状态向量的维度
 * @note 输出状态维度更高，包含更多的状态估计变量
 */
Eigen::Matrix<double, 30, 30> process_noise_cov_output();

// ==================== 动力学模型函数 ====================

//double L_offset_to_I[3] = {0.04165, 0.02326, -0.0284}; // Avia
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);

/**
 * @brief 计算输入状态的状态转移函数 f(x, u)
 * @param s 当前输入状态
 * @param in IMU输入测量值（角速度和加速度）
 * @return 24x1的状态导数向量
 * @note 实现ESKF的状态预测方程
 */
Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in);

/**
 * @brief 计算输出状态的状态转移函数 f(x, u)
 * @param s 当前输出状态
 * @param in IMU输入测量值
 * @return 30x1的状态导数向量
 */
Eigen::Matrix<double, 30, 1> get_f_output(state_output &s, const input_ikfom &in);

// ==================== 雅可比矩阵计算函数 ====================

/**
 * @brief 计算输入状态转移函数关于状态的雅可比矩阵 ∂f/∂x
 * @param s 当前输入状态
 * @param in IMU输入测量值
 * @return 24x24的雅可比矩阵
 * @note 用于ESKF的协方差传播
 */
Eigen::Matrix<double, 24, 24> df_dx_input(state_input &s, const input_ikfom &in);

// Eigen::Matrix<double, 24, 12> df_dw_input(state_input &s, const input_ikfom &in);

/**
 * @brief 计算输出状态转移函数关于状态的雅可比矩阵 ∂f/∂x
 * @param s 当前输出状态
 * @param in IMU输入测量值
 * @return 30x30的雅可比矩阵
 */
Eigen::Matrix<double, 30, 30> df_dx_output(state_output &s, const input_ikfom &in);

// Eigen::Matrix<double, 30, 15> df_dw_output(state_output &s);

// ==================== 观测模型函数 ====================

/**
 * @brief 输入状态的观测模型（点到平面距离）
 * @param s 当前输入状态
 * @param cov_p 位置协方差矩阵（3x3）
 * @param cov_R 旋转协方差矩阵（3x3）
 * @param ekfom_data ESKF动态共享数据，包含观测雅可比、残差等
 * @note 构建点云特征与地图的关联约束，用于ESKF更新步骤
 */
void h_model_input(state_input &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

/**
 * @brief 输出状态的观测模型（点到平面距离）
 * @param s 当前输出状态
 * @param cov_p 位置协方差矩阵（3x3）
 * @param cov_R 旋转协方差矩阵（3x3）
 * @param ekfom_data ESKF动态共享数据
 */
void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

/**
 * @brief 输出状态的IMU观测模型
 * @param s 当前输出状态
 * @param ekfom_data ESKF动态共享数据
 * @note 直接使用IMU测量作为观测，用于状态校正
 */
void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data);

// ==================== 坐标转换函数 ====================

/**
 * @brief 将点从机体坐标系转换到世界坐标系
 * @param pi 输入点（机体坐标系）
 * @param po 输出点（世界坐标系）
 * @note 使用当前状态估计的位姿进行坐标变换
 */
void pointBodyToWorld(PointType const * const pi, PointType * const po);

#endif