/**
 * @file IMU_Processing.h
 * @brief IMU数据处理与点云去畸变模块
 *
 * 主要功能：
 * 1. IMU数据初始化和预处理
 * 2. 激光雷达点云去畸变（消除运动畸变）
 * 3. IMU与激光雷达数据融合处理
 * 4. IMU噪声协方差管理
 * 5. 重力向量和初始姿态估计
 */

#pragma once
#include <cmath>
#include <math.h>
// #include <deque>
// #include <mutex>
// #include <thread>
#include <csignal>
#include <ros/ros.h>
// #include <so3_math.h>
#include <Eigen/Eigen>
// #include "Estimator.h"
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

/// *************预配置参数 (Preconfiguration)

#define MAX_INI_COUNT (100)  // IMU初始化的最大迭代次数

/**
 * @brief 点云时间戳排序比较函数
 * @param x 第一个点
 * @param y 第二个点
 * @return 如果x的时间戳小于y则返回true
 * @note 使用点的curvature字段存储时间戳信息
 */
const bool time_list(PointType &x, PointType &y); // {return (x.curvature < y.curvature);};

/// *************IMU处理与去畸变 (IMU Process and undistortion)

/**
 * @class ImuProcess
 * @brief IMU数据处理类，负责IMU初始化、点云去畸变和状态估计
 *
 * 该类实现了基于IMU的点云运动畸变去除功能，通过融合IMU数据对激光雷达扫描期间
 * 的运动进行补偿，生成去畸变后的点云。同时负责IMU的初始化和噪声协方差管理。
 */
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen库内存对齐宏，确保动态分配时的正确对齐

  /**
   * @brief 构造函数
   */
  ImuProcess();

  /**
   * @brief 析构函数
   */
  ~ImuProcess();

  /**
   * @brief 重置IMU处理器状态
   * @note 将所有状态变量恢复到初始值，用于重新开始处理
   */
  void Reset();

  /**
   * @brief 处理IMU和激光雷达测量数据，执行点云去畸变
   * @param meas 包含IMU和激光雷达数据的测量组
   * @param pcl_un_ 输出的去畸变点云指针
   * @note 这是核心处理函数，整合IMU数据对点云进行运动补偿
   */
  void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr pcl_un_);

  /**
   * @brief 设置陀螺仪噪声协方差缩放因子
   * @param scaler 3维向量，表示x/y/z三个轴的协方差缩放系数
   */
  void set_gyr_cov(const V3D &scaler);

  /**
   * @brief 设置加速度计噪声协方差缩放因子
   * @param scaler 3维向量，表示x/y/z三个轴的协方差缩放系数
   */
  void set_acc_cov(const V3D &scaler);

  /**
   * @brief 设置初始化参数（重力向量和旋转矩阵）
   * @param tmp_gravity 重力向量（通常为[0, 0, -9.81]在世界坐标系下）
   * @param rot 初始旋转矩阵，表示从IMU坐标系到世界坐标系的变换
   */
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);

  // ========== 公有成员变量 (Public Member Variables) ==========

  MD(12, 12) state_cov = MD(12, 12)::Identity();  ///< 12x12状态协方差矩阵，初始化为单位矩阵
  int    lidar_type;           ///< 激光雷达类型标识（如Velodyne、Livox等）
  V3D    gravity_;             ///< 重力向量，单位m/s²，用于IMU姿态估计
  bool   imu_en;               ///< IMU使能标志，指示是否使用IMU数据
  V3D    mean_acc;             ///< 加速度均值，用于IMU初始化时估计重力方向
  bool   imu_need_init_ = true;     ///< IMU需要初始化标志，true表示尚未完成初始化
  bool   after_imu_init_ = false;   ///< IMU初始化完成标志，true表示已完成初始化
  bool   b_first_frame_ = true;     ///< 首帧标志，用于标识是否为第一帧数据
  double time_last_scan = 0.0;      ///< 上一次扫描的时间戳，用于时间间隔计算
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001);  ///< 陀螺仪协方差缩放因子（rad/s）²
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001);  ///< 速度协方差缩放因子（m/s）²

 private:
  // ========== 私有成员函数 (Private Member Functions) ==========

  /**
   * @brief IMU初始化函数
   * @param meas 测量数据组，包含IMU和激光雷达数据
   * @param N 初始化迭代计数器（输出参数）
   * @note 通过静态状态下的IMU数据估计零偏和初始姿态
   */
  void IMU_init(const MeasureGroup &meas, int &N);

  // ========== 私有成员变量 (Private Member Variables) ==========

  V3D mean_gyr;              ///< 陀螺仪均值，用于估计陀螺仪零偏
  int init_iter_num = 1;     ///< 初始化迭代次数，记录已完成的初始化迭代数
};

