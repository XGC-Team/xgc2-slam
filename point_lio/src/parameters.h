/**
 * @file parameters.h
 * @brief Point-LIO系统参数定义头文件
 *
 * 本文件定义了Point-LIO（Point-based LiDAR-Inertial Odometry）系统中使用的全局参数和外部变量。
 * 主要包括：
 * - IMU和LiDAR传感器配置参数
 * - 时间同步相关参数
 * - 状态估计协方差参数
 * - iVox地图配置参数
 * - 数据预处理和发布选项
 * - 外参标定参数
 *
 * 这些参数通常从ROS参数服务器读取，用于配置整个LIO系统的行为。
 */

// #ifndef PARAM_H
// #define PARAM_H
#pragma once

// ROS相关头文件
#include <ros/ros.h>

// Eigen线性代数库
#include <Eigen/Eigen>
#include <Eigen/Core>

// 标准库头文件
#include <cstring>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <condition_variable>

// Point-LIO内部头文件
#include "preprocess.h"       // 点云预处理
#include "IMU_Processing.h"   // IMU数据处理

// ROS消息类型
#include <sensor_msgs/NavSatFix.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

// 第三方库
#include <omp.h>                        // OpenMP并行计算
#include <math.h>                       // 数学函数库
#include <ivox/ivox3d.h>                // iVox增量式体素地图
#include <Python.h>                     // Python接口
#include <pcl/common/transforms.h>      // PCL变换工具

// #define IVOX_NODE_TYPE_PHC

// ============================================================================
// iVox类型定义
// ============================================================================
#ifdef IVOX_NODE_TYPE_PHC
    // 使用PHC（Perfect Hash Coding）节点类型的iVox
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
    // 使用默认节点类型的iVox（三维增量式体素地图数据结构）
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

// ============================================================================
// 时间同步与帧管理相关参数
// ============================================================================
extern bool is_first_frame;                      // 是否为第一帧数据的标志
extern double lidar_end_time;                    // 当前LiDAR帧结束时间戳
extern double first_lidar_time;                  // 第一帧LiDAR数据的时间戳
extern double time_con;                          // 时间连续性检查参数
extern double last_timestamp_lidar;              // 上一帧LiDAR数据的时间戳
extern double last_timestamp_imu;                // 上一帧IMU数据的时间戳
extern int pcd_index;                            // 点云保存索引计数器

// ============================================================================
// iVox地图配置参数
// ============================================================================
extern IVoxType::Options ivox_options_;          // iVox体素地图配置选项
extern int ivox_nearby_type;                     // iVox近邻搜索类型（6邻域/18邻域/26邻域）

// ============================================================================
// 状态估计输入输出
// ============================================================================
extern state_input state_in;                     // ESKF状态估计器输入状态
extern state_output state_out;                   // ESKF状态估计器输出状态

// ============================================================================
// ROS话题配置
// ============================================================================
extern std::string lid_topic;                    // LiDAR数据话题名称
extern std::string imu_topic;                    // IMU数据话题名称

// ============================================================================
// 系统功能开关
// ============================================================================
extern bool prop_at_freq_of_imu;                 // 是否以IMU频率进行状态传播
extern bool check_satu;                          // 是否检查IMU饱和状态
extern bool con_frame;                           // 是否启用连续帧模式
extern bool cut_frame;                           // 是否启用帧切割模式
extern bool use_imu_as_input;                    // 是否使用IMU作为输入（影响状态表示）
extern bool space_down_sample;                   // 是否启用空间降采样
extern bool extrinsic_est_en;                    // 是否启用外参在线估计
extern bool publish_odometry_without_downsample; // 是否发布未降采样的里程计数据

// ============================================================================
// 地图与匹配参数
// ============================================================================
extern int init_map_size;                        // 初始化地图大小
extern int con_frame_num;                        // 连续帧数量
extern double match_s;                           // 匹配得分阈值
extern double satu_acc;                          // 加速度计饱和阈值（m/s²）
extern double satu_gyro;                         // 陀螺仪饱和阈值（rad/s）
extern double cut_frame_time_interval;           // 帧切割时间间隔（秒）
extern float plane_thr;                          // 平面拟合阈值（点到平面距离）

// ============================================================================
// 滤波与视场参数
// ============================================================================
extern double filter_size_surf_min;              // 表面点云最小滤波器尺寸（体素大小）
extern double filter_size_map_min;               // 地图点云最小滤波器尺寸（体素大小）
extern double fov_deg;                           // LiDAR视场角（度）
// extern double cube_len;                       // 局部地图立方体边长（已弃用）
extern float DET_RANGE;                          // 特征点检测范围（米）

// ============================================================================
// IMU配置参数
// ============================================================================
extern bool imu_en;                              // 是否启用IMU
extern double imu_time_inte;                     // IMU时间积分间隔

// ============================================================================
// 传感器噪声协方差参数
// ============================================================================
extern double laser_point_cov;                   // LiDAR点云测量协方差
extern double acc_norm;                          // 重力加速度归一化值（用于初始化）

// 输入状态噪声协方差（用于状态传播）
extern double acc_cov_input;                     // 加速度计输入噪声协方差
extern double gyr_cov_input;                     // 陀螺仪输入噪声协方差
extern double vel_cov;                           // 速度噪声协方差

// 输出状态噪声协方差
extern double gyr_cov_output;                    // 陀螺仪输出噪声协方差
extern double acc_cov_output;                    // 加速度计输出噪声协方差
extern double b_gyr_cov;                         // 陀螺仪零偏噪声协方差
extern double b_acc_cov;                         // 加速度计零偏噪声协方差

// IMU测量噪声协方差
extern double imu_meas_acc_cov;                  // IMU加速度测量噪声协方差
extern double imu_meas_omg_cov;                  // IMU角速度测量噪声协方差

// ============================================================================
// LiDAR类型与数据保存参数
// ============================================================================
extern int lidar_type;                           // LiDAR类型（1-Livox, 2-Velodyne, 3-Ouster等）
extern int pcd_save_interval;                    // 点云保存间隔（-1表示不保存）
extern std::vector<double> gravity_init;         // 初始重力向量（用于初始对齐）
extern std::vector<double> gravity;              // 当前重力向量

// ============================================================================
// 日志与发布选项
// ============================================================================
extern bool runtime_pos_log;                     // 是否记录运行时位置日志
extern bool pcd_save_en;                         // 是否启用点云保存
extern bool path_en;                             // 是否启用路径发布
extern bool scan_pub_en;                         // 是否发布降采样后的扫描点云
extern bool scan_body_pub_en;                    // 是否发布Body坐标系下的扫描点云

// ============================================================================
// 预处理与IMU处理对象
// ============================================================================
extern shared_ptr<Preprocess> p_pre;             // 点云预处理器智能指针
extern shared_ptr<ImuProcess> p_imu;             // IMU数据处理器智能指针

// ============================================================================
// 外参标定参数
// ============================================================================
extern std::vector<double> extrinT;              // LiDAR到IMU的外参平移向量 [tx, ty, tz]
extern std::vector<double> extrinR;              // LiDAR到IMU的外参旋转矩阵 [r11, r12, ..., r33]
extern double time_diff_lidar_to_imu;            // LiDAR与IMU之间的时间偏移（秒）

// ============================================================================
// 时间管理参数
// ============================================================================
extern double lidar_time_inte;                   // LiDAR时间积分间隔
extern double first_imu_time;                    // 第一帧IMU数据的时间戳
extern int cut_frame_num;                        // 已切割的帧数量计数器
extern int orig_odom_freq;                       // 原始里程计输出频率（Hz）
extern double online_refine_time;                // 在线优化时间（单位：秒）
extern bool cut_frame_init;                      // 帧切割是否已初始化的标志
extern double time_update_last;                  // 上一次状态更新的时间戳
extern double time_current;                      // 当前时间戳
extern double time_predict_last_const;           // 上一次预测的常量时间
extern double t_last;                            // 上一个时间点

// ============================================================================
// 测量数据组
// ============================================================================
extern MeasureGroup Measures;                    // 存储同步后的LiDAR和IMU测量数据组

// ============================================================================
// 文件输出流
// ============================================================================
extern ofstream fout_out;                        // 输出文件流（保存位姿等结果）
extern ofstream fout_imu_pbp;                    // IMU point-by-point输出文件流

// ============================================================================
// 函数声明
// ============================================================================

/**
 * @brief 从ROS参数服务器读取所有配置参数
 * @param n ROS节点句柄
 *
 * 该函数从launch文件或参数服务器读取所有系统配置参数，包括：
 * - 传感器话题名称
 * - IMU和LiDAR参数
 * - 噪声协方差
 * - 功能开关
 * - 外参等
 */
void readParameters(ros::NodeHandle &n);

/**
 * @brief 打开用于保存结果的文件
 *
 * 根据配置打开相应的文件流，用于保存：
 * - 位姿轨迹
 * - IMU数据
 * - 点云数据等
 */
void open_file();

/**
 * @brief 将SO3旋转矩阵转换为欧拉角
 * @param orient SO3旋转对象
 * @return 欧拉角向量 [roll, pitch, yaw]（弧度）
 *
 * 该函数将李群SO(3)表示的旋转转换为ZYX顺序的欧拉角表示
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &orient);

/**
 * @brief 重置输入状态的协方差矩阵
 * @param P_init 24x24的协方差矩阵，包含位置、姿态、速度、零偏等状态的不确定性
 *
 * 该函数用于初始化或重置ESKF滤波器的输入状态协方差矩阵
 * 状态向量维度：24（位置3 + 姿态3 + 速度3 + 加速度零偏3 + 陀螺仪零偏3 + 重力3 + 其他6）
 */
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init);

/**
 * @brief 重置输出状态的协方差矩阵
 * @param P_init_output 30x30的协方差矩阵，包含扩展状态的不确定性
 *
 * 该函数用于初始化或重置ESKF滤波器的输出状态协方差矩阵
 * 状态向量维度：30（在输入状态基础上增加了额外的6维状态）
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output);