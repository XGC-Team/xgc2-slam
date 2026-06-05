/**
 * @file li_initialization.h
 * @brief Point-LIO系统初始化头文件
 *
 * 本文件定义了Point-LIO（激光惯性里程计）系统初始化所需的全局变量和函数声明。
 * 主要功能包括：
 * - IMU和激光雷达数据的回调函数声明
 * - 数据同步和缓冲区管理相关的全局变量
 * - 在线标定和初始化状态控制变量
 * - 传感器时间戳对齐和重力向量估计相关变量
 */

#pragma once

#include <common_lib.h>
#include "Estimator.h"
#define MAXN                (720000)  // 最大数据点数量定义，用于调试数组大小

// ============ 初始化和标定状态标志 ============
extern bool data_accum_finished;      // 数据累积是否完成的标志
extern bool data_accum_start;         // 数据累积是否开始的标志
extern bool online_calib_finish;      // 在线标定是否完成的标志
extern bool refine_print;             // 是否打印精细化信息的标志
extern int frame_num_init;            // 初始化阶段的帧数计数器
extern double time_lag_IMU_wtr_lidar; // IMU相对于激光雷达的时间延迟（秒）
extern double move_start_time;        // 运动开始的时间戳
extern double online_calib_starts_time; // 在线标定开始的时间戳 //, mean_acc_norm = 9.81;

// ============ 时间同步相关变量 ============
extern double timediff_imu_wrt_lidar; // IMU相对于激光雷达的时间差（用于时间戳对齐）
extern bool timediff_set_flg;         // 时间差是否已设置的标志
extern V3D gravity_lio;               // LIO系统估计的重力向量（3维向量）
// ============ 线程同步和缓冲区控制 ============
extern mutex mtx_buffer;              // 缓冲区访问的互斥锁，保护共享数据
extern condition_variable sig_buffer; // 条件变量，用于线程间的数据就绪信号通知
extern int scan_count;                // 激光扫描帧的总计数器
extern int frame_ct;                  // 当前处理的帧计数
extern int wait_num;                  // 等待处理的帧数量
// ============ 数据缓冲队列 ============
extern std::deque<PointCloudXYZI::Ptr>  lidar_buffer; // 激光雷达点云数据缓冲队列
extern std::deque<double>               time_buffer;  // 激光雷达时间戳缓冲队列
extern std::deque<sensor_msgs::Imu::Ptr> imu_deque;   // IMU数据缓冲队列
extern std::mutex m_time;                             // 时间相关操作的互斥锁
// ============ 数据推送和状态标志 ============
extern bool lidar_pushed;              // 激光雷达数据是否已推送到缓冲区的标志
extern bool imu_pushed;                // IMU数据是否已推送到缓冲区的标志
extern double imu_first_time;          // 第一个IMU数据的时间戳，用于时间对齐
extern bool lose_lid;                  // 是否丢失激光雷达数据的标志
extern sensor_msgs::Imu imu_last;      // 上一个IMU测量数据
extern sensor_msgs::Imu imu_next;      // 下一个IMU测量数据
// ============ 点云和调试数据 ============
extern PointCloudXYZI::Ptr  ptr_con;   // 点云数据指针（用于数据处理和转换）
extern double T1[MAXN];                // 调试用时间数组
extern double s_plot[MAXN];            // 调试用绘图数据数组1
extern double s_plot2[MAXN];           // 调试用绘图数据数组2
extern double s_plot3[MAXN];           // 调试用绘图数据数组3
extern double s_plot11[MAXN];          // 调试用绘图数据数组11

// extern sensor_msgs::Imu::ConstPtr imu_last_ptr;

// ============ 回调函数声明 ============

/**
 * @brief 标准点云数据回调函数
 *
 * 处理标准ROS PointCloud2格式的激光雷达数据
 * @param msg 接收到的PointCloud2消息指针
 */
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);

/**
 * @brief Livox激光雷达自定义消息回调函数
 *
 * 处理Livox激光雷达特有的CustomMsg格式数据
 * @param msg 接收到的Livox CustomMsg消息指针
 */
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg);

/**
 * @brief IMU数据回调函数
 *
 * 接收并处理IMU传感器数据，包括加速度计和陀螺仪测量值
 * @param msg_in 接收到的IMU消息指针
 */
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);

/**
 * @brief 传感器数据同步函数
 *
 * 将IMU和激光雷达数据进行时间对齐和同步，组装成测量组
 * @param meas 输出参数，同步后的测量数据组
 * @return true 数据同步成功
 * @return false 数据同步失败或数据不足
 */
bool sync_packages(MeasureGroup &meas);

// #endif