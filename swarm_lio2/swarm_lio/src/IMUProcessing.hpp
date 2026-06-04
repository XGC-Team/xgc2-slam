/**
 * @file IMUProcessing.hpp
 * @brief IMU数据处理类头文件，负责IMU初始化、前向传播和点云去畸变
 * @author Swarm-LIO2团队
 */

#ifndef _IMU_PROCESSING_HPP
#define _IMU_PROCESSING_HPP

#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <swarm_lio/States.h>
#include <geometry_msgs/Vector3.h>
//#include "MultiUAV.hpp"
#include "MultiUAV.h"


/// *************预配置参数

#define MAX_INI_COUNT (200)  // IMU初始化最大迭代次数


/**
 * @brief 点云按时间排序的比较函数
 * @param x 第一个点
 * @param y 第二个点
 * @return 如果x的时间戳小于y则返回true
 */
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

/// *************IMU处理和去畸变类
/**
 * @class ImuProcess
 * @brief IMU数据处理类，负责IMU初始化、状态传播和激光雷达点云去畸变
 */
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
     */
    void Reset();

    /**
     * @brief 重置IMU处理器到指定时间戳
     * @param start_timestamp 起始时间戳
     * @param lastimu 最后的IMU数据
     */
    void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

    /**
     * @brief 设置LiDAR到IMU的外参
     * @param rot 旋转矩阵
     * @param trans 平移向量
     */
    void set_extrinsic(const M3D &rot, const V3D &trans);

    /**
     * @brief 设置陀螺仪协方差缩放因子
     * @param scaler 缩放向量
     */
    void set_gyr_cov(const V3D &scaler);

    /**
     * @brief 设置加速度计协方差缩放因子
     * @param scaler 缩放向量
     */
    void set_acc_cov(const V3D &scaler);

    /**
     * @brief 设置陀螺仪偏置协方差
     * @param b_g 陀螺仪偏置协方差
     */
    void set_gyr_bias_cov(const V3D &b_g);

    /**
     * @brief 设置加速度计偏置协方差
     * @param b_a 加速度计偏置协方差
     */
    void set_acc_bias_cov(const V3D &b_a);

    /**
     * @brief 设置全局外参协方差
     * @param cov_global_extrinsic_rot 全局外参旋转协方差
     * @param cov_global_extrinsic_trans 全局外参平移协方差
     */
    void set_global_extrinsic_cov(const V3D &cov_global_extrinsic_rot, const V3D &cov_global_extrinsic_trans);

    /**
     * @brief 设置强度阈值
     * @param threshold 阈值
     */
    void set_inten_threshold(const int &threshold);

    /**
     * @brief 主处理函数：IMU前向传播和点云去畸变
     * @param meas 测量数据组（包含IMU和LiDAR数据）
     * @param state 状态组（输入输出）
     * @param orig_pcl_un_ 去畸变后的点云
     */
    void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZI::Ptr orig_pcl_un_);


    ros::NodeHandle nh;                       // ROS节点句柄
    V3D cov_acc;                              // 加速度计协方差
    V3D cov_gyr;                              // 陀螺仪协方差
    V3D cov_acc_scale;                        // 加速度计协方差缩放因子
    V3D cov_gyr_scale;                        // 陀螺仪协方差缩放因子
    V3D cov_bias_gyr;                         // 陀螺仪偏置协方差
    V3D cov_bias_acc;                         // 加速度计偏置协方差
    int lidar_type;                           // 激光雷达类型

    V3D unbiased_gyr;                         // 去偏置后的陀螺仪数据
    M3D offset_R_L_I;                         // LiDAR到IMU的旋转外参
    V3D offset_T_L_I;                         // LiDAR到IMU的平移外参
    V3D cov_global_extrinsic_rot;             // 全局外参旋转协方差
    V3D cov_global_extrinsic_trans;           // 全局外参平移协方差
    double IMU_mean_acc_norm = 0.0;           // IMU平均加速度模值
    bool imu_need_init_ = true;               // IMU是否需要初始化标志

private:

    /**
     * @brief IMU初始化函数
     * @param meas 测量数据组
     * @param state 状态组（输出）
     * @param N 初始化迭代计数器
     * @details 初始化重力、陀螺仪偏置、加速度计和陀螺仪协方差，并将加速度归一化为单位重力
     */
    void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);

    /**
     * @brief IMU状态传播和点云去畸变函数
     * @param meas 测量数据组
     * @param state_inout 状态组（输入输出）
     * @param orig_pcl_out 去畸变后的点云（输出）
     * @details 通过IMU测量进行前向传播，并对点云进行反向去畸变
     */
    void propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &orig_pcl_out);

    PointCloudXYZI::Ptr cur_pcl_un_;          // 当前去畸变点云指针
    sensor_msgs::ImuConstPtr last_imu_;       // 上一帧最后的IMU数据
    deque<sensor_msgs::ImuConstPtr> v_imu_;   // IMU数据队列
    vector<Pose6D> IMUpose;                   // IMU位姿序列（用于点云去畸变）
    V3D mean_acc;                             // 加速度均值（用于初始化）
    V3D mean_gyr;                             // 陀螺仪均值（用于初始化）
    V3D angvel_last;                          // 上一时刻角速度
    V3D acc_s_last;                           // 上一时刻加速度
    double last_lidar_end_time_;              // 上一帧LiDAR结束时间戳
    double time_last_scan;                    // 上一帧开头的时间戳
    int init_iter_num = 1;                    // 初始化迭代次数
    bool b_first_frame_ = true;               // 是否为第一帧标志

};

/**
 * @brief ImuProcess构造函数
 * @details 初始化所有IMU处理相关参数和协方差矩阵
 */
ImuProcess::ImuProcess()
        : b_first_frame_(true), imu_need_init_(true) {
    init_iter_num = 1;
    cov_acc = V3D(0.1, 0.1, 0.1);                            // 加速度计协方差初始值
    cov_gyr = V3D(0.1, 0.1, 0.1);                            // 陀螺仪协方差初始值
    cov_bias_gyr = V3D(0.0001, 0.0001, 0.0001);              // 陀螺仪偏置协方差初始值
    cov_bias_acc = V3D(0.0001, 0.0001, 0.0001);              // 加速度计偏置协方差初始值
    mean_acc = V3D(0, 0, -1.0);                              // 加速度均值初始值（假设重力向下）
    mean_gyr = V3D(0, 0, 0);                                 // 陀螺仪均值初始值
    angvel_last = Zero3d;                                    // 上一时刻角速度初始化为零
    offset_R_L_I = M3D::Identity();                          // LiDAR到IMU外参旋转初始化为单位矩阵
    offset_T_L_I = Zero3d;                                   // LiDAR到IMU外参平移初始化为零
    cov_global_extrinsic_rot = V3D(0.0001, 0.0001, 0.0001);  // 全局外参旋转协方差
    cov_global_extrinsic_trans = V3D(0.0001, 0.0001, 0.0001);// 全局外参平移协方差
    last_imu_.reset(new sensor_msgs::Imu());                 // 重置上一帧IMU数据
}

/**
 * @brief ImuProcess析构函数
 */
ImuProcess::~ImuProcess() {}

/**
 * @brief 重置IMU处理器
 * @details 清空所有缓存数据，重置状态到初始值
 */
void ImuProcess::Reset() {
    ROS_WARN("Reset ImuProcess");
    mean_acc = V3D(0, 0, -1.0);                  // 重置加速度均值
    mean_gyr = V3D(0, 0, 0);                     // 重置陀螺仪均值
    angvel_last = Zero3d;                        // 重置角速度
    imu_need_init_ = true;                       // 标记需要重新初始化
    init_iter_num = 1;                           // 重置初始化迭代计数
    v_imu_.clear();                              // 清空IMU队列
    IMUpose.clear();                             // 清空IMU位姿缓存
    last_imu_.reset(new sensor_msgs::Imu());     // 重置上一帧IMU数据
    cur_pcl_un_.reset(new PointCloudXYZI());     // 重置点云数据
}


/**
 * @brief 设置陀螺仪协方差缩放因子
 */
void ImuProcess::set_gyr_cov(const V3D &scaler) {
    cov_gyr_scale = scaler;
}

/**
 * @brief 设置加速度计协方差缩放因子
 */
void ImuProcess::set_acc_cov(const V3D &scaler) {
    cov_acc_scale = scaler;
}

/**
 * @brief 设置陀螺仪偏置协方差
 */
void ImuProcess::set_gyr_bias_cov(const V3D &b_g) {
    cov_bias_gyr = b_g;
}

/**
 * @brief 设置加速度计偏置协方差
 */
void ImuProcess::set_acc_bias_cov(const V3D &b_a) {
    cov_bias_acc = b_a;
}

/**
 * @brief 设置LiDAR到IMU的外参
 */
void ImuProcess::set_extrinsic(const M3D &rot, const V3D &trans) {
    offset_R_L_I = rot;
    offset_T_L_I = trans;
}

/**
 * @brief 设置全局外参协方差
 */
void ImuProcess::set_global_extrinsic_cov(const V3D &cov_global_extrin_rot, const V3D &cov_global_extrin_trans) {
    cov_global_extrinsic_rot = cov_global_extrin_rot;
    cov_global_extrinsic_trans = cov_global_extrin_trans;
}

/**
 * @brief IMU初始化函数
 * @param meas 测量数据组
 * @param state_inout 状态组（输出）
 * @param N 初始化迭代计数器
 * @details 主要功能：
 *          1. 初始化重力、陀螺仪偏置、加速度计和陀螺仪协方差
 *          2. 将加速度测量值归一化为单位重力
 */
void ImuProcess::IMU_init(const MeasureGroup &meas, StatesGroup &state_inout, int &N) {
    ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
    V3D cur_acc, cur_gyr;

    // 第一帧：重置状态并初始化均值
    if (b_first_frame_) {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
    }

    // 遍历所有IMU数据，增量更新均值和协方差
    for (const auto &imu: meas.imu) {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        // 增量更新均值
        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        // 增量更新协方差（使用Welford算法）
        cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        N++;
    }
    // 根据加速度均值估计重力方向和大小
    state_inout.gravity = - mean_acc / mean_acc.norm() * G_m_s2;
    state_inout.rot_end = Eye3d;          // 初始旋转为单位矩阵
    state_inout.bias_g.setZero();         // 初始陀螺仪偏置为零
    last_imu_ = meas.imu.back();          // 保存最后一帧IMU数据
}

/**
 * @brief IMU状态传播和点云去畸变函数
 * @param meas 测量数据组
 * @param state_inout 状态组（输入输出）
 * @param orig_pcl_out 去畸变后的点云（输出）
 * @details 主要步骤：
 *          1. 将上一帧末尾的IMU数据添加到当前帧头部
 *          2. 在每个IMU时刻进行前向传播
 *          3. 反向传播对点云进行去畸变
 */
void
ImuProcess::propagation_and_undist(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &orig_pcl_out) {
    /*** 将上一帧末尾的IMU数据添加到当前帧头部 ***/
    orig_pcl_out = *(meas.lidar);  // 有畸变的原始点云
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);   // 添加上一帧最后的IMU数据，保证时间连续性
    double imu_end_time = v_imu.back()->header.stamp.toSec();
    double pcl_beg_time, pcl_end_time;

    // 根据LiDAR类型确定点云时间范围
    if (lidar_type == SIM) {
        pcl_beg_time = last_lidar_end_time_;
        pcl_end_time = meas.lidar_beg_time;
    } else {
        pcl_beg_time = meas.lidar_beg_time;
        /*** 按偏移时间对点云进行排序 ***/
//        sort(orig_pcl_out.points.begin(), orig_pcl_out.points.end(), time_list);
        pcl_end_time = pcl_beg_time + orig_pcl_out.points.back().curvature / double(1000);
    }


    /*** 初始化IMU位姿序列 ***/
    IMUpose.clear();
    IMUpose.push_back(
            set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

    /*** 在每个IMU时刻进行前向传播 ***/
    V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);

    MatrixXd F_x, cov_w;
    F_x.resize(18, 18);                // 状态转移矩阵（18维：旋转、位置、速度、陀螺仪偏置、加速度偏置各3维）
    cov_w.resize(DIM_STATE, DIM_STATE);// 过程噪声协方差矩阵


    double dt = 0.0;
    // 遍历所有IMU数据对，进行前向传播
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto &&head = *(it_imu);      // 当前IMU数据
        auto &&tail = *(it_imu + 1);  // 下一个IMU数据

        // 跳过早于上一帧LiDAR结束时间的IMU数据
        if (tail->header.stamp.toSec() < last_lidar_end_time_) continue;

        // 计算角速度平均值（中值法）
        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);

        // 计算加速度平均值（中值法）
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        V3D angvel_now(head->angular_velocity.x, head->angular_velocity.y, head->angular_velocity.z);
        V3D acc_now(head->linear_acceleration.x, head->linear_acceleration.y, head->linear_acceleration.z);

        // 去除陀螺仪偏置
        angvel_avr -= state_inout.bias_g;
        // 校正加速度并去除偏置
        acc_avr = acc_avr / IMU_mean_acc_norm * G_m_s2 - state_inout.bias_a;

        // 计算时间间隔
        if (head->header.stamp.toSec() < last_lidar_end_time_)
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        else
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();

        /* 协方差传播 */
        M3D acc_avr_skew;
        M3D Exp_f = Exp(angvel_avr, dt);           // 旋转传播矩阵
        acc_avr_skew << SKEW_SYM_MATRX(acc_avr);   // 加速度反对称矩阵

        // 构建状态转移矩阵F_x
        F_x.setIdentity();
        memset(cov_w.data(), 0, cov_w.size() * sizeof(double)); //初始化为0，比setzero快很多

        F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);      // 旋转对旋转的雅可比
        F_x.block<3, 3>(0, 9) = -Eye3d * dt;               // 旋转对陀螺仪偏置的雅可比
        F_x.block<3, 3>(3, 6) = Eye3d * dt;                // 位置对速度的雅可比
        F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;// 速度对旋转的雅可比
        F_x.block<3, 3>(6, 12) = -R_imu * dt;              // 速度对加速度偏置的雅可比
        F_x.block<3, 3>(6, 15) = Eye3d * dt;               // 速度对重力的雅可比

        // 构建过程噪声协方差矩阵
        cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;      // 陀螺仪噪声
        cov_w.block<3, 3>(6, 6) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt; // 加速度噪声
        cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt;  // 陀螺仪偏置随机游走
        cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt;// 加速度偏置随机游走

        // 多机外参协方差
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            cov_w.block<3, 3>(start_row, start_row).diagonal() =
                    cov_global_extrinsic_rot * dt * dt;       // 全局外参旋转协方差
            cov_w.block<3, 3>(start_row + 3, start_row + 3).diagonal() =
                    cov_global_extrinsic_trans * dt * dt;     // 全局外参平移协方差
        }

        // EKF协方差传播
        state_inout.cov.block<18,18>(0,0) = F_x * state_inout.cov.block<18,18>(0,0) * F_x.transpose();
        state_inout.cov.block<18, 6*MAX_UAV_NUM>(0,18) = F_x * state_inout.cov.block<18, 6*MAX_UAV_NUM>(0,18);
        state_inout.cov.block<6*MAX_UAV_NUM, 18>(18,0) = state_inout.cov.block<18, 6*MAX_UAV_NUM>(0,18).transpose();
        state_inout.cov += cov_w;

        /* IMU姿态传播（全局坐标系）*/
        R_imu = R_imu * Exp_f;

        /* IMU比力（全局坐标系）*/
        acc_imu = R_imu * acc_avr + state_inout.gravity;

        /* IMU位置传播（全局坐标系）*/
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* IMU速度传播（全局坐标系）*/
        vel_imu = vel_imu + acc_imu * dt;

        /* 保存每个IMU时刻的位姿（全局坐标系）*/
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }

    // 保存去偏置后的陀螺仪数据
    unbiased_gyr = V3D(IMUpose.back().gyr[0], IMUpose.back().gyr[1], IMUpose.back().gyr[2]);

    /*** 计算帧末时刻的位置和姿态预测 ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;  // 判断点云结束时间在IMU之后还是之前
    dt = note * (pcl_end_time - imu_end_time);
    state_inout.vel_end = vel_imu + note * acc_imu * dt;     // 预测速度
    state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);  // 预测旋转
    state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;  // 预测位置



    last_imu_ = meas.imu.back();           // 保存最后的IMU数据
    last_lidar_end_time_ = pcl_end_time;   // 保存LiDAR结束时间

    // 对于非仿真LiDAR，需要对点云进行去畸变
    if (lidar_type != SIM) {
        /*** 对每个激光点进行去畸变（反向传播）***/
        auto it_pcl = orig_pcl_out.points.end() - 1;  // 从点云末尾开始（最新的点）
        for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
            auto head = it_kp - 1;  // t_i时刻的IMU位姿，满足t_i < t_j
            R_imu << MAT_FROM_ARRAY(head->rot);
            acc_imu << VEC_FROM_ARRAY(head->acc);
            // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
            vel_imu << VEC_FROM_ARRAY(head->vel);
            pos_imu << VEC_FROM_ARRAY(head->pos);
            angvel_avr << VEC_FROM_ARRAY(head->gyr);

            // 对该IMU时段内的所有点进行去畸变
            for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
                dt = it_pcl->curvature / double(1000) - head->offset_time;  // dt = t_j - t_i > 0
                /* 变换到扫描结束时刻的IMU坐标系（I_k坐标系）*/
                M3D R_i(R_imu * Exp(angvel_avr, dt));  // 点所在时刻的旋转
                V3D P_i = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;  // 点所在时刻的位置
                V3D p_in(it_pcl->x, it_pcl->y, it_pcl->z);
                // 去畸变变换：LiDAR坐标 -> IMU坐标(t_j) -> 世界坐标 -> IMU坐标(t_end) -> LiDAR坐标(t_end)
                V3D P_compensate = offset_R_L_I.transpose() * (state_inout.rot_end.transpose() * (R_i * (offset_R_L_I * p_in + offset_T_L_I) + P_i - state_inout.pos_end) - offset_T_L_I);
                /// 保存去畸变后的点
                it_pcl->x = P_compensate(0);
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);
                if (it_pcl == orig_pcl_out.points.begin()) break;
            }
        }
    }
}

/**
 * @brief IMU处理主函数
 * @param meas 测量数据组
 * @param stat 状态组
 * @param orig_pcl_un_ 去畸变后的点云输出
 * @details 包含两个主要阶段：
 *          1. IMU初始化阶段：收集足够的IMU数据估计重力和偏置
 *          2. 正常处理阶段：进行IMU前向传播和点云去畸变
 */
void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr orig_pcl_un_) {
    if (meas.imu.empty()) return;  // 如果没有IMU数据则直接返回
    ROS_ASSERT(meas.lidar != nullptr);

    // IMU初始化阶段
    if (imu_need_init_) {

        /// 处理第一帧LiDAR数据
        IMU_init(meas, stat, init_iter_num);
        imu_need_init_ = true;
        last_imu_ = meas.imu.back();
        // 检查是否达到初始化迭代次数
        if (init_iter_num > MAX_INI_COUNT) {
            // 根据重力大小缩放加速度协方差
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;  // 标记初始化完成

            // 使用配置的协方差值
            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;


            ROS_INFO("IMU Initialization Done: Gravity: %.4f %.4f %.4f, Acc norm: %.4f", stat.gravity[0],
                     stat.gravity[1], stat.gravity[2], mean_acc.norm());
            IMU_mean_acc_norm = mean_acc.norm();  // 保存加速度模值用于后续归一化
        }
        return;
    }
    // 正常处理阶段：IMU前向传播和点云去畸变
    propagation_and_undist(meas, stat, *orig_pcl_un_);
}

#endif
