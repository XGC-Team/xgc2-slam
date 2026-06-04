/*
 * ESIKF_TRACKER.HPP
 * 误差状态卡尔曼滤波器（Error-State Iterative Kalman Filter）
 *
 * 功能说明：用于跟踪多无人机集群中其他无人机的位置和速度
 * 主要特点：
 * - 基于误差状态的卡尔曼滤波，提高数值稳定性
 * - 支持迭代更新，提高收敛速度和精度
 * - 专门用于集群SLAM中的队友状态估计
 *
 * Created by fangcheng on 2022/6/1.
 */

#ifndef ESIKF_TRACKER_HPP
#define ESIKF_TRACKER_HPP

#include "common_lib.h"

#define init_cov 0.1;  // 初始协方差值

/**
 * @brief 误差状态迭代卡尔曼滤波器类
 *
 * 状态向量定义（默认6维）：
 * x = [位置(3), 速度(3)]^T
 *
 * 应用场景：跟踪其他无人机在本机世界坐标系下的位置和速度
 */
class ESIKF{
public:
    /**
     * @brief 构造函数 - 初始化ESIKF滤波器
     * @param dim_x 状态维度（默认6：3位置 + 3速度）
     * @param dim_w 过程噪声维度（默认3：速度噪声）
     * @param dim_z 测量维度（默认6：3位置 + 3速度）
     */
    ESIKF(const int &dim_x = 6, const int &dim_w = 3, const int &dim_z = 6){
        x_.resize(dim_x,1);                    // 状态向量
        x_.setZero();
        w_.resize(dim_w,1);                    // 过程噪声向量
        w_.setZero();
        delta_x_.resize(dim_x,1);              // 误差状态向量
        delta_x_.setZero();
        F_x_.resize(dim_x,dim_x);              // 状态转移矩阵（雅可比）
        F_x_.setIdentity();
        F_w_.resize(dim_x,dim_w);              // 噪声雅可比矩阵
        F_w_.setZero();
        Q_.resize(dim_w,dim_w);                // 过程噪声协方差矩阵
        Q_.setIdentity() * init_cov;
        R_.resize(dim_z,dim_z);                // 测量噪声协方差矩阵
        R_.setIdentity() * init_cov;
        P_.resize(dim_x,dim_x);                // 状态协方差矩阵
        P_.setIdentity() * init_cov;
        K_.resize(dim_x,dim_z);                // 卡尔曼增益矩阵
        K_.setZero();
        H_.resize(dim_z,dim_x);                // 测量矩阵（观测雅可比）
        H_.setIdentity();
        z_.resize(dim_z,1);                    // 测量向量
        z_.setZero();
        iter_num_ = 1;                         // 迭代次数
        last_predict_time_ = 0.0;              // 上次预测时间
        last_update_time_ = 0.0;               // 上次更新时间
        last_teammate_update_time_ = 0.0;      // 上次队友更新时间
    }

    /**
     * @brief 析构函数
     */
    ~ESIKF(){};

    /**
     * @brief 初始化滤波器状态
     * @param state 初始状态向量 [位置, 速度]
     * @param lidar_end_time 激光雷达帧结束时间
     *
     * 功能说明：设置队友在本机世界坐标系下的初始位置和速度
     */
    void init(const VectorXd &state, const double &lidar_end_time){
        x_ = state;                                   // 队友在自己世界系下的位置和速度
        w_ = Vector3d(0.01,0.01,0.01);               // 速度的过程噪声标准差
        Q_ = w_.asDiagonal();                        // 构造对角协方差矩阵
        F_x_ = Matrix<double,6,6>::Identity();
        F_x_.block<3,3>(0,3) = Matrix3d::Identity() * 0.001;  // 位置对速度的微小耦合
        F_w_.block<3,3>(0,0) = Matrix3d::Zero();     // 位置不受过程噪声直接影响
        F_w_.block<3,3>(3,0) = Matrix3d::Identity() * 0.001;  // 速度受过程噪声影响
        H_ = Matrix<double,6,6>::Identity();         // 全状态观测
        last_predict_time_ = lidar_end_time - 0.001;
        last_update_time_ = lidar_end_time - 0.001;
        last_teammate_update_time_ = lidar_end_time - 0.001;
    }

    /**
     * @brief 重置滤波器 - 当长时间未更新时使用
     * @param meas 测量值
     * @param lidar_end_time 当前时间
     *
     * 功能说明：
     * - 状态完全由观测决定
     * - 协方差重置为0（高置信度）
     * - 通常在丢失跟踪后重新获得观测时调用
     */
    void reset( const VectorXd &meas, const double &lidar_end_time) {
        ROS_WARN("No update for too long time! EKF Tracker reset!");
        z_ = meas;
        x_ = z_;                 // 状态直接设为测量值
        P_.setZero();            // 协方差设为0（完全相信测量）
        last_predict_time_ = lidar_end_time;
    }

    /**
     * @brief 卡尔曼滤波预测步骤
     * @param lidar_end_time 当前时刻
     *
     * 运动模型（匀速运动）：
     * - 位置: p_k = p_{k-1} + v_{k-1} * dt
     * - 速度: v_k = v_{k-1}  （假设速度恒定）
     *
     * 协方差预测：P_k = F_x * P_{k-1} * F_x^T + F_w * Q * F_w^T
     */
    void predict(const double &lidar_end_time){
        double dt = lidar_end_time - last_predict_time_;  // 计算时间间隔
        // 计算状态增量
        delta_x_.block<3,1>(0,0) = x_.block<3,1>(3,0) * dt;  // 位置增量 = 速度 * dt
        delta_x_.block<3,1>(3,0) = Zero3d;                   // 速度增量 = 0（匀速模型）

        // 更新状态转移矩阵
        F_x_.block<3,3>(0,3) = Matrix3d::Identity() * dt;    // ∂p/∂v = I * dt
        F_w_.block<3,3>(3,0) = Matrix3d::Identity() * dt;    // 速度噪声的影响

        // 状态预测
        x_ += delta_x_;

        // 协方差预测（考虑过程噪声）
        P_ = F_x_ * P_ * F_x_.transpose() + F_w_ * Q_ * F_w_.transpose();
        last_predict_time_ = lidar_end_time;
    }

    /**
     * @brief 迭代卡尔曼滤波更新步骤
     * @param meas 测量值 [位置, 速度]
     * @param iter_num 迭代次数
     * @param lidar_end_time 当前时刻
     * @param meas_noise 测量噪声标准差
     *
     * 功能说明：
     * - 使用迭代方法提高非线性系统的估计精度
     * - Z轴噪声设置为较大值，使Z轴估计更平滑
     * - 迭代终止条件：状态变化小于阈值 或 达到最大迭代次数
     *
     * 迭代更新公式（IEKF）：
     * 1. 计算残差: r = z - H*x
     * 2. 计算卡尔曼增益: K = P*H^T*(H*P*H^T + R)^{-1}
     * 3. 状态更新: δx = K*r - (x - x_pred) + K*H*(x - x_pred)
     * 4. 收敛检查: ||δx|| < threshold
     */
    void update(const VectorXd &meas, const int &iter_num, const double &lidar_end_time, const double &meas_noise) {
        z_ = meas;
        R_.setIdentity();
        R_ *= meas_noise;
        R_(2,2) = meas_noise * 100;  // Z轴测量噪声设为较大值，使Z轴估计更平滑

        iter_num_ = iter_num;
        MatrixXd x_predicted = x_;  // 保存预测值用于迭代

        // 迭代更新循环
        for (int i = 0; i < iter_num; ++i) {
            MatrixXd residual = z_ - H_ * x_;  // 计算残差（测量值 - 预测值）

            // 计算卡尔曼增益
            K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();

            // 迭代状态更新（考虑预测值的影响）
            MatrixXd vec = x_ - x_predicted;
            delta_x_ = K_ * residual - vec + K_ * H_ * vec;
            x_ += delta_x_;

            // 收敛性检查
            if (delta_x_.norm() < 0.0001 || i == iter_num_ - 1){
                // 协方差更新（Joseph形式的简化版）
                P_ = P_ - K_ * H_ * P_;
                break;
            }
        }
    }

    /**
     * @brief 获取估计的位置
     * @return 位置向量（3×1）
     */
    MatrixXd get_state_pos(){
        return x_.block<3,1>(0,0);
    }

    /**
     * @brief 设置速度观测的雅可比矩阵
     * @param H_vel 速度观测雅可比（3×3矩阵）
     *
     * 功能说明：允许自定义速度观测模型的雅可比矩阵
     * 默认为单位矩阵（直接观测速度）
     */
    void set_H_vel(const Matrix3d &H_vel){
        H_.block<3,3>(3,3) = H_vel;
    }

    // ==================== 公有成员变量 ====================
    int iter_num_;                          // 迭代更新次数
    double last_predict_time_;              // 上次预测的时间戳
    double last_update_time_;               // 上次测量更新的时间戳
    double last_teammate_update_time_;      // 上次队友信息更新的时间戳

private:
    // ==================== 私有成员变量 ====================
    MatrixXd x_;        // 状态向量 [位置(3), 速度(3)]
    MatrixXd w_;        // 过程噪声向量
    MatrixXd delta_x_;  // 误差状态向量（状态增量）
    MatrixXd F_x_;      // 状态转移矩阵的雅可比
    MatrixXd F_w_;      // 噪声雅可比矩阵
    MatrixXd Q_;        // 过程噪声协方差矩阵
    MatrixXd R_;        // 测量噪声协方差矩阵
    MatrixXd P_;        // 状态协方差矩阵
    MatrixXd K_;        // 卡尔曼增益矩阵
    MatrixXd H_;        // 观测矩阵（测量雅可比）
    MatrixXd z_;        // 测量向量
};

#endif
