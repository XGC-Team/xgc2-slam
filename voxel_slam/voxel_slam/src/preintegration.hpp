#pragma once

#include "tools.hpp"
#include <deque>
#include <sensor_msgs/Imu.h>

// IMU预积分全局参数
// Don't forget to init
double imupre_scale_gravity = 1.0;  // IMU重力缩放因子，用于校正加速度计测量值
Eigen::Matrix<double, 6, 6> noiseMeas, noiseWalk;  // 测量噪声和随机游走噪声协方差矩阵

/**
 * @brief IMU预积分类
 * 实现IMU数据的预积分，包括旋转、速度、位置的增量计算，
 * 以及对陀螺仪和加速度计偏置的雅可比矩阵计算，用于后续的状态估计和优化
 */
class IMU_PRE
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen库内存对齐宏

  // 预积分测量值（从时刻i到时刻j的增量）
  Eigen::Matrix3d R_delta;  // 旋转增量矩阵 R_i^j
  Eigen::Vector3d p_delta, v_delta;  // 位置增量和速度增量 p_i^j, v_i^j
  Eigen::Vector3d bg, ba;  // 陀螺仪偏置和加速度计偏置

  // 预积分测量值对偏置的雅可比矩阵
  Eigen::Matrix3d R_bg;  // 旋转增量对陀螺仪偏置的雅可比 ∂R/∂bg
  Eigen::Matrix3d p_bg, p_ba;  // 位置增量对陀螺仪和加速度计偏置的雅可比 ∂p/∂bg, ∂p/∂ba
  Eigen::Matrix3d v_bg, v_ba;  // 速度增量对陀螺仪和加速度计偏置的雅可比 ∂v/∂bg, ∂v/∂ba

  double dtime;  // 预积分的总时间间隔

  Eigen::Vector3d dbg, dba;  // 偏置的增量（用于优化更新）
  Eigen::Vector3d dbg_buf, dba_buf;  // 偏置增量的备份缓存

  Eigen::Matrix<double, DIM, DIM> cov;  // 预积分的协方差矩阵

  deque<sensor_msgs::ImuPtr> _imus;  // IMU数据队列

  /**
   * @brief 构造函数，初始化IMU预积分
   * @param bg1 陀螺仪偏置初始值，默认为零向量
   * @param ba1 加速度计偏置初始值，默认为零向量
   */
  IMU_PRE(const Eigen::Vector3d &bg1 = Eigen::Vector3d::Zero(), const Eigen::Vector3d &ba1 = Eigen::Vector3d::Zero())
  {
    bg = bg1; ba = ba1;  // 设置偏置初始值
    R_delta.setIdentity();  // 旋转增量初始化为单位矩阵
    p_delta.setZero(); v_delta.setZero();  // 位置和速度增量初始化为零

    R_bg.setZero();  // 雅可比矩阵初始化为零
    p_bg.setZero(); p_ba.setZero();
    v_bg.setZero(); v_ba.setZero();

    dtime = 0;  // 时间间隔初始化为0

    dbg.setZero(); dba.setZero();  // 偏置增量初始化为零
    dbg_buf.setZero(); dba_buf.setZero();  // 偏置增量缓存初始化为零

    cov.setZero();  // 协方差矩阵初始化为零
  }

  /**
   * @brief 批量添加IMU数据并进行预积分
   * @param imus IMU数据队列
   *
   * 此函数将输入的IMU数据添加到内部队列，然后使用中值积分法进行预积分计算。
   * 使用相邻两帧IMU数据的平均值作为当前时刻的测量值（中值积分）。
   */
  void push_imu(deque<sensor_msgs::ImuPtr> &imus)
  {
    _imus.insert(_imus.end(), imus.begin(), imus.end());  // 将IMU数据插入内部队列
    Eigen::Vector3d cur_gyr, cur_acc;
    for(auto it_imu=imus.begin()+1; it_imu!=imus.end(); it_imu++)
    {
      sensor_msgs::Imu &imu1 = **(it_imu-1);  // 前一帧IMU数据
      sensor_msgs::Imu &imu2 = **it_imu;  // 当前帧IMU数据

      double dt = imu2.header.stamp.toSec() - imu1.header.stamp.toSec();  // 计算时间间隔

      // 使用中值积分：取相邻两帧的平均值
      cur_gyr << 0.5*(imu1.angular_velocity.x + imu2.angular_velocity.x),
                 0.5*(imu1.angular_velocity.y + imu2.angular_velocity.y),
                 0.5*(imu1.angular_velocity.z + imu2.angular_velocity.z);
      cur_acc << 0.5*(imu1.linear_acceleration.x + imu2.linear_acceleration.x),
                 0.5*(imu1.linear_acceleration.y + imu2.linear_acceleration.y),
                 0.5*(imu1.linear_acceleration.z + imu2.linear_acceleration.z);

      cur_gyr = cur_gyr - bg;  // 减去陀螺仪偏置
      cur_acc = cur_acc * imupre_scale_gravity - ba;  // 应用重力缩放并减去加速度计偏置

      add_imu(cur_gyr, cur_acc, dt);  // 执行预积分更新
    }
  }

  /**
   * @brief 添加单个IMU测量值并更新预积分
   * @param cur_gyr 去偏置后的角速度测量值
   * @param cur_acc 去偏置后的加速度测量值
   * @param dt 时间间隔
   *
   * 此函数实现预积分的核心递推公式，包括：
   * 1. 更新预积分测量值（旋转、速度、位置增量）
   * 2. 更新预积分测量值对偏置的雅可比矩阵
   * 3. 更新预积分协方差矩阵
   */
  void add_imu(Eigen::Vector3d &cur_gyr, Eigen::Vector3d &cur_acc, double dt)
  {
    dtime += dt;  // 累加总时间
    Eigen::Matrix3d R_inc = Exp(cur_gyr, dt);  // 计算旋转增量：Exp(ω*dt)
    Eigen::Matrix3d R_jr(jr(cur_gyr * dt));  // 计算SO(3)右雅可比矩阵

    Eigen::Matrix3d R_dt = dt * R_delta;  // dt * R_delta，用于速度雅可比更新
    Eigen::Matrix3d R_dt2_2 = 0.5*dt*dt*R_delta;  // 0.5*dt²*R_delta，用于位置雅可比更新

    Eigen::Matrix3d acc_skew;  // 加速度的反对称矩阵
    acc_skew << SKEW_SYM_MATRX(cur_acc);

    // 更新雅可比矩阵（对偏置的导数）
    p_ba = p_ba + v_ba*dt - R_dt2_2;  // 位置对加速度计偏置的雅可比
    p_bg = p_bg + v_bg*dt - R_dt2_2*acc_skew*R_bg;  // 位置对陀螺仪偏置的雅可比
    v_ba = v_ba - R_dt;  // 速度对加速度计偏置的雅可比
    v_bg = v_bg - R_dt * acc_skew * R_bg;  // 速度对陀螺仪偏置的雅可比
    R_bg = R_inc.transpose() * R_bg - R_jr*dt;  // 旋转对陀螺仪偏置的雅可比
    
    // 以下为协方差传播的完整版本（已注释）
    // Eigen::Matrix<double, DIM, DIM> Ai;
    // Eigen::Matrix<double, DIM, DNOI> Bi;
    // Ai.setIdentity(); Bi.setZero();
    // Ai.block<3, 3>(0, 0) = R_inc.transpose();
    // Ai.block<3, 3>(0, 9) = -I33 * dt;
    // // Ai.block<3, 3>(3, 0) = -R_dt2_2 * acc_skew;
    // Ai.block<3, 3>(3, 6) = I33 * dt;
    // // Ai.block<3, 3>(3, 12) = -R_dt2_2;
    // Ai.block<3, 3>(6, 0) = -R_dt * acc_skew;
    // Ai.block<3, 3>(6, 12) = -R_dt;
    // // Ai.block<3, 3>(6, 15) = I33 * dt;

    // // Bi.block<3, 3>(0, 0) = R_jr * dt;
    // Bi.block<3, 3>(0, 0) = dt * I33;
    // // Bi.block<3, 3>(3, 3) = R_dt2_2;
    // Bi.block<3, 3>(6, 3) = R_dt;
    // Bi.block<3, 3>(9, 6) = I33 * dt;
    // Bi.block<3, 3>(12, 9) = I33 * dt;

    // cov = Ai*cov*Ai.transpose() + Bi*noi_imu*Bi.transpose();

    // 协方差传播：更新预积分的协方差矩阵
    // 使用一阶线性化误差状态传播模型：cov = A * cov * A^T + B * noise * B^T
    Eigen::Matrix<double, 9, 9> A;  // 状态转移矩阵（9维：旋转3+位置3+速度3）
    Eigen::Matrix<double, 9, 6> B;  // 噪声转移矩阵（6维噪声：陀螺仪3+加速度计3）
    A.setIdentity(); B.setZero();

    // 构建状态转移矩阵A的非零块
    A.block<3, 3>(0, 0) = R_inc.transpose();  // 旋转误差的传播
    A.block<3, 3>(3, 0) = -R_dt2_2 * acc_skew;  // 旋转误差对位置误差的影响
    A.block<3, 3>(3, 6) = I33 * dt;  // 速度误差对位置误差的影响
    A.block<3, 3>(6, 0) = -R_dt * acc_skew;  // 旋转误差对速度误差的影响

    // 构建噪声转移矩阵B的非零块
    B.block<3, 3>(0, 0) = R_jr * dt;  // 陀螺仪噪声对旋转误差的影响
    B.block<3, 3>(3, 3) = R_dt2_2;  // 加速度计噪声对位置误差的影响
    B.block<3, 3>(6, 3) = R_dt;  // 加速度计噪声对速度误差的影响

    // 更新协方差矩阵：前9x9块（旋转、位置、速度）
    // cov.block<9, 9>(0, 0) = A * cov.block<9, 9>(0, 0) * A.transpose() + B * noiseMeas * B.transpose() * dt * dt;
    cov.block<9, 9>(0, 0) = A * cov.block<9, 9>(0, 0) * A.transpose() + B * noiseMeas * B.transpose();
    // 更新协方差矩阵：后6x6块（偏置随机游走）
    // cov.block<6, 6>(9, 9) += noiseWalk * dt * dt;
    cov.block<6, 6>(9, 9) += noiseWalk * dt;

    // 更新预积分测量值
    p_delta += v_delta*dt + R_dt2_2*cur_acc;  // 位置增量更新
    v_delta += R_dt * cur_acc;  // 速度增量更新
    R_delta = R_delta * R_inc;  // 旋转增量更新
  }

  /**
   * @brief 计算预积分残差和雅可比矩阵（不优化重力）
   * @param st1 时刻i的IMU状态（位姿、速度、偏置、重力）
   * @param st2 时刻j的IMU状态
   * @param jtj 输出的Hessian矩阵（J^T * Σ^-1 * J）
   * @param gg 输出的梯度向量（J^T * Σ^-1 * r）
   * @param jac_enable 是否计算雅可比矩阵
   * @return 残差的马氏距离平方（r^T * Σ^-1 * r）
   *
   * 此函数用于优化IMU状态，包括旋转、位置、速度和偏置，但不优化重力向量。
   */
  double give_evaluate(IMUST &st1, IMUST &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable)
  {
    Eigen::Matrix<double, DIM, DIM> joca, jocb;  // 对状态st1和st2的雅可比矩阵
    Eigen::Matrix<double, DIM, 1> rr;  // 残差向量（15维）
    joca.setZero(); jocb.setZero(); rr.setZero();

    // 使用当前偏置增量修正预积分测量值
    Eigen::Matrix3d R_correct = R_delta * Exp(R_bg * dbg);  // 修正后的旋转增量
    Eigen::Vector3d t_correct = p_delta + p_bg*dbg + p_ba*dba;  // 修正后的位置增量
    Eigen::Vector3d v_correct = v_delta + v_bg*dbg + v_ba*dba;  // 修正后的速度增量

    // 计算残差：预期值与修正后的预积分测量值之差
    Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;  // 旋转残差
    Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime*st1.g);  // 速度预期值
    Eigen::Vector3d res_v = exp_v - v_correct;  // 速度残差
    Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v*dtime - 0.5*dtime*dtime*st1.g);  // 位置预期值
    Eigen::Vector3d res_t = exp_t - t_correct;  // 位置残差

    Eigen::Vector3d res_bg = st2.bg - st1.bg;  // 陀螺仪偏置残差（偏置应该缓慢变化）
    Eigen::Vector3d res_ba = st2.ba - st1.ba;  // 加速度计偏置残差

    double b_wei = 1;  // 偏置残差的权重

    // 组装残差向量（15维：旋转3+位置3+速度3+bg偏置3+ba偏置3）
    rr.block<3, 1>(0, 0) = Log(res_r);  // 旋转残差（李代数形式）
    rr.block<3, 1>(3, 0) = res_t;  // 位置残差
    rr.block<3, 1>(6, 0) = res_v;  // 速度残差
    rr.block<3, 1>(9, 0) = res_bg*b_wei;  // 陀螺仪偏置残差
    rr.block<3, 1>(12, 0) = res_ba*b_wei;  // 加速度计偏置残差

    // rr.block<3, 1>(15, 0) = st2.g - st1.g;  // 重力残差（此版本不优化重力）

    Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();  // 协方差矩阵的逆（信息矩阵）

    if(jac_enable)  // 如果需要计算雅可比矩阵
    {
      Eigen::Matrix3d JR_inv = jr_inv(res_r);  // 旋转残差的右雅可比逆矩阵
      // 构建对状态st1的雅可比矩阵joca（15x15）
      // joca.block<3, 3>(0, 0) = -JR_inv * st1.R.transpose() * st2.R;
      joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;  // ∂r_R/∂R_i
      jocb.block<3, 3>(0, 0) =  JR_inv;  // ∂r_R/∂R_j
      joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * jr(R_bg*dbg) * R_bg;  // ∂r_R/∂bg_i

      joca.block<3, 3>(3, 0) = hat(exp_t);  // ∂r_p/∂R_i (使用反对称矩阵)
      joca.block<3, 3>(3, 3) = -st1.R.transpose();  // ∂r_p/∂p_i
      joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;  // ∂r_p/∂v_i
      joca.block<3, 3>(3, 9) = -p_bg;  // ∂r_p/∂bg_i
      joca.block<3, 3>(3, 12) = -p_ba;  // ∂r_p/∂ba_i
      jocb.block<3, 3>(3, 3) = st1.R.transpose();  // ∂r_p/∂p_j

      joca.block<3, 3>(6, 0) = hat(exp_v);  // ∂r_v/∂R_i
      joca.block<3, 3>(6, 6) = -st1.R.transpose();  // ∂r_v/∂v_i
      joca.block<3, 3>(6, 9) = -v_bg;  // ∂r_v/∂bg_i
      joca.block<3, 3>(6, 12) = -v_ba;  // ∂r_v/∂ba_i
      jocb.block<3, 3>(6, 6) = st1.R.transpose();  // ∂r_v/∂v_j

      joca.block<3, 3>(9, 9) = -I33*b_wei;  // ∂r_bg/∂bg_i
      joca.block<3, 3>(12, 12) = -I33*b_wei;  // ∂r_ba/∂ba_i
      jocb.block<3, 3>(9, 9) = I33*b_wei;  // ∂r_bg/∂bg_j
      jocb.block<3, 3>(12, 12) = I33*b_wei;  // ∂r_ba/∂ba_j

      // 重力雅可比（此版本不优化重力）
      // joca.block<3, 3>(3, 15) = -0.5 * st1.R.transpose() * dtime * dtime;
      // joca.block<3, 3>(6, 15) = -st1.R.transpose() * dtime;
      // joca.block<3, 3>(15, 15) = -I33;
      // jocb.block<3, 3>(15, 15) = I33;

      // 组合雅可比矩阵：[joca | jocb] (15x30)
      Eigen::Matrix<double, DIM, 2*DIM> joc;
      joc.block<DIM, DIM>(0, 0) = joca;
      joc.block<DIM, DIM>(0, DIM) = jocb;

      // 计算Hessian矩阵和梯度向量（考虑协方差）
      // jtj = joc.transpose() * joc;
      // gg = joc.transpose() * rr;

      jtj = joc.transpose() * cov_inv * joc;  // H = J^T * Σ^-1 * J
      gg = joc.transpose() * cov_inv * rr;  // g = J^T * Σ^-1 * r
    }

    // return rr.squaredNorm();  // 残差的平方和
    return rr.dot(cov_inv * rr);  // 马氏距离：r^T * Σ^-1 * r
  }

  /**
   * @brief 计算预积分残差和雅可比矩阵（同时优化重力）
   * @param st1 时刻i的IMU状态（位姿、速度、偏置、重力）
   * @param st2 时刻j的IMU状态
   * @param jtj 输出的Hessian矩阵（J^T * Σ^-1 * J）
   * @param gg 输出的梯度向量（J^T * Σ^-1 * r）
   * @param jac_enable 是否计算雅可比矩阵
   * @return 残差的马氏距离平方（r^T * Σ^-1 * r）
   *
   * 此函数与give_evaluate类似，但额外优化重力向量（3个自由度）。
   * 雅可比矩阵维度为 15 x 33（两个状态各15维 + 重力3维）。
   */
  double give_evaluate_g(IMUST &st1, IMUST &st2, Eigen::MatrixXd &jtj, Eigen::VectorXd &gg, bool jac_enable)
  {
    Eigen::Matrix<double, DIM, DIM> joca, jocb;  // 对状态st1和st2的雅可比矩阵
    Eigen::Matrix<double, DIM, 1> rr;  // 残差向量
    joca.setZero(); jocb.setZero(); rr.setZero();
    Eigen::Matrix<double, DIM, 3> jocg;  // 对重力向量的雅可比矩阵
    jocg.setZero();

    // 使用当前偏置增量修正预积分测量值
    Eigen::Matrix3d R_correct = R_delta * Exp(R_bg * dbg);
    Eigen::Vector3d t_correct = p_delta + p_bg*dbg + p_ba*dba;
    Eigen::Vector3d v_correct = v_delta + v_bg*dbg + v_ba*dba;

    // 计算残差
    Eigen::Matrix3d res_r = R_correct.transpose() * st1.R.transpose() * st2.R;
    Eigen::Vector3d exp_v = st1.R.transpose() * (st2.v - st1.v - dtime*st1.g);
    Eigen::Vector3d res_v = exp_v - v_correct;
    Eigen::Vector3d exp_t = st1.R.transpose() * (st2.p - st1.p - st1.v*dtime - 0.5*dtime*dtime*st1.g);
    Eigen::Vector3d res_t = exp_t - t_correct;

    Eigen::Vector3d res_bg = st2.bg - st1.bg;
    Eigen::Vector3d res_ba = st2.ba - st1.ba;

    double b_wei = 1;

    // 组装残差向量
    rr.block<3, 1>(0, 0) = Log(res_r);
    rr.block<3, 1>(3, 0) = res_t;
    rr.block<3, 1>(6, 0) = res_v;
    rr.block<3, 1>(9, 0) = res_bg*b_wei;
    rr.block<3, 1>(12, 0) = res_ba*b_wei;

    // rr.block<3, 1>(15, 0) = st2.g - st1.g;  // 重力残差（这里不约束重力变化）
    Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();

    if(jac_enable)
    {
      Eigen::Matrix3d JR_inv = jr_inv(res_r);
      // 构建对状态st1的雅可比矩阵
      // joca.block<3, 3>(0, 0) = -JR_inv * st1.R.transpose() * st2.R;
      joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
      jocb.block<3, 3>(0, 0) =  JR_inv;
      joca.block<3, 3>(0, 9) = -JR_inv * res_r.transpose() * jr(R_bg*dbg) * R_bg;

      joca.block<3, 3>(3, 0) = hat(exp_t);
      joca.block<3, 3>(3, 3) = -st1.R.transpose();
      joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
      joca.block<3, 3>(3, 9) = -p_bg;
      joca.block<3, 3>(3, 12) = -p_ba;
      jocb.block<3, 3>(3, 3) = st1.R.transpose();

      joca.block<3, 3>(6, 0) = hat(exp_v);
      joca.block<3, 3>(6, 6) = -st1.R.transpose();
      joca.block<3, 3>(6, 9) = -v_bg;
      joca.block<3, 3>(6, 12) = -v_ba;
      jocb.block<3, 3>(6, 6) = st1.R.transpose();

      joca.block<3, 3>(9, 9) = -I33*b_wei;
      joca.block<3, 3>(12, 12) = -I33*b_wei;
      jocb.block<3, 3>(9, 9) = I33*b_wei;
      jocb.block<3, 3>(12, 12) = I33*b_wei;

      // 重力雅可比（如果优化重力，取消注释）
      // joca.block<3, 3>(3, 15) = -0.5 * st1.R.transpose() * dtime * dtime;
      // joca.block<3, 3>(6, 15) = -st1.R.transpose() * dtime;
      // joca.block<3, 3>(15, 15) = -I33;
      // jocb.block<3, 3>(15, 15) = I33;

      // 构建对重力向量的雅可比矩阵（额外的3维）
      jocg.block<3, 3>(3, 0) = st1.R.transpose() * (-0.5*dtime*dtime);  // ∂r_p/∂g
      jocg.block<3, 3>(6, 0) = st1.R.transpose() * (-dtime);  // ∂r_v/∂g

      // 组合雅可比矩阵：[joca | jocb | jocg] (15x33)
      Eigen::Matrix<double, DIM, 2*DIM+3> joc;
      joc.block<DIM, DIM>(0, 0) = joca;
      joc.block<DIM, DIM>(0, DIM) = jocb;
      joc.block<DIM, 3>(0, 2*DIM) = jocg;

      // 计算Hessian矩阵和梯度向量
      // jtj = joc.transpose() * joc;
      // gg = joc.transpose() * rr;

      jtj = joc.transpose() * cov_inv * joc;  // H = J^T * Σ^-1 * J
      gg = joc.transpose() * cov_inv * rr;  // g = J^T * Σ^-1 * r
    }

    // return rr.squaredNorm();
    return rr.dot(cov_inv * rr);  // 马氏距离
  }

  /**
   * @brief 更新偏置增量
   * @param dxi 状态增量向量（DIM维，通常为15维）
   *
   * 从优化得到的状态增量中提取偏置增量，并更新陀螺仪和加速度计偏置的增量值。
   * 在优化迭代过程中调用，用于累积偏置的修正量。
   */
  void update_state(const Eigen::Matrix<double, DIM, 1> &dxi)
  {
    dbg_buf = dbg;  // 备份当前偏置增量
    dba_buf = dba;

    dbg += dxi.block<3, 1>(9, 0);  // 更新陀螺仪偏置增量（第9-11维）
    dba += dxi.block<3, 1>(12, 0);  // 更新加速度计偏置增量（第12-14维）
  }

  /**
   * @brief 合并两个预积分结果
   * @param imu2 第二段预积分对象
   *
   * 将当前预积分（时刻i到j）与另一个预积分（时刻j到k）合并，
   * 得到从时刻i到k的完整预积分。包括：
   * 1. 合并预积分测量值（旋转、速度、位置增量）
   * 2. 合并雅可比矩阵（考虑链式法则）
   * 3. 合并协方差矩阵（考虑误差传播）
   */
  void merge(IMU_PRE &imu2)
  {
    // 更新雅可比矩阵：考虑两段预积分的组合效应
    p_bg += v_bg*imu2.dtime + R_delta*(imu2.p_bg-hat(imu2.p_delta)*R_bg);
    p_ba += v_ba*imu2.dtime + R_delta*imu2.p_ba;
    v_bg += R_delta*(imu2.v_bg - hat(imu2.v_delta)*R_bg);
    v_ba += R_delta*imu2.v_ba;
    R_bg = imu2.R_delta.transpose()*R_bg + imu2.R_bg;

    // 协方差传播：合并两段预积分的不确定性
    Eigen::Matrix<double, DIM, DIM> Ai, Bi;  // 状态转移矩阵
    Ai.setIdentity(); Bi.setIdentity();
    Ai.block<3, 3>(0, 0) = imu2.R_delta.transpose();  // 旋转的组合
    Ai.block<3, 3>(3, 0) = -R_delta * hat(imu2.p_delta);  // 旋转误差对位置的影响
    Ai.block<3, 3>(3, 6) = I33 * imu2.dtime;  // 速度误差对位置的影响
    Ai.block<3, 3>(6, 0) = -R_delta * hat(imu2.v_delta);  // 旋转误差对速度的影响

    Bi.block<3, 3>(3, 3) = R_delta;  // 第二段位置误差的旋转变换
    Bi.block<3, 3>(6, 6) = R_delta;  // 第二段速度误差的旋转变换
    cov = Ai*cov*Ai.transpose() + Bi*imu2.cov*Bi.transpose();  // 合并协方差

    // 更新预积分测量值：组合两段预积分
    p_delta += v_delta*imu2.dtime + R_delta*imu2.p_delta;  // 位置增量组合
    v_delta += R_delta*imu2.v_delta;  // 速度增量组合
    R_delta = R_delta * imu2.R_delta;  // 旋转增量组合（旋转矩阵相乘）

    dtime += imu2.dtime;  // 累加时间间隔
  }

};  // class IMU_PRE

