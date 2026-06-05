#ifndef EKF_IMU_HPP
#define EKF_IMU_HPP

#include "tools.hpp"
#include <deque>
#include <sensor_msgs/Imu.h>

/**
 * @brief IMU扩展卡尔曼滤波器类
 *
 * 该类实现了基于IMU的扩展卡尔曼滤波(EKF)，用于状态估计和点云运动补偿。
 * 主要功能包括：
 * 1. IMU初始化和零偏估计
 * 2. 基于IMU的运动模糊补偿
 * 3. 状态预测和协方差更新
 */
class IMUEKF
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen库内存对齐宏，确保正确的内存分配

  // ===== 初始化相关变量 =====
  bool init_flag;                  // 初始化完成标志
  double pcl_beg_time, pcl_end_time, last_pcl_end_time;  // 点云时间戳：当前帧起始、结束时间，上一帧结束时间
  int init_num;                     // 用于初始化的IMU数据计数
  Eigen::Vector3d mean_acc, mean_gyr;  // 初始化期间的加速度和角速度均值（用于估计零偏）
  sensor_msgs::Imu::Ptr last_imu;   // 上一个IMU数据指针
  int min_init_num = 30;            // 初始化所需的最小IMU数据量
  Eigen::Vector3d angvel_last, acc_s_last;  // 上一时刻的角速度和加速度

  // ===== 噪声协方差参数 =====
  Eigen::Vector3d cov_acc, cov_gyr;        // 加速度和角速度测量噪声协方差
  Eigen::Vector3d cov_bias_gyr, cov_bias_acc;  // 陀螺仪和加速度计零偏的过程噪声协方差
  // Eigen::Matrix<double, DIM, DIM> cov, cov_inv;

  // ===== 外参标定参数 =====
  Eigen::Matrix3d Lid_rot_to_IMU;    // LiDAR到IMU的旋转外参
  Eigen::Vector3d Lid_offset_to_IMU; // LiDAR到IMU的平移外参

  // ===== 其他参数 =====
  double scale_gravity = 1.0;         // 重力缩放因子（根据IMU输出单位调整）
  vector<IMUST> imu_poses;            // 存储IMU位姿序列，用于点云去畸变
  string imu_topic = "/livox/imu";    // IMU话题名称

  int point_notime = 0;               // 点云是否无时间戳标志（1为无时间戳）

  /**
   * @brief 构造函数
   *
   * 初始化所有标志位和向量为零
   */
  IMUEKF()
  {
    init_flag = false;              // 未初始化状态
    init_num = 0;                   // 初始化计数清零
    mean_acc.setZero(); mean_gyr.setZero();  // 均值向量清零
    angvel_last.setZero(); acc_s_last.setZero();  // 上一时刻测量值清零
  }

  /**
   * @brief 运动模糊补偿函数（点云去畸变）
   *
   * 使用IMU数据进行前向积分，预测扫描期间的运动轨迹，
   * 并将点云从采样时刻变换到扫描结束时刻，消除运动畸变
   *
   * @param xc 当前状态（位置、速度、姿态等）
   * @param pcl_in 输入点云（会被就地修改）
   * @param imus IMU数据队列
   */
  void motion_blur(IMUST &xc, pcl::PointCloud<PointType> &pcl_in, deque<sensor_msgs::Imu::Ptr> &imus)
  {
    // 将上一个IMU数据加入队列前端，确保时间连续性
    imus.push_front(last_imu);

    // 时间检查：确保点云时间戳单调递增
    if(last_pcl_end_time - pcl_beg_time > 0.01)
    {
      printf("%lf %lf\n", pcl_beg_time, last_pcl_end_time);
      printf("LiDAR time regress. Please check data\n"); exit(0);
    }

    // 清空IMU位姿缓存，准备存储新的轨迹
    imu_poses.clear();
    // imu_poses.emplace_back(0, xc.R, xc.p, xc.v, angvel_last, acc_s_last);

    // ===== 初始化状态变量 =====
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);  // IMU系加速度、平均角速度/加速度、速度、位置
    Eigen::Matrix3d R_imu(xc.R);        // IMU姿态旋转矩阵
    Eigen::Matrix<double, DIM, DIM> F_x, cov_w;  // 状态转移矩阵和过程噪声协方差

    double dt = 0;  // 时间间隔

    // ===== 前向积分：遍历IMU数据进行状态预测 =====
    for(auto it_imu=imus.begin(); it_imu!=imus.end()-1; it_imu++)
    {
      sensor_msgs::Imu &head = **(it_imu);      // 当前IMU数据
      sensor_msgs::Imu &tail = **(it_imu+1);    // 下一个IMU数据

      // 跳过上一帧点云结束时间之前的IMU数据
      if(tail.header.stamp.toSec() < last_pcl_end_time) continue;

      // 使用中值法计算平均角速度和加速度（提高积分精度）
      angvel_avr << 0.5*(head.angular_velocity.x + tail.angular_velocity.x),
                    0.5*(head.angular_velocity.y + tail.angular_velocity.y),
                    0.5*(head.angular_velocity.z + tail.angular_velocity.z);
      acc_avr << 0.5*(head.linear_acceleration.x + tail.linear_acceleration.x),
                 0.5*(head.linear_acceleration.y + tail.linear_acceleration.y),
                 0.5*(head.linear_acceleration.z + tail.linear_acceleration.z);

      // 零偏补偿
      angvel_avr -= xc.bg;                          // 减去陀螺仪零偏
      acc_avr = acc_avr * scale_gravity - xc.ba;    // 缩放并减去加速度计零偏
      acc_imu = R_imu * acc_avr + xc.g;             // 转换到世界坐标系并加上重力

      // 计算时间间隔
      // 如果当前IMU时间早于上一帧点云结束时间，则从上一帧结束时间开始积分
      // if(head.header.stamp.toSec() < last_pcl_end_time)
      //   dt = tail.header.stamp.toSec() - last_pcl_end_time;
      // else
      //   dt = tail.header.stamp.toSec() - head.header.stamp.toSec();
      double cur_time = head.header.stamp.toSec();
      if(cur_time < last_pcl_end_time)
        cur_time = last_pcl_end_time;
      dt = tail.header.stamp.toSec() - cur_time;

      // 记录当前时刻的IMU位姿（用于后续点云去畸变）
      double offt = cur_time - pcl_beg_time;  // 相对于点云起始时间的偏移
      imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
      
      // ===== EKF预测步骤：更新协方差矩阵 =====
      Eigen::Matrix3d acc_avr_skew = hat(acc_avr);  // 加速度反对称矩阵（用于叉乘运算）
      Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);  // 旋转增量矩阵

      // 构建状态转移矩阵F_x（雅可比矩阵）
      F_x.setIdentity();
      cov_w.setZero();

      // 状态向量: [旋转(0:2), 位置(3:5), 速度(6:8), 陀螺零偏(9:11), 加速度零偏(12:14)]
      F_x.block<3,3>(0,0)  = Exp(angvel_avr, - dt);      // ∂R/∂R
      F_x.block<3,3>(0,9)  = -I33 * dt;                  // ∂R/∂bg
      F_x.block<3,3>(3,6)  = I33 * dt;                   // ∂p/∂v
      F_x.block<3,3>(6,0)  = - R_imu * acc_avr_skew * dt;  // ∂v/∂R
      F_x.block<3,3>(6,12) = - R_imu * dt;               // ∂v/∂ba

      // 构建过程噪声协方差矩阵
      cov_w.block<3,3>(0,0).diagonal() = cov_gyr * dt * dt;      // 角速度噪声
      cov_w.block<3,3>(6,6) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;  // 加速度噪声
      cov_w.block<3,3>(9,9).diagonal()   = cov_bias_gyr * dt * dt;   // 陀螺零偏随机游走
      cov_w.block<3,3>(12,12).diagonal() = cov_bias_acc * dt * dt;   // 加速度零偏随机游走

      // 协方差传播：P = F*P*F^T + Q
      xc.cov = F_x * xc.cov * F_x.transpose() + cov_w;
      // ===== 状态更新：运动学方程积分 =====
      // R_imu = R_imu * Exp_f;
      // acc_imu = R_imu * acc_avr + xc.g;
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;  // 位置更新（二阶积分）
      vel_imu = vel_imu + acc_imu * dt;                            // 速度更新（一阶积分）
      R_imu = R_imu * Exp_f;                                       // 姿态更新（李群指数映射）

      // double offt = tail.header.stamp.toSec() - pcl_beg_time;
      // double offt = max(head.header.stamp.toSec() - pcl_beg_time, 0.0);
      // imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
    }

    // ===== 外推到点云结束时刻 =====
    // 如果点云结束时间晚于最后一个IMU时间，进行前向外推；否则进行后向外推
    double imu_end_time = imus.back()->header.stamp.toSec();
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;  // 外推方向标志
    dt = note * (pcl_end_time - imu_end_time);
    xc.v = vel_imu + note * acc_imu * dt;                                    // 速度外推
    xc.R = R_imu * Exp(note*angvel_avr, dt);                                 // 姿态外推
    xc.p = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;  // 位置外推
    xc.t = pcl_end_time;                                                     // 更新时间戳

    // ===== 更新IMU队列的时间戳 =====
    // 创建两个新的IMU数据，时间戳分别对应上一帧和当前帧的点云结束时间
    sensor_msgs::ImuPtr imu1(new sensor_msgs::Imu(*imus.front()));
    sensor_msgs::ImuPtr imu2(new sensor_msgs::Imu(*imus.back()));
    imu1->header.stamp.fromSec(last_pcl_end_time);  // 队列首部对应上一帧结束时间
    imu2->header.stamp.fromSec(pcl_end_time);       // 队列尾部对应当前帧结束时间
    // imus.pop_front();
    last_imu = imus.back();                         // 保存最后一个IMU数据
    last_pcl_end_time = pcl_end_time;               // 更新点云结束时间
    imus.front() = imu1;
    imus.back()  = imu2;

    // 如果点云没有时间戳，跳过去畸变步骤
    if(point_notime)
      return;

    // ===== 点云去畸变：反向遍历点云和IMU位姿 =====
    auto it_pcl = pcl_in.end() - 1;  // 从点云最后一个点开始
    for(int i=imu_poses.size()-1; i>=0; i--)  // 从最新的IMU位姿开始反向遍历
    {
      IMUST &head = imu_poses[i];  // 当前IMU位姿
      R_imu = head.R;              // 旋转
      acc_imu = head.ba;           // 加速度
      vel_imu = head.v;            // 速度
      pos_imu = head.p;            // 位置
      angvel_avr = head.bg;        // 角速度

      // 处理时间戳大于当前IMU时刻的所有点
      for(; it_pcl->curvature > head.t; it_pcl--)
      {
        dt = it_pcl->curvature - head.t;  // 点采样时刻相对于当前IMU时刻的时间差

        // 计算点采样时刻的IMU姿态和位置
        Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);                          // 姿态
        Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;  // 相对位置

        // 点云去畸变变换：
        // 1. LiDAR系 -> IMU系（通过外参）
        // 2. 采样时刻IMU系 -> 世界系（通过R_i和T_ei）
        // 3. 世界系 -> 扫描结束时刻IMU系（通过xc.R和xc.p）
        // 4. IMU系 -> LiDAR系（通过外参逆变换）
        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        Eigen::Vector3d P_compensate = Lid_rot_to_IMU.transpose() * (xc.R.transpose() * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) - Lid_offset_to_IMU);

        // 更新点云坐标
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
        if(it_pcl == pcl_in.begin()) break;  // 已到达第一个点
      }
    }

  }

  /**
   * @brief IMU初始化函数
   *
   * 通过累积多帧IMU数据计算加速度和角速度的均值，
   * 用于估计初始重力方向和传感器零偏
   *
   * @param imus IMU数据队列
   */
  void IMU_init(deque<sensor_msgs::Imu::Ptr> &imus)
  {
    Eigen::Vector3d cur_acc, cur_gyr;

    // 遍历所有IMU数据，计算在线均值
    for(sensor_msgs::Imu::Ptr imu: imus)
    {
      // 提取加速度和角速度测量值
      cur_acc << imu->linear_acceleration.x,
                 imu->linear_acceleration.y,
                 imu->linear_acceleration.z;
      cur_gyr << imu->angular_velocity.x,
                 imu->angular_velocity.y,
                 imu->angular_velocity.z;

      if(init_num != 0)
      {
        // 增量式均值更新：mean_new = mean_old + (x - mean_old) / n
        mean_acc += (cur_acc - mean_acc) / init_num;
        mean_gyr += (cur_gyr - mean_gyr) / init_num;
      }
      else
      {
        // 第一个数据点直接赋值
        mean_acc = cur_acc; // modify
        mean_gyr = cur_gyr;
        init_num = 1;
      }

      init_num++;  // 计数器递增
    }

    last_imu = imus.back();  // 保存最后一个IMU数据
  }

  /**
   * @brief IMU处理主函数
   *
   * 根据初始化状态决定执行初始化或运动补偿
   *
   * @param x_curr 当前状态
   * @param pcl_in 输入点云
   * @param imus IMU数据队列
   * @return 0表示正在初始化，1表示初始化完成并进行了运动补偿
   */
  int process(IMUST &x_curr, pcl::PointCloud<PointType> &pcl_in, deque<sensor_msgs::Imu::Ptr> &imus)
  {
    // ===== 初始化阶段 =====
    if(!init_flag)
    {
      IMU_init(imus);  // 累积IMU数据计算均值

      // 自动检测重力缩放因子
      // 如果加速度均值模长远小于重力加速度，说明IMU输出单位可能是g而非m/s²
      if(mean_acc.norm() < 2 && imu_topic == "/livox/imu")
        scale_gravity = G_m_s2;  // 使用重力加速度常量进行缩放

      printf("scale_gravity: %lf %lf %d\n", scale_gravity, mean_acc.norm(), init_num);

      // 初始化重力向量（加速度均值的反方向即为重力方向）
      x_curr.g = -mean_acc * scale_gravity;

      // 检查是否收集了足够的初始化数据
      if(init_num > min_init_num) init_flag = true;
      last_pcl_end_time = pcl_end_time;

      return 0;  // 返回0表示仍在初始化
    }

    // ===== 正常运行阶段：执行运动补偿 =====
    motion_blur(x_curr, pcl_in, imus);
    return 1;  // 返回1表示初始化完成
  }

};

#endif

