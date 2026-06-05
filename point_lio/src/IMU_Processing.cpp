/**
 * @file IMU_Processing.cpp
 * @brief IMU数据处理类实现文件
 *
 * 本文件实现了Point-LIO算法中的IMU数据处理功能，主要包括：
 * 1. IMU初始化：通过静止状态下的IMU数据估计重力方向和陀螺仪零偏
 * 2. 重力对齐：计算从测量重力到标准重力的旋转矩阵
 * 3. 协方差设置：配置陀螺仪和加速度计的测量噪声协方差
 * 4. 点云去畸变：利用IMU数据对激光点云进行运动补偿
 *
 * 主要功能：
 * - IMU预积分和状态预测
 * - 重力方向估计和坐标系对齐
 * - IMU-LiDAR数据同步处理
 */
#include "IMU_Processing.h"

/**
 * @brief 点云时间戳排序比较函数
 * @param x 第一个点
 * @param y 第二个点
 * @return 如果x的时间戳小于y的时间戳则返回true
 * @note 这里使用curvature字段存储相对时间戳
 */
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

/**
 * @brief 设置陀螺仪测量协方差缩放因子
 * @param scaler 3维协方差缩放向量(对应x,y,z三个轴)
 */
void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;  // 保存陀螺仪协方差缩放因子
}

/**
 * @brief 设置加速度计测量协方差缩放因子
 * @param scaler 3维协方差缩放向量(对应x,y,z三个轴)
 */
void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_vel_scale = scaler;  // 保存加速度计协方差缩放因子
}

/**
 * @brief ImuProcess类构造函数
 *
 * 初始化IMU处理器的各项参数：
 * - 设置首帧标志和初始化标志
 * - 初始化均值加速度和角速度为零向量
 * - 设置状态协方差矩阵为单位矩阵
 */
ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true)
{
  imu_en = true;                        // 使能IMU数据处理
  init_iter_num = 1;                    // 初始化迭代计数器
  mean_acc      = V3D(0, 0, 0.0);       // 平均加速度初始化为零
  mean_gyr      = V3D(0, 0, 0);         // 平均角速度初始化为零
  after_imu_init_ = false;              // IMU初始化完成标志
  state_cov.setIdentity();              // 状态协方差矩阵初始化为单位矩阵
}

/**
 * @brief ImuProcess类析构函数
 */
ImuProcess::~ImuProcess() {}

/**
 * @brief 重置IMU处理器状态
 *
 * 将IMU处理器重置为初始状态，用于系统重启或异常恢复：
 * - 清零平均加速度和角速度
 * - 重置初始化标志和计数器
 * - 清零上次扫描时间戳
 */
void ImuProcess::Reset()
{
  ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, 0.0);       // 重置平均加速度
  mean_gyr      = V3D(0, 0, 0);         // 重置平均角速度
  imu_need_init_    = true;             // 标记需要重新初始化
  init_iter_num     = 1;                // 重置初始化迭代计数
  after_imu_init_   = false;            // 重置初始化完成标志

  time_last_scan = 0.0;                 // 重置上次扫描时间戳
}

/**
 * @brief 计算重力对齐旋转矩阵
 * @param tmp_gravity 从IMU测量数据估计的重力向量(输入)
 * @param rot 输出的旋转矩阵，将测量重力对齐到标准重力方向(输出)
 *
 * 功能说明：
 * 1. 计算从测量重力向量到标准重力向量的旋转矩阵
 * 2. 使用Rodrigues旋转公式，通过叉乘和点乘计算旋转轴和角度
 * 3. 处理特殊情况：重力向量平行(正向或反向)时的旋转
 *
 * 算法步骤：
 * - 构造重力向量的反对称矩阵(叉乘矩阵)
 * - 计算旋转轴的模(用于判断是否平行)和旋转角度的余弦值
 * - 如果向量平行，返回单位矩阵或负单位矩阵
 * - 否则通过旋转轴和角度计算旋转矩阵(使用李代数指数映射)
 */
void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;

  // 构造标准重力向量的反对称矩阵(叉乘矩阵)，用于计算 gravity × tmp_gravity
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1),
              -gravity_(2), 0.0, gravity_(0),
              gravity_(1), -gravity_(0), 0.0;

  // 计算旋转轴的归一化模长，用于判断两个向量是否平行
  // 如果 gravity × tmp_gravity ≈ 0，说明两向量平行
  double align_norm = (hat_grav * tmp_gravity).norm() / gravity_.norm() / tmp_gravity.norm();

  // 计算两个重力向量夹角的余弦值
  // align_cos = (gravity · tmp_gravity) / (|gravity| * |tmp_gravity|)
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();

  // 判断两个重力向量是否平行(叉乘结果接近零)
  if (align_norm < 1e-6)
  {
    // 如果余弦值为正，说明两向量同向，旋转矩阵为单位矩阵
    if (align_cos > 1e-6)
    {
      rot = Eye3d;
    }
    // 如果余弦值为负，说明两向量反向，旋转矩阵为负单位矩阵(旋转180度)
    else
    {
      rot = -Eye3d;
    }
  }
  // 两向量不平行，需要计算一般情况下的旋转矩阵
  else
  {
    // 计算旋转轴和旋转角度
    // 旋转轴 = gravity × tmp_gravity (归一化后)
    // 旋转角度 = arccos(align_cos)
    // align_angle 是旋转向量(轴角表示)，其方向为旋转轴，模长为旋转角度
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);

    // 将旋转向量通过指数映射转换为旋转矩阵
    // Exp函数实现了 SO(3) 李代数到李群的指数映射
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}

/**
 * @brief IMU初始化函数
 * @param meas 测量数据组，包含IMU和LiDAR数据
 * @param N 初始化样本计数(输入输出参数)
 *
 * 功能说明：
 * 1. 在静止状态下收集IMU数据，计算加速度计和陀螺仪的均值
 * 2. 加速度均值用于估计重力方向
 * 3. 陀螺仪均值用于估计零偏(bias)
 * 4. 使用增量平均算法避免数值溢出
 *
 * 初始化过程：
 * - 第一帧：重置状态并使用第一个IMU数据初始化均值
 * - 后续帧：使用增量平均公式更新均值
 *   mean_new = mean_old + (cur_value - mean_old) / N
 */
void ImuProcess::IMU_init(const MeasureGroup &meas, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;  // 当前帧的加速度和角速度

  // 处理第一帧数据：重置状态并初始化均值
  if (b_first_frame_)
  {
    Reset();                          // 重置所有状态
    N = 1;                            // 初始化样本计数
    b_first_frame_ = false;           // 清除首帧标志
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;  // 使用第一个IMU数据初始化加速度均值
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;  // 使用第一个IMU数据初始化角速度均值
  }

  // 遍历当前测量组中的所有IMU数据
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;   // 提取当前加速度
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;   // 提取当前角速度

    // 使用增量平均公式更新均值，避免累加导致的数值溢出
    // 公式: mean_new = mean_old + (current - mean_old) / count
    mean_acc      += (cur_acc - mean_acc) / N;    // 更新加速度均值
    mean_gyr      += (cur_gyr - mean_gyr) / N;    // 更新角速度均值

    N ++;  // 递增样本计数
  }
}

/**
 * @brief 处理IMU和LiDAR数据的主函数
 * @param meas 测量数据组，包含同步的IMU和LiDAR数据
 * @param cur_pcl_un_ 输出的去畸变点云
 *
 * 功能说明：
 * 1. 如果启用IMU，执行初始化流程直到收集足够样本
 * 2. 初始化完成后，直接传递点云数据(本简化版本未实现完整的去畸变)
 * 3. 如果未启用IMU，直接传递原始点云
 *
 * 处理流程：
 * - IMU启用且需要初始化：调用IMU_init收集数据，达到MAX_INI_COUNT后完成初始化
 * - IMU启用且已初始化：传递点云数据(完整版本会进行运动补偿)
 * - IMU未启用：直接传递原始点云
 */
void ImuProcess::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr cur_pcl_un_)
{
  // 检查是否启用IMU处理
  if (imu_en)
  {
    // 如果IMU数据为空，直接返回
    if(meas.imu.empty())  return;

    // IMU初始化阶段
    if (imu_need_init_)
    {

      {
        /// The very first lidar frame
        // 执行IMU初始化，累积静止状态下的IMU数据
        IMU_init(meas, init_iter_num);

        imu_need_init_ = true;  // 保持初始化标志，直到样本数足够

        // 检查是否收集了足够的初始化样本
        if (init_iter_num > MAX_INI_COUNT)
        {
          ROS_INFO("IMU Initializing: %.1f %%", 100.0);
          imu_need_init_ = false;           // 清除初始化标志
          *cur_pcl_un_ = *(meas.lidar);     // 输出当前帧点云
        }
        // *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    // 初始化完成后的首次处理
    if (!after_imu_init_) after_imu_init_ = true;

    // 简化版本：直接传递点云数据
    // 完整版本会在此处进行IMU预积分和点云去畸变
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
  // IMU未启用的情况
  else
  {
    // 直接传递原始点云，不进行任何处理
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}