/**
 * @file voxelslam.hpp
 * @brief VoxelSLAM主头文件 - 基于体素地图的SLAM系统实现
 * @details 该文件定义了VoxelSLAM系统的核心功能，包括：
 *          - IMU和点云数据的同步处理
 *          - 点云特征提取和变换
 *          - 体素地图管理
 *          - 回环检测和优化
 *          - GTSAM因子图优化
 */

#pragma once

// 自定义工具和核心模块
#include "tools.hpp"           // 基础工具函数
#include "ekf_imu.hpp"          // IMU扩展卡尔曼滤波器
#include "voxel_map.hpp"        // 体素地图实现
#include "feature_point.hpp"    // 点云特征提取
#include "loop_refine.hpp"      // 回环检测和优化
#include "BTC.h"                // 偏置补偿模块

// 标准库
#include <mutex>

// Eigen库 - 线性代数运算
#include <Eigen/Eigenvalues>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>

// ROS相关
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

// PCL点云库
#include <pcl/kdtree/kdtree_flann.h>

// GTSAM图优化库
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// 内存管理
#include <malloc.h>

using namespace std;

// ============== ROS发布器和订阅器 ==============
ros::Publisher pub_scan;        // 发布当前扫描点云
ros::Publisher pub_cmap;        // 发布当前地图
ros::Publisher pub_init;        // 发布初始化相关信息
ros::Publisher pub_pmap;        // 发布全局地图
ros::Publisher pub_test;        // 发布测试数据
ros::Publisher pub_prev_path;   // 发布之前的路径
ros::Publisher pub_curr_path;   // 发布当前路径
ros::Subscriber sub_imu;        // 订阅IMU数据
ros::Subscriber sub_pcl;        // 订阅点云数据

/**
 * @brief 发布点云数据到ROS话题
 * @tparam T 点云类型（支持PCL点云类型）
 * @param pl 输入点云
 * @param pub ROS发布器
 * @details 将PCL点云转换为ROS消息格式并发布，坐标系设置为"camera_init"
 */
template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
  pl.height = 1; pl.width = pl.size();  // 设置点云为无序点云格式
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);  // PCL点云转ROS消息
  output.header.frame_id = "camera_init";  // 设置坐标系
  output.header.stamp = ros::Time::now();   // 设置时间戳
  pub.publish(output);  // 发布消息
}

// ============== 全局数据缓冲区和状态变量 ==============
mutex mBuf;                                        // 缓冲区互斥锁，保护多线程访问
Features feat;                                     // 特征提取器对象
deque<sensor_msgs::Imu::Ptr> imu_buf;             // IMU数据缓冲队列
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;  // 点云数据缓冲队列
deque<double> time_buf;                           // 点云时间戳缓冲队列

double imu_last_time = -1;   // 最后一次IMU时间戳
int point_notime = 0;        // 点云是否无时间戳标志
double last_pcl_time = -1;   // 上一次点云时间戳

/**
 * @brief IMU数据回调函数
 * @param msg_in IMU消息指针
 * @details 接收IMU数据并存入缓冲区，线程安全处理
 *          - 首次调用时打印初始时间戳
 *          - 使用互斥锁保护缓冲区访问
 */
void imu_handler(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  static int flag = 1;
  if(flag)
  {
    flag = 0;
    printf("Time0: %lf\n", msg_in->header.stamp.toSec());  // 打印第一帧IMU时间
  }

  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  // Hilti 2022 exp03数据集特殊处理（已注释）
  // 用于修正特定时间段的加速度计数据
  // double t0 = 1646320760 + 255.5;
  // double t1 = 1646320760 + 256.2;
  // double tc = msg->header.stamp.toSec();
  // if(tc > t0 && tc < t1)
  //   msg->linear_acceleration.z = -9.7;

  mBuf.lock();
  imu_last_time = msg->header.stamp.toSec();  // 更新最后IMU时间
  imu_buf.push_back(msg);                     // 加入缓冲队列
  mBuf.unlock();
}

/**
 * @brief 点云数据回调函数（模板函数，支持多种点云消息类型）
 * @tparam T 点云消息类型
 * @param msg 点云消息
 * @details 处理流程：
 *          1. 通过特征提取器处理原始点云
 *          2. 如果点云为空，添加两个虚拟点以保持数据流
 *          3. 按curvature字段排序点云
 *          4. 移除curvature > 0.11的无效点
 *          5. 将处理后的点云存入缓冲区
 */
template<class T>
void pcl_handler(T &msg)
{
  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
  double t0 = feat.process(msg, *pl_ptr);  // 提取特征点并返回时间戳

  // 如果点云为空，添加两个占位点
  if(pl_ptr->empty())
  {
    PointType ap;
    ap.x = 0; ap.y = 0; ap.z = 0;
    ap.intensity = 0; ap.curvature = 0;
    pl_ptr->push_back(ap);      // 第一个占位点
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);      // 第二个占位点，curvature设为扫描时长
  }

  // 按curvature排序（curvature存储点的相对时间戳）
  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType &x, PointType &y)
  {
    return x.curvature < y.curvature;
  });

  // 移除超出扫描时间范围的点（curvature > 0.11s）
  while(pl_ptr->back().curvature > 0.11)
    pl_ptr->points.pop_back();

  // 线程安全地将点云和时间戳存入缓冲区
  mBuf.lock();
  time_buf.push_back(t0);
  pcl_buf.push_back(pl_ptr);
  mBuf.unlock();
}

/**
 * @brief 同步点云和IMU数据包
 * @param pl_ptr 输出的点云指针
 * @param imus 输出的IMU数据队列
 * @param p_imu IMU-EKF对象，用于存储时间信息
 * @return 是否成功同步（IMU数量>4返回true）
 * @details 同步策略：
 *          1. 从缓冲区获取一帧点云，设置起始和结束时间
 *          2. 等待IMU数据覆盖整个点云扫描周期
 *          3. 提取该时间段内的所有IMU数据
 *          4. 处理无时间戳点云的特殊情况
 */
bool sync_packages(pcl::PointCloud<PointType>::Ptr &pl_ptr, deque<sensor_msgs::Imu::Ptr> &imus, IMUEKF &p_imu)
{
  static bool pl_ready = false;  // 点云是否准备好的标志

  // 第一阶段：获取点云数据并设置时间范围
  if(!pl_ready)
  {
    if(pcl_buf.empty()) return false;  // 点云缓冲区为空，等待

    // 从缓冲区取出一帧点云
    mBuf.lock();
    pl_ptr = pcl_buf.front();
    p_imu.pcl_beg_time = time_buf.front();  // 点云起始时间
    pcl_buf.pop_front(); time_buf.pop_front();
    mBuf.unlock();

    // 计算点云结束时间（起始时间 + 扫描时长）
    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    // 处理无时间戳点云的情况
    if(point_notime)
    {
      if(last_pcl_time < 0)  // 首次处理
      {
        last_pcl_time = p_imu.pcl_beg_time;
        return false;
      }

      // 使用上一帧的结束时间作为本帧的起始时间
      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;  // 点云已准备好
  }

  // 第二阶段：等待IMU数据覆盖点云时间范围
  if(!pl_ready || imu_last_time <= p_imu.pcl_end_time) return false;

  // 第三阶段：提取时间范围内的IMU数据
  mBuf.lock();
  double imu_time = imu_buf.front()->header.stamp.toSec();
  while((!imu_buf.empty()) && (imu_time < p_imu.pcl_end_time))
  {
    imu_time = imu_buf.front()->header.stamp.toSec();
    if(imu_time > p_imu.pcl_end_time) break;  // 超出点云时间范围
    imus.push_back(imu_buf.front());
    imu_buf.pop_front();
  }
  mBuf.unlock();

  // 检查IMU缓冲区是否意外为空
  if(imu_buf.empty())
  {
    printf("imu buf empty\n"); exit(0);
  }

  pl_ready = false;  // 重置标志，准备处理下一帧

  // 返回是否有足够的IMU数据（至少5个）
  if(imus.size() > 4)
    return true;
  else
    return false;
}

// ============== 点云不确定性建模 ==============
double dept_err, beam_err;  // 深度误差和角度误差参数

/**
 * @brief 计算激光点在body坐标系下的协方差矩阵
 * @param pb 点在body坐标系下的位置（输入输出参数）
 * @param range_inc 距离测量增量误差
 * @param degree_inc 角度测量增量误差（度）
 * @param var 输出的3x3协方差矩阵
 * @details 误差模型：
 *          1. 距离误差：沿测量方向的误差
 *          2. 角度误差：垂直于测量方向的误差
 *          将两种误差组合成完整的协方差矩阵
 */
void calcBodyVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var)
{
  // 避免除零错误
  if (pb[2] == 0)
    pb[2] = 0.0001;

  // 计算测量距离和距离方差
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;

  // 角度方差矩阵（2x2，两个正交方向）
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);

  // 测量方向单位向量
  Eigen::Vector3d direction(pb);
  direction.normalize();

  // 构造方向向量的反对称矩阵（用于叉乘）
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1),
                   direction(2), 0, -direction(0),
                   -direction(1), direction(0), 0;

  // 构造正交基向量（垂直于测量方向）
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();

  // 零空间矩阵N（3x2）：两个正交基向量
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0),
       base_vector1(1), base_vector2(1),
       base_vector1(2), base_vector2(2);

  // 角度误差在3D空间的投影矩阵
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;

  // 总协方差 = 距离方差 + 角度方差
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

/**
 * @brief 初始化点云的方差信息
 * @param ext 外参变换（从lidar到body坐标系）
 * @param pl_cur 当前点云
 * @param pptr 输出的点云方差向量指针
 * @param dept_err 深度误差参数
 * @param beam_err 光束角度误差参数
 * @details 为每个点计算：
 *          1. body坐标系下的协方差矩阵
 *          2. 应用外参变换到IMU坐标系
 *          3. 传播不确定性到新坐标系
 */
void var_init(IMUST &ext, pcl::PointCloud<PointType> &pl_cur, PVecPtr pptr, double dept_err, double beam_err)
{
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);

  for(int i=0; i<plsize; i++)
  {
    PointType &ap = pl_cur[i];
    pointVar &pv = pptr->at(i);

    // 提取点坐标
    pv.pnt << ap.x, ap.y, ap.z;

    // 计算body坐标系下的协方差
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);

    // 应用外参变换：从lidar坐标系到IMU坐标系
    pv.pnt = ext.R * pv.pnt + ext.p;

    // 传播协方差到IMU坐标系
    pv.var = ext.R * pv.var * ext.R.transpose();
  }
}

/**
 * @brief 更新点云方差并变换到世界坐标系
 * @param pptr 点云方差向量指针（包含IMU坐标系下的点）
 * @param x_curr 当前IMU状态（位姿和协方差）
 * @param pwld 输出的世界坐标系点云
 * @details 不确定性传播：
 *          1. 提取旋转和平移的协方差
 *          2. 将点从IMU坐标系变换到世界坐标系
 *          3. 传播位姿不确定性到点的协方差
 *          公式：var = R*var*R^T + [p]×*var_rot*[p]×^T + var_tsl
 */
void pvec_update(PVecPtr pptr, IMUST &x_curr, PLV(3) &pwld)
{
  // 提取位姿协方差的旋转和平移部分
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);  // 旋转协方差
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);  // 平移协方差

  for(pointVar &pv: *pptr)
  {
    // 点的反对称矩阵（用于旋转误差的传播）
    Eigen::Matrix3d phat = hat(pv.pnt);

    // 更新点的协方差：观测误差 + 旋转误差 + 平移误差
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() +  // 观测误差传播
             phat * rot_var * phat.transpose() +          // 旋转误差贡献
             tsl_var;                                      // 平移误差贡献

    // 变换点到世界坐标系
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

/**
 * @brief 从文件读取LiDAR状态信息
 * @param filename 状态文件路径（alidarstate.txt）
 * @param bl_tem 输出的扫描位姿向量
 * @details 文件格式（每行包含以空格分隔的数值）：
 *          - 基础信息（8个）：时间戳 tx ty tz qx qy qz qw
 *          - 扩展信息（12个）：vx vy vz bgx bgy bgz bax bay baz gx gy gz
 *          - 额外参数（6个）：v6[0-5]（如果有26个以上数值）
 */
void read_lidarstate(string filename, vector<ScanPose*> &bl_tem)
{
  ifstream file(filename);
  if(!file.is_open())
  {
    printf("Error: %s not found\n", filename.c_str());
    exit(0);
  }

  string lineStr, str;
  vector<double> nums;

  // 逐行读取文件
  while(getline(file, lineStr))
  {
    nums.clear();
    stringstream ss(lineStr);

    // 解析每行的数值（空格分隔）
    while(getline(ss, str, ' '))
      nums.push_back(stod(str));

    // 构造IMU状态
    IMUST xx;
    xx.t = nums[0];                                                          // 时间戳
    xx.p << nums[1], nums[2], nums[3];                                       // 位置 (x,y,z)
    xx.R = Eigen::Quaterniond(nums[7], nums[4], nums[5], nums[6]).matrix();  // 旋转（四元数 w,x,y,z）

    // 如果有完整的状态信息（>=20个数值）
    if(nums.size() >= 20)
    {
      xx.v << nums[8], nums[9], nums[10];      // 速度
      xx.bg << nums[11], nums[12], nums[13];   // 陀螺仪偏置
      xx.ba << nums[14], nums[15], nums[16];   // 加速度计偏置
      xx.g << nums[17], nums[18], nums[19];    // 重力向量
    }

    // 创建扫描位姿对象
    ScanPose* blp = new ScanPose(xx, nullptr);
    bl_tem.push_back(blp);

    // 如果有额外参数（>=26个数值）
    if(nums.size() >= 26)
      for(int i=0; i<6; i++)
        blp->v6[i] = nums[i + 20];  // 读取6个额外参数
  }
}

/**
 * @brief 获取当前进程的内存使用量（GB）
 * @return 内存使用量（单位：GB）
 * @details 通过读取 /proc/self/status 文件中的 VmRSS 字段
 *          VmRSS (Resident Set Size)：进程实际占用的物理内存
 *          原始单位为KB，转换为GB（除以1048576 = 1024*1024）
 */
double get_memory()
{
  ifstream infile("/proc/self/status");
  double mem = -1;
  string lineStr, str;

  // 逐行查找VmRSS字段
  while(getline(infile, lineStr))
  {
    stringstream ss(lineStr);
    bool is_find = false;

    while(ss >> str)
    {
      if(str == "VmRSS:")  // 找到内存使用标记
      {
        is_find = true;
        continue;
      }

      if(is_find)  // 读取内存数值
        mem = stod(str);
      break;
    }

    if(is_find) break;  // 已找到，退出循环
  }

  return mem / (1048576);  // KB转换为GB
}

/**
 * @brief ICP回环检测验证函数
 * @param pl_src 源点云（回环前的点云）
 * @param pl_tar 目标点云（回环候选点云）
 * @param pub_src 源点云发布器
 * @param pub_tar 目标点云发布器
 * @param loop_transform 回环变换（平移，旋转）
 * @param xx 当前IMU状态
 * @details 功能：
 *          1. 应用回环变换到源点云
 *          2. 应用当前位姿变换到两个点云
 *          3. 发布变换后的点云用于可视化验证
 *          这允许在RViz中检查回环检测的准确性
 */
void icp_check(pcl::PointCloud<PointType> &pl_src, pcl::PointCloud<PointType> &pl_tar, ros::Publisher &pub_src, ros::Publisher &pub_tar, pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform, IMUST &xx)
{
  pcl::PointCloud<PointType> pl1, pl2;

  // 处理源点云：先应用回环变换，再应用当前位姿
  for(PointType ap: pl_src.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = loop_transform.second * v + loop_transform.first;  // 回环变换（R*p + t）
    v = xx.R * v + xx.p;                                    // 当前位姿变换
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl1.push_back(ap);
  }

  // 处理目标点云：只应用当前位姿
  for(PointType ap: pl_tar.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = xx.R * v + xx.p;  // 当前位姿变换
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl2.push_back(ap);
  }

  // 发布点云用于可视化
  pub_pl_func(pl1, pub_src);
  pub_pl_func(pl2, pub_tar);
}

