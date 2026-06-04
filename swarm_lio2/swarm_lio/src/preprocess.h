#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "common_lib.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

// 特征点类型枚举：普通点、可能平面、真实平面、边缘跳跃、边缘平面、线状特征、零点
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
// 周围方向枚举：前一个点、后一个点
enum Surround{Prev, Next};
// 边缘跳跃类型枚举：正常、零度跳跃、180度跳跃、无穷远跳跃、盲区跳跃
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

// 按时间戳排序点云的比较函数
const bool time_list_cut_frame(PointType &x, PointType &y);

// 点云原始特征信息结构体
struct orgtype
{
  double range;          // 点到激光雷达的距离
  double dista;          // 当前点与下一个点之间的距离
  double angle[2];       // 当前点与前后两个点的夹角
  double intersect;      // 前后两个方向向量的夹角余弦值
  E_jump edj[2];         // 前后两个方向的边缘跳跃类型
  Feature ftype;         // 点的特征类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};
// Velodyne激光雷达点云数据结构定义
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;        // 添加点云的x, y, z, padding字段
        float intensity;        // 反射强度
        float time;             // 时间戳（相对于帧起始时间）
        uint16_t ring;          // 激光束编号（第几线）
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (float, time, time)
        (std::uint16_t, ring, ring)
)

// Ouster激光雷达点云数据结构定义
namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;           // 添加点云的x, y, z, padding字段
      float intensity;           // 反射强度
      uint32_t t;                // 时间戳（纳秒）
      uint16_t reflectivity;     // 反射率
      uint8_t  ring;             // 激光束编号（第几线）
      uint16_t ambient;          // 环境光强度
      uint32_t range;            // 距离测量值
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// Pandar激光雷达点云数据结构定义
namespace pandar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;           // 添加点云的x, y, z, padding字段
        float intensity;           // 反射强度
        double timestamp;          // 时间戳（秒）
        uint16_t  ring;            // 激光束编号（第几线）
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (double, timestamp, timestamp)
                                          (std::uint16_t, ring, ring)
)

// 点云预处理类：负责不同类型激光雷达数据的预处理、特征提取和点云分帧
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();

  // 处理Livox激光雷达数据，提取特征点
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // 处理Livox激光雷达数据并进行分帧切割
  void process_cut_frame_livox(const livox_ros_driver::CustomMsg::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);
  // 处理PointCloud2格式的激光雷达数据
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // 处理PointCloud2格式数据并进行分帧切割
  void process_cut_frame_pcl2(const sensor_msgs::PointCloud2::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);
  // 累积处理Livox激光雷达帧数据
  void process_accumulate_frame_livox(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out, const double &first_lidar_head_time);
  // 设置预处理参数：特征提取使能、激光雷达类型、盲区距离、点滤波数量
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;  // 完整点云、角点云、平面点云
  PointCloudXYZI pl_buff[128];               // 每条激光线的点云缓存（最多支持128线激光雷达）
  vector<orgtype> typess[128];               // 每条激光线的点特征类型（最多支持128线激光雷达）
  int lidar_type, point_filter_num, N_SCANS; // 激光雷达类型、点滤波数量、扫描线数
  double blind, DET_RANGE;                   // 盲区距离、最大检测距离
  bool feature_enabled, given_offset_time;   // 特征提取使能标志、是否给定偏移时间标志
  ros::Publisher pub_full, pub_surf, pub_corn; // ROS发布器：完整点云、平面点云、角点云
  double original_freq;                      // 激光雷达原始频率
    

  private:
  // Livox Avia激光雷达数据处理函数
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  // Ouster激光雷达数据处理函数
  void oust_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Velodyne激光雷达数据处理函数
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // Intel RealSense L515深度相机数据处理函数
  void l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // 仿真数据处理函数
  void sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // 提取点云特征（平面点、边缘点等）
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  // 判断点群是否构成平面，返回平面类型
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  // 判断是否为小平面
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  // 判断是否为边缘跳跃点
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;                                  // 平面检测的点群大小
  double disA, disB, inf_bound;                    // 距离参数A、B，无穷远边界
  double limit_maxmid, limit_midmin, limit_maxmin; // 平面判断的距离比率限制
  double p2l_ratio;                                // 点到线的距离比率阈值
  double jump_up_limit, jump_down_limit;           // 向上/向下跳跃角度限制（余弦值）
  double cos160;                                   // 160度角的余弦值
  double edgea, edgeb;                             // 边缘检测参数
  double smallp_intersect, smallp_ratio;           // 小平面交角和比率阈值
  double vx, vy, vz;                               // 临时向量坐标

};
#endif