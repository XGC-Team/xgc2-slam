// 特征点头文件保护宏
#ifndef FEATURE_POINT_HPP
#define FEATURE_POINT_HPP

// ROS相关头文件
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

// 定义点类型为PCL的PointXYZINormal（包含xyz坐标、强度和法向量）
typedef pcl::PointXYZINormal PointType;
using namespace std;

// 激光雷达类型枚举：支持LIVOX、VELODYNE、OUSTER、HESAI、ROBOSENSE、TARTANAIR
enum LID_TYPE{LIVOX, VELODYNE, OUSTER, HESAI, ROBOSENSE, TARTANAIR};

// Velodyne激光雷达点云数据结构命名空间
namespace velodyne_ros {
  // Velodyne点结构体，采用16字节对齐以优化性能
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;                  // 添加x, y, z, w坐标字段
      // float intensity;               // 强度（已注释）
      float time;                       // 点的时间戳（相对于帧起始时间）
      std::uint16_t ring;               // 激光束环号（垂直角度索引）
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 确保动态分配的内存对齐
  };
}  // namespace velodyne_ros

// 向PCL注册Velodyne自定义点类型，定义各字段的映射关系
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    // (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

// Ouster激光雷达点云数据结构命名空间
namespace ouster_ros
{
  // Ouster点结构体，采用16字节对齐
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                    // 添加x, y, z, w坐标字段
    float intensity;                    // 点云强度值
    uint32_t t;                         // 时间戳（纳秒级）
    uint16_t reflectivity;              // 反射率
    uint8_t  ring;                      // 激光束环号
    // uint16_t ambient;                // 环境光强度（已注释）
    uint32_t range;                     // 距离测量值
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // 确保动态分配的内存对齐
  };
}

// 向PCL注册Ouster自定义点类型，定义各字段的映射关系
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  // use std::uint32_t to avoid conflicting with pcl::uint32_t
  (std::uint32_t, t, t)                 // 使用std::uint32_t避免与pcl::uint32_t冲突
  // (std::uint16_t, reflectivity, reflectivity)
  // (std::uint8_t, ring, ring)
  // (std::uint16_t, ambient, ambient)
  // (std::uint32_t, range, range)
)

// 禾赛XT32激光雷达点云数据结构命名空间
namespace xt32_ros {
  // XT32点结构体，采用16字节对齐
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;                  // 添加x, y, z, w坐标字段
      float intensity;                  // 点云强度值
      double timestamp;                 // 时间戳（双精度浮点）
      uint16_t ring;                    // 激光束环号
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 确保动态分配的内存对齐
  };
}  // namespace xt32_ros

// 向PCL注册XT32自定义点类型，定义各字段的映射关系
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)


// RoboSense激光雷达点云数据结构命名空间
namespace rslidar_ros {
  // RoboSense点结构体，采用16字节对齐
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;                  // 添加x, y, z, w坐标字段
      float intensity;                  // 点云强度值
      std::uint16_t ring;               // 激光束环号
      double timestamp;                 // 时间戳（双精度浮点）
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 确保动态分配的内存对齐
  };
}

// 向PCL注册RoboSense自定义点类型，定义各字段的映射关系
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

/**
 * @brief 特征点处理类
 *
 * 该类负责处理多种激光雷达的点云数据，支持LIVOX、VELODYNE、OUSTER、
 * HESAI、ROBOSENSE和TARTANAIR等类型的激光雷达。主要功能包括点云数据
 * 的解析、滤波和时间戳处理。
 */
class Features
{
public:
  int lidar_type;                       // 激光雷达类型
  int point_filter_num;                 // 点云降采样参数（每隔N个点取一个）
  double blind = 1;                     // 盲区阈值（距离的平方），过滤掉距离过近的点
  double omega_l = 3610;                // 激光雷达旋转角速度（度/秒），用于计算时间戳

  /**
   * @brief 处理Livox激光雷达的自定义消息格式
   * @param msg Livox自定义消息的智能指针
   * @param pl_full 输出的完整点云（经过滤波和处理）
   * @return 点云帧的时间戳（秒）
   */
  double process(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    livox_handler(msg, pl_full);
    return msg->header.stamp.toSec();
  }

  /**
   * @brief 处理标准PointCloud2消息格式（支持多种激光雷达类型）
   * @param msg PointCloud2消息的智能指针
   * @param pl_full 输出的完整点云（经过滤波和处理）
   * @return 点云帧的时间戳（秒）
   *
   * 根据lidar_type成员变量，自动调用对应的处理函数：
   * - VELODYNE: Velodyne系列激光雷达
   * - OUSTER: Ouster系列激光雷达
   * - HESAI: 禾赛系列激光雷达
   * - ROBOSENSE: RoboSense系列激光雷达
   * - TARTANAIR: TartanAir数据集
   */
  double process(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    double t0 = msg->header.stamp.toSec();
    switch(lidar_type)
    {
    case VELODYNE:
      velodyne_handler(msg, pl_full);
      break;

    case OUSTER:
      ouster_handler(msg, pl_full);
      break;

    case HESAI:
      hesai_handler(msg, pl_full);
      break;

    case ROBOSENSE:
      t0 = robosense_handler(msg, pl_full);
      break;

    case TARTANAIR:
      tartanair_handler(msg, pl_full);
      break;

    default:
      printf("Lidar Type Error\n");
      exit(0);
    }

    return t0;
  }

  /**
   * @brief Livox激光雷达数据处理函数
   * @param msg Livox自定义消息的智能指针
   * @param pl_full 输出的完整点云
   *
   * 功能：
   * 1. 提取点的xyz坐标和反射率
   * 2. 将偏移时间转换为秒（curvature字段存储时间戳）
   * 3. 进行降采样（每point_filter_num个点取一个）
   * 4. 过滤盲区内的点（距离小于blind阈值）
   */
  void livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    int plsize = msg->point_num;
    pl_full.reserve(plsize);

    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = msg->points[i].x;
      ap.y = msg->points[i].y;
      ap.z = msg->points[i].z;
      ap.intensity = msg->points[i].reflectivity;
      // ap.curvature = msg->points[i].offset_time / float(1000000); // ms
      ap.curvature = msg->points[i].offset_time / float(1000000000); // s（将纳秒转换为秒）

      // 降采样：每point_filter_num个点取一个
      if(i % point_filter_num == 0)
      {
        // 过滤盲区：距离平方大于盲区阈值
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.push_back(ap);
        }
      }

    }

  }

  /**
   * @brief Velodyne激光雷达数据处理函数
   * @param msg PointCloud2消息的智能指针
   * @param pl_full 输出的完整点云
   *
   * 功能：
   * 1. 支持两种时间戳模式：
   *    - 直接时间戳模式（time在0.01-0.12秒范围内）
   *    - 角度计算模式（通过旋转角度计算相对时间）
   * 2. 进行降采样和盲区过滤
   * 3. 处理激光束的顺时针旋转，计算每个点的相对时间戳
   */
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.size();
    if(plsize == 0) return;

    // 判断是否为直接时间戳模式（时间戳在合理范围内）
    if(pl_orig.back().time > 0.01 && pl_orig.back().time < 0.12)
    {
      // for(velodyne_ros::Point &iter : pl_orig.points)
      for(int i=0; i<plsize; i++)
      {
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;

        // ap.intensity = iter.intensity;
        // ap.curvature = iter.time * 1e-3; // ms
        // ap.curvature = iter.time * 1e-6;
        ap.curvature = iter.time;         // 直接使用时间戳

        if(i % point_filter_num == 0)
        {
          if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
          {
            pl_full.push_back(ap);
          }
        }
      }

    }
    else
    {
      // 角度计算模式：激光雷达顺时针旋转，通过角度变化计算时间戳
      bool first_point = true;
      double yaw_first = 0;           // 第一个点的偏航角（度）
      double yaw_last = 0;            // 上一个点的偏航角（度）
      double yaw_bias = 0;            // 角度偏置，用于处理跨越360度的情况
      int cool = 0;                   // 冷却计数器，防止频繁触发角度跳变处理
      float max_ang = 0;
      for(int i=0; i<plsize; i++)
      {
        cool--;
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;

        // 过滤x坐标过小的点（可能是噪声）
        if(fabs(ap.x) < 0.1)
          continue;

        // 计算点的偏航角（度），57.2957 = 180/π（弧度转角度）
        double yaw_angle = atan2(ap.y, ap.x) * 57.2957 - yaw_bias;
        if(first_point)
        {
          yaw_first = yaw_angle;
          yaw_last  = yaw_angle;
          first_point = false;
        }

        // 过滤盲区内的点
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z < blind)
          continue;

        // 处理跨越360度的情况（从359度跳到0度）
        if(yaw_angle - yaw_last > 180 && cool <= 0)
        {
          yaw_bias += 360; yaw_angle-= 360; cool = 1000;
        }

        // 处理角度差异过大的情况
        if(fabs(yaw_angle - yaw_last) > 180)
        {
          yaw_angle += 360;
        }

        // 根据角度变化计算相对时间：角度差 / 角速度 = 时间
        ap.curvature = (yaw_first - yaw_angle) / omega_l;
        yaw_last = yaw_angle;

        if(ap.curvature > max_ang)
          max_ang = ap.curvature;

        // 只保留时间戳在合理范围内的点（0到0.1秒）
        if(ap.curvature >= 0 && ap.curvature < 0.1)
          if(i % point_filter_num == 0)
            pl_full.push_back(ap);
      }

      // printf("maxang: %f\n", max_ang);
    }

  }

  /**
   * @brief Ouster激光雷达数据处理函数
   * @param msg PointCloud2消息的智能指针
   * @param pl_full 输出的完整点云
   *
   * 功能：
   * 1. 提取点的xyz坐标和强度
   * 2. 将时间戳从纳秒转换为秒（curvature字段存储时间戳）
   * 3. 进行降采样和盲区过滤
   */
  void ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig[i].intensity;
      // ap.curvature = pl_orig[i].t / float(1e6); // ms
      ap.curvature = pl_orig[i].t / float(1e9); // s（将纳秒转换为秒）

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }

    }

  }

  /**
   * @brief 禾赛激光雷达数据处理函数
   * @param msg PointCloud2消息的智能指针
   * @param pl_full 输出的完整点云
   *
   * 功能：
   * 1. 提取点的xyz坐标和强度
   * 2. 初始化法向量为0
   * 3. 计算相对于第一个点的时间戳差值
   * 4. 进行降采样和盲区过滤
   */
  void hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<xt32_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double time_head = pl_orig.points[0].timestamp;  // 第一个点的时间戳作为基准
    for(int i=0; i<plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;                         // 初始化法向量x分量
      added_pt.normal_y = 0;                         // 初始化法向量y分量
      added_pt.normal_z = 0;                         // 初始化法向量z分量
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head);  // 相对时间戳

      if (i % point_filter_num == 0)
      {
        if (added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > blind)
        {
          pl_full.points.push_back(added_pt);
        }
      }


    }

  }

  /**
   * @brief RoboSense激光雷达数据处理函数
   * @param msg PointCloud2消息的智能指针
   * @param pl_full 输出的完整点云
   * @return 第一个点的时间戳（秒）
   *
   * 功能：
   * 1. 提取点的xyz坐标和强度
   * 2. 计算相对于第一个点的时间戳差值
   * 3. 进行降采样和盲区过滤
   * 4. 返回基准时间戳
   */
  double robosense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<rslidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double t0 = pl_orig[0].timestamp;                // 第一个点的时间戳作为基准
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig.points[i].intensity;
      // ap.curvature = (pl_orig[i].timestamp - t0) * float(1e3); //
      ap.curvature = (pl_orig[i].timestamp - t0);    // 相对时间戳

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }

    }

    return t0;
  }

  /**
   * @brief TartanAir数据集处理函数
   * @param msg PointCloud2消息的智能指针
   * @param pl_full 输出的完整点云
   *
   * 功能：
   * 1. 处理TartanAir数据集的点云（仅包含xyz坐标）
   * 2. 设置时间戳为0（curvature字段）
   * 3. 不进行降采样和盲区过滤
   */
  void tartanair_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<pcl::PointXYZ> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    pl_full.reserve(pl_orig.size());

    PointType pp; pp.curvature = 0;              // 时间戳设置为0
    for(pcl::PointXYZ &ap: pl_orig.points)
    {
      pp.x = ap.x;
      pp.y = ap.y;
      pp.z = ap.z;
      pl_full.push_back(pp);
    }

    return;
  }

};

#endif
