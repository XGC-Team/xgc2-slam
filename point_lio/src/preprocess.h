/**
 * @file preprocess.h
 * @brief 激光雷达点云预处理模块
 *
 * 该文件定义了点云预处理类，用于处理不同类型激光雷达的原始数据，包括：
 * - 支持多种雷达类型：Livox AVIA、Velodyne VLP-16、Ouster OS1-64、HESAI XT32
 * - 点云特征提取：平面点、边缘点的检测和分类
 * - 点云滤波和去噪处理
 * - 时间戳同步和帧切分
 * - 盲区处理和有效点过滤
 */

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

// 判断浮点数是否有效（绝对值大于1e8视为无效）
#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

// 点类型定义：使用PCL的PointXYZINormal（包含xyz坐标、强度和法向量）
typedef pcl::PointXYZINormal PointType;
// 点云类型定义
typedef pcl::PointCloud<PointType> PointCloudXYZI;

/**
 * @brief 激光雷达类型枚举
 * AVIA: Livox AVIA 固态雷达
 * VELO16: Velodyne VLP-16 机械旋转雷达
 * OUST64: Ouster OS1-64 机械旋转雷达
 * HESAIxt32: HESAI XT32 机械旋转雷达
 */
enum LID_TYPE{AVIA = 1, VELO16, OUST64, HESAIxt32}; //{1, 2, 3, 4}

/**
 * @brief 时间单位枚举
 * SEC: 秒
 * MS: 毫秒
 * US: 微秒
 * NS: 纳秒
 */
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};

/**
 * @brief 点云特征类型枚举
 * Nor: 正常点（未分类）
 * Poss_Plane: 可能的平面点
 * Real_Plane: 确定的平面点
 * Edge_Jump: 边缘跳变点
 * Edge_Plane: 边缘平面点
 * Wire: 线状特征点
 * ZeroPoint: 零点（无效点）
 */
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};

/**
 * @brief 相邻点方向枚举
 * Prev: 前一个点
 * Next: 后一个点
 */
enum Surround{Prev, Next};

/**
 * @brief 边缘跳变类型枚举
 * Nr_nor: 正常边缘
 * Nr_zero: 零值边缘
 * Nr_180: 180度跳变
 * Nr_inf: 无穷远跳变
 * Nr_blind: 盲区边缘
 */
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

// 根据时间戳判断是否需要切分帧的比较函数
const bool time_list_cut_frame(PointType &x, PointType &y);

/**
 * @brief 原始点云数据的组织结构
 *
 * 该结构体存储每个点的几何和特征信息，用于特征提取和分类
 */
struct orgtype
{
  double range;          // 点到雷达中心的距离
  double dista;          // 点到前一个点的距离
  double angle[2];       // 与前后两个点形成的夹角
  double intersect;      // 交点参数，用于平面判断
  E_jump edj[2];         // 前后两个方向的边缘跳变类型
  Feature ftype;         // 该点的特征类型

  // 构造函数：初始化所有成员变量为默认值
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

/**
 * @brief Velodyne雷达点云数据结构
 *
 * 定义了Velodyne系列雷达的点云格式，包含位置、强度、时间戳和线束信息
 */
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;         // 添加xyz坐标和对齐填充
      float intensity;         // 激光反射强度
      float time;              // 点的时间戳（相对于帧起始时间）
      uint16_t ring;           // 激光线束编号（0-15 for VLP-16）
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

/**
 * @brief HESAI雷达点云数据结构
 *
 * 定义了HESAI系列雷达的点云格式，使用double类型的时间戳以提高精度
 */
namespace hesai_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;         // 添加xyz坐标和对齐填充
      float intensity;         // 激光反射强度
      double timestamp;        // 点的时间戳（双精度）
      uint16_t ring;           // 激光线束编号
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace hesai_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (double, timestamp, timestamp)
    (std::uint16_t, ring, ring)
)

/**
 * @brief Ouster雷达点云数据结构
 *
 * 定义了Ouster系列雷达的点云格式，包含丰富的传感器信息
 */
namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;         // 添加xyz坐标和对齐填充
      float intensity;         // 激光反射强度
      uint32_t t;              // 点的时间戳（纳秒）
      uint16_t reflectivity;   // 反射率（原始值）
      uint8_t  ring;           // 激光线束编号（0-63 for OS1-64）
      uint16_t ambient;        // 环境光强度
      uint32_t range;          // 距离原始值（毫米）
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
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

/**
 * @brief 点云预处理类
 *
 * 该类负责处理不同类型激光雷达的原始数据，包括：
 * - 点云格式转换和标准化
 * - 特征提取（平面点和边缘点）
 * - 点云滤波和去噪
 * - 时间戳同步
 * - 盲区和有效距离过滤
 */
class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 构造函数
   */
  Preprocess();

  /**
   * @brief 析构函数
   */
  ~Preprocess();

  /**
   * @brief 处理Livox雷达数据并切分帧
   * @param msg Livox自定义消息指针
   * @param pcl_out 输出点云队列
   * @param time_lidar 输出时间戳队列
   * @param required_frame_num 需要的帧数
   * @param scan_count 当前扫描计数
   */
  void process_cut_frame_livox(const livox_ros_driver::CustomMsg::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);

  /**
   * @brief 处理PointCloud2格式数据并切分帧
   * @param msg PointCloud2消息指针
   * @param pcl_out 输出点云队列
   * @param time_lidar 输出时间戳队列
   * @param required_frame_num 需要的帧数
   * @param scan_count 当前扫描计数
   */
  void process_cut_frame_pcl2(const sensor_msgs::PointCloud2::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);

  /**
   * @brief 处理Livox雷达数据
   * @param msg Livox自定义消息指针
   * @param pcl_out 输出点云指针
   */
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  /**
   * @brief 处理PointCloud2格式数据
   * @param msg PointCloud2消息指针
   * @param pcl_out 输出点云指针
   */
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

  /**
   * @brief 设置预处理参数
   * @param feat_en 是否启用特征提取
   * @param lid_type 雷达类型
   * @param bld 盲区距离
   * @param pfilt_num 点滤波数量
   */
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;  // 完整点云、角点点云、平面点云
  PointCloudXYZI pl_buff[128];               // 按线束缓存的点云（最多支持128线雷达）
  vector<orgtype> typess[128];               // 每条线束的点特征信息（最多支持128线雷达）
  float time_unit_scale;                     // 时间单位缩放因子
  int lidar_type;                            // 雷达类型
  int point_filter_num;                      // 点滤波采样间隔
  int N_SCANS;                               // 雷达线束数量
  int SCAN_RATE;                             // 雷达扫描频率（Hz）
  int time_unit;                             // 时间单位
  double blind;                              // 盲区距离（米）
  double det_range;                          // 有效检测距离（米）
  bool given_offset_time;                    // 是否给定时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn; // ROS发布器：完整点云、平面点、角点
    

  private:
  /**
   * @brief 处理Livox AVIA雷达数据
   * @param msg Livox自定义消息指针
   */
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);

  /**
   * @brief 处理Ouster OS1-64雷达数据
   * @param msg PointCloud2消息指针
   */
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief 处理Velodyne雷达数据
   * @param msg PointCloud2消息指针
   */
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief 处理HESAI雷达数据
   * @param msg PointCloud2消息指针
   */
  void hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief 对点云进行特征提取和分类
   * @param pl 输入点云
   * @param types 输出点特征类型数组
   */
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);

  /**
   * @brief 发布处理后的点云
   * @param pl 待发布的点云
   * @param ct 当前时间戳
   */
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);

  /**
   * @brief 判断点是否属于平面
   * @param pl 输入点云
   * @param types 点特征类型数组
   * @param i 当前点索引
   * @param i_nex 输出下一个有效点索引
   * @param curr_direct 当前方向向量
   * @return 平面判断结果（1: 平面, 2: 可能是平面, 0: 不是平面）
   */
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);

  /**
   * @brief 判断是否为小平面
   * @param pl 输入点云
   * @param types 点特征类型数组
   * @param i_cur 当前点索引
   * @param i_nex 输出下一个有效点索引
   * @param curr_direct 当前方向向量
   * @return 是否为小平面
   */
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);

  /**
   * @brief 判断边缘跳变类型
   * @param pl 输入点云
   * @param types 点特征类型数组
   * @param i 当前点索引
   * @param nor_dir 相邻点方向（前/后）
   * @return 是否存在边缘跳变
   */
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  int group_size;                                  // 特征提取的点组大小
  double disA, disB;                               // 距离阈值A和B
  double inf_bound;                                // 无穷远边界阈值
  double limit_maxmid, limit_midmin, limit_maxmin; // 平面判断的距离比例限制
  double p2l_ratio;                                // 点到线的距离比例阈值
  double jump_up_limit, jump_down_limit;           // 跳变的上下限阈值
  double cos160;                                   // 160度的余弦值，用于角度判断
  double edgea, edgeb;                             // 边缘判断参数a和b
  double smallp_intersect, smallp_ratio;           // 小平面的交点和比例参数
  double vx, vy, vz;                               // 速度向量（用于运动补偿）
};
