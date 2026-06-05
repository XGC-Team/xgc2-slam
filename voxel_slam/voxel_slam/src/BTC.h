/**
 * @file BTC.h
 * @brief BTC (Binary Triangle Code) 描述符相关的数据结构和类定义
 *
 * 本文件定义了用于场景识别和回环检测的BTC描述符系统，包括：
 * - 体素化和平面检测
 * - 二进制描述符提取
 * - STD（Spatial Triangle Descriptor）描述符生成和匹配
 * - 几何验证和位姿估计
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <fstream>
#include <mutex>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// 哈希函数的质数参数，用于体素位置哈希计算
#define HASH_P 116101
// 哈希值的最大范围
#define MAX_N 10000000000

/**
 * @brief 配置参数结构体
 *
 * 包含BTC描述符系统的所有可配置参数，分为以下几类：
 * - 点云预处理参数
 * - 关键点提取参数
 * - STD描述符参数
 * - 场景识别参数
 */
typedef struct ConfigSetting {
  /* 点云预处理参数 */
  int useful_corner_num_ = 30;  // 有用的角点数量

  /* 关键点提取参数 */
  float plane_merge_normal_thre_;      // 平面合并法向量阈值
  float plane_merge_dis_thre_;         // 平面合并距离阈值
  float plane_detection_thre_ = 0.01;  // 平面检测阈值（特征值比）
  float voxel_size_ = 1.0;             // 体素大小（米）
  int voxel_init_num_ = 10;            // 体素初始化所需最少点数
  int proj_plane_num_ = 3;             // 投影平面数量
  float proj_image_resolution_ = 0.5;  // 投影图像分辨率（米/像素）
  float proj_image_high_inc_ = 0.5;    // 投影图像高度增量
  float proj_dis_min_ = 0;             // 投影最小距离
  float proj_dis_max_ = 5;             // 投影最大距离
  float summary_min_thre_ = 10;        // 二进制描述符最小占用阈值
  int line_filter_enable_ = 0;         // 线特征滤波开关
  int touch_filter_enable_ = 0;        // 接触滤波开关

  /* STD描述符参数 */
  float descriptor_near_num_ = 10;             // 描述符邻近点数量
  float descriptor_min_len_ = 1;               // 描述符三角形最小边长（米）
  float descriptor_max_len_ = 10;              // 描述符三角形最大边长（米）
  float non_max_suppression_radius_ = 3.0;     // 非极大值抑制半径（米）
  float std_side_resolution_ = 0.2;            // STD边长分辨率（米）

  /* 场景识别参数 */
  int skip_near_num_ = 20;              // 跳过的邻近帧数量（避免短期重复）
  int candidate_num_ = 50;              // 候选帧数量
  float rough_dis_threshold_ = 0.03;    // 粗匹配距离阈值
  float similarity_threshold_ = 0.7;    // 相似度阈值
  float icp_threshold_ = 0.5;           // ICP匹配阈值
  float normal_threshold_ = 0.1;        // 法向量阈值
  float dis_threshold_ = 0.3;           // 距离阈值

} ConfigSetting;

/**
 * @brief 二进制描述符（双精度版本）
 *
 * 用于描述投影平面上的局部几何特征，通过二进制占用数组编码
 */
typedef struct BinaryDescriptor {
  std::vector<bool> occupy_array_;  // 二进制占用数组（7x7网格）
  unsigned char summary_;           // 占用总和（用于快速过滤）
  Eigen::Vector3d location_;        // 描述符在3D空间中的位置
} BinaryDescriptor;

/**
 * @brief 二进制描述符（单精度版本）
 *
 * 内存优化版本，用于需要存储大量描述符的场景
 */
typedef struct BinaryDescriptorF {
  // std::vector<bool> occupy_array_;  // 已注释：占用数组
  // bool occupy_array_[49];           // 已注释：固定大小占用数组
  unsigned char summary_;              // 占用总和
  Eigen::Vector3f location_;           // 3D位置（单精度）
} BinaryDescriptorF;

/**
 * @brief STD（Spatial Triangle Descriptor）空间三角形描述符
 *
 * 由三个角点的二进制描述符构成的三角形特征，用于场景识别
 * 大小约1KB，用于快速匹配和回环检测
 */
typedef struct STD {
  Eigen::Vector3d triangle_;   // 三角形三条边的长度 (a, b, c)
  Eigen::Vector3d angle_;      // 三角形三个内角 (角A, 角B, 角C)
  Eigen::Vector3d center_;     // 三角形中心点坐标
  // unsigned short frame_number_;        // 已注释：旧版帧号（短整型）
  int frame_number_;                      // 所属帧的编号
  // std::vector<unsigned short> score_frame_;  // 已注释：评分帧列表
  // std::vector<Eigen::Matrix3d> position_list_; // 已注释：位姿列表
  BinaryDescriptor binary_A_;  // 顶点A的二进制描述符
  BinaryDescriptor binary_B_;  // 顶点B的二进制描述符
  BinaryDescriptor binary_C_;  // 顶点C的二进制描述符
} STD;

/**
 * @brief BTC平面结构体
 *
 * 表示从点云中提取的平面特征，包含平面的几何属性和统计信息
 */
typedef struct BTCPlane {
  pcl::PointXYZINormal p_center_;  // 平面中心点（PCL格式，包含法向量）
  Eigen::Vector3d center_;         // 平面中心坐标
  Eigen::Vector3d normal_;         // 平面法向量
  Eigen::Matrix3d covariance_;     // 点云协方差矩阵
  float radius_ = 0;               // 平面半径（点云分布范围）
  float min_eigen_value_ = 1;      // 协方差矩阵最小特征值（平面度量）
  float d_ = 0;                    // 平面方程的d参数（ax+by+cz+d=0）
  int id_ = 0;                     // 平面ID
  int sub_plane_num_ = 0;          // 子平面数量（合并后）
  int points_size_ = 0;            // 平面包含的点数
  bool is_plane_ = false;          // 是否为有效平面
} BTCPlane;

/**
 * @brief STD匹配列表结构体
 *
 * 存储两帧之间的STD描述符匹配结果
 */
typedef struct STDMatchList {
  std::vector<std::pair<STD, STD>> match_list_;  // STD匹配对列表
  std::pair<int, int> match_id_;                 // 匹配的两帧ID
  int match_frame_;                              // 匹配到的历史帧ID
  double mean_dis_;                              // 平均匹配距离
} STDMatchList;

/**
 * @brief 体素位置类
 *
 * 用于表示3D空间中的体素网格位置，支持哈希映射
 */
class BTCVOXEL_LOC {
public:
  int64_t x, y, z;  // 体素在网格中的整数坐标

  /**
   * @brief 构造函数
   * @param vx x坐标（默认0）
   * @param vy y坐标（默认0）
   * @param vz z坐标（默认0）
   */
  BTCVOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  /**
   * @brief 相等运算符
   * @param other 另一个体素位置
   * @return 如果两个体素位置相同返回true
   */
  bool operator==(const BTCVOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

/**
 * @brief 体素位置的哈希函数
 *
 * 用于std::unordered_map中的体素位置哈希计算
 */
namespace std {
template <> struct hash<BTCVOXEL_LOC> {
  int64_t operator()(const BTCVOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    // 使用质数HASH_P进行哈希计算，避免冲突
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
} // namespace std

/**
 * @brief STD描述符位置类
 *
 * 用于STD描述符的哈希索引，包含三角形中心坐标和边长信息
 */
class STD_LOC {
public:
  int64_t x, y, z, a, b, c;  // x,y,z: 三角形中心坐标; a,b,c: 三角形三边长度

  /**
   * @brief 构造函数
   * @param vx 中心x坐标（默认0）
   * @param vy 中心y坐标（默认0）
   * @param vz 中心z坐标（默认0）
   * @param va 边长a（默认0）
   * @param vb 边长b（默认0）
   * @param vc 边长c（默认0）
   */
  STD_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0, int64_t va = 0,
          int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  /**
   * @brief 相等运算符
   * @param other 另一个STD位置
   * @return 如果位置相同返回true（仅比较中心坐标）
   */
  bool operator==(const STD_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
    // 已注释：完整比较（包括边长）
    // return (x == other.x && y == other.y && z == other.z && a == other.a &&
    //         b == other.b && c == other.c);
  }
};

/**
 * @brief STD位置的哈希函数
 *
 * 用于std::unordered_map中的STD描述符哈希计算
 */
namespace std {
template <> struct hash<STD_LOC> {
  int64_t operator()(const STD_LOC &s) const {
    using std::hash;
    using std::size_t;
    // 仅使用中心坐标进行哈希
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};
} // namespace std

/**
 * @brief BTC八叉树类
 *
 * 用于点云的层次化体素表示和平面提取
 * 通过递归划分空间，在叶节点进行平面检测
 */
class BTCOctoTree {
public:
  ConfigSetting config_setting_;                // 配置参数
  std::vector<Eigen::Vector3d> voxel_points_;   // 当前节点包含的点云
  BTCPlane *plane_ptr_;                         // 当前节点提取的平面
  int layer_;                                   // 八叉树层级
  int octo_state_;                              // 八叉树状态：0表示叶节点，1表示有子节点
  int merge_num_ = 0;                           // 合并的平面数量
  bool is_project_ = false;                     // 是否为投影平面
  std::vector<Eigen::Vector3d> project_normal;  // 投影法向量列表
  bool is_publish_ = false;                     // 是否已发布（用于可视化）
  BTCOctoTree *leaves_[8];                      // 8个子节点指针
  double voxel_center_[3];                      // 体素中心坐标 (x, y, z)
  float quater_length_;                         // 体素四分之一边长
  bool init_octo_;                              // 八叉树是否已初始化

  // 用于可视化的连接信息
  bool is_check_connect_[6];  // 是否已检查连接（6个面）
  bool connect_[6];            // 是否连接（6个面）
  BTCOctoTree *connect_tree_[6];  // 连接的邻居节点（6个面）

  /**
   * @brief 构造函数
   * @param config_setting 配置参数
   */
  BTCOctoTree(const ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    // 初始化8个子节点为空
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    // 初始化连接信息（用于可视化）
    for (int i = 0; i < 6; i++) {
      is_check_connect_[i] = false;
      connect_[i] = false;
      connect_tree_[i] = nullptr;
    }
    plane_ptr_ = new BTCPlane;
  }

  /**
   * @brief 初始化平面参数
   */
  void init_plane();

  /**
   * @brief 初始化八叉树结构
   */
  void init_octo_tree();

  /**
   * @brief 析构函数
   */
  ~BTCOctoTree()
  {
    delete plane_ptr_;
  }

};

/**
 * @brief 从配置文件加载参数
 * @param config_file 配置文件路径
 * @param config_setting 配置参数结构体（输出）
 */
void load_config_setting(std::string &config_file,
                         ConfigSetting &config_setting);

/**
 * @brief 计算两个二进制描述符的相似度
 * @param b1 第一个二进制描述符
 * @param b2 第二个二进制描述符
 * @return 相似度值（0-1之间）
 */
double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2);

/**
 * @brief 二进制描述符排序比较函数（按summary_降序）
 * @param a 第一个描述符
 * @param b 第二个描述符
 * @return a的summary_是否大于b
 */
bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b);

/**
 * @brief 平面排序比较函数
 * @param plane1 第一个平面
 * @param plane2 第二个平面
 * @return plane1是否应排在plane2前面
 */
bool plane_greater_sort(BTCPlane *plane1, BTCPlane *plane2);

// 已注释：计算匹配STD对的三角形距离
// double
// calc_triangle_dis(const std::vector<std::pair<STD, STD>> &match_std_list);

/**
 * @brief 从ROS参数服务器读取参数
 * @param nh ROS节点句柄
 * @param config_setting 配置参数结构体（输出）
 * @param isHighFly 是否为高空飞行模式
 */
void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting, int isHighFly);

/**
 * @brief 从PCL点的法向量提取为Eigen向量
 * @param pi 包含法向量的PCL点
 * @return 法向量（Eigen格式）
 */
Eigen::Vector3d normal2vec(const pcl::PointXYZINormal &pi);

/**
 * @brief 从任意类型的点提取坐标为Eigen向量
 * @tparam T 点类型（需包含x,y,z成员）
 * @param pi 输入点
 * @return 坐标向量
 */
template <typename T> Eigen::Vector3d point2vec(const T &pi) {
  Eigen::Vector3d vec(pi.x, pi.y, pi.z);
  return vec;
}

/**
 * @brief 计算时间增量（毫秒）
 * @param t_end 结束时间点
 * @param t_begin 开始时间点
 * @return 时间差（毫秒）
 */
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin);


/**
 * @brief STD描述符管理器类
 *
 * 管理STD描述符的生成、存储、检索和匹配
 * 核心功能包括：
 * - 从点云生成STD描述符
 * - 维护描述符数据库
 * - 执行回环检测
 * - 进行几何验证
 */
class STDescManager {
public:
  STDescManager() = default;

  ConfigSetting config_setting_;  // 配置参数

  unsigned int current_frame_id_;  // 当前帧ID

  /**
   * @brief 构造函数
   * @param config_setting 配置参数
   */
  STDescManager(ConfigSetting &config_setting)
      : config_setting_(config_setting) {
    current_frame_id_ = 0;
  };

  // 已注释：二进制描述符向量
  // std::vector<BinaryDescriptor> vec_binary;

  // 哈希表，保存所有STD描述符
  std::unordered_map<STD_LOC, std::vector<STD>> data_base_;

  // 已注释：保存所有关键帧点云（可选）
  // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;

  // 已注释：保存所有关键帧的二进制描述符
  // std::vector<std::vector<BinaryDescriptor>> history_binary_list_;

  // 保存所有关键帧的平面点云（必需，用于几何验证）
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

  /* ========== 三个主要处理函数 ========== */

  /**
   * @brief 从点云生成STD描述符
   *
   * 主要流程：
   * 1. 体素化和平面检测
   * 2. 提取二进制描述符
   * 3. 构建STD三角形描述符
   *
   * @param input_cloud 输入点云
   * @param stds_vec 输出的STD描述符列表
   * @param id 当前帧ID
   */
  void GenerateSTDescs(pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                       std::vector<STD> &stds_vec, int id);

  /**
   * @brief 搜索回环
   *
   * 在历史帧中搜索与当前帧匹配的回环候选
   *
   * @param stds_vec 当前帧的STD描述符
   * @param loop_result 回环结果 <候选帧ID, ICP评分>，-1表示无回环
   * @param loop_transform 回环变换 <平移向量, 旋转矩阵>
   * @param loop_std_pair 匹配的STD对列表
   * @param pl_cur 当前帧平面点云
   */
  void SearchLoop(std::vector<STD> &stds_vec,
                  std::pair<int, double> &loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
                  std::vector<std::pair<STD, STD>> &loop_std_pair, pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_cur);

  /**
   * @brief 将描述符添加到数据库
   *
   * @param stds_vec STD描述符列表
   */
  void AddSTDescs(const std::vector<STD> &stds_vec);

  /**
   * @brief 平面到平面的几何ICP优化
   *
   * 使用平面约束进行精确的位姿估计
   *
   * @param source_cloud 源平面点云
   * @param target_cloud 目标平面点云
   * @param transform 输出变换 <平移向量, 旋转矩阵>
   */
  void PlaneGeomrtricIcp(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);

private:
  /* ========== 以下是子处理函数 ========== */

  /**
   * @brief 体素化和平面检测
   *
   * 将输入点云划分为体素网格，并在每个体素中检测平面
   *
   * @param input_cloud 输入点云
   * @param voxel_map 输出的体素地图（体素位置->八叉树节点）
   */
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> &voxel_map);

  /**
   * @brief 从体素地图中提取平面
   *
   * @param voxel_map 体素地图
   * @param plane_cloud 输出的平面点云（每个点代表一个平面）
   */
  void get_plane(const std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> &voxel_map,
                 pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud);

  /**
   * @brief 获取投影平面
   *
   * 从特征地图中提取用于投影的平面
   *
   * @param feat_map 特征地图
   * @param project_plane_list 输出的投影平面列表
   */
  void get_project_plane(std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> &feat_map,
                         std::vector<BTCPlane *> &project_plane_list);

  /**
   * @brief 合并相似平面
   *
   * 根据法向量和距离阈值合并相似平面
   *
   * @param origin_list 原始平面列表
   * @param merge_plane_list 输出的合并后平面列表
   */
  void merge_plane(std::vector<BTCPlane *> &origin_list,
                   std::vector<BTCPlane *> &merge_plane_list);

  /**
   * @brief 二进制描述符提取器
   *
   * 从投影平面和点云中提取角点的二进制描述符
   *
   * @param proj_plane_list 投影平面列表
   * @param input_cloud 输入点云
   * @param binary_descriptor_list 输出的二进制描述符列表
   */
  void binary_extractor(const std::vector<BTCPlane *> proj_plane_list,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                        std::vector<BinaryDescriptor> &binary_descriptor_list);

  /**
   * @brief 提取单个投影平面的二进制描述符
   *
   * @param project_center 投影中心
   * @param project_normal 投影法向量
   * @param input_cloud 输入点云
   * @param binary_list 输出的二进制描述符列表
   */
  void extract_binary(const Eigen::Vector3d &project_center,
                      const Eigen::Vector3d &project_normal,
                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
                      std::vector<BinaryDescriptor> &binary_list);

  /**
   * @brief 非极大值抑制
   *
   * 控制角点数量，保留局部最优的二进制描述符
   *
   * @param binary_list 二进制描述符列表（输入输出）
   */
  void non_maxi_suppression(std::vector<BinaryDescriptor> &binary_list);

  /**
   * @brief 从角点构建STD描述符
   *
   * 通过连接三个角点形成三角形，构建STD描述符
   *
   * @param binary_list 二进制描述符列表（角点）
   * @param frame_id 当前帧ID
   * @param std_list 输出的STD描述符列表
   */
  void generate_std(const std::vector<BinaryDescriptor> &binary_list,
                    const int &frame_id, std::vector<STD> &std_list);

  /**
   * @brief 候选帧选择器
   *
   * 根据STD粗匹配数量选择指定数量的候选帧
   *
   * @param stds_vec 当前帧的STD描述符
   * @param candidate_matcher_vec 输出的候选匹配列表
   */
  void candidate_selector(std::vector<STD> &stds_vec,
                          std::vector<STDMatchList> &candidate_matcher_vec);

  /**
   * @brief 候选帧几何验证
   *
   * 通过几何一致性检查获取最佳候选帧
   *
   * @param candidate_matcher 候选匹配
   * @param verify_score 输出的验证评分
   * @param relative_pose 输出的相对位姿 <平移, 旋转>
   * @param sucess_match_vec 输出的成功匹配STD对
   * @param pl_cur 当前帧平面点云
   */
  void
  candidate_verify(STDMatchList &candidate_matcher, double &verify_score,
                   std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
                   std::vector<std::pair<STD, STD>> &sucess_match_vec, pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_cur);

  /**
   * @brief 三角形求解器
   *
   * 从匹配的STD对计算相对变换
   *
   * @param std_pair 匹配的STD对
   * @param t 输出的平移向量
   * @param rot 输出的旋转矩阵
   */
  void triangle_solver(std::pair<STD, STD> &std_pair, Eigen::Vector3d &t,
                       Eigen::Matrix3d &rot);

  /**
   * @brief 平面几何验证
   *
   * 使用平面到平面ICP阈值进行几何验证
   *
   * @param source_cloud 源平面点云
   * @param target_cloud 目标平面点云
   * @param transform 变换 <平移向量, 旋转矩阵>
   * @return ICP评分（越小越好）
   */
  double plane_geometric_verify(
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
      const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform);
};