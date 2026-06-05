/**
 * @file loop_refine.hpp
 * @brief 回环检测优化模块头文件
 *
 * 本文件实现了基于GTSAM的位姿图优化(PGO)和回环检测功能
 * 主要包括：ICP配准、位姿图边缘管理、八叉树全局Bundle Adjustment
 */

#ifndef LOOP_REFINE_HPP
#define LOOP_REFINE_HPP

#include "tools.hpp"
#include "voxel_map.hpp"

// #include "STDesc.h"
#include <gtsam/geometry/Pose3.h>        // GTSAM位姿表示
#include <gtsam/slam/PriorFactor.h>      // GTSAM先验因子
#include <gtsam/slam/BetweenFactor.h>    // GTSAM相对位姿因子
#include <gtsam/nonlinear/Values.h>      // GTSAM优化变量
#include <gtsam/nonlinear/ISAM2.h>       // GTSAM增量式优化器
#include <pcl/kdtree/kdtree_flann.h>     // PCL KD树最近邻搜索

using namespace std;

/**
 * @struct ScanPose
 * @brief 扫描位姿结构体
 *
 * 存储单次扫描的位姿信息和点云数据
 */
struct ScanPose
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUST x;                              // IMU状态（包含位姿、速度等）
  PVecPtr pvec;                         // 点云数据指针
  Eigen::Matrix<double, 6, 1> v6;       // 6维向量（可能用于存储协方差等信息）

  /**
   * @brief 构造函数
   * @param _x IMU状态
   * @param _pvec 点云数据指针
   */
  ScanPose(IMUST &_x, PVecPtr _pvec): x(_x), pvec(_pvec)
  {
    v6.setZero();
  }

  /**
   * @brief 更新位姿
   * @param dx 增量位姿变换
   *
   * 使用增量变换更新当前位姿的旋转、平移和速度
   */
  void update(IMUST dx)
  {
    x.v = dx.R * x.v;           // 更新速度
    x.p = dx.R * x.p + dx.p;    // 更新位置
    x.R = dx.R * x.R;           // 更新旋转
  }

  /**
   * @brief 从GTSAM位姿设置状态
   * @param pose GTSAM的Pose3位姿
   *
   * 将GTSAM优化后的位姿转换为内部状态表示
   */
  void set_state(const gtsam::Pose3 &pose)
  {
    Eigen::Matrix3d rot = pose.rotation().matrix();
    rot = rot * x.R.transpose();          // 计算旋转差
    x.R = pose.rotation().matrix();       // 设置新旋转
    x.p = pose.translation();             // 设置新平移
    x.v = rot * x.v;                      // 调整速度方向
  }

};

/**
 * @brief 基于法向量的ICP配准算法
 * @param pl_src 源点云（待配准）
 * @param pl_tar 目标点云（参考）
 * @param pose 输入输出参数，存储变换（平移向量，旋转矩阵）
 * @param icp_eigval ICP收敛判断的最小特征值阈值
 * @return 配准是否成功（基于特征值和收敛性判断）
 *
 * 使用点到平面的ICP方法进行配准，考虑法向量信息
 */
bool icp_normal(pcl::PointCloud<PointType> &pl_src, pcl::PointCloud<PointType> &pl_tar, pair<Eigen::Vector3d, Eigen::Matrix3d> &pose, double icp_eigval)
{
  // 构建目标点云的KD树用于最近邻搜索
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  for(PointType &ap: pl_tar.points)
  {
    pcl::PointXYZ pi;
    pi.x = ap.x; pi.y = ap.y; pi.z = ap.z;
    input_cloud.push_back(pi);
  }
  kd_tree.setInputCloud(input_cloud.makeShared());

  // 最近邻搜索结果存储
  vector<int> pointIdxNKNSearch(1);
  vector<float> pointNKNSquaredDistance(1);

  // ICP参数：法向量差阈值1，法向量差阈值2，点到平面距离阈值，点到点距离阈值
  Eigen::Vector4d paras(0.2, 0.2, 0.5, 3);
  int is_converge = 0;  // 收敛标志

  int ssize = pl_src.size();
  int match_num = 0;           // 匹配点对数量
  Eigen::Matrix3d mat_norm;    // 法向量矩阵用于特征值分析
  // for(int iterCount=0; iterCount<10; iterCount++)
  for(int iterCount=0; iterCount<20; iterCount++)  // 最多迭代20次
  {
    // 初始化Hessian矩阵和雅可比转置向量
    Eigen::Matrix<double, 6, 6> Hess; Hess.setZero();  // 6x6 Hessian矩阵（3旋转+3平移）
    Eigen::Matrix<double, 6, 1> JacT; JacT.setZero();  // 雅可比转置
    double resi = 0;  // 残差
    match_num = 0;
    mat_norm.setZero();

    // 遍历源点云中的所有点
    for(int i=0; i<ssize; i++)
    {
      PointType &searchPoint = pl_src[i];
      Eigen::Vector3d plocal(searchPoint.x, searchPoint.y, searchPoint.z);
      // 将源点转换到目标坐标系
      Eigen::Vector3d pi = pose.second * plocal + pose.first;
      pcl::PointXYZ use_search_point;
      use_search_point.x = pi[0];
      use_search_point.y = pi[1];
      use_search_point.z = pi[2];
      // 源点法向量转换到目标坐标系
      Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y, searchPoint.normal_z);
      ni = pose.second * ni;

      // 在目标点云中搜索最近邻点
      if (kd_tree.nearestKSearch(use_search_point, 1, pointIdxNKNSearch,pointNKNSquaredDistance) > 0)
      {
        pcl::PointXYZINormal nearstPoint = pl_tar[pointIdxNKNSearch[0]];
        Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
        Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,nearstPoint.normal_z);

        // 计算法向量差异和距离
        Eigen::Vector3d normal_inc = ni - tni;  // 法向量差
        Eigen::Vector3d normal_add = ni + tni;  // 法向量和（用于判断是否反向）
        double point_to_point_dis = (pi - tpi).norm();           // 点到点距离
        double point_to_plane = fabs(tni.transpose() * (pi - tpi));  // 点到平面距离

        // 匹配条件：法向量相似、点到平面距离小、点到点距离小
        if ((normal_inc.norm() < paras[0] || normal_add.norm() < paras[1]) && point_to_plane < paras[2] && point_to_point_dis < paras[3])
        // if ((normal_inc.norm() < 0.1 || normal_add.norm() < 0.1) &&   point_to_plane < 0.1 && point_to_point_dis < 1)
        {
          // 计算点到平面残差
          double rr = tni.dot(pi - tpi);
          // 计算雅可比矩阵（对旋转和平移的导数）
          Eigen::Matrix<double, 6, 1> jac;
          jac.head(3) = hat(plocal) * pose.second.transpose() * tni;  // 对旋转的导数
          jac.tail(3) = tni;  // 对平移的导数

          // 累加Hessian矩阵和雅可比转置
          Hess += jac * jac.transpose();
          JacT += jac * rr;
          resi += 0.5 * rr * rr;
          match_num++;
          mat_norm += tni * tni.transpose();  // 累加法向量外积（用于特征值分析）
        }

      }
    }

    // 求解增量：H * dx = -J^T
    Eigen::Matrix<double, 6, 1> dxi = Hess.ldlt().solve(-JacT);
    // 更新位姿
    pose.second = pose.second * Exp(dxi.head(3));  // 更新旋转
    pose.first = pose.first + dxi.tail(3);         // 更新平移

    // printf("icp%d: %lf %d\n", iterCount, resi, match_num);

    // 收敛判断
    if(dxi.head(3).norm()<1e-3 && dxi.tail(3).norm()<1e-3)
    {
      if(is_converge)
        break;  // 已经收敛过一次，退出
      else
      {
        // 第一次收敛，缩小阈值继续优化
        paras << 0.1, 0.1, 0.1, 1;
        is_converge = 1;
      }
    }

    // if(is_converge == 0 && iterCount > 4)
    // {
    //   paras << 0.1, 0.1, 0.1, 1;
    //   is_converge = 1;
    // }
  }

  // 计算法向量矩阵的特征值，用于判断配准质量
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(mat_norm);
  Eigen::Vector3d eig_vec = saes.eigenvalues();
  printf("eigvalue: %lf %lf %lf %d\n", eig_vec[0], eig_vec[1], eig_vec[2], is_converge);
  // 返回成功条件：最小特征值大于阈值且已收敛
  return eig_vec[0] > icp_eigval && is_converge == 1;
  // return eig_vec[0] > icp_eigval;

}

/**
 * @brief 向因子图添加边（使用IMUST状态）
 * @param pos1 第一个位姿节点索引
 * @param pos2 第二个位姿节点索引
 * @param x1 第一个位姿状态
 * @param x2 第二个位姿状态
 * @param graph GTSAM因子图
 * @param odometryNoise 里程计噪声模型
 *
 * 计算两个位姿之间的相对变换并添加到因子图中
 */
void add_edge(int pos1, int pos2, IMUST &x1, IMUST &x2, gtsam::NonlinearFactorGraph &graph, gtsam::noiseModel::Diagonal::shared_ptr odometryNoise)
{
  // 计算相对平移（在pos1坐标系下）
  gtsam::Point3 tt(x1.R.transpose() * (x2.p - x1.p));
  // 计算相对旋转
  gtsam::Rot3 RR(x1.R.transpose() * x2.R);
  // 创建BetweenFactor并添加到图中
  gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(pos1, pos2, gtsam::Pose3(RR, tt), odometryNoise));
  graph.push_back(factor);
}

/**
 * @brief 向因子图添加边（使用旋转矩阵和平移向量）
 * @param pos1 第一个位姿节点索引
 * @param pos2 第二个位姿节点索引
 * @param rot 相对旋转矩阵
 * @param tra 相对平移向量
 * @param graph GTSAM因子图
 * @param odometryNoise 里程计噪声模型
 */
void add_edge(int pos1, int pos2, Eigen::Matrix3d &rot, Eigen::Vector3d &tra, gtsam::NonlinearFactorGraph &graph, gtsam::noiseModel::Diagonal::shared_ptr odometryNoise)
{
  gtsam::Point3 tt(tra);
  gtsam::Rot3 RR(rot);
  gtsam::NonlinearFactor::shared_ptr factor(new gtsam::BetweenFactor<gtsam::Pose3>(pos1, pos2, gtsam::Pose3(RR, tt), odometryNoise));
  graph.push_back(factor);
}

/**
 * @struct PGO_Edge
 * @brief 位姿图优化(PGO)的边结构
 *
 * 存储两个地图/子图之间的约束关系，包括多个观测的旋转、平移和协方差
 */
struct PGO_Edge
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int m1, m2;                // 连接的两个地图/子图的ID
  vector<int> ids1, ids2;    // 对应的帧ID列表
  PLM(3) rots;               // 旋转矩阵列表
  PLV(3) tras;               // 平移向量列表
  PLV(6) covs;               // 协方差列表（6维：3旋转+3平移）

  /**
   * @brief 构造函数
   * @param _m1 第一个地图ID
   * @param _m2 第二个地图ID
   * @param id1 第一个帧ID
   * @param id2 第二个帧ID
   * @param rot 相对旋转
   * @param tra 相对平移
   * @param v6 协方差
   */
  PGO_Edge(int _m1, int _m2, int id1, int id2, Eigen::Matrix3d &rot, Eigen::Vector3d &tra, Eigen::Matrix<double, 6, 1> &v6): m1(_m1), m2(_m2)
  {
    push(id1, id2, rot, tra, v6);
  }

  /**
   * @brief 添加一个新的观测
   * @param id1 第一个帧ID
   * @param id2 第二个帧ID
   * @param rot 相对旋转
   * @param tra 相对平移
   * @param v6 协方差
   */
  void push(int id1, int id2, Eigen::Matrix3d &rot, Eigen::Vector3d &tra, Eigen::Matrix<double, 6, 1> &v6)
  {
    ids1.push_back(id1); ids2.push_back(id2);
    rots.push_back(rot); tras.push_back(tra);
    covs.push_back(v6);
  }

  /**
   * @brief 检查该边是否适配给定的地图列表
   * @param maps 地图ID列表
   * @param step 输出参数，存储m1和m2在maps中的索引
   * @return 如果m1和m2都在maps中返回true
   */
  bool is_adapt(vector<int> &maps, vector<int> &step)
  {
    bool f1 = false, f2 = false;
    for(int i=0; i<maps.size(); i++)
    {
      if(m1 == maps[i])
      {
        f1 = true;
        step[0] = i;
      }
      if(m2 == maps[i])
      {
        f2 = true;
        step[1] = i;
      }
    }

    return f1 && f2;
  }

};

/**
 * @struct PGO_Edges
 * @brief 位姿图优化的边集合管理器
 *
 * 管理所有PGO边，并提供图连通性分析功能
 */
struct PGO_Edges
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector<PGO_Edge> edges;      // 所有的PGO边
  vector<vector<int>> mates;   // 邻接表，mates[i]存储与地图i相连的所有地图ID

  /**
   * @brief 添加一条边或向已有边添加观测
   * @param _m1 第一个地图ID
   * @param _m2 第二个地图ID
   * @param _id1 第一个帧ID
   * @param _id2 第二个帧ID
   * @param rot 相对旋转
   * @param tra 相对平移
   * @param v6 协方差
   */
  void push(int _m1, int _m2, int _id1, int _id2, Eigen::Matrix3d &rot, Eigen::Vector3d &tra, Eigen::Matrix<double, 6, 1> &v6)
  {
    bool is_lack = true;
    // 检查是否已存在相同的边
    for(PGO_Edge &e: edges)
    {
      if(e.m1 == _m1 && e.m2 == _m2)
      {
        is_lack = false;
        e.push(_id1, _id2, rot, tra, v6);  // 向已有边添加新观测
        break;
      }
    }

    // 如果是新边，创建并更新邻接表
    if(is_lack)
    {
      edges.emplace_back(_m1, _m2, _id1, _id2, rot, tra, v6);
      int msize = mates.size();
      // 扩展邻接表大小
      for(int i=msize; i<_m2+1; i++)
        mates.emplace_back();
      // 添加双向连接
      mates[_m1].push_back(_m2);
      mates[_m2].push_back(_m1);
    }

  }

  /**
   * @brief 从根节点开始找出所有连通的地图ID
   * @param root 根地图ID
   * @param ids 输出参数，存储所有连通的地图ID（已排序）
   */
  void connect(int root, vector<int> &ids)
  {
    ids.clear();
    ids.push_back(root);
    tras(root, ids);  // 递归遍历
    sort(ids.begin(), ids.end());
  }

  /**
   * @brief 递归遍历图，查找所有连通节点
   * @param ord 当前节点ID
   * @param ids 已访问节点ID列表（输入输出参数）
   */
  void tras(int ord, vector<int> &ids)
  {
    if(ord < mates.size())
    for(int id: mates[ord])
    {
      bool is_exist = false;
      // 检查该节点是否已访问
      for(int i: ids)
      if(id == i)
      {
        is_exist = true;
        break;
      }

      // 如果未访问，添加并递归遍历
      if(!is_exist)
      {
        ids.push_back(id);
        tras(id, ids);
      }
    }

  }

};

// 全局BA（Bundle Adjustment）参数
vector<double> gba_eigen_value_array;  // 各层的特征值阈值数组
double gba_min_eigen_value;            // 最小特征值阈值
double gba_voxel_size;                 // 体素大小

/**
 * @class OctreeGBA
 * @brief 用于全局Bundle Adjustment的八叉树结构
 *
 * 这个八叉树用于组织点云数据，自适应细分空间，并提取平面特征用于BA优化
 */
class OctreeGBA
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector<PLV(3)> locals, worlds;  // 局部坐标和世界坐标的点集（每个窗口一个）
  PointCluster pcr_add;           // 累加的点簇
  int layer;                      // 当前层级
  int octo_state;                 // 八叉树状态（0=叶节点，1=已细分）
  int wdsize;                     // 窗口数量
  OctreeGBA* leaves[8];           // 8个子节点指针
  double voxel_center[3];         // 体素中心坐标
  float quater_length;            // 四分之一边长（用于细分）
  bool is_plane;                  // 是否为平面特征

  float ref;                      // 随机颜色引用（用于可视化）

  /**
   * @brief 构造函数
   * @param _l 层级
   * @param _w 窗口数量
   */
  OctreeGBA(int _l, int _w): layer(_l), wdsize(_w)
  {
    locals.resize(wdsize);
    worlds.resize(wdsize);
    for(int i=0; i<8; i++) leaves[i] = nullptr;
    is_plane = false;
    octo_state = 0;

    ref = 255.0*rand()/(RAND_MAX + 1.0f);  // 随机颜色
  }

  /**
   * @brief 重置八叉树节点
   * @param _l 新层级
   * @param _w 新窗口数量
   */
  void reset(int _l, int _w)
  {
    layer = _l; wdsize = _w;
    locals.clear(); worlds.clear();
    locals.resize(wdsize);
    worlds.resize(wdsize);
    pcr_add.clear();
    octo_state = 0;
    is_plane = false;
    for(int i=0; i<8; i++) leaves[i] = nullptr;
  }

  /**
   * @brief 判断当前体素是否为平面
   * @param eig_values 协方差矩阵的特征值（升序）
   * @return 如果满足平面条件返回true
   *
   * 平面判断基于两个条件：
   * 1. 最小特征值小于阈值
   * 2. 特征值比率小于层级阈值
   */
  inline bool plane_judge(Eigen::Vector3d &eig_values)
  {
    // return (eig_values[0] < min_eigen_value);
    return (eig_values[0] < gba_min_eigen_value && (eig_values[0]/eig_values[2])<gba_eigen_value_array[layer]);
  }

  /**
   * @brief 向体素添加一个点
   * @param ord 窗口序号
   * @param local 局部坐标
   * @param world 世界坐标
   */
  void push(int ord, Eigen::Vector3d &local, Eigen::Vector3d &world)
  {
    locals[ord].push_back(local);
    worlds[ord].push_back(world);
    pcr_add.push(world);  // 累加到点簇
  }

  /**
   * @brief 细分当前体素为8个子体素
   * @param oct_buf 八叉树节点缓冲池（用于复用节点，避免频繁内存分配）
   *
   * 将当前体素中的所有点分配到对应的子体素中
   */
  void subdivide(vector<OctreeGBA*> &oct_buf)
  {
    // 遍历所有窗口的所有点
    for(int i=0; i<wdsize; i++)
    for(int j=0; j<locals[i].size(); j++)
    {
      Eigen::Vector3d &pl = locals[i][j];
      Eigen::Vector3d &pw = worlds[i][j];
      // 确定点属于哪个子体素（8个）
      int xyz[3] = {0, 0, 0};
      for(int k=0; k<3; k++)
        if(pw[k] > voxel_center[k])
          xyz[k] = 1;  // 在中心的正方向
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];  // 计算子节点索引（0-7）

      // 如果子节点不存在，创建它
      if(leaves[leafnum] == nullptr)
      {
        if(oct_buf.size() > 0)
        {
          // 从缓冲池复用节点
          leaves[leafnum] = oct_buf.back();
          leaves[leafnum]->reset(layer+1, wdsize);
          oct_buf.pop_back();
        }
        else
        {
          // 创建新节点
          leaves[leafnum] = new OctreeGBA(layer+1, wdsize);
        }

        // 设置子节点的体素中心和大小
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      // 将点添加到对应的子节点
      leaves[leafnum]->push(i, pl, pw);
    }
  }

  /**
   * @brief 递归切分体素并提取平面特征
   * @param vox_opt 激光雷达因子优化器（用于添加体素约束）
   * @param oct_buf 八叉树节点缓冲池
   *
   * 这是八叉树的核心功能：
   * 1. 如果体素是平面，提取为优化约束
   * 2. 如果不是平面且未达到最大层级，继续细分
   * 3. 递归处理所有子节点
   */
  void recut(LidarFactor &vox_opt, vector<OctreeGBA*> &oct_buf)
  {
    // 点数太少，跳过
    if(pcr_add.N <= 10)
      return;

    // 计算协方差矩阵的特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
    Eigen::Vector3d eig_value = saes.eigenvalues();
    Eigen::Matrix3d eig_vector = saes.eigenvectors();
    is_plane = plane_judge(eig_value);

    // 如果是平面特征，添加到优化器
    if(is_plane)
    {
      if(pcr_add.N < 10) return;

      // 检查有效窗口数量（至少2个窗口有点）
      int exi = 0;
      for(int i=0; i<wdsize; i++)
        if(locals[i].size() != 0)
          exi++;
      if(exi <= 1) return;

      // 检查平面质量（特征值比率）
      if(eig_value[0]/eig_value[1] > 0.12) return;

      double coe = 1.0;  // 权重系数
      // double coe = 1.0 / pcr_add.N;
      // 为每个窗口创建点簇
      vector<PointCluster> pcrs(wdsize);
      for(int i=0; i<wdsize; i++)
        for(Eigen::Vector3d &v: locals[i])
          pcrs[i].push(v);
      PointCluster pcr_fix;
      // 将体素添加到优化器
      vox_opt.push_voxel(pcrs, pcr_fix, coe, eig_value, eig_vector, pcr_add);

      return;
    }
    else if(layer >= max_layer)
    {
      // 达到最大层级，停止细分
      return;
    }
    else
    {
      // 不是平面且未达到最大层级，继续细分
      subdivide(oct_buf);
      octo_state = 1;  // 标记为已细分
    }

    // 递归处理所有子节点
    for(int i=0; i<8; i++)
      if(leaves[i] != nullptr)
        leaves[i]->recut(vox_opt, oct_buf);

  }

  /**
   * @brief 递归回收所有子节点到缓冲池
   * @param oct_buf 八叉树节点缓冲池
   *
   * 将所有子节点回收到缓冲池以便复用，避免频繁的内存分配
   */
  void tras_ptr(vector<OctreeGBA*> &oct_buf)
  {
    if(octo_state == 1)
    for(int i=0; i<8; i++)
    if(leaves[i] != nullptr)
    {
      leaves[i]->tras_ptr(oct_buf);     // 递归回收子节点
      oct_buf.push_back(leaves[i]);     // 添加到缓冲池
      leaves[i] = nullptr;              // 清空指针
    }
  }

  /**
   * @brief 递归遍历八叉树，将所有点添加到点云用于可视化
   * @param pl_send 输出点云
   *
   * 为每个叶节点的点分配相同的颜色（强度值）
   */
  void tras_display(pcl::PointCloud<PointType> &pl_send)
  {
    if(octo_state == 0)  // 叶节点
    {
      PointType ap; ap.intensity = ref;  // 设置颜色
      for(int i=0; i<wdsize; i++)
      for(Eigen::Vector3d &v: worlds[i])
      {
        ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
        pl_send.push_back(ap);
      }
    }
    else  // 内部节点，递归处理子节点
    {
      for(int i=0; i<8; i++)
        if(leaves[i] != nullptr)
          leaves[i]->tras_display(pl_send);
    }
  }

  // 析构函数（已注释，因为使用缓冲池管理内存）
  // ~OctreeGBA()
  // {
  //   for(int i=0; i<8; i++)
  //     if(leaves[i] != nullptr)
  //       delete leaves[i];
  // }

  /**
   * @brief 静态方法：将点云切分到体素地图中
   * @param feat_map 特征体素地图（哈希表）
   * @param xc 当前帧的位姿
   * @param plptr 点云指针
   * @param win_count 窗口计数（帧索引）
   * @param wdsize 窗口大小
   *
   * 将点云中的每个点转换到世界坐标系，然后分配到对应的体素中
   */
  static void cut_voxel(unordered_map<VOXEL_LOC, OctreeGBA*> &feat_map, IMUST &xc, pcl::PointCloud<PointType>::Ptr plptr, int win_count, int wdsize)
  {
    for(PointType &ap: plptr->points)
    {
      // 局部坐标
      Eigen::Vector3d local(ap.x, ap.y, ap.z);
      // 转换到世界坐标系
      Eigen::Vector3d world = xc.R * local + xc.p;

      // 计算体素位置（整数坐标）
      float loc[3];
      for(int j=0; j<3; j++)
      {
        loc[j] = world[j] / gba_voxel_size;
        if(loc[j] < 0) loc[j] -= 1;  // 负数向下取整
      }

      VOXEL_LOC position(loc[0], loc[1], loc[2]);
      auto iter = feat_map.find(position);
      if(iter != feat_map.end())
      {
        // 体素已存在，添加点
        iter->second->push(win_count, local, world);
      }
      else
      {
        // 创建新体素
        OctreeGBA *ot = new OctreeGBA(0, wdsize);
        ot->push(win_count, local, world);
        // 设置体素中心和大小
        ot->voxel_center[0] = (0.5+position.x) * gba_voxel_size;
        ot->voxel_center[1] = (0.5+position.y) * gba_voxel_size;
        ot->voxel_center[2] = (0.5+position.z) * gba_voxel_size;
        ot->quater_length = gba_voxel_size / 4.0;
        feat_map[position] = ot;
      }

    }


  }

};

/**
 * @brief 多线程执行八叉树切分和平面特征提取
 * @param feat_map 特征体素地图
 * @param voxhess 激光雷达因子优化器（输出参数）
 * @param thd_num 线程数量
 *
 * 将体素地图均匀分配给多个线程并行处理，提高处理效率
 * 最后合并所有线程的结果到主优化器中
 */
void OctreeGBA_multi_recut(unordered_map<VOXEL_LOC, OctreeGBA*> &feat_map, LidarFactor &voxhess, int thd_num)
{
  // 为每个线程分配八叉树列表
  vector<vector<OctreeGBA*>> octss(thd_num);
  // 为每个线程创建独立的LidarFactor副本
  vector<LidarFactor> vec_voxhess(thd_num, voxhess);
  int g_size = feat_map.size();
  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;  // 每个线程的平均工作量
  int cnt = 0;

  // 将体素均匀分配给各个线程
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
  {
    octss[cnt].push_back(iter->second);
    if(octss[cnt].size() >= part && cnt < thd_num-1)
      cnt++;
  }

  // Lambda函数：处理一批八叉树
  auto recut_func = [](vector<OctreeGBA*> &octs, LidarFactor &voxhess)
  {
    vector<OctreeGBA*> oct_buf;  // 节点缓冲池
    for(OctreeGBA *oc: octs)
    {
      oc->recut(voxhess, oct_buf);    // 递归切分并提取平面
      oc->tras_ptr(oct_buf);          // 回收子节点
      delete oc;                      // 删除根节点
      // 清理缓冲池中的所有节点
      for(OctreeGBA *oct: oct_buf)
        delete oct;
      oct_buf.clear();
    }

    // for(OctreeGBA *oc: oct_buf)
    //   delete oc;
  };

  // 启动多个工作线程（第0个线程在主线程执行）
  for(int i=1; i<thd_num; i++)
    mthreads[i] = new thread(recut_func, ref(octss[i]), ref(vec_voxhess[i]));

  // 等待所有线程完成并合并结果
  for(int i=0; i<thd_num; i++)
  {
    if(i == 0)
    {
      // 主线程执行第0个任务
      recut_func(octss[0], voxhess);
    }
    else
    {
      // 等待子线程完成
      mthreads[i]->join();
      delete mthreads[i];

      // 合并该线程的结果到主优化器
      voxhess.plvec_voxels.insert(voxhess.plvec_voxels.end(), vec_voxhess[i].plvec_voxels.begin(), vec_voxhess[i].plvec_voxels.end());
      voxhess.sig_vecs.insert(voxhess.sig_vecs.end(), vec_voxhess[i].sig_vecs.begin(), vec_voxhess[i].sig_vecs.end());
      voxhess.coeffs.insert(voxhess.coeffs.end(), vec_voxhess[i].coeffs.begin(), vec_voxhess[i].coeffs.end());
      voxhess.eig_values.insert(voxhess.eig_values.end(), vec_voxhess[i].eig_values.begin(), vec_voxhess[i].eig_values.end());
      voxhess.eig_vectors.insert(voxhess.eig_vectors.end(), vec_voxhess[i].eig_vectors.begin(), vec_voxhess[i].eig_vectors.end());
      voxhess.pcr_adds.insert(voxhess.pcr_adds.end(), vec_voxhess[i].pcr_adds.begin(), vec_voxhess[i].pcr_adds.end());
    }
  }
}

#endif
