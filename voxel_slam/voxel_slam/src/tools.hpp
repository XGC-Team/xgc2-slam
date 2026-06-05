// tools.hpp - Voxel-SLAM工具函数和数据结构定义
// 包含体素定位、旋转矩阵操作、IMU状态表示和点云降采样等核心工具

#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <Eigen/Core>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 哈希函数使用的质数，用于体素位置的哈希计算
#define HASH_P 116101
// 哈希函数中使用的最大数值，防止溢出
#define MAX_N 10000000000
// 反对称矩阵宏，用于将向量转换为反对称矩阵形式（叉乘矩阵）
// 输入3D向量v，输出3x3反对称矩阵的9个元素（按行展开）
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
// PLM: Point List Matrix - 定义Eigen矩阵向量，使用对齐分配器
#define PLM(a) vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
// PLV: Point List Vector - 定义Eigen列向量向量，使用对齐分配器
#define PLV(a) vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

// 重力加速度常量 (m/s²)
#define G_m_s2 9.8
// IMU状态维度：旋转(3) + 位置(3) + 速度(3) + 陀螺仪偏差(3) + 加速度计偏差(3) = 15
#define DIM 15
// 匹配点数量阈值
#define NMATCH 5

// 点类型定义：带强度和法向量的点云类型
typedef pcl::PointXYZINormal PointType;
using namespace std;

// 3x3单位矩阵，用于旋转计算
Eigen::Matrix3d I33(Eigen::Matrix3d::Identity());

/**
 * @class VOXEL_LOC
 * @brief 体素位置类，用于三维空间的离散化表示
 *
 * 在哈希表中作为键值使用，实现点云的体素化存储和快速查询
 */
class VOXEL_LOC
{
public:
  int64_t x, y, z;  // 体素在三维网格中的整数坐标

  /**
   * @brief 构造函数
   * @param vx x方向体素索引（默认为0）
   * @param vy y方向体素索引（默认为0）
   * @param vz z方向体素索引（默认为0）
   */
  VOXEL_LOC(int64_t vx=0, int64_t vy=0, int64_t vz=0): x(vx), y(vy), z(vz){}

  /**
   * @brief 相等运算符重载
   * @param other 待比较的体素位置
   * @return 如果两个体素位置完全相同返回true
   */
  bool operator == (const VOXEL_LOC &other) const
  {
    return (x==other.x && y==other.y && z==other.z);
  }
};

/**
 * @brief 为VOXEL_LOC类特化std::hash，使其可以在unordered_map中作为键使用
 */
namespace std
{
  template<>
  struct hash<VOXEL_LOC>
  {
    /**
     * @brief 哈希函数运算符
     * @param s 待哈希的体素位置
     * @return 哈希值
     *
     * 使用质数HASH_P和模运算，将三维坐标映射到一维哈希值
     * 计算公式：((z * HASH_P + y) * HASH_P + x) % MAX_N
     * 这种方法能够有效减少哈希碰撞
     */
    size_t operator() (const VOXEL_LOC &s) const
    {
      using std::size_t; using std::hash;
      // 旧方法（已注释）：使用异或和移位操作
      // return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^ (hash<int64_t>()(s.z) << 1);
      // 当前方法：使用质数乘法和模运算，提供更好的哈希分布
      return (((hash<int64_t>()(s.z)*HASH_P)%MAX_N + hash<int64_t>()(s.y))*HASH_P)%MAX_N + hash<int64_t>()(s.x);
    }
  };
}

/**
 * @brief SO(3)李代数的指数映射，将旋转向量转换为旋转矩阵
 * @param ang 旋转向量（轴角表示），方向为旋转轴，模长为旋转角度
 * @return 对应的3x3旋转矩阵
 *
 * 使用Rodrigues公式：R = I + sin(θ)K + (1-cos(θ))K²
 * 其中θ是旋转角度，K是旋转轴的反对称矩阵
 * 当角度很小时（< 1e-11），返回单位矩阵以避免数值不稳定
 */
Eigen::Matrix3d Exp(const Eigen::Vector3d &ang)
{
  double ang_norm = ang.norm();  // 计算旋转角度（向量模长）
  // if (ang_norm > 0.00001)  // 旧阈值
  if (ang_norm >= 1e-11)  // 角度阈值，避免除零和数值不稳定
  {
    Eigen::Vector3d r_axis = ang / ang_norm;  // 归一化得到旋转轴
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);  // 构造旋转轴的反对称矩阵
    /// Rodrigues变换公式
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }

  // 角度接近零时，返回单位矩阵
  return I33;

}

/**
 * @brief SO(3)李代数的指数映射（带时间间隔版本）
 * @param ang_vel 角速度向量
 * @param dt 时间间隔
 * @return 对应的旋转矩阵增量
 *
 * 功能同上一个Exp函数，但输入为角速度和时间间隔
 * 旋转角度 = 角速度 × 时间间隔
 * 常用于IMU积分中计算旋转更新
 */
Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt)
{
  double ang_vel_norm = ang_vel.norm();  // 计算角速度的模长
  if (ang_vel_norm > 1e-7)  // 角速度阈值检查
  {
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;  // 归一化得到旋转轴
    Eigen::Matrix3d K;

    K << SKEW_SYM_MATRX(r_axis);  // 构造反对称矩阵
    double r_ang = ang_vel_norm * dt;  // 计算实际旋转角度

    /// Rodrigues变换公式
    return I33 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }

  // 角速度接近零时，返回单位矩阵（无旋转）
  return I33;
}

/**
 * @brief SO(3)李代数的对数映射，将旋转矩阵转换为旋转向量
 * @param R 输入的3x3旋转矩阵
 * @return 对应的旋转向量（轴角表示）
 *
 * 这是Exp函数的逆运算
 * 从旋转矩阵的迹（trace）计算旋转角度：θ = acos((trace(R)-1)/2)
 * 从旋转矩阵的反对称部分提取旋转轴
 */
Eigen::Vector3d Log(const Eigen::Matrix3d &R)
{
  // 计算旋转角度，trace(R) = 1 + 2cos(θ)
  // 如果trace接近3（单位矩阵），则旋转角为0
  double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  // 从旋转矩阵的反对称部分提取旋转轴信息
  Eigen::Vector3d K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
  // 根据旋转角度大小选择计算方式，避免小角度时的数值不稳定
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/**
 * @brief hat运算符，将3D向量转换为反对称矩阵（叉乘矩阵）
 * @param v 输入的3D向量
 * @return 对应的3x3反对称矩阵
 *
 * 反对称矩阵满足：hat(v) * w = v × w (叉乘)
 * 用于李代数中的旋转操作
 */
Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

/**
 * @brief 右雅可比矩阵（Right Jacobian）
 * @param vec 旋转向量
 * @return 右雅可比矩阵
 *
 * SO(3)的右雅可比矩阵，用于李代数的微分运算
 * Jr = (sin(θ)/θ)I + (1-sin(θ)/θ)aa^T - ((1-cos(θ))/θ)hat(a)
 * 其中θ是旋转角度，a是归一化的旋转轴
 * 当角度很小时返回单位矩阵
 */
Eigen::Matrix3d jr(Eigen::Vector3d vec)
{
  double ang = vec.norm();  // 旋转角度

  if(ang < 1e-9)  // 角度很小时的特殊处理
  {
    return I33;
  }
  else
  {
    vec /= ang;  // 归一化得到旋转轴
    double ra = sin(ang)/ang;
    return ra*I33 + (1-ra)*vec*vec.transpose() - (1-cos(ang))/ang * hat(vec);
  }
}

/**
 * @brief 右雅可比矩阵的逆（Inverse of Right Jacobian）
 * @param rotR 旋转矩阵
 * @return 右雅可比逆矩阵
 *
 * 计算SO(3)右雅可比矩阵的逆，用于状态更新和优化
 * Jr_inv = (θ/2/tan(θ/2))I + (1-θ/2/tan(θ/2))aa^T + (θ/2)hat(a)
 * 当角度很小时返回单位矩阵
 */
Eigen::Matrix3d jr_inv(const Eigen::Matrix3d &rotR)
{
  Eigen::AngleAxisd rot_vec(rotR);  // 将旋转矩阵转换为轴角表示
  Eigen::Vector3d axi = rot_vec.axis();  // 提取旋转轴
  double ang = rot_vec.angle();  // 提取旋转角度

  if(ang < 1e-9)  // 角度很小时的特殊处理
  {
    return I33;
  }
  else
  {
    double ctt = ang / 2 / tan(ang/2);  // 系数计算
    return ctt*I33 + (1-ctt)*axi*axi.transpose() + ang/2 * hat(axi);
  }
}

/**
 * @struct IMUST
 * @brief IMU状态结构体，表示机器人在某一时刻的完整状态
 *
 * 包含位姿、速度、IMU偏差和协方差矩阵等15维状态信息
 * 用于SLAM系统中的状态估计和滤波
 */
struct IMUST
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏，确保SSE优化正常工作
  double t;                        // 时间戳
  Eigen::Matrix3d R;               // 旋转矩阵（姿态）
  Eigen::Vector3d p;               // 位置（3D坐标）
  Eigen::Vector3d v;               // 速度（3D速度向量）
  Eigen::Vector3d bg;              // 陀螺仪偏差
  Eigen::Vector3d ba;              // 加速度计偏差
  Eigen::Vector3d g;               // 重力向量
  Eigen::Matrix<double, DIM, DIM> cov;  // 15x15协方差矩阵

  /**
   * @brief 默认构造函数，将所有状态初始化为零
   */
  IMUST()
  {
    setZero();
  }

  /**
   * @brief 带参数的构造函数
   * @param _t 时间戳
   * @param _R 旋转矩阵
   * @param _p 位置
   * @param _v 速度
   * @param _bg 陀螺仪偏差
   * @param _ba 加速度计偏差
   * @param _g 重力向量（默认为(0,0,-9.8)）
   */
  IMUST(double _t, const Eigen::Matrix3d &_R, const Eigen::Vector3d &_p, const Eigen::Vector3d &_v, const Eigen::Vector3d &_bg, const Eigen::Vector3d &_ba, const Eigen::Vector3d &_g = Eigen::Vector3d(0, 0, -G_m_s2)) : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g) {}

  /**
   * @brief 复合赋值运算符，用于状态更新（加法）
   * @param ist 15维状态增量向量
   * @return 更新后的状态引用
   *
   * 状态增量分布：
   * [0:2]   - 旋转增量（李代数）
   * [3:5]   - 位置增量
   * [6:8]   - 速度增量
   * [9:11]  - 陀螺仪偏差增量
   * [12:14] - 加速度计偏差增量
   */
  IMUST &operator+=(const Eigen::Matrix<double, DIM, 1> &ist)
  {
    this->R = this->R * Exp(ist.block<3, 1>(0, 0));  // 旋转更新（李群乘法）
    this->p += ist.block<3, 1>(3, 0);                // 位置更新
    this->v += ist.block<3, 1>(6, 0);                // 速度更新
    this->bg += ist.block<3, 1>(9, 0);               // 陀螺仪偏差更新
    this->ba += ist.block<3, 1>(12, 0);              // 加速度计偏差更新
    return *this;
  }

  /**
   * @brief 减法运算符，计算两个状态之间的差值
   * @param b 被减状态
   * @return 15维状态差值向量
   *
   * 计算this - b的状态差，旋转部分使用李代数表示
   */
  Eigen::Matrix<double, DIM, 1> operator-(const IMUST &b)
  {
    Eigen::Matrix<double, DIM, 1> a;
    a.block<3, 1>(0, 0) = Log(b.R.transpose() * this->R);  // 旋转差（转换为李代数）
    a.block<3, 1>(3, 0) = this->p - b.p;                   // 位置差
    a.block<3, 1>(6, 0) = this->v - b.v;                   // 速度差
    a.block<3, 1>(9, 0) = this->bg - b.bg;                 // 陀螺仪偏差差
    a.block<3, 1>(12, 0) = this->ba - b.ba;                // 加速度计偏差差
    return a;
  }

  /**
   * @brief 赋值运算符，复制另一个状态
   * @param b 源状态
   * @return 当前状态的引用
   */
  IMUST &operator=(const IMUST &b)
  {
    this->R = b.R;      // 复制旋转
    this->p = b.p;      // 复制位置
    this->v = b.v;      // 复制速度
    this->bg = b.bg;    // 复制陀螺仪偏差
    this->ba = b.ba;    // 复制加速度计偏差
    this->g = b.g;      // 复制重力向量
    this->t = b.t;      // 复制时间戳
    this->cov = b.cov;  // 复制协方差矩阵
    return *this;
  }

  /**
   * @brief 将状态重置为零/初始值
   *
   * 时间戳置0，旋转矩阵设为单位矩阵
   * 位置、速度、IMU偏差均置零
   * 协方差矩阵初始化：
   * - 前9维（旋转、位置、速度）：0.0001 * I
   * - 后6维（陀螺仪和加速度计偏差）：0.00001 * I
   */
  void setZero()
  {
    t = 0; R.setIdentity();  // 时间戳置0，旋转为单位矩阵
    p.setZero(); v.setZero();  // 位置和速度置零
    bg.setZero(); ba.setZero();  // IMU偏差置零
    // g << 0, 0, -G_m_s2;  // 重力向量（已在构造函数中设置）
    cov.setIdentity();  // 协方差初始化为单位矩阵
    cov *= 0.0001;  // 前9维状态的初始协方差
    cov.block<6, 6>(9, 9) = Eigen::Matrix<double, 6, 6>::Identity() * 0.00001;  // 偏差的初始协方差更小
  }

};

/**
 * @brief 基于体素的点云降采样（均值法）
 * @param pl_feat 输入/输出点云，函数会直接修改此点云
 * @param voxel_size 体素大小（米）
 *
 * 将空间划分为规则体素网格，每个体素内的所有点计算平均值作为代表点
 * 使用curvature字段存储体素内点的数量，用于增量计算均值
 * 如果voxel_size < 0.001，则不进行降采样
 */
void down_sampling_voxel(pcl::PointCloud<PointType> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.001) return;  // 体素尺寸太小则跳过

  unordered_map<VOXEL_LOC, PointType> feat_map;  // 体素位置到点的映射
  float loc_xyz[3];
  for(PointType &p_c : pl_feat.points)
  {
    // 计算点所属的体素坐标
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)  // 负坐标需要向下取整
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())  // 该体素首次遇到
    {
      PointType pp = p_c;
      pp.curvature = 1;  // 使用curvature记录点数
      feat_map[position] = pp;
    }
    else  // 该体素已有点，增量更新均值
    {
      PointType &pp = iter->second;
      // 增量计算均值：new_mean = (old_mean * n + new_point) / (n + 1)
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1);
      pp.curvature += 1;  // 点数加1
    }
  }

  // 用降采样后的点替换原点云
  pl_feat.clear();
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
    pl_feat.push_back(iter->second);

}

/**
 * @brief 基于体素的点云降采样（最近点法）
 * @param pl_feat 输入/输出点云，函数会直接修改此点云
 * @param voxel_size 体素大小（米）
 *
 * 与down_sampling_voxel不同，此函数选择距离体素中心最近的实际点作为代表点
 * 而不是计算均值点，这样能保留原始点的特性（如强度、法向量等）
 * 算法步骤：
 * 1. 将点分配到各个体素
 * 2. 计算每个体素内所有点的均值中心
 * 3. 选择距离均值中心最近的点作为该体素的代表点
 */
void down_sampling_close(pcl::PointCloud<PointType> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.001) return;  // 体素尺寸太小则跳过

  // 体素位置到点云指针的映射，存储每个体素内的所有点
  unordered_map<VOXEL_LOC, pcl::PointCloud<PointType>::Ptr> feat_map;
  float loc_xyz[3];
  for(PointType &p_c: pl_feat.points)
  {
    // 计算点所属的体素坐标
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)  // 负坐标需要向下取整
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())  // 该体素首次遇到，创建新点云
    {
      pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>);
      pl_ptr->push_back(p_c);
      feat_map[position] = pl_ptr;
    }
    else  // 该体素已存在，添加点到点云
    {
      iter->second->push_back(p_c);
    }
  }

  // 对每个体素，选择最接近体素中心的点
  pl_feat.clear();
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
  {
    pcl::PointCloud<PointType>::Ptr pl_ptr = iter->second;

    // 计算体素内所有点的均值（作为体素中心）
    PointType pb = pl_ptr->points[0];
    int plsize = pl_ptr->size();
    for(int i=1; i<plsize; i++)
    {
      PointType &pp = pl_ptr->points[i];
      pb.x += pp.x; pb.y += pp.y; pb.z += pp.z;
    }
    pb.x /= plsize; pb.y /=plsize; pb.z /= plsize;

    // 找到距离均值中心最近的点
    double ndis = 100;  // 初始化为较大值
    int mnum = 0;       // 最近点的索引
    for(int i=0; i<plsize; i++)
    {
      PointType &pp = pl_ptr->points[i];
      double xx = pb.x - pp.x;
      double yy = pb.y - pp.y;
      double zz = pb.z - pp.z;
      double dis = xx*xx + yy*yy + zz*zz;  // 欧氏距离平方
      if(dis < ndis)
      {
        mnum = i;
        ndis = dis;
      }
    }

    // 将最接近中心的点添加到结果点云
    pl_feat.push_back(pl_ptr->points[mnum]);
  }

}

/**
 * @class PointCluster
 * @brief 点聚类类，用于增量计算点云的统计信息（均值和协方差）
 *
 * 通过增量方式维护点的和(v)、外积和(P)和数量(N)
 * 可以高效地计算点云的中心和协方差矩阵，常用于平面拟合等应用
 */
class PointCluster
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen内存对齐宏
  Eigen::Matrix3d P;  // 点外积的累加和：Σ(p_i * p_i^T)
  Eigen::Vector3d v;  // 点的累加和：Σ(p_i)
  int N;              // 点的数量

  /**
   * @brief 默认构造函数，初始化为空
   */
  PointCluster()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  /**
   * @brief 清空所有统计数据
   */
  void clear()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  /**
   * @brief 增量添加一个点
   * @param vec 3D点坐标
   *
   * 更新点的数量、累加和和外积累加和
   */
  void push(const Eigen::Vector3d &vec)
  {
    N++;
    P += vec * vec.transpose();  // 累加外积
    v += vec;                    // 累加坐标
  }

  /**
   * @brief 计算点云的协方差矩阵
   * @return 3x3协方差矩阵
   *
   * 使用公式：Cov = E[xx^T] - E[x]E[x]^T = (Σxx^T)/N - (Σx/N)(Σx/N)^T
   */
  Eigen::Matrix3d cov()
  {
    Eigen::Vector3d center = v / N;  // 计算中心点
    return P/N - center*center.transpose();
  }

  /**
   * @brief 复合赋值运算符（加法），合并两个点聚类
   * @param sigv 待合并的点聚类
   * @return 当前点聚类的引用
   *
   * 将另一个点聚类的统计信息合并到当前聚类
   * 用于分布式或增量式的点云处理
   */
  PointCluster & operator+=(const PointCluster &sigv)
  {
    this->P += sigv.P;  // 合并外积和
    this->v += sigv.v;  // 合并坐标和
    this->N += sigv.N;  // 合并点数

    return *this;
  }

  /**
   * @brief 复合赋值运算符（减法），移除一个点聚类
   * @param sigv 待移除的点聚类
   * @return 当前点聚类的引用
   *
   * 从当前聚类中减去另一个聚类的统计信息
   * 用于滑动窗口等场景
   */
  PointCluster & operator-=(const PointCluster &sigv)
  {
    this->P -= sigv.P;  // 减去外积和
    this->v -= sigv.v;  // 减去坐标和
    this->N -= sigv.N;  // 减去点数

    return *this;
  }

  /**
   * @brief 将点聚类在给定的位姿下进行坐标变换
   * @param sigv 源点聚类（局部坐标系）
   * @param stat IMU状态，包含旋转R和平移p
   *
   * 将点从局部坐标系变换到全局坐标系
   * 变换公式：p' = R*p + t
   * 同时更新外积和：P' = R*P*R^T + R*v*t^T + t*v^T*R^T + N*t*t^T
   */
  void transform(const PointCluster &sigv, const IMUST &stat)
  {
    N = sigv.N;
    v = stat.R*sigv.v + N*stat.p;  // 变换点的累加和
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    // 变换外积和，考虑旋转和平移的影响
    P = stat.R*sigv.P*stat.R.transpose() + rp + rp.transpose() + N*stat.p*stat.p.transpose();
  }

};

#endif
