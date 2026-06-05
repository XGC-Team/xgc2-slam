/**
 * @file common_lib.h
 * @brief Point-LIO系统的公共库头文件
 *
 * 本文件定义了Point-LIO激光惯性里程计系统中使用的所有通用数据结构、类型定义和工具函数，
 * 包括：
 * - IKFoM（Iterated Kalman Filter on Manifold）流形卡尔曼滤波器的状态定义
 * - 点云和IMU数据的相关类型定义
 * - 平面拟合、法向量估计等几何计算函数
 * - 常用的数学常量和宏定义
 */

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include <queue>

using namespace std;
using namespace Eigen;


// ==================== MTK流形类型定义 ====================
// MTK (Manifold ToolKit) 是用于流形空间运算的工具库
typedef MTK::vect<3, double> vect3;  // 三维向量类型（双精度）
typedef MTK::SO3<double> SO3;        // SO(3)旋转群类型，表示三维旋转
typedef MTK::S2<double, 98090, 10000, 1> S2;  // S2球面类型，用于单位向量表示
typedef MTK::vect<1, double> vect1;  // 一维向量类型
typedef MTK::vect<2, double> vect2;  // 二维向量类型

/**
 * @brief 输入状态流形定义（用于ESEKF滤波器的输入状态）
 *
 * 该流形定义了系统的完整状态向量，包括位姿、外参、速度、偏置和重力
 * 总维度：24维（pos:3, rot:3, offset_R_L_I:3, offset_T_L_I:3, vel:3, bg:3, ba:3, gravity:3）
 */
MTK_BUILD_MANIFOLD(state_input,
((vect3, pos))              // 位置（世界坐标系下的IMU位置）
((SO3, rot))                // 旋转（世界坐标系到IMU坐标系的旋转）
((SO3, offset_R_L_I))       // 外参：激光雷达到IMU的旋转外参
((vect3, offset_T_L_I))     // 外参：激光雷达到IMU的平移外参
((vect3, vel))              // 速度（世界坐标系下的IMU速度）
((vect3, bg))               // 陀螺仪零偏
((vect3, ba))               // 加速度计零偏
((vect3, gravity))          // 重力向量（世界坐标系下）
);

/**
 * @brief 输出状态流形定义（用于ESEKF滤波器的输出状态）
 *
 * 相比输入状态，输出状态额外包含角速度和加速度信息
 * 总维度：30维（pos:3, rot:3, offset_R_L_I:3, offset_T_L_I:3, vel:3, omg:3, acc:3, gravity:3, bg:3, ba:3）
 */
MTK_BUILD_MANIFOLD(state_output,
((vect3, pos))              // 位置（世界坐标系下的IMU位置）
((SO3, rot))                // 旋转（世界坐标系到IMU坐标系的旋转）
((SO3, offset_R_L_I))       // 外参：激光雷达到IMU的旋转外参
((vect3, offset_T_L_I))     // 外参：激光雷达到IMU的平移外参
((vect3, vel))              // 速度（世界坐标系下的IMU速度）
((vect3, omg))              // 角速度（IMU坐标系下）
((vect3, acc))              // 加速度（IMU坐标系下）
((vect3, gravity))          // 重力向量（世界坐标系下）
((vect3, bg))               // 陀螺仪零偏
((vect3, ba))               // 加速度计零偏
);

/**
 * @brief IKFoM输入流形定义
 *
 * 定义了卡尔曼滤波器的输入向量，即IMU的测量值
 */
MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))              // 加速度计测量值（包含零偏的原始测量）
((vect3, gyro))             // 陀螺仪测量值（包含零偏的原始测量）
);

/**
 * @brief 输入状态的过程噪声流形定义
 *
 * 定义了输入状态的过程噪声向量
 */
MTK_BUILD_MANIFOLD(process_noise_input,
((vect3, ng))               // 陀螺仪测量噪声
((vect3, na))               // 加速度计测量噪声
((vect3, nbg))              // 陀螺仪零偏随机游走噪声
((vect3, nba))              // 加速度计零偏随机游走噪声
);

/**
 * @brief 输出状态的过程噪声流形定义
 *
 * 定义了输出状态的过程噪声向量，相比输入噪声额外包含速度噪声
 */
MTK_BUILD_MANIFOLD(process_noise_output,
((vect3, vel))              // 速度过程噪声
((vect3, ng))               // 陀螺仪测量噪声
((vect3, na))               // 加速度计测量噪声
((vect3, nbg))              // 陀螺仪零偏随机游走噪声
((vect3, nba))              // 加速度计零偏随机游走噪声
);

// ==================== ESEKF滤波器实例声明 ====================
extern esekfom::esekf<state_input, 24, input_ikfom> kf_input;    // 输入状态的ESEKF滤波器（24维状态）
extern esekfom::esekf<state_output, 30, input_ikfom> kf_output;  // 输出状态的ESEKF滤波器（30维状态）

// ==================== 进度条显示相关宏 ====================
#define PBWIDTH 30  // 进度条宽度
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"  // 进度条字符串

// ==================== 系统常量定义 ====================
#define PI_M (3.14159265358)     // 圆周率
// #define G_m_s2 (9.81)         // 重力加速度（广东/中国地区）
#define DIM_STATE (24)           // 状态维度（SO(3)的维度记为3）
#define DIM_PROC_N (12)          // 过程噪声维度（SO(3)的维度记为3）
#define CUBE_LEN  (6.0)          // 立方体边长（用于ikdtree的体素大小）
#define LIDAR_SP_LEN    (2)      // 激光雷达扫描分割长度
#define INIT_COV   (0.0001)      // 初始协方差值
#define NUM_MATCH_POINTS    (5)  // 平面拟合使用的匹配点数量
#define MAX_MEAS_DIM        (10000)  // 最大测量维度

// ==================== 工具宏定义 ====================
#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]  // 从数组提取3元素作为参数
#define VEC_FROM_ARRAY_SIX(v)    v[0],v[1],v[2],v[3],v[4],v[5]  // 从数组提取6元素作为参数
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]  // 从数组提取9元素（3x3矩阵）作为参数
#define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)  // 数值约束到[min,max]范围
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()  // Eigen矩阵转换为数组范围
#define STD_VEC_FROM_EIGEN(mat)  vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())  // Eigen矩阵转换为std::vector
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))  // 调试文件路径生成宏

// ==================== PCL点云类型定义 ====================
typedef pcl::PointXYZINormal PointType;        // 带强度和法向量的点类型（用于LiDAR数据）
typedef pcl::PointXYZRGB     PointTypeRGB;     // 带RGB颜色的点类型（用于可视化）
typedef pcl::PointCloud<PointType>    PointCloudXYZI;     // 强度点云类型
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;   // 彩色点云类型
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;  // 点向量（使用Eigen内存对齐）

// ==================== Eigen类型简写定义 ====================
typedef Vector3d V3D;  // 三维双精度向量
typedef Matrix3d M3D;  // 三阶双精度矩阵
typedef Vector3f V3F;  // 三维单精度向量
typedef Matrix3f M3F;  // 三阶单精度矩阵

// ==================== Eigen矩阵/向量类型生成宏 ====================
#define MD(a,b)  Matrix<double, (a), (b)>  // 生成双精度矩阵类型
#define VD(a)    Matrix<double, (a), 1>    // 生成双精度列向量类型
#define MF(a,b)  Matrix<float, (a), (b)>   // 生成单精度矩阵类型
#define VF(a)    Matrix<float, (a), 1>     // 生成单精度列向量类型

// ==================== 常用常量定义 ====================
const M3D Eye3d(M3D::Identity());  // 三阶双精度单位矩阵
const M3F Eye3f(M3F::Identity());  // 三阶单精度单位矩阵
const V3D Zero3d(0, 0, 0);         // 三维双精度零向量
const V3F Zero3f(0, 0, 0);         // 三维单精度零向量

/**
 * @brief 测量数据组结构体
 *
 * 用于存储当前处理帧的激光雷达数据和对应时间段内的IMU数据
 * 该结构体将LiDAR扫描和IMU测量数据打包在一起进行融合处理
 */
struct MeasureGroup
{
    /**
     * @brief 构造函数，初始化时间戳和点云指针
     */
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        lidar_last_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };

    double lidar_beg_time;                      // 激光雷达扫描开始时间戳
    double lidar_last_time;                     // 激光雷达扫描结束时间戳
    PointCloudXYZI::Ptr lidar;                  // 激光雷达点云数据指针
    deque<sensor_msgs::Imu::ConstPtr> imu;     // IMU数据队列（存储该扫描周期内的所有IMU测量）
};

/**
 * @brief 计算两个点之间的欧氏距离平方
 *
 * @tparam T 返回值类型（通常为float或double）
 * @param p1 第一个点
 * @param p2 第二个点
 * @return 两点之间的距离平方
 */
template <typename T>
T calc_dist(PointType p1, PointType p2){
    T d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

/**
 * @brief 计算Eigen向量和PCL点之间的欧氏距离平方
 *
 * @tparam T 返回值类型（通常为float或double）
 * @param p1 Eigen三维向量
 * @param p2 PCL点
 * @return 向量和点之间的距离平方
 */
template <typename T>
T calc_dist(Eigen::Vector3d p1, PointType p2){
    T d = (p1(0) - p2.x) * (p1(0) - p2.x) + (p1(1) - p2.y) * (p1(1) - p2.y) + (p1(2) - p2.z) * (p1(2) - p2.z);
    return d;
}

/**
 * @brief 时间压缩函数 - 将点云按时间戳分组
 *
 * 该函数利用点云中curvature字段存储的时间信息，将点云分割成多个时间段
 * 在Point-LIO中，curvature字段被重用来存储点的相对时间戳
 * 当检测到时间戳跳变（下一个点的时间戳大于当前点）时，认为进入了新的时间段
 *
 * @tparam T 模板类型参数（未使用）
 * @param point_cloud 输入点云指针
 * @return 每个时间段内的点数序列
 *
 * @note 返回的vector中，每个元素表示对应时间段内的点数
 * @note 这种分组方式用于增量式的点云处理和运动补偿
 */
template<typename T>
std::vector<int> time_compressing(const PointCloudXYZI::Ptr &point_cloud)
{
  int points_size = point_cloud->points.size();
  int j = 0;
  std::vector<int> time_seq;
  // time_seq.clear();
  time_seq.reserve(points_size);
  for(int i = 0; i < points_size - 1; i++)
  {
    j++;
    if (point_cloud->points[i+1].curvature > point_cloud->points[i].curvature)
    {
      time_seq.emplace_back(j);  // 记录当前时间段的点数
      j = 0;  // 重置计数器，开始新时间段
    }
  }
//   if (j == 0)
//   {
//     time_seq.emplace_back(1);
//   }
//   else
  {
    time_seq.emplace_back(j+1);  // 添加最后一个时间段的点数
  }
  return time_seq;
}

/**
 * @brief 估计平面法向量
 *
 * 通过最小二乘法从一组点中估计平面的法向量
 *
 * 原理说明：
 * 平面方程: Ax + By + Cz + D = 0
 * 转换为: A/D*x + B/D*y + C/D*z = -1
 * 求解线性方程组: A0*x0 = b0
 * 其中 A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
 * normvec: 归一化后的 x0
 *
 * @tparam T 数据类型（float或double）
 * @param normvec 输出参数，估计得到的单位法向量
 * @param point 输入点集
 * @param threshold 点到平面距离的阈值，用于验证平面拟合质量
 * @param point_num 参与拟合的点数
 * @return 如果所有点到拟合平面的距离都小于阈值，返回true；否则返回false
 */
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    // 构造超定方程组的系数矩阵和右端项
    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    // 使用列主元QR分解求解最小二乘问题
    normvec = A.colPivHouseholderQr().solve(b);

    // 验证拟合质量：检查所有点到平面的距离
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;  // 有点偏离平面过远，拟合失败
        }
    }

    normvec.normalize();  // 归一化为单位法向量
    return true;
}

/**
 * @brief 估计平面参数（包含法向量和距离）
 *
 * 使用固定数量的匹配点（NUM_MATCH_POINTS=5）通过最小二乘法估计平面参数
 * 平面方程表示为: nx*x + ny*y + nz*z + d = 0
 * 其中 (nx, ny, nz) 是单位法向量，d 是原点到平面的有向距离
 *
 * @tparam T 数据类型（float或double）
 * @param pca_result 输出参数，四维向量 [nx, ny, nz, d]，表示平面的法向量和距离
 * @param point 输入点集（至少包含NUM_MATCH_POINTS个点）
 * @param threshold 点到平面距离的阈值，用于验证拟合质量
 * @return 如果所有点到拟合平面的距离都小于阈值，返回true；否则返回false
 *
 * @note 该函数用于Point-LIO中的点到面ICP配准
 */
template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    // 构造超定方程组
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    // 求解最小二乘问题得到法向量（未归一化）
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    // 归一化并构造平面参数
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;  // 归一化法向量x分量
    pca_result(1) = normvec(1) / n;  // 归一化法向量y分量
    pca_result(2) = normvec(2) / n;  // 归一化法向量z分量
    pca_result(3) = 1.0 / n;         // 原点到平面的有向距离

    // 验证拟合质量
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;  // 拟合误差超过阈值
        }
    }
    return true;
}

// ==================== 注释掉的代码 ====================
// 以下是按时间排序的比较函数（已注释，未使用）
// const bool time_list(PointType &x, PointType &y); // {return (x.curvature < y.curvature);};
// template<typename T>
// const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

#endif  // COMMON_LIB_H