/*
 * COMMON_LIB.H
 * Swarm-LIO2 公共库 - 定义系统中使用的数据结构、类型别名和工具函数
 * 主要功能：
 * - 定义状态向量结构（位置、姿态、速度、偏差等）
 * - 定义测量数据结构（激光雷达、IMU）
 * - 提供坐标转换和数学工具函数
 * - 支持多无人机集群的外参标定
 */

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <swarm_lio/States.h>
#include <swarm_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <color.h>
#include <boost/filesystem.hpp>
#include "scope_timer.hpp"

using namespace std;
using namespace Eigen;

// ==================== 系统参数定义 ====================
#define MAX_DRONE_ID (40)                     // 集群中最大的飞机编号（支持ID 0-40）
// #define DEBUG_PRINT                         // 调试打印开关
#define DEPLOY                                 // 部署模式标志
#define MAX_UAV_NUM (MAX_DRONE_ID + 1)        // 集群中最大的飞机数量（41架）
#define PI_M (3.14159265358)                  // 圆周率常数
#define G_m_s2 (9.81)                         // 重力加速度常数（广东/中国地区）
#define DIM_STATE (18 + 6 * MAX_UAV_NUM)      // 状态维度：18(自身状态) + 6*N(N架无人机的外参：3旋转+3平移)

// ==================== 算法参数定义 ====================
#define LIDAR_SP_LEN    (2)                   // 激光雷达样条长度参数
#define INIT_COV   (1)                        // 初始协方差值
#define NUM_MATCH_POINTS    (5)               // 平面拟合所需的匹配点数量

// ==================== 宏工具定义 ====================
#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]                                  // 从数组展开为3个向量元素
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]  // 从数组展开为9个矩阵元素
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))      // 调试文件目录路径

// ==================== 类型别名定义 ====================
typedef swarm_lio::Pose6D     Pose6D;             // 6自由度位姿类型（位置+姿态）
typedef pcl::PointXYZINormal PointType;           // 点云点类型（位置+强度+法向量）
typedef pcl::PointXYZRGB     PointTypeRGB;        // RGB点云点类型
typedef pcl::PointCloud<PointType>    PointCloudXYZI;      // 强度点云类型
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;    // RGB点云类型
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;  // 内存对齐的点向量
typedef Vector3d V3D;                             // 3维双精度向量
typedef Matrix3d M3D;                             // 3x3双精度矩阵
const M3D Eye3d(M3D::Identity());                 // 3x3单位矩阵常量
const V3D Zero3d(0, 0, 0);                        // 3维零向量常量
typedef Vector3f V3F;                             // 3维单精度向量

// 动态矩阵和向量类型定义宏
#define MD(a,b)  Matrix<double, (a), (b)>         // a×b 双精度矩阵
#define VD(a)    Matrix<double, (a), 1>           // a维双精度列向量


// 不同激光雷达型号的外参标定参考值
// Vector3d Lidar_offset_to_IMU(0.05512, 0.02226, -0.0297); // Livox Horizon 激光雷达外参
// Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Livox Avia 激光雷达外参

/**
 * @brief 激光雷达类型枚举
 * 支持的激光雷达型号：
 * AVIA(1)   - Livox Avia
 * VELO(2)   - Velodyne系列
 * OUSTER(3) - Ouster系列
 * L515(4)   - Intel RealSense L515
 * PANDAR(5) - Pandar系列
 * SIM(6)    - 仿真激光雷达
 */
enum LID_TYPE{AVIA = 1, VELO, OUSTER, L515, PANDAR, SIM}; //{1, 2, 3, 4, 5, 6}

/**
 * @brief 测量数据组结构 - 存储一帧激光雷达数据及对应的IMU数据
 *
 * 数据同步说明：
 * - lidar_beg_time: 激光雷达帧的起始时间戳
 * - lidar: 点云数据指针
 * - imu: 与该帧点云对应的IMU数据队列
 *
 * 用途：将时间同步的激光雷达和IMU数据打包，便于融合处理
 */
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;                    // 激光雷达帧起始时间
    PointCloudXYZI::Ptr lidar;                // 点云数据智能指针
    deque<sensor_msgs::Imu::ConstPtr> imu;   // IMU数据队列
};

/**
 * @brief 状态组结构 - 存储ESKF滤波器的完整状态向量
 *
 * 状态向量组成（总维度 = 18 + 6*MAX_UAV_NUM）：
 * [0-2]   : 旋转增量（李代数so(3)）
 * [3-5]   : 位置 (世界坐标系)
 * [6-8]   : 速度 (世界坐标系)
 * [9-11]  : 陀螺仪偏差
 * [12-14] : 加速度计偏差
 * [15-17] : 重力向量
 * [18+6i : 18+6i+5] : 第i架无人机的外参（3旋转 + 3平移）
 *
 * 用于多无人机协同SLAM的状态估计和协方差传播
 */
struct StatesGroup
{
    /**
     * @brief 默认构造函数 - 初始化所有状态为零或单位值
     */
    StatesGroup() {
		this->rot_end = M3D::Identity();              // 姿态初始化为单位矩阵
		this->pos_end = Zero3d;                       // 位置初始化为零
        this->vel_end = Zero3d;                       // 速度初始化为零
        this->bias_g  = Zero3d;                       // 陀螺仪偏差初始化为零
        this->bias_a  = Zero3d;                       // 加速度计偏差初始化为零
        this->gravity = Zero3d;                       // 重力向量初始化为零
        // 初始化所有无人机的外参
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            this->global_extrinsic_rot[id] = M3D::Identity();    // 外参旋转初始化为单位矩阵
            this->global_extrinsic_trans[id] = Zero3d;           // 外参平移初始化为零
        }
        // 初始化协方差矩阵
        this->cov.resize(DIM_STATE, DIM_STATE);
        this->cov.setIdentity();
        this->cov *= INIT_COV;                        // 设置初始协方差
        this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;  // IMU偏差和重力的协方差设为较小值
	};

    /**
     * @brief 拷贝构造函数 - 深拷贝另一个状态组
     */
    StatesGroup(const StatesGroup& b) {
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            this->global_extrinsic_rot[id] = b.global_extrinsic_rot[id];
            this->global_extrinsic_trans[id] = b.global_extrinsic_trans[id];
        }
        this->cov = b.cov;
	};

    /**
     * @brief 赋值运算符重载 - 实现状态组的赋值操作
     */
    StatesGroup& operator=(const StatesGroup& b)
	{
        this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            this->global_extrinsic_rot[id] = b.global_extrinsic_rot[id];
            this->global_extrinsic_trans[id] = b.global_extrinsic_trans[id];
        }
        return *this;
	};

    /**
     * @brief 加法运算符重载 - 将误差状态增量添加到当前状态
     * @param state_add 误差状态向量（DIM_STATE维）
     * @return 更新后的新状态
     *
     * ESKF更新公式：
     * - 旋转：R_new = R_old * Exp(δθ)  （李群上的右乘更新）
     * - 向量：v_new = v_old + δv       （向量空间的加法）
     *
     * 用于卡尔曼滤波的状态更新步骤
     */
    StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));  // 旋转更新
		a.pos_end = this->pos_end + state_add.block<3,1>(3,0);      // 位置更新
        a.vel_end = this->vel_end + state_add.block<3,1>(6,0);      // 速度更新
        a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);      // 陀螺仪偏差更新
        a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);     // 加速度计偏差更新
        a.gravity = this->gravity + state_add.block<3,1>(15,0);     // 重力向量更新
        // 更新所有无人机的外参
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            a.global_extrinsic_rot[id] = this->global_extrinsic_rot[id] * Exp(state_add(start_row,0), state_add(start_row+1,0), state_add(start_row+2,0));
            a.global_extrinsic_trans[id] = this->global_extrinsic_trans[id] + state_add.block<3,1>(start_row+3,0);
        }
        a.cov = this->cov;  // 协方差矩阵保持不变
		return a;
	};

    /**
     * @brief 复合赋值运算符重载 - 在当前状态上直接添加误差增量
     * @param state_add 误差状态向量
     * @return 更新后的当前状态引用
     *
     * 就地更新版本，避免创建临时对象
     */
    StatesGroup& operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		this->pos_end += state_add.block<3,1>(3,0);
        this->vel_end += state_add.block<3,1>(6,0);
        this->bias_g  += state_add.block<3,1>(9,0);
        this->bias_a  += state_add.block<3,1>(12,0);
        this->gravity += state_add.block<3,1>(15,0);
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            this->global_extrinsic_rot[id] = this->global_extrinsic_rot[id] * Exp(state_add(start_row,0), state_add(start_row+1,0), state_add(start_row+2,0));
            this->global_extrinsic_trans[id] +=  state_add.block<3,1>(start_row+3,0);
        }
		return *this;
	};

    /**
     * @brief 减法运算符重载 - 计算两个状态之间的误差
     * @param b 另一个状态
     * @return 误差状态向量
     *
     * ESKF误差计算：
     * - 旋转误差：δθ = Log(R_b^T * R_this)  （李代数上的差值）
     * - 向量误差：δv = v_this - v_b         （向量空间的减法）
     *
     * 用于计算预测值与测量值之间的残差
     */
    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
	{
        Matrix<double, DIM_STATE, 1> a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);  // 计算相对旋转
        a.block<3,1>(0,0)  = Log(rotd);                   // 转换为李代数表示
        a.block<3,1>(3,0)  = this->pos_end - b.pos_end;   // 位置差
        a.block<3,1>(6,0)  = this->vel_end - b.vel_end;   // 速度差
        a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;    // 陀螺仪偏差差
        a.block<3,1>(12,0) = this->bias_a  - b.bias_a;    // 加速度计偏差差
        a.block<3,1>(15,0) = this->gravity - b.gravity;   // 重力向量差

        // 计算所有无人机外参的差值
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            M3D global_extrinsic_rotd(b.global_extrinsic_rot[id].transpose() * this->global_extrinsic_rot[id]);
            a.block<3,1>(start_row, 0) = Log(global_extrinsic_rotd);
            a.block<3,1>(start_row+3, 0) = this->global_extrinsic_trans[id] - b.global_extrinsic_trans[id];
        }
		return a;
	};

    // ==================== 状态成员变量 ====================
	M3D rot_end;      // 估计的姿态（旋转矩阵），对应激光雷达帧末尾时刻
    V3D pos_end;      // 估计的位置（世界坐标系），对应激光雷达帧末尾时刻
    V3D vel_end;      // 估计的速度（世界坐标系），对应激光雷达帧末尾时刻
    V3D bias_g;       // 陀螺仪偏差（bias）
    V3D bias_a;       // 加速度计偏差（bias）
    V3D gravity;      // 估计的重力加速度向量
    M3D global_extrinsic_rot[MAX_UAV_NUM];   // 全局外参旋转矩阵数组（本机与其他无人机之间的坐标系变换）
    V3D global_extrinsic_trans[MAX_UAV_NUM]; // 全局外参平移向量数组（本机与其他无人机之间的坐标系变换）
    MatrixXd cov;     // 协方差矩阵（动态大小：DIM_STATE × DIM_STATE）
};

/**
 * @brief 将任意类型转换为字符串
 * @param param_in 输入参数（支持任何可以输出到流的类型）
 * @return 转换后的字符串
 */
template<class T>
string SetString(T &param_in){
    stringstream ss;
    ss << param_in;
    string str = ss.str();
    return str;
}

/**
 * @brief 弧度转角度
 * @param radians 弧度值
 * @return 角度值
 */
template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

/**
 * @brief 角度转弧度
 * @param degrees 角度值
 * @return 弧度值
 */
template<typename T>
T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

/**
 * @brief 构造6自由度位姿对象
 * @param t 时间偏移
 * @param a 加速度向量
 * @param g 角速度向量（陀螺仪）
 * @param v 速度向量
 * @param p 位置向量
 * @param R 旋转矩阵
 * @return Pose6D对象（包含完整的位姿信息）
 *
 * 功能说明：将各个分量组合成一个完整的6自由度位姿消息
 * 应用场景：发布里程计信息、保存轨迹等
 */
template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;  // 设置时间偏移
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);  // 设置加速度
        rot_kp.gyr[i] = g(i);  // 设置角速度
        rot_kp.vel[i] = v(i);  // 设置速度
        rot_kp.pos[i] = p(i);  // 设置位置
        // 将旋转矩阵展平存储（行优先）
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    return move(rot_kp);
}

/**
 * @brief 估计平面法向量
 * @param normvec 输出的归一化法向量
 * @param point 输入的点云数据
 * @param threshold 平面拟合的距离阈值
 * @param point_num 使用的点数量
 * @return 是否成功拟合（所有点到平面距离是否都小于阈值）
 *
 * 数学原理：
 * 平面方程: Ax + By + Cz + D = 0
 * 转换为: (A/D)*x + (B/D)*y + (C/D)*z = -1
 * 求解最小二乘问题: A0*x0 = b0
 * 其中: A0_i = [x_i, y_i, z_i]
 *       x0 = [A/D, B/D, C/D]^T
 *       b0 = [-1, ..., -1]^T
 *
 * 应用场景：从多个点拟合平面，用于点到面的ICP匹配
 */
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    // 构造最小二乘矩阵
    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    // 使用QR分解求解最小二乘问题
    normvec = A.colPivHouseholderQr().solve(b);

    // 验证所有点到平面的距离是否小于阈值
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;  // 存在点超出阈值，拟合失败
        }
    }

    normvec.normalize();  // 归一化法向量
    return true;
}

/**
 * @brief 估计平面参数（包含法向量和距离）
 * @param pca_result 输出的平面参数 [nx, ny, nz, d]，满足 nx*x + ny*y + nz*z + d = 0
 * @param point 输入的点云数据
 * @param threshold 平面拟合的距离阈值
 * @return 是否成功拟合
 *
 * 功能说明：固定使用NUM_MATCH_POINTS个点拟合平面
 * 与esti_normvector的区别：
 * - 本函数返回完整的平面方程参数（法向量+距离）
 * - 点的数量固定为NUM_MATCH_POINTS（5个）
 *
 * 平面方程形式: nx*x + ny*y + nz*z + d = 0
 * 其中: (nx, ny, nz) 为归一化的法向量，d 为原点到平面的有向距离
 */
template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    // 构造最小二乘矩阵
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    // 求解平面参数
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    // 归一化平面参数
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;  // 法向量x分量
    pca_result(1) = normvec(1) / n;  // 法向量y分量
    pca_result(2) = normvec(2) / n;  // 法向量z分量
    pca_result(3) = 1.0 / n;         // 平面距离参数d

    // 验证拟合质量：检查所有点到平面的距离
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;  // 拟合误差超出阈值
        }
    }

    return true;  // 拟合成功
}


#endif
