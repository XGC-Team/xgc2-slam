/**
 * @file parameters.cpp
 * @brief Point-LIO系统参数管理模块
 *
 * 本文件负责管理Point-LIO激光-惯性里程计系统的所有全局参数，包括：
 * - 从ROS参数服务器读取配置参数
 * - 维护系统运行时的全局变量
 * - 提供坐标转换和协方差重置等工具函数
 * - 管理传感器外参、噪声模型、优化参数等核心配置
 */

#include "parameters.h"

// ==================== 时间戳和帧管理相关变量 ====================
bool is_first_frame = true;  // 标记是否为第一帧数据
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;  // 激光雷达时间戳：结束时间、首帧时间、时间连续性
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;  // 上一次激光雷达和IMU的时间戳
int pcd_index = 0;  // 点云索引，用于保存点云文件的编号
IVoxType::Options ivox_options_;  // iVox体素化地图配置选项
int ivox_nearby_type = 6;  // iVox邻近体素类型（6/18/26邻域）

// ==================== 外参和状态相关变量 ====================
std::vector<double> extrinT(3, 0.0);  // 激光雷达到IMU的平移外参 [x, y, z]
std::vector<double> extrinR(9, 0.0);  // 激光雷达到IMU的旋转外参（3x3矩阵展开）
state_input state_in;   // 输入状态（用于IMU预测）
state_output state_out; // 输出状态（用于观测更新）
std::string lid_topic, imu_topic;  // 激光雷达和IMU的ROS话题名称

// ==================== 系统功能开关 ====================
bool prop_at_freq_of_imu = true;      // 是否以IMU频率进行状态传播
bool check_satu = true;                // 是否检查IMU饱和
bool con_frame = false;                // 是否连续帧处理
bool cut_frame = false;                // 是否切割帧
bool use_imu_as_input = false;         // 是否使用IMU作为输入
bool space_down_sample = true;         // 是否进行空间降采样
bool publish_odometry_without_downsample = false;  // 是否发布未降采样的里程计

// ==================== 地图和匹配相关参数 ====================
int  init_map_size = 10, con_frame_num = 1;  // 初始地图大小、连续帧数量
double match_s = 81;  // 匹配搜索半径的平方
double satu_acc, satu_gyro;  // 加速度计和陀螺仪的饱和阈值
double cut_frame_time_interval = 0.1;  // 切割帧的时间间隔
float  plane_thr = 0.1f;  // 平面拟合的距离阈值

// ==================== 滤波器参数 ====================
double filter_size_surf_min = 0.5;  // 表面点云滤波器的最小尺寸
double filter_size_map_min = 0.5;   // 地图点云滤波器的最小尺寸
double fov_deg = 180;  // 激光雷达视场角（度）
// double cube_len = 2000;  // 地图立方体边长（已弃用）
float  DET_RANGE = 450;  // 有效检测范围（米）

// ==================== IMU相关参数 ====================
bool   imu_en = true;  // 是否启用IMU
double imu_time_inte = 0.005;  // IMU时间积分步长
double laser_point_cov = 0.01;  // 激光点测量协方差
double acc_norm;  // 加速度范数（用于重力对齐）

// ==================== 噪声协方差参数 ====================
double vel_cov, acc_cov_input, gyr_cov_input;  // 速度、输入加速度、输入角速度的协方差
double gyr_cov_output, acc_cov_output;  // 输出角速度、输出加速度的协方差
double b_gyr_cov, b_acc_cov;  // 陀螺仪偏置、加速度计偏置的协方差
double imu_meas_acc_cov, imu_meas_omg_cov;  // IMU加速度和角速度测量噪声协方差

// ==================== 传感器类型和日志相关 ====================
int    lidar_type, pcd_save_interval;  // 激光雷达类型、点云保存间隔
std::vector<double> gravity_init, gravity;  // 初始重力向量、当前重力向量

// ==================== 发布和日志开关 ====================
bool   runtime_pos_log;  // 是否记录运行时位置日志
bool   pcd_save_en;      // 是否保存点云
bool   path_en;          // 是否发布路径
bool   extrinsic_est_en = true;  // 是否进行外参估计
bool   scan_pub_en, scan_body_pub_en;  // 是否发布扫描点云、是否发布机体坐标系下的点云

// ==================== 预处理和IMU处理对象 ====================
shared_ptr<Preprocess> p_pre;  // 点云预处理对象
shared_ptr<ImuProcess> p_imu;  // IMU处理对象

// ==================== 时间管理变量 ====================
double time_update_last = 0.0;  // 上次更新时间
double time_current = 0.0;  // 当前时间
double time_predict_last_const = 0.0;  // 上次预测时间（常量）
double t_last = 0.0;  // 上一时刻
double time_diff_lidar_to_imu = 0.0;  // 激光雷达到IMU的时间差

// ==================== 激光雷达时间和帧处理参数 ====================
double lidar_time_inte = 0.1;  // 激光雷达时间积分步长
double first_imu_time = 0.0;   // 第一个IMU时间戳
int cut_frame_num = 1;  // 切割帧数量
int orig_odom_freq = 10;  // 原始里程计频率
double online_refine_time = 20.0;  // 在线优化时间（单位：秒）
bool cut_frame_init = false;  // 切割帧初始化标志

// ==================== 测量数据组 ====================
MeasureGroup Measures;  // 存储同步后的激光雷达和IMU测量数据

// ==================== 文件输出流 ====================
ofstream fout_out, fout_imu_pbp;  // 输出文件流：主输出文件、IMU逐点输出文件

/**
 * @brief 从ROS参数服务器读取所有配置参数
 *
 * 该函数从ROS参数服务器读取Point-LIO系统所需的所有参数，包括：
 * - 传感器话题和类型配置
 * - IMU和激光雷达的噪声模型参数
 * - 外参（激光雷达到IMU的变换）
 * - 滤波器和地图参数
 * - 发布选项和日志开关
 *
 * @param nh ROS节点句柄，用于访问参数服务器
 * @return 无返回值
 */
void readParameters(ros::NodeHandle &nh)
{
  // 初始化点云预处理和IMU处理对象
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  // ========== 系统基础配置 ==========
  nh.param<bool>("prop_at_freq_of_imu", prop_at_freq_of_imu, 1);  // 是否以IMU频率传播状态
  nh.param<bool>("use_imu_as_input", use_imu_as_input, 0);  // 是否使用IMU作为输入
  nh.param<bool>("check_satu", check_satu, 1);  // 是否检查IMU饱和
  nh.param<int>("init_map_size", init_map_size, 100);  // 初始化地图大小
  nh.param<bool>("space_down_sample", space_down_sample, 1);  // 是否进行空间降采样

  // ========== 映射和优化参数 ==========
  nh.param<double>("mapping/satu_acc",satu_acc,3.0);  // 加速度计饱和阈值（m/s^2）
  nh.param<double>("mapping/satu_gyro",satu_gyro,35.0);  // 陀螺仪饱和阈值（rad/s）
  nh.param<double>("mapping/acc_norm",acc_norm,1.0);  // 加速度范数（用于重力对齐）
  nh.param<float>("mapping/plane_thr", plane_thr, 0.05f);  // 平面拟合阈值（米）
  nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);  // 点云滤波器采样间隔

  // ========== 传感器话题配置 ==========
  nh.param<std::string>("common/lid_topic",lid_topic,"/livox/lidar");  // 激光雷达话题
  nh.param<std::string>("common/imu_topic", imu_topic,"/livox/imu");  // IMU话题

  // ========== 帧处理配置 ==========
  nh.param<bool>("common/con_frame",con_frame,false);  // 是否连续帧处理
  nh.param<int>("common/con_frame_num",con_frame_num,1);  // 连续帧数量
  nh.param<bool>("common/cut_frame",cut_frame,false);  // 是否切割帧
  nh.param<double>("common/cut_frame_time_interval",cut_frame_time_interval,0.1);  // 切割帧时间间隔（秒）
  nh.param<double>("common/time_diff_lidar_to_imu",time_diff_lidar_to_imu,0.0);  // 激光雷达到IMU的时间偏移

  // ========== 滤波器尺寸参数 ==========
  nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);  // 表面点滤波尺寸（米）
  nh.param<double>("filter_size_map",filter_size_map_min,0.5);  // 地图点滤波尺寸（米）
  // nh.param<double>("cube_side_length",cube_len,2000);  // 地图立方体边长（已弃用）

  // ========== 激光雷达参数 ==========
  nh.param<float>("mapping/det_range",DET_RANGE,300.f);  // 有效检测范围（米）
  nh.param<double>("mapping/fov_degree",fov_deg,180);  // 视场角（度）

  // ========== IMU配置 ==========
  nh.param<bool>("mapping/imu_en",imu_en,true);  // 是否启用IMU
  nh.param<bool>("mapping/extrinsic_est_en",extrinsic_est_en,true);  // 是否估计外参
  nh.param<double>("mapping/imu_time_inte",imu_time_inte,0.005);  // IMU时间积分步长（秒）

  // ========== 测量噪声协方差参数 ==========
  nh.param<double>("mapping/lidar_meas_cov",laser_point_cov,0.1);  // 激光点测量协方差
  nh.param<double>("mapping/acc_cov_input",acc_cov_input,0.1);  // 输入加速度协方差
  nh.param<double>("mapping/vel_cov",vel_cov,20);  // 速度协方差
  nh.param<double>("mapping/gyr_cov_input",gyr_cov_input,0.1);  // 输入角速度协方差
  nh.param<double>("mapping/gyr_cov_output",gyr_cov_output,0.1);  // 输出角速度协方差
  nh.param<double>("mapping/acc_cov_output",acc_cov_output,0.1);  // 输出加速度协方差
  nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);  // 陀螺仪偏置协方差
  nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);  // 加速度计偏置协方差
  nh.param<double>("mapping/imu_meas_acc_cov",imu_meas_acc_cov,0.1);  // IMU加速度测量噪声
  nh.param<double>("mapping/imu_meas_omg_cov",imu_meas_omg_cov,0.1);  // IMU角速度测量噪声

  // ========== 点云预处理参数 ==========
  nh.param<double>("preprocess/blind", p_pre->blind, 1.0);  // 盲区距离（米）
  nh.param<int>("preprocess/lidar_type", lidar_type, 1);  // 激光雷达类型（0:AVIA, 1:Velodyne等）
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);  // 扫描线数
  nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);  // 扫描频率（Hz）
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, 1);  // 时间戳单位（1:s, 1000:ms等）

  // ========== 匹配和外参参数 ==========
  nh.param<double>("mapping/match_s", match_s, 81);  // 匹配搜索半径的平方
  nh.param<std::vector<double>>("mapping/gravity", gravity, std::vector<double>());  // 重力向量
  nh.param<std::vector<double>>("mapping/gravity_init", gravity_init, std::vector<double>());  // 初始重力向量
  nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());  // 外参平移向量
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());  // 外参旋转矩阵

  // ========== 发布选项 ==========
  nh.param<bool>("odometry/publish_odometry_without_downsample", publish_odometry_without_downsample, false);  // 发布未降采样的里程计
  nh.param<bool>("publish/path_en",path_en, true);  // 发布路径
  nh.param<bool>("publish/scan_publish_en",scan_pub_en,1);  // 发布扫描点云
  nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en,1);  // 发布机体坐标系点云

  // ========== 日志和保存选项 ==========
  nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);  // 启用运行时位置日志
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);  // 启用点云保存
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);  // 点云保存间隔

  // ========== 激光雷达时间参数 ==========
  nh.param<double>("mapping/lidar_time_inte",lidar_time_inte,0.1);  // 激光雷达时间积分步长
  nh.param<double>("mapping/lidar_meas_cov",laser_point_cov,0.1);  // 激光测量协方差（重复读取以覆盖）

  // ========== iVox体素地图配置 ==========
  nh.param<float>("mapping/ivox_grid_resolution", ivox_options_.resolution_, 0.2);  // iVox网格分辨率（米）
  nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);  // iVox邻近体素搜索类型

  // 根据邻近类型设置iVox选项
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;  // 仅中心体素
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;  // 6邻域（面邻接）
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;  // 18邻域（面+边邻接）
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;  // 26邻域（面+边+角邻接）
  } else {
    // LOG(WARNING) << "unknown ivox_nearby_type, use NEARBY18";
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;  // 默认使用18邻域
  }

  // 设置IMU处理器的重力向量
  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

/**
 * @brief 将SO(3)旋转矩阵转换为欧拉角（ZYX顺序）
 *
 * 该函数将3x3旋转矩阵转换为欧拉角表示，使用ZYX（yaw-pitch-roll）旋转顺序。
 * 转换公式基于旋转矩阵的分解，处理了万向节锁（gimbal lock）的奇异情况。
 *
 * 旋转顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)
 * - roll (x): 绕X轴旋转（横滚角）
 * - pitch (y): 绕Y轴旋转（俯仰角）
 * - yaw (z): 绕Z轴旋转（偏航角）
 *
 * @param rot 输入的SO(3)旋转矩阵（3x3）
 * @return Eigen::Matrix<double, 3, 1> 欧拉角向量 [roll, pitch, yaw]，单位为弧度
 */
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot)
{
    // 计算sy = sqrt(R[0,0]^2 + R[1,0]^2)，用于判断是否接近万向节锁
    double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));

    // 判断是否为奇异情况（万向节锁）：当pitch接近±90度时，sy接近0
    bool singular = sy < 1e-6;

    double x, y, z;  // 分别对应roll, pitch, yaw

    if(!singular)
    {
        // 非奇异情况：使用标准的欧拉角提取公式
        x = atan2(rot(2, 1), rot(2, 2));   // roll = atan2(R[2,1], R[2,2])
        y = atan2(-rot(2, 0), sy);         // pitch = atan2(-R[2,0], sy)
        z = atan2(rot(1, 0), rot(0, 0));   // yaw = atan2(R[1,0], R[0,0])
    }
    else
    {
        // 奇异情况处理：当pitch接近±90度时
        x = atan2(-rot(1, 2), rot(1, 1));  // roll = atan2(-R[1,2], R[1,1])
        y = atan2(-rot(2, 0), sy);         // pitch = atan2(-R[2,0], sy)
        z = 0;  // yaw设为0（因为此时yaw和roll耦合，无法唯一确定）
    }

    // 构造欧拉角向量并返回
    Eigen::Matrix<double, 3, 1> ang(x, y, z);
    return ang;
}

/**
 * @brief 打开调试输出文件
 *
 * 该函数打开两个用于调试的输出文件：
 * 1. mat_out.txt - 主要的数据输出文件，用于记录系统状态、优化结果等
 * 2. imu_pbp.txt - IMU点对点（point-by-point）数据文件，用于记录IMU处理的详细信息
 *
 * 文件路径通过DEBUG_FILE_DIR宏定义，通常位于项目的调试输出目录。
 *
 * @return 无返回值，但会在控制台输出文件打开状态信息
 */
void open_file()
{
    // 打开主输出文件，用于记录优化和状态估计结果
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);

    // 打开IMU逐点数据文件，用于详细的IMU数据分析
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);

    // 检查文件是否成功打开
    if (fout_out && fout_imu_pbp)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;  // 文件打开成功
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;  // 目录不存在或文件打开失败

}

/**
 * @brief 重置输入状态的协方差矩阵
 *
 * 该函数初始化24维输入状态（state_input）的协方差矩阵P_init。
 * 状态向量维度分配（24维）：
 * - [0:2]   位置 (3维)
 * - [3:5]   速度 (3维)
 * - [6:8]   姿态（旋转向量表示） (3维)
 * - [9:14]  IMU偏置（陀螺仪偏置3维 + 加速度计偏置3维） (6维)
 * - [15:20] 外参（激光雷达到IMU的旋转和平移） (6维)
 * - [21:23] 重力向量 (3维)
 *
 * 不同状态量被赋予不同的初始不确定度：
 * - 大部分状态（位置、速度、姿态等）：0.1
 * - 外参：0.001（较小的不确定度，因为外参相对稳定）
 * - 重力向量：0.0001（非常小的不确定度，重力方向已知）
 *
 * @param P_init 输入/输出参数，24x24的协方差矩阵，将被重置为初始值
 * @return 无返回值
 */
void reset_cov(Eigen::Matrix<double, 24, 24> & P_init)
{
    // 将整个协方差矩阵初始化为0.1倍的单位矩阵
    P_init = MD(24, 24)::Identity() * 0.1;

    // 重力向量部分（索引21-23）使用更小的协方差，因为重力方向已知
    P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;

    // 外参部分（索引15-20）使用中等协方差，外参相对稳定但需要估计
    P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
}

/**
 * @brief 重置输出状态的协方差矩阵
 *
 * 该函数初始化30维输出状态（state_output）的协方差矩阵P_init_output。
 * 输出状态相比输入状态增加了额外的维度用于更精确的状态表示。
 *
 * 状态向量维度分配（30维）：
 * - [0:2]   位置 (3维)
 * - [3:5]   速度 (3维)
 * - [6:11]  姿态（可能使用四元数或其他表示） (6维)
 * - [12:17] IMU偏置（陀螺仪偏置3维 + 加速度计偏置3维） (6维)
 * - [18:23] 其他辅助状态 (6维)
 * - [21:23] 重力向量（在索引21-23） (3维)
 * - [24:29] 外参（激光雷达到IMU的旋转和平移） (6维)
 *
 * 不同状态量的初始不确定度：
 * - 大部分状态：0.01（比输入状态更小，表示输出状态更确定）
 * - 重力向量：0.0001（非常小的不确定度）
 * - 外参：0.001（中等不确定度）
 *
 * @param P_init_output 输入/输出参数，30x30的协方差矩阵，将被重置为初始值
 * @return 无返回值
 */
void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    // 将整个协方差矩阵初始化为0.01倍的单位矩阵
    P_init_output = MD(30, 30)::Identity() * 0.01;

    // 重力向量部分（索引21-23）使用更小的协方差
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;

    // 注释掉的代码：可能是姿态部分的特殊处理，当前未使用
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;

    // 外参部分（索引24-29）使用中等协方差
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}