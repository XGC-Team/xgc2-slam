# FAST-LIO 学习笔记


## laserMapping.cpp 主函数分析

### 主函数工作流程概述

main 函数位于 `/home/lxk/Paper/motion-planning/src/FAST_LIO/src/laserMapping.cpp:756`，主要完成以下几个部分的工作：

### 1. 初始化阶段

#### 1.1 ROS节点初始化
```cpp
ros::init(argc, argv, "laserMapping");   // 初始化ROS节点，节点名为"laserMapping"
ros::NodeHandle nh;                       // 创建节点句柄，用于参数读取和话题订阅/发布
```

#### 1.2 参数配置
从 ROS 参数服务器读取配置参数：

```cpp
// ========== 发布控制参数 ==========
nh.param<bool>("publish/path_en",path_en, true);                      // 是否发布路径轨迹
nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);          // 是否发布配准后的点云
nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);        // 是否发布稠密点云(未降采样)
nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true); // 是否发布机体坐标系点云

// ========== 算法核心参数 ==========
nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);                  // 迭代卡尔曼滤波最大迭代次数
nh.param<string>("map_file_path",map_file_path,"");                   // 地图文件保存路径

// ========== 话题配置 ==========
nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");        // LiDAR点云话题名
nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");         // IMU数据话题名
nh.param<bool>("common/time_sync_en", time_sync_en, false);           // 是否启用时间同步
nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0); // LiDAR与IMU时间偏移

// ========== 滤波器参数 ==========
nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);    // 角点体素滤波器大小(m)
nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);        // 平面点体素滤波器大小(m)
nh.param<double>("filter_size_map",filter_size_map_min,0.5);          // 地图体素滤波器大小(m)

// ========== 地图参数 ==========
nh.param<double>("cube_side_length",cube_len,200);                    // 局部地图立方体边长(m)
nh.param<float>("mapping/det_range",DET_RANGE,300.f);                 // 有效检测范围(m)
nh.param<double>("mapping/fov_degree",fov_deg,180);                   // 视场角(度)

// ========== IMU噪声参数 ==========
nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);                      // 陀螺仪测量噪声协方差
nh.param<double>("mapping/acc_cov",acc_cov,0.1);                      // 加速度计测量噪声协方差
nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);              // 陀螺仪偏差随机游走噪声
nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);              // 加速度计偏差随机游走噪声

// ========== 预处理参数 ==========
nh.param<double>("preprocess/blind", p_pre->blind, 0.01);             // 雷达近距离盲区(m)
nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);             // 雷达类型(AVIA/VELO/OUST等)
nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);            // 雷达扫描线数
nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);     // 时间戳单位(US/MS)
nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);          // 扫描频率(Hz)
nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);        // 点云滤波最少点数阈值
nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false); // 是否启用特征提取

// ========== 调试与日志 ==========
nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);         // 是否记录运行时位置日志
nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);   // 是否在线估计外参
nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);           // 是否保存PCD文件
nh.param<int>("pcd_save/interval", pcd_save_interval, -1);            // PCD保存间隔(扫描数)

// ========== 外参标定参数 ==========
nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); // LiDAR到IMU平移向量
nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); // LiDAR到IMU旋转矩阵(行优先)
```

##### 通过 mapping_mid360.launch 启动时的参数设置

当使用 `roslaunch fast_lio mapping_mid360.launch` 启动时，会覆盖以下参数：

**Launch文件直接设置的参数:**
```xml
<param name="feature_extract_enable" type="bool" value="0"/>     # 关闭特征提取
<param name="point_filter_num" type="int" value="3"/>            # 点云滤波阈值设为3
<param name="max_iteration" type="int" value="3"/>               # 迭代次数设为3(比默认4少)
<param name="filter_size_surf" type="double" value="0.5"/>       # 平面点滤波器0.5m
<param name="filter_size_map" type="double" value="0.5"/>        # 地图滤波器0.5m
<param name="cube_side_length" type="double" value="1000"/>      # 局部地图1000m(比默认200大)
<param name="runtime_pos_log_enable" type="bool" value="0"/>     # 关闭运行时日志
```

**通过 mid360.yaml 配置文件设置的参数:**
```yaml
# 话题配置
common/lid_topic: "/livox/lidar"           # LiDAR话题
common/imu_topic: "/livox/imu"             # IMU话题
common/time_sync_en: false                 # 不启用时间同步
common/time_offset_lidar_to_imu: 0.0       # 时间偏移设为0

# 预处理参数 (针对MID360雷达)
preprocess/lidar_type: 1                   # Livox系列雷达
preprocess/scan_line: 4                    # 4线扫描(MID360特性)
preprocess/blind: 0.5                       # 盲区0.5m

# IMU噪声参数
mapping/acc_cov: 0.1                       # 加速度计协方差
mapping/gyr_cov: 0.1                       # 陀螺仪协方差
mapping/b_acc_cov: 0.0001                  # 加速度计偏差协方差
mapping/b_gyr_cov: 0.0001                  # 陀螺仪偏差协方差

# 地图参数
mapping/fov_degree: 360                    # 360度全向视场(MID360特性)
mapping/det_range: 100.0                   # 检测范围100m(比默认300m小)
mapping/extrinsic_est_en: false            # 不在线估计外参(使用标定值)

# MID360外参标定值
mapping/extrinsic_T: [-0.011, -0.02329, 0.04412]  # 平移向量
mapping/extrinsic_R: [1,0,0, 0,1,0, 0,0,1]        # 单位旋转矩阵

# 发布设置
publish/path_en: false                     # 不发布路径
publish/scan_publish_en: true              # 发布点云
publish/dense_publish_en: true             # 发布稠密点云
publish/scan_bodyframe_pub_en: true        # 发布机体坐标系点云

# PCD保存
pcd_save/pcd_save_en: true                 # 启用PCD保存
pcd_save/interval: -1                      # 保存所有帧到一个文件
```

**关键差异:**
- MID360配置针对360°全向扫描优化(FOV=360°)
- 检测范围减小到100m以提高实时性
- 局部地图扩大到1000m以适应大场景
- 使用预标定的外参而非在线估计
- 关闭路径发布以减少计算负担

#### 1.3 系统组件初始化

```cpp
// ========== 预处理器配置 ==========
p_pre->lidar_type = lidar_type;                          // 设置雷达类型

// ========== 路径消息初始化 ==========
path.header.stamp    = ros::Time::now();                 // 设置路径时间戳
path.header.frame_id ="camera_init";                     // 设置坐标系为camera_init

// ========== 变量定义 ==========
int effect_feat_num = 0, frame_num = 0;                  // 有效特征数、帧数计数器
double deltaT, deltaR, aver_time_consu = 0, ...;         // 时间统计变量
bool flg_EKF_converged, EKF_stop_flg = 0;                // EKF收敛标志

// ========== 视场角计算 ==========
FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);  // 限制视场角最大179.9度
HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);             // 计算半视场角余弦值

// ========== 点云容器初始化 ==========
_featsArray.reset(new PointCloudXYZI());                 // 初始化特征点云数组

// ========== 数组初始化 ==========
memset(point_selected_surf, true, sizeof(point_selected_surf));  // 选中点标记初始化为true
memset(res_last, -1000.0f, sizeof(res_last));                   // 残差数组初始化为-1000

// ========== 体素滤波器设置 ==========
downSizeFilterSurf.setLeafSize(filter_size_surf_min, ...);      // 设置平面点体素滤波器
downSizeFilterMap.setLeafSize(filter_size_map_min, ...);        // 设置地图体素滤波器

// ========== 外参矩阵转换 ==========
Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);               // 将外参平移向量转为Eigen格式
Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);               // 将外参旋转矩阵转为Eigen格式

// ========== IMU处理器配置 ==========
p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU); // 设置LiDAR-IMU外参
p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));     // 设置陀螺仪协方差(对角阵)
p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));     // 设置加速度计协方差
p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, ...));           // 设置陀螺仪偏差协方差
p_imu->set_acc_bias_cov(V3D(b_acc_cov, ...));           // 设置加速度计偏差协方差
p_imu->lidar_type = lidar_type;                         // 设置雷达类型

// ========== 扩展卡尔曼滤波器初始化 ==========
double epsi[23] = {0.001};                              // 收敛阈值数组
fill(epsi, epsi+23, 0.001);                             // 全部设为0.001
kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model,   // 初始化ESEKF
                  NUM_MAX_ITERATIONS, epsi);             // 传入函数指针和参数

// ========== 调试日志文件 ==========
FILE *fp;
string pos_log_dir = root_dir + "/Log/pos_log.txt";     // 位置日志路径
fp = fopen(pos_log_dir.c_str(),"w");                    // 打开位置日志文件

ofstream fout_pre, fout_out, fout_dbg;                  // 调试输出流
fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);  // 预测状态日志
fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);  // 输出状态日志
fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);      // 调试信息日志
```

**关键组件说明：**
- **p_pre**: 点云预处理器，处理原始雷达数据
- **p_imu**: IMU处理器，负责IMU预积分和状态预测
- **kf**: 迭代误差状态卡尔曼滤波器(IESEKF)，FAST-LIO核心算法
- **体素滤波器**: 降采样以减少计算量
- **外参**: LiDAR到IMU的刚体变换关系

#### 1.4 ROS通信设置

```cpp
// ========== ROS 订阅器 ==========
ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
    nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \         // AVIA雷达使用livox回调
    nh.subscribe(lid_topic, 200000, standard_pcl_cbk);         // 其他雷达使用标准回调
ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);  // IMU订阅器

// ========== ROS 发布器 ==========
ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_registered", 100000);                         // 世界坐标系配准点云

ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_registered_body", 100000);                    // 机体坐标系配准点云

ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_effected", 100000);                           // 有效特征点云(用于配准的点)

ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
        ("/Laser_map", 100000);                                // 局部地图点云

ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
        ("/Odometry", 100000);                                 // 里程计(位姿+协方差)

ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
        ("/path", 100000);                                     // 运动轨迹路径
```

**关键设计点：**
- **大缓冲区**: 队列大小设为200000/100000，避免高频数据丢失
- **条件编译**: 根据雷达类型选择不同的回调函数处理不同数据格式
- **多种输出**: 提供多个坐标系下的点云供不同用途使用
- **回调函数**:
  - `livox_pcl_cbk`: 处理Livox自定义消息格式（CustomMsg）
  - `standard_pcl_cbk`: 处理标准PointCloud2格式
  - `imu_cbk`: 处理IMU数据并进行时间同步

**MID360使用的回调函数**：
- MID360配置中 `lidar_type: 1`，对应 `AVIA` 类型
- 因此MID360使用 **`livox_pcl_cbk`** 回调函数
- 该回调处理Livox特有的 `CustomMsg` 消息格式，包含点云和精确时间戳信息

#### 1.5 信号处理函数注册

```cpp
signal(SIGINT, SigHandle);   // 注册SIGINT信号处理函数
```

**信号处理函数实现：**
```cpp
void SigHandle(int sig)
{
    flg_exit = true;                    // 设置退出标志
    ROS_WARN("catch sig %d", sig);      // 打印警告信息
    sig_buffer.notify_all();            // 通知所有等待的线程
}
```

**功能说明：**
- **SIGINT信号**：当用户按下 `Ctrl+C` 时触发
- **优雅退出**：不直接终止程序，而是设置标志让主循环自然退出
- **线程同步**：通过条件变量通知其他可能阻塞的线程
- **资源清理**：允许程序在退出前保存地图、关闭文件等

**标志位检查位置：**
```cpp
while (status)
{
    if (flg_exit) break;    // 检查退出标志，如果为true则跳出循环
    ros::spinOnce();
    // ... 其他处理
}
```

**设计特点：**
- 避免强制终止导致的数据丢失
- 确保PCD文件正确保存
- 允许日志文件正常关闭
- 保证内存和资源正确释放

### 2. 核心循环 while(status)

循环以 5000Hz 的频率运行，执行以下核心任务：

#### 2.1 ROS回调处理 (ros::spinOnce)

##### 2.1.1 功能说明

```cpp
ros::spinOnce();  // 处理所有待处理的回调函数
```

- 处理ROS消息队列中的所有待处理事件
- 非阻塞调用，处理完当前队列立即返回
- 触发订阅器的回调函数（LiDAR点云回调 + IMU数据回调）

##### 2.1.2 互斥锁与缓冲区机制

**缓冲区定义：**

```cpp
// 全局变量定义（文件开头，第81-106行）
mutex mtx_buffer;                                    // 互斥锁
condition_variable sig_buffer;                       // 条件变量
deque<double>                     time_buffer;       // 时间戳缓冲区
deque<PointCloudXYZI::Ptr>        lidar_buffer;      // LiDAR点云缓冲区
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;        // IMU数据缓冲区
```

**互斥锁的使用分析：**
- **保护目标**：三个缓冲区（time_buffer、lidar_buffer、imu_buffer）
- **临界区**：
  - 回调函数中：向缓冲区添加数据
  - sync_packages中：从缓冲区读取和删除数据
- **实际情况**：由于使用`ros::spinOnce()`而非多线程Spinner，所有操作都在**主线程串行执行**
- **结论**：互斥锁在当前实现中是**冗余的**，可能是防御性编程或历史遗留

**条件变量 sig_buffer 的使用分析：**
- **notify_all()调用位置**：
  - SigHandle信号处理（第147行）
  - 三个回调函数结束时（第297、333、363行）
- **wait()调用**：**代码中没有任何wait调用**！
- **结论**：`sig_buffer.notify_all()`完全**无效**，因为没有线程在等待
- **推测**：可能是多线程版本的遗留代码，或为未来扩展预留

**执行流程（单线程）：**
```
主线程循环：
1. ros::spinOnce() → 串行执行所有回调
2. sync_packages() → 同步处理数据
3. 无并发，无需锁保护
```

##### 2.1.3 LiDAR点云回调 (livox_pcl_cbk)

**函数签名：**
```cpp
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
```

**完整实现：**
```cpp
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();                                              // 加锁（实际冗余）
    double preprocess_start_time = omp_get_wtime();                 // 记录预处理开始时间
    scan_count++;                                                    // 扫描帧计数

    // ===== 时间戳检查，防止数据乱序 =====
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");                 // 检测到时间倒退
        lidar_buffer.clear();                                        // 清空缓冲区
    }
    last_timestamp_lidar = msg->header.stamp.toSec();               // 更新最新时间戳

    // ===== 时间同步诊断信息 =====
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0
        && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
               last_timestamp_imu, last_timestamp_lidar);            // 警告：时间差超过10秒
    }

    // ===== 自动时间同步（首次） =====
    if (time_sync_en && !timediff_set_flg
        && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;                                     // 标记已计算时间偏移
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    // ===== 点云预处理 =====
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);                                        // 预处理：格式转换、去畸变
    lidar_buffer.push_back(ptr);                                     // 存入点云缓冲队列
    time_buffer.push_back(last_timestamp_lidar);                     // 存入时间戳队列

    // 详见下方"点云预处理详解"章节

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time; // 记录预处理耗时
    mtx_buffer.unlock();                                             // 解锁（实际冗余）
    sig_buffer.notify_all();                                         // 通知（实际无效）
}
```

**执行步骤分解：**

###### 时间戳检查与乱序处理

检测时间戳倒退，防止数据乱序：
```cpp
if (msg->header.stamp.toSec() < last_timestamp_lidar)
{
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
}
last_timestamp_lidar = msg->header.stamp.toSec();
```

###### 时间同步诊断

关闭自动同步时，检测时间差异并发出警告：
```cpp
if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0
    && !imu_buffer.empty() && !lidar_buffer.empty())
{
    printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
           last_timestamp_imu, last_timestamp_lidar);
}
```

###### 自动时间同步计算

首次检测到时间差>1秒时，计算偏移量：
```cpp
if (time_sync_en && !timediff_set_flg
    && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
{
    timediff_set_flg = true;
    timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
    printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
}
```

###### 点云预处理调用

```cpp
PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
p_pre->process(msg, ptr);                      // 预处理：格式转换、去畸变
lidar_buffer.push_back(ptr);                   // 存入点云缓冲队列
time_buffer.push_back(last_timestamp_lidar);   // 存入时间戳队列
```

###### 点云预处理详解

在 LiDAR 回调函数中执行的 `p_pre->process(msg, ptr)` 是点云预处理的核心步骤。

##### 2.1.4 IMU数据回调 (imu_cbk)

**函数签名：**
```cpp
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
```

**完整实现：**
```cpp
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;                                                    // 发布计数

    // ===== 复制IMU消息（避免修改原始数据） =====
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // ===== 时间戳校正（方案1：手动配置偏移） =====
    msg->header.stamp = ros::Time().fromSec(
        msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);         // 应用手动偏移

    // ===== 时间戳校正（方案2：自动计算偏移，优先级更高） =====
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
        // 注：如果启用自动同步且偏移>0.1秒，覆盖上面的手动偏移
    }

    double timestamp = msg->header.stamp.toSec();                       // 获取校正后的时间戳

    mtx_buffer.lock();                                                  // 加锁（实际冗余）

    // ===== 时间戳检查，防止数据乱序 =====
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");                        // 检测到时间倒退
        imu_buffer.clear();                                              // 清空缓冲区
    }

    last_timestamp_imu = timestamp;                                     // 更新最新时间戳
    imu_buffer.push_back(msg);                                          // 存入IMU缓冲队列

    mtx_buffer.unlock();                                                // 解锁（实际冗余）
    sig_buffer.notify_all();                                            // 通知（实际无效）
}
```

**执行步骤分解：**

###### 时间戳校正：手动配置模式

应用手动配置的偏移量（从ROS参数 `time_offset_lidar_to_imu` 读取）：
```cpp
sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
msg->header.stamp = ros::Time().fromSec(
    msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
```

###### 时间戳校正：自动同步模式

如果启用自动同步且计算出有效偏移量，覆盖手动配置：
```cpp
if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
{
    msg->header.stamp = \
    ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
}
```

###### 时间戳检查与缓冲

检测时间倒退并将数据存入缓冲区：
```cpp
double timestamp = msg->header.stamp.toSec();

mtx_buffer.lock();
if (timestamp < last_timestamp_imu)
{
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer.clear();
}
last_timestamp_imu = timestamp;
imu_buffer.push_back(msg);
mtx_buffer.unlock();
```

###### 时间戳校正逻辑总结

- **MID360配置** (`time_sync_en=false, offset=0.0`):
  - 执行: `timestamp = 原始值 - 0.0`
  - 跳过自动同步代码块
  - 结果: 保持原始时间戳

- **自动同步配置** (`time_sync_en=true`):
  - 首次执行手动偏移（通常为0）
  - 如果`timediff_lidar_wrt_imu > 0.1`，覆盖为自动计算值
  - 执行: `timestamp = 原始值 + timediff_lidar_wrt_imu`
  - 结果: 所有IMU时间戳调整到LiDAR时间基准

详见 [`timestamp_synchronization_analysis.md`](./timestamp_synchronization_analysis.md)

**PointCloudXYZI 类型定义**

位于 `/home/lxk/Paper/motion-planning/src/FAST_LIO/include/common_lib.h:38`:

```cpp
typedef pcl::PointXYZINormal PointType;           // 单点类型
typedef pcl::PointCloud<PointType> PointCloudXYZI; // 点云类型
```

- **基础类型**：`pcl::PointXYZINormal` 包含以下字段：
  - `x, y, z`: 3D坐标
  - `intensity`: 反射强度
  - `normal_x, normal_y, normal_z`: 法向量（用于存储附加信息）
  - `curvature`: 曲率（**被复用存储时间戳**）

- **智能指针封装**：
  ```cpp
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());  // 创建共享指针
  ```

**Preprocess 类处理流程**

位于 `/home/lxk/Paper/motion-planning/src/FAST_LIO/src/preprocess.cpp:44-48`:

```cpp
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
    avia_handler(msg);        // 调用 Livox 专用处理函数
    *pcl_out = pl_surf;       // 输出平面点云
}
```

**Livox 雷达处理 (avia_handler)**

针对 **MID360 配置**（`feature_enabled=false, point_filter_num=3`），执行以下步骤：

```cpp
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pl_surf.clear();   // 清空上一帧平面点云
    pl_full.clear();   // 清空上一帧完整点云

    int plsize = msg->point_num;  // 获取点数
    pl_full.resize(plsize);       // 预分配内存

    uint valid_num = 0;

    // ===== 非特征提取模式（MID360默认） =====
    for(uint i=1; i<plsize; i++)
    {
        // 1) 质量检查：线束号有效 && 点状态正常
        if((msg->points[i].line < N_SCANS) &&
           ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num++;

            // 2) 点云降采样：每 point_filter_num 个点保留一个
            if (valid_num % point_filter_num == 0)  // MID360: 每3个点保留1个
            {
                // 3) 坐标转换：CustomMsg -> PointXYZINormal
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time / 1000000.0;  // 微秒->秒

                // 4) 去重检查：与前一点位置不同
                if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) ||
                    (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7) ||
                    (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)) &&

                   // 5) 盲区滤除：距离 > blind（MID360: 0.5m）
                   (pl_full[i].x*pl_full[i].x + pl_full[i].y*pl_full[i].y +
                    pl_full[i].z*pl_full[i].z > (blind * blind)))
                {
                    pl_surf.push_back(pl_full[i]);  // 加入输出点云
                }
            }
        }
    }
}
```

**预处理步骤总结**

| 步骤 | 操作 | MID360 参数 | 作用 |
|------|------|-------------|------|
| 1 | **质量过滤** | `tag & 0x30` 检查 | 去除异常点（遮挡、噪声等） |
| 2 | **降采样** | `point_filter_num=3` | 保留1/3点云，提高实时性 |
| 3 | **格式转换** | CustomMsg→PointXYZINormal | 统一点云数据结构 |
| 4 | **去重** | 阈值 `1e-7` | 去除静止时的重复点 |
| 5 | **盲区滤除** | `blind=0.5m` | 去除近距离无效数据 |
| 6 | **时间戳转换** | `offset_time/1e6` | 微秒→秒，存入curvature字段 |

**特征提取模式（可选）**

当 `feature_enabled=true` 时，会执行更复杂的处理：
- **平面检测** (`plane_judge`): 识别平面区域
- **边缘检测** (`edge_jump_judge`): 检测深度跳变边缘
- **特征分类**: 区分 Real_Plane、Edge_Jump、Edge_Plane、Wire 等
- **输出**: `pl_surf`（平面点）和 `pl_corn`（角点）

**输出点云特点**

- **稀疏但均匀**: 1/3采样率保证覆盖范围
- **时间对齐**: 每个点携带精确时间戳（存于curvature）
- **质量保证**: 经过多重滤波，适合后续配准
- **内存效率**: 点数减少2/3，降低KD树构建开销

#### 2.2 数据同步 (sync_packages)

##### 2.2.1 功能概述

`sync_packages` 函数负责将 LiDAR 点云和 IMU 数据时间对齐，确保每一帧 LiDAR 扫描都有对应的 IMU 数据覆盖整个扫描周期。

**函数签名：**
```cpp
bool sync_packages(MeasureGroup &meas)
```

**返回值：**
- `true`: 成功同步一组数据，可以进行处理
- `false`: 数据不足或时间未对齐，等待更多数据

**MeasureGroup 数据结构：**
```cpp
struct MeasureGroup {
    double lidar_beg_time;                      // LiDAR扫描起始时间
    double lidar_end_time;                      // LiDAR扫描结束时间
    PointCloudXYZI::Ptr lidar;                  // LiDAR点云数据
    deque<sensor_msgs::Imu::ConstPtr> imu;      // 对应时间段的IMU数据队列
};
```

##### 2.2.2 完整实现

```cpp
bool sync_packages(MeasureGroup &meas)
{
    // ===== 步骤1: 检查缓冲区 =====
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;  // 任一缓冲区为空，无法同步
    }

    // ===== 步骤2: 提取LiDAR数据（仅执行一次，由lidar_pushed标志控制） =====
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();          // 取出最早的LiDAR点云
        meas.lidar_beg_time = time_buffer.front();  // 取出起始时间戳

        // --- 计算LiDAR扫描结束时间 ---

        // 情况1: 点云过少（≤1个点）
        if (meas.lidar->points.size() <= 1)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        // 情况2: 最后一个点的时间戳异常（< 平均扫描时间的一半）
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        // 情况3: 正常情况，使用最后一个点的时间戳
        else
        {
            scan_num++;
            // 最后一个点的curvature字段存储了相对起始时间的偏移（单位：微秒）
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);

            // 增量更新平均扫描时间（滑动平均）
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        // 特殊情况: MARSIM雷达类型，扫描时间视为0
        if(lidar_type == MARSIM)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;  // 标记已提取LiDAR数据
    }

    // ===== 步骤3: 检查IMU数据是否覆盖LiDAR扫描周期 =====
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;  // IMU数据尚未到达扫描结束时间，等待更多IMU数据
    }

    // ===== 步骤4: 提取对应时间段的IMU数据 =====
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();  // 清空上一次的IMU数据

    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;  // 超过扫描结束时间，停止

        meas.imu.push_back(imu_buffer.front());  // 添加到测量组
        imu_buffer.pop_front();                   // 从缓冲区移除
    }

    // ===== 步骤5: 清理已处理的LiDAR数据 =====
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;  // 重置标志，准备下一帧

    return true;  // 成功同步一组数据
}
```

##### 2.2.3 关键机制解析

###### 扫描时间计算策略

函数通过三种方式计算 LiDAR 扫描结束时间：

| 情况 | 条件 | 计算方式 | 原因 |
|------|------|----------|------|
| **异常1** | 点数 ≤ 1 | `起始时间 + 平均扫描时间` | 点云数据不足，使用历史平均值 |
| **异常2** | 最后点时间戳 < 0.5×平均时间 | `起始时间 + 平均扫描时间` | 时间戳异常，使用历史平均值 |
| **正常** | 最后点时间戳正常 | `起始时间 + 最后点时间偏移` | 使用实际扫描时间 |

**时间戳存储方式：**
- 点云中每个点的 `curvature` 字段被复用存储相对起始时间的偏移
- 单位：微秒（在预处理时从 Livox 的 `offset_time` 转换而来）
- 使用时需除以 1000 转换为秒

###### 滑动平均扫描时间

```cpp
lidar_mean_scantime += (当前扫描时间 - lidar_mean_scantime) / scan_num;
```

这是一个**增量平均算法**，避免存储所有历史扫描时间：
- `scan_num`: 已处理的扫描帧数
- 新平均值 = 旧平均值 + (新值 - 旧平均值) / 总数
- 作用: 用于异常情况下估算扫描时间

###### lidar_pushed 标志机制

**作用**: 确保同一帧 LiDAR 数据只被提取一次

**工作流程：**
```
第1次调用 sync_packages():
├─ lidar_pushed = false
├─ 提取 LiDAR 数据，计算结束时间
├─ 设置 lidar_pushed = true
├─ 检查 IMU 数据不足 → 返回 false

第2次调用 sync_packages():
├─ lidar_pushed = true → 跳过 LiDAR 提取
├─ 检查 IMU 数据充足
├─ 提取 IMU 数据
├─ 清理缓冲区，设置 lidar_pushed = false
└─ 返回 true
```

**设计意图**:
- LiDAR 数据先到，IMU 数据可能需要等待
- 避免重复提取同一帧 LiDAR，直到 IMU 数据就绪

##### 2.2.4 时间对齐示意图

```
LiDAR扫描周期:
  |<------------- lidar_mean_scantime (约0.1秒) ------------->|
  ▼                                                           ▼
  ┌───────────────────────────────────────────────────────────┐
  │  LiDAR点云                                                │
  └───────────────────────────────────────────────────────────┘
  t0 (lidar_beg_time)                          t1 (lidar_end_time)

IMU数据流:
  ─────●────●────●────●────●────●────●────●────●────●────●──────→ 时间
       ↑                                                   ↑
   第1个IMU                                          last_timestamp_imu

选取的IMU数据:
  ┌────────────────────────────────────────────────────────┐
  │  t0 <= IMU时间戳 < t1 的所有IMU数据                     │
  └────────────────────────────────────────────────────────┘
```

**同步条件：**
- `last_timestamp_imu >= lidar_end_time`: IMU数据必须覆盖整个扫描周期
- 提取 `[lidar_beg_time, lidar_end_time)` 区间内的所有 IMU 数据

##### 2.2.5 主循环调用逻辑

```cpp
while (ros::ok())
{
    ros::spinOnce();  // 触发回调，填充 lidar_buffer 和 imu_buffer

    if(sync_packages(Measures))  // 尝试同步数据
    {
        // 成功获取一组同步的数据，开始处理
        if (flg_first_scan) {
            // 首帧初始化
            first_lidar_time = Measures.lidar_beg_time;
            p_imu->first_lidar_time = first_lidar_time;
            flg_first_scan = false;
            continue;
        }

        // 正常处理流程
        // ... IMU预积分
        // ... 点云配准
        // ... 状态更新
    }
    // 如果返回false，继续循环等待更多数据
}
```

**关键特点：**
- **非阻塞**: 数据不足时立即返回 `false`，不等待
- **增量处理**: 每次成功同步一组数据后立即处理
- **时间保证**: 确保 IMU 数据完整覆盖 LiDAR 扫描时间，为后续去畸变提供基础

##### 2.2.6 全局变量

```cpp
// 数据缓冲区（由回调函数填充）
deque<PointCloudXYZI::Ptr> lidar_buffer;      // LiDAR点云队列
deque<double> time_buffer;                    // 对应的时间戳队列
deque<sensor_msgs::Imu::ConstPtr> imu_buffer; // IMU数据队列

// 同步状态变量
bool lidar_pushed = false;                    // 标记当前LiDAR帧是否已提取
double lidar_mean_scantime = 0.0;             // 平均扫描时间（秒）
int scan_num = 0;                             // 已处理的扫描帧数
double lidar_end_time;                        // 当前LiDAR扫描结束时间
double last_timestamp_imu;                    // 最新IMU数据的时间戳
```

##### 2.2.7 sync_packages 执行频率分析

**核心问题**：`sync_packages(Measures)` 什么时候返回 `true`？执行频率是多少？

###### 返回 true 的条件

`sync_packages` 返回 `true` 需要同时满足以下条件：

1. **LiDAR 和 IMU 缓冲区都非空**
   ```cpp
   if (lidar_buffer.empty() || imu_buffer.empty()) {
       return false;  // ❌ 任一缓冲区为空
   }
   ```

2. **IMU 数据覆盖完整的 LiDAR 扫描周期**
   ```cpp
   if (last_timestamp_imu < lidar_end_time) {
       return false;  // ❌ IMU数据不足
   }
   // ✅ 只有当 last_timestamp_imu >= lidar_end_time 时才返回 true
   ```

###### 主循环调用频率

主循环以**极高频率**运行（理论上接近 5000 Hz，实际受 `ros::spinOnce()` 限制）：

```cpp
while (ros::ok())
{
    ros::spinOnce();              // 处理回调，填充缓冲区
    if(sync_packages(Measures))   // 频繁调用，但大多数返回 false
    {
        // 仅在数据就绪时执行（约 10 Hz）
    }
}
```

###### 实际执行频率

**`sync_packages` 返回 `true` 的频率 ≈ LiDAR 发布频率**

对于 Livox MID360：**约 10 Hz**（每秒处理 10 帧点云）

**时间轴示意**：
```
时间   0ms    100ms   200ms   300ms   400ms
       │       │       │       │       │
LiDAR  帧1     帧2     帧3     帧4     帧5
       │       │       │       │       │
       └─wait──┘       │       │       │
               └─wait──┘       │       │
                       └─wait──┘       │
                               └─wait──┘

sync_packages() 在每帧 LiDAR 到达后：
- 多次调用返回 false（等待 IMU 数据）
- IMU 数据覆盖扫描周期后返回 true（每帧 1 次）
```

**详细执行流程**：

```
t=100ms: LiDAR 帧到达
├─ sync_packages() 第1次调用
│  ├─ 提取 LiDAR 数据
│  ├─ 计算 lidar_end_time ≈ 200ms
│  ├─ last_timestamp_imu = 90ms < 200ms
│  └─ 返回 false ❌

t=110-190ms: IMU 陆续到达
├─ sync_packages() 多次调用
│  ├─ lidar_pushed = true（跳过 LiDAR 提取）
│  ├─ last_timestamp_imu < 200ms
│  └─ 返回 false ❌ （持续等待）

t=200ms: IMU 覆盖到扫描结束时间
├─ sync_packages() 最终成功
│  ├─ last_timestamp_imu = 200ms >= 200ms ✅
│  ├─ 提取 [100ms, 200ms) 区间的所有 IMU 数据
│  ├─ 清理缓冲区
│  └─ 返回 true ✅ → 执行括号内代码
```

###### 10 Hz 的来源解析

**重要澄清**：10 Hz **不是固定参数**，也**不是代码控制**的，而是由 **LiDAR 驱动程序决定**。

**1. `scan_rate` 参数的真实作用**

```yaml
# 仅在 velodyne.yaml 中出现
preprocess:
    scan_rate: 10    # only need to be set for velodyne
```

- **用途**：仅用于 **Velodyne 等老式雷达**，当点云数据**没有精确时间戳**时，用于估算每个点的时间
  ```cpp
  // preprocess.cpp:297
  double omega_l = 0.361 * SCAN_RATE;  // 扫描角速度
  // 根据点的角度估算时间: time = angle / omega_l
  ```

- **MID360 不需要**：Livox 雷达每个点都有硬件时间戳（`offset_time`），直接使用：
  ```cpp
  pl_full[i].curvature = msg->points[i].offset_time / 1000000.0;
  ```

**2. 实际频率由什么决定**

```
频率决定链：
┌─────────────────┐
│ LiDAR 硬件      │ MID360: 10 Hz 扫描
└────────┬────────┘
         ↓
┌─────────────────┐
│ Livox 驱动程序  │ 以 10 Hz 发布点云到 ROS
└────────┬────────┘
         ↓
┌─────────────────┐
│ FAST-LIO 订阅   │ 被动接收，处理频率 = 10 Hz
└─────────────────┘
```

**答案**：**LiDAR 驱动程序发布 ROS 消息的频率**

- FAST-LIO **不控制** LiDAR 发布频率
- 只是被动订阅 `/livox/lidar` 话题
- 实际频率由 Livox ROS 驱动决定（通常 10 Hz）

**3. 如何验证实际频率**

```bash
# 查看 LiDAR 话题的实际发布频率
rostopic hz /livox/lidar

# 输出示例：
# subscribed to [/livox/lidar]
# average rate: 10.002
#   min: 0.098s max: 0.102s std dev: 0.00123s window: 30
```

###### 括号内代码执行频率

**结论**：`if(sync_packages(Measures)) { ... }` 括号内的代码以 **LiDAR 发布频率**执行。

对于 MID360：**10 Hz**（每秒 10 次）

| 指标 | 值 |
|------|------|
| **sync_packages 调用频率** | 极高（~5000 Hz 理论值） |
| **返回 false 频率** | 大部分调用（等待数据） |
| **返回 true 频率** | **~10 Hz**（与 LiDAR 同步） |
| **括号内代码执行** | **10 Hz**（每帧 1 次） |
| **每帧可用时间** | 100ms |
| **典型处理时间** | 30-70ms（保证实时性） |

**关键点**：
- 主循环高速运行，不断检查数据同步状态
- 只有当 IMU 数据覆盖完整 LiDAR 扫描周期时才返回 `true`
- 核心算法以 **LiDAR 驱动发布频率**执行，对 MID360 约为 10 Hz

#### 2.3 IMU预积分与状态预测
- `p_imu->Process()`: 处理IMU数据，进行预积分
- 获取预测的状态 (位置、姿态、速度等)

#### 2.4 地图管理
- `lasermap_fov_segment()`: 根据视场对地图进行分割
- 动态调整局部地图范围，删除远离当前位置的点

#### 2.5 点云降采样
- 使用体素滤波器对输入点云进行降采样
- 减少计算量同时保持特征

#### 2.6 KD树初始化与更新
- 首次运行时构建 ikd-tree
- 后续循环中进行增量式更新

#### 2.7 迭代卡尔曼滤波更新
- `kf.update_iterated_dyn_share_modified()`:
  - 执行点到面ICP配准
  - 迭代更新状态估计
  - 这是FAST-LIO的核心算法部分

#### 2.8 地图增量更新
- `map_incremental()`: 将新的特征点添加到地图KD树中
- 智能降采样策略，避免地图过度稠密

#### 2.9 点云话题发布

FAST-LIO 发布 **4 个点云话题**，但默认只启用 **2 个**。

##### 2.9.1 话题总览

| 话题名称 | 坐标系 (frame_id) | 默认状态 | 发布频率 | 用途 |
|----------|-------------------|----------|----------|------|
| `/cloud_registered` | `camera_init` (世界坐标系) | ✅ 启用 | ~10 Hz | 配准后的世界坐标系点云 |
| `/cloud_registered_body` | `body` (IMU坐标系) | ✅ 启用 | ~10 Hz | 配准后的机体坐标系点云 |
| `/cloud_effected` | `camera_init` | ❌ 注释 | - | 用于配准的有效特征点 |
| `/Laser_map` | `camera_init` | ❌ 注释 | - | 局部地图点云 |

##### 2.9.2 发布器定义

在主函数初始化时创建（第849-856行）：

```cpp
// ROS 点云发布器
ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_registered", 100000);              // 世界坐标系配准点云

ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_registered_body", 100000);         // 机体坐标系配准点云

ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_effected", 100000);                // 有效特征点云

ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
        ("/Laser_map", 100000);                     // 局部地图点云
```

**队列大小**：100000（非常大的缓冲区，避免高频数据丢失）

##### 2.9.3 话题详解

###### 1. /cloud_registered（世界坐标系配准点云）

**坐标系**：`camera_init`（世界坐标系，SLAM 初始化时的坐标系）

**数据来源**：
```cpp
// 根据 dense_pub_en 参数选择
dense_pub_en = true  → feats_undistort   // 去畸变点云（稠密）
dense_pub_en = false → feats_down_body   // 降采样点云（稀疏）
```

**发布函数**（第478-530行）：
```cpp
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)  // 检查发布开关
    {
        // 选择点云数据源
        PointCloudXYZI::Ptr laserCloudFullRes(
            dense_pub_en ? feats_undistort : feats_down_body);

        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        // 转换到世界坐标系
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        // 转换为 ROS 消息并发布
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
    }
}
```

**发布条件**（第981行）：
```cpp
if (scan_pub_en || pcd_save_en)  // 启用点云发布 或 启用PCD保存
    publish_frame_world(pubLaserCloudFull);
```

**配置参数**（mid360.yaml）：
```yaml
publish:
    scan_publish_en: true      # 启用点云发布
    dense_publish_en: true     # 发布稠密点云
```

**MID360 配置**：
- `scan_publish_en = true` → 发布
- `dense_publish_en = true` → 使用 `feats_undistort`（稠密，~10000 点/帧）

**用途**：
- RViz 可视化（世界坐标系点云地图）
- 保存为 PCD 文件（地图重建）
- 下游算法使用（路径规划、障碍检测等）

###### 2. /cloud_registered_body（机体坐标系配准点云）

**坐标系**：`body`（IMU/机体坐标系）

**数据来源**：`feats_undistort`（去畸变点云，固定使用稠密点云）

**发布函数**（第532-549行）：
```cpp
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    // 转换到 IMU 机体坐标系
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";  // IMU 机体坐标系
    pubLaserCloudFull_body.publish(laserCloudmsg);
}
```

**发布条件**（第982行）：
```cpp
if (scan_pub_en && scan_body_pub_en)  // 需同时启用两个开关
    publish_frame_body(pubLaserCloudFull_body);
```

**配置参数**（mid360.yaml）：
```yaml
publish:
    scan_publish_en: true
    scan_bodyframe_pub_en: true  # 启用机体坐标系发布
```

**MID360 配置**：两个开关都为 `true` → 发布

**用途**：
- 机器人局部避障（相对机体的障碍物位置）
- 传感器校验（检查 LiDAR-IMU 外参）
- 在线外参标定

###### 3. /cloud_effected（有效特征点云）

**坐标系**：`camera_init`（世界坐标系）

**数据来源**：`laserCloudOri`（用于 ICP 配准的有效特征点）

**发布函数**（第551-565行）：
```cpp
void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld(
        new PointCloudXYZI(effct_feat_num, 1));

    // 转换有效特征点到世界坐标系
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}
```

**发布条件**（第983行）：
```cpp
// publish_effect_world(pubLaserCloudEffect);  // ❌ 默认注释
```

**状态**：**默认关闭**（代码被注释）

**用途**：
- 调试：可视化实际用于配准的特征点
- 性能分析：查看有效特征点的分布
- 算法研究：验证特征提取效果

**点云特点**：
- 点数少（通常几百到几千点）
- 是降采样后用于 ICP 的点
- 包含有效残差的点（满足平面拟合条件）

###### 4. /Laser_map（局部地图点云）

**坐标系**：`camera_init`（世界坐标系）

**数据来源**：`featsFromMap`（从 KD 树中提取的局部地图点）

**发布函数**（第567-574行）：
```cpp
void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}
```

**发布条件**（第984行）：
```cpp
// publish_map(pubLaserCloudMap);  // ❌ 默认注释
```

**状态**：**默认关闭**（代码被注释）

**用途**：
- 可视化当前使用的局部地图
- 验证地图管理策略（FOV 裁剪、动态删除）
- 调试地图构建

**点云特点**：
- 点数中等（取决于 `det_range` 和点云密度）
- 只包含视场范围内的地图点
- 用于当前帧的 ICP 匹配

##### 2.9.4 发布时机与频率

**发布时机**：在主循环 `if(sync_packages(Measures))` 成功后，每帧处理结束时（第979-984行）：

```cpp
if(sync_packages(Measures))  // 数据同步成功
{
    // ... IMU 预积分
    // ... 点云去畸变
    // ... 状态更新
    // ... 地图更新

    /******* Publish points *******/
    if (path_en)                         publish_path(pubPath);
    if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
    if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
    // publish_effect_world(pubLaserCloudEffect);  // 注释
    // publish_map(pubLaserCloudMap);              // 注释
}
```

**发布频率**：
- 与 `sync_packages` 返回 `true` 的频率一致
- 对于 MID360：**~10 Hz**（每秒 10 次）
- 每处理完一帧 LiDAR 数据，发布一次点云

##### 2.9.5 配置参数总结

```yaml
# mid360.yaml 中的发布配置
publish:
    path_en: false                    # 路径发布开关
    scan_publish_en: true             # 点云发布总开关 ⭐
    dense_publish_en: true            # 稠密点云开关 ⭐
    scan_bodyframe_pub_en: true       # 机体坐标系点云开关 ⭐
```

**参数作用**：

| 参数 | 作用 | MID360 默认值 |
|------|------|---------------|
| `scan_publish_en` | 启用/禁用点云发布 | `true` |
| `dense_publish_en` | 选择稠密/稀疏点云 | `true`（稠密） |
| `scan_bodyframe_pub_en` | 启用/禁用机体坐标系点云 | `true` |

**发布逻辑**：
```cpp
// /cloud_registered 发布条件
scan_pub_en = true  || pcd_save_en = true  → 发布

// /cloud_registered_body 发布条件
scan_pub_en = true  && scan_body_pub_en = true  → 发布

// dense_pub_en 影响
dense_pub_en = true  → 使用 feats_undistort（稠密，~10000 点）
dense_pub_en = false → 使用 feats_down_body（稀疏，~1000 点）
```

##### 2.9.6 坐标系说明

**camera_init**（世界坐标系）：
- SLAM 初始化时刻的坐标系
- 固定不变的全局参考系
- 所有历史轨迹和地图点都在这个坐标系下

**body**（IMU/机体坐标系）：
- 以 IMU 为原点的机体坐标系
- 随机器人运动而运动
- 用于表示相对机体的障碍物位置

**坐标变换关系**：
```
LiDAR 坐标系 --[外参]--> IMU 坐标系 (body) --[状态估计]--> 世界坐标系 (camera_init)
```

##### 2.9.7 性能考虑

**发布点云对性能的影响**：

```cpp
// laserMapping.cpp:482
PointCloudXYZI::Ptr laserCloudFullRes(
    dense_pub_en ? feats_undistort : feats_down_body);
```

| 配置 | 点数 | 数据量 | 网络带宽 | 建议使用场景 |
|------|------|--------|----------|--------------|
| `dense_pub_en=true` | ~10000 点/帧 | ~400 KB/帧 | ~4 MB/s @ 10Hz | 离线地图构建、精细可视化 |
| `dense_pub_en=false` | ~1000 点/帧 | ~40 KB/帧 | ~0.4 MB/s @ 10Hz | 实时性要求高、低带宽网络 |

**优化建议**：
- 实时机器人：`dense_pub_en=false`（减少网络负载）
- 地图构建：`dense_pub_en=true`（保证地图质量）
- 调试模式：启用 `/cloud_effected` 和 `/Laser_map`
- 生产环境：只发布必要的话题

##### 2.9.8 与其他发布的关系

除了点云，还发布：

```cpp
// 里程计（第857-858行）
ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
        ("/Odometry", 100000);

// 路径轨迹（第859-860行）
ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
        ("/path", 100000);
```

**发布时机**：与点云同时（第980行）
**发布频率**：~10 Hz

**完整的发布流程**：
```
每帧处理完成后（~10 Hz）
├─ /Odometry            (位姿 + 协方差)
├─ /path                (轨迹路径，如果 path_en=true)
├─ /cloud_registered    (世界坐标系点云)
└─ /cloud_registered_body (机体坐标系点云)
```

### 3. 循环退出机制

有两种退出方式：

1. **信号中断** (SIGINT):
   - 通过 `SigHandle` 函数捕获 Ctrl+C 信号
   - 设置 `flg_exit = true`
   - 通知条件变量，优雅退出

2. **ROS关闭**:
   - `status = ros::ok()` 检查ROS状态
   - 当ROS节点关闭时退出循环

### 4. 退出后处理

当 `while(status)` 循环结束后（通过 `Ctrl+C` 或 `ros::ok()` 返回 false），程序执行清理和保存工作。

#### 4.1 地图保存

**功能**: 将累积的点云地图保存为 PCD 文件

**代码实现**:
```cpp
/**************** save map ****************/
/* 1. make sure you have enough memories
/* 2. pcd save will largely influence the real-time performences **/
if (pcl_wait_save->size() > 0 && pcd_save_en)
{
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    cout << "current scan saved to /PCD/" << file_name<<endl;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
}
```

**执行条件**:
- `pcd_save_en = true`: 启用PCD保存功能（从配置参数读取）
- `pcl_wait_save->size() > 0`: 缓冲区中有待保存的点云数据

**数据来源**:
在主循环的 `publish_frame_world()` 函数中，每一帧处理后的点云会累积到 `pcl_wait_save`：
```cpp
// 位于 publish_frame_world() 函数（第501-529行）
if (pcd_save_en)
{
    // 将当前帧点云转换到世界坐标系
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&feats_undistort->points[i],
                        &laserCloudWorld->points[i]);
    }

    *pcl_wait_save += *laserCloudWorld;  // 累积到缓冲区

    // 如果设置了间隔保存（pcd_save_interval > 0）
    // 每隔 N 帧保存一次到 scans_X.pcd
    // 退出时保存最终累积的点云到 scans.pcd
}
```

**保存路径**:
- `ROOT_DIR/PCD/scans.pcd`（`ROOT_DIR` 在编译时定义）
- MID360 配置中 `pcd_save_en=true, interval=-1`，表示累积所有帧到一个文件

**文件格式**:
- 使用 PCL 库的 `writeBinary()` 方法
- 二进制格式，文件更小，加载更快
- 包含所有历史帧累积的世界坐标系点云

**注意事项**:
1. **内存消耗**: 累积所有帧需要大量内存（注释提醒 "make sure you have enough memories"）
2. **实时性影响**: 保存过程会影响性能（注释 "pcd save will largely influence the real-time performences"）
3. **间隔保存**: 如果 `pcd_save_interval > 0`，会在循环中定期保存，减少退出时的负担

#### 4.2 调试日志文件关闭

**功能**: 关闭运行时打开的调试输出文件流

**代码实现**:
```cpp
fout_out.close();  // 关闭状态输出日志
fout_pre.close();  // 关闭预测状态日志
```

**文件用途**:
这些文件流在主函数初始化时打开（第188-191行）：
```cpp
ofstream fout_pre, fout_out, fout_dbg;
fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);  // 预测状态
fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);  // 更新状态
fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);      // 调试信息
```

**日志内容**:
- `mat_pre.txt`: 每次迭代前的状态预测值
- `mat_out.txt`: 每次迭代后的状态估计值（含位置、姿态、速度、IMU偏差等）
- `dbg.txt`: 调试信息（视具体需要写入）

**作用**: 确保文件正确关闭，避免数据丢失

#### 4.3 运行时统计日志生成

**功能**: 生成性能统计 CSV 文件，记录每一帧的处理耗时和资源使用情况

**执行条件**:
```cpp
if (runtime_pos_log)  // 配置参数 runtime_pos_log_enable 为 true
```

**完整实现**:
```cpp
if (runtime_pos_log)
{
    vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
    FILE *fp2;
    string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
    fp2 = fopen(log_dir.c_str(), "w");

    // CSV 表头
    fprintf(fp2, "time_stamp, total time, scan point size, incremental time, "
                 "search time, delete size, delete time, tree size st, "
                 "tree size end, add point size, preprocess time\n");

    // 写入每一帧的统计数据
    for (int i = 0; i < time_log_counter; i++) {
        fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
                T1[i],           // 时间戳
                s_plot[i],       // 总处理时间
                int(s_plot2[i]), // 扫描点数量
                s_plot3[i],      // KD树增量更新时间
                s_plot4[i],      // KD树搜索时间
                int(s_plot5[i]), // KD树删除点数
                s_plot6[i],      // KD树删除时间
                int(s_plot7[i]), // KD树起始大小
                int(s_plot8[i]), // KD树结束大小
                int(s_plot10[i]),// 添加点数量
                s_plot11[i]);    // 预处理时间

        // 同时构建向量（可能用于后续分析）
        t.push_back(T1[i]);
        s_vec.push_back(s_plot9[i]);
        s_vec2.push_back(s_plot3[i] + s_plot6[i]);
        s_vec3.push_back(s_plot4[i]);
        s_vec5.push_back(s_plot[i]);
    }
    fclose(fp2);
}
```

**统计变量定义**:
```cpp
#define MAXN (720000)  // 最大记录帧数

// 统计数组（每帧一个元素）
double T1[MAXN];       // 时间戳
double s_plot[MAXN];   // 总处理时间
double s_plot2[MAXN];  // 扫描点云大小
double s_plot3[MAXN];  // KD树增量更新时间
double s_plot4[MAXN];  // KD树搜索时间
double s_plot5[MAXN];  // KD树删除点数
double s_plot6[MAXN];  // KD树删除时间
double s_plot7[MAXN];  // KD树起始大小
double s_plot8[MAXN];  // KD树结束大小
double s_plot9[MAXN];  // 平均处理时间
double s_plot10[MAXN]; // 添加点数量
double s_plot11[MAXN]; // 点云预处理时间

int time_log_counter = 0;  // 当前记录的帧数
```

**数据收集时机**:
在主循环的每一帧处理结束时（第997-1008行）：
```cpp
T1[time_log_counter] = Measures.lidar_beg_time;
s_plot[time_log_counter] = t5 - t0;                    // 总时间
s_plot2[time_log_counter] = feats_undistort->points.size();
s_plot3[time_log_counter] = kdtree_incremental_time;
s_plot4[time_log_counter] = kdtree_search_time;
s_plot5[time_log_counter] = kdtree_delete_counter;
s_plot6[time_log_counter] = kdtree_delete_time;
s_plot7[time_log_counter] = kdtree_size_st;
s_plot8[time_log_counter] = kdtree_size_end;
s_plot9[time_log_counter] = aver_time_consu;
s_plot10[time_log_counter] = add_point_size;
time_log_counter++;
```

**CSV 文件列说明**:

| 列名 | 变量 | 单位 | 含义 |
|------|------|------|------|
| time_stamp | T1[i] | 秒 | LiDAR扫描起始时间 |
| total time | s_plot[i] | 秒 | 该帧总处理时间 |
| scan point size | s_plot2[i] | 个 | 去畸变后的点云大小 |
| incremental time | s_plot3[i] | 秒 | KD树增量更新耗时 |
| search time | s_plot4[i] | 秒 | KD树搜索最近邻耗时 |
| delete size | s_plot5[i] | 个 | 从KD树删除的点数 |
| delete time | s_plot6[i] | 秒 | KD树删除操作耗时 |
| tree size st | s_plot7[i] | 个 | 更新前KD树点数 |
| tree size end | s_plot8[i] | 个 | 更新后KD树点数 |
| add point size | s_plot10[i] | 个 | 添加到KD树的点数 |
| preprocess time | s_plot11[i] | 秒 | 点云预处理耗时 |

**用途**:
- **性能分析**: 识别性能瓶颈（搜索、更新、删除等）
- **参数调优**: 评估不同参数对性能的影响
- **算法对比**: 与其他SLAM算法对比效率
- **实时性验证**: 检查是否满足实时性要求

**保存路径**:
- `root_dir/Log/fast_lio_time_log.csv`

**MID360 配置**:
- `runtime_pos_log_enable: false`（默认关闭，避免影响性能）
- 需要时可在 launch 文件或 yaml 中启用

### 关键特点总结

1. **实时性设计**: 5000Hz的循环频率，确保及时处理传感器数据
2. **鲁棒性**: 多重检查机制，处理数据丢失和时间不同步
3. **效率优化**: 增量式地图更新，避免重复计算
4. **模块化**: 清晰的功能划分，便于维护和扩展
5. **完整的日志系统**: 详细的运行时统计和调试信息

---

*更新于: 2025-10-04*