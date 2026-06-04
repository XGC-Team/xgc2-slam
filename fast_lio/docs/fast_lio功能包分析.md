# 功能包: fast_lio

## 1. 功能概述

*   **一句话总结**: FAST-LIO (Fast LiDAR-Inertial Odometry) 是一个高效鲁棒的LiDAR-IMU紧耦合里程计系统，通过迭代扩展卡尔曼滤波器(IEKF)和增量式kdtree实现快速运动、噪声环境和退化场景下的实时定位与建图。
*   **核心节点**: `fastlio_mapping`
*   **算法类型**: 基于误差状态迭代扩展卡尔曼滤波(Error-State Iterated Extended Kalman Filter, ESIEKF)的LiDAR-IMU紧耦合里程计算法，采用点到面ICP匹配和ikd-Tree增量式动态KD树进行地图管理。

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: C++客户端库，用于ROS节点通信
    *   `geometry_msgs`: 几何消息类型，用于位姿和坐标变换
    *   `nav_msgs`: 导航消息类型，包含里程计(Odometry)和路径(Path)消息
    *   `sensor_msgs`: 传感器消息类型，用于点云(PointCloud2)和IMU数据
    *   `pcl_ros`: PCL与ROS的接口库
    *   `tf`: 坐标变换库
    *   `livox_ros_driver`: Livox系列LiDAR的ROS驱动
    *   `message_generation/message_runtime`: ROS消息生成支持
    *   `eigen_conversions`: Eigen与ROS消息的转换库
    *   `Eigen3 (>= 3.3.4)`: 线性代数库，用于矩阵运算
    *   `PCL (>= 1.8)`: 点云库，用于点云处理
    *   `PythonLibs`: Python库，用于matplotlib可视化
    *   `OpenMP`: 多线程并行计算库

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/livox/lidar` (可配置) | `livox_ros_driver/CustomMsg` 或 `sensor_msgs/PointCloud2` | LiDAR点云数据，支持Livox系列(使用CustomMsg)、Velodyne、Ouster等多种LiDAR |
| `/livox/imu` (可配置) | `sensor_msgs/Imu` | IMU数据，包含加速度计和陀螺仪测量值，支持6轴和9轴IMU |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/Odometry` | `nav_msgs/Odometry` | ~10-100 | 机器人在世界坐标系下的里程计信息，包含位姿和协方差 |
| `/path` | `nav_msgs/Path` | ~1-10 | 机器人运动轨迹路径 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | ~10-100 | 配准后的点云(世界坐标系) |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | ~10-100 | IMU体坐标系下的点云 |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | ~10-100 | 有效匹配特征点点云 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | ~1-10 | 局部地图点云 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无（外参通过配置文件设置）
*   **发布的TF**: `camera_init` → `body` (世界坐标系到IMU体坐标系的变换)

### 3.4 提供的服务 (Services)

本功能包不提供ROS服务接口。

## 4. 核心算法原理

FAST-LIO采用基于误差状态的迭代扩展卡尔曼滤波器(Error-State IEKF)实现LiDAR和IMU的紧耦合融合。其核心特点是直接使用原始点云(无需特征提取)进行点到面的ICP匹配，并通过增量式kdTree(ikd-Tree)实现高效的地图管理和近邻搜索。

### 4.1 状态定义

系统状态向量为18维，包括：

$$
\mathbf{x} = [\mathbf{R}^T, \mathbf{p}^T, \mathbf{v}^T, \mathbf{b}_g^T, \mathbf{b}_a^T, \mathbf{g}^T]^T
$$

其中：
- $\mathbf{R} \in SO(3)$: 旋转矩阵(3维李群表示)
- $\mathbf{p} \in \mathbb{R}^3$: 位置
- $\mathbf{v} \in \mathbb{R}^3$: 速度
- $\mathbf{b}_g \in \mathbb{R}^3$: 陀螺仪偏置
- $\mathbf{b}_a \in \mathbb{R}^3$: 加速度计偏置
- $\mathbf{g} \in \mathbb{R}^3$: 重力向量

同时估计LiDAR到IMU的外参：
- $\mathbf{T}_{L}^{I}$: LiDAR相对IMU的平移
- $\mathbf{R}_{L}^{I}$: LiDAR相对IMU的旋转

### 4.2 算法流程

#### 步骤1: 数据同步与预处理 (sync_packages & preprocess)

1. **时间同步**: 将LiDAR扫描时间段内的IMU数据进行同步配对，形成`MeasureGroup`
2. **点云预处理**:
   - 去除盲区内的点（距离<blind参数）
   - 进行降采样（point_filter_num）
   - 提取时间戳信息（存储在curvature字段）
   - 支持特征提取模式和直接使用原始点云模式

#### 步骤2: IMU预积分与状态前向传播 (IMU_Processing)

使用IMU数据对状态进行前向传播：

$$
\mathbf{R}_{t+1} = \mathbf{R}_t \cdot \text{Exp}((\boldsymbol{\omega}_t - \mathbf{b}_g) \Delta t)
$$

$$
\mathbf{v}_{t+1} = \mathbf{v}_t + (\mathbf{R}_t (\mathbf{a}_t - \mathbf{b}_a) + \mathbf{g}) \Delta t
$$

$$
\mathbf{p}_{t+1} = \mathbf{p}_t + \mathbf{v}_t \Delta t + \frac{1}{2}(\mathbf{R}_t (\mathbf{a}_t - \mathbf{b}_a) + \mathbf{g}) \Delta t^2
$$

同时对点云进行运动畸变补偿，将扫描期间的点投影到扫描结束时刻。

#### 步骤3: 局部地图管理 (lasermap_fov_segment)

采用立方体滑动窗口管理局部地图：
- 当LiDAR位置距离地图边缘过近时，移动地图窗口
- 使用ikd-Tree删除窗口外的点云，保持地图大小固定
- 动态调整以适应机器人运动

#### 步骤4: 点云降采样 (VoxelGrid Filter)

对去畸变后的点云进行体素降采样：
- 降采样尺寸: `filter_size_surf` (如0.5m)
- 减少点云数量，提高匹配效率

#### 步骤5: 迭代EKF更新 (update_iterated_dyn_share_modified)

这是FAST-LIO的核心步骤，采用迭代方式进行状态估计：

**测量模型** (h_share_model):

对每个降采样点，在ikd-Tree中搜索最近的5个点（NUM_MATCH_POINTS），拟合局部平面：

$$
Ax + By + Cz + D = 0
$$

点到平面距离作为观测残差：

$$
h_i = \frac{|A x_i + B y_i + C z_i + D|}{\sqrt{A^2 + B^2 + C^2}}
$$

**雅可比矩阵计算**:

计算观测对状态的雅可比矩阵 $\mathbf{H}$：

$$
\mathbf{H}_i = [\mathbf{n}^T, \mathbf{n}^T \frac{\partial \mathbf{p}_w}{\partial \boldsymbol{\theta}}, \mathbf{n}^T \frac{\partial \mathbf{p}_w}{\partial \mathbf{t}_{L}^{I}}, \mathbf{n}^T \frac{\partial \mathbf{p}_w}{\partial \boldsymbol{\phi}_{L}^{I}}, \mathbf{0}_{1 \times 9}]
$$

其中 $\mathbf{n}$ 是平面法向量。

**迭代更新**:

重复以下步骤直到收敛（max_iteration次）：
1. 计算卡尔曼增益: $\mathbf{K} = \mathbf{P} \mathbf{H}^T (\mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R})^{-1}$
2. 状态更新: $\hat{\mathbf{x}} = \mathbf{x} + \mathbf{K}(\mathbf{z} - h(\mathbf{x}))$
3. 协方差更新: $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}$
4. 重新搜索最近邻点并重新计算残差

#### 步骤6: 地图增量更新 (map_incremental)

将当前帧的点云添加到ikd-Tree中：
- 检查新点与地图点的距离，避免重复添加
- 采用体素网格降采样策略（filter_size_map）
- ikd-Tree支持并行的增量式插入和删除操作

### 4.3 ikd-Tree特性

ikd-Tree是FAST-LIO 2.0的关键创新，相比传统KD-Tree具有以下优势：
- **增量式构建**: 支持高效的点插入和删除，无需重建整树
- **并行搜索**: 支持多线程并行k近邻搜索
- **动态平衡**: 自动维护树的平衡性，保证搜索效率
- **适用于高频率点云**: 支持100Hz以上的LiDAR频率

### 4.4 算法特点

1. **直接点云匹配**: 无需特征提取，可使用所有原始点云，提高精度
2. **紧耦合融合**: IMU和LiDAR在状态估计层面紧密融合，鲁棒性高
3. **高计算效率**: 通过ikd-Tree和并行计算实现实时性
4. **在线外参标定**: 可选的LiDAR-IMU外参在线估计
5. **支持多种LiDAR**: 统一接口支持机械旋转式(Velodyne, Ouster)和固态(Livox)雷达

## 5. 使用与配置

### 5.1 启动示例

```bash
# 启动Livox Avia LiDAR的建图节点
roslaunch fast_lio mapping_avia.launch

# 启动Velodyne LiDAR的建图节点
roslaunch fast_lio mapping_velodyne.launch

# 启动Ouster 64线LiDAR的建图节点
roslaunch fast_lio mapping_ouster64.launch

# 播放rosbag进行离线测试
rosbag play your_bag.bag
```

### 5.2 主要参数配置 (config/avia.yaml)

#### 5.2.1 通用参数 (common)
```yaml
common:
    lid_topic: "/livox/lidar"        # LiDAR话题名
    imu_topic: "/livox/imu"          # IMU话题名
    time_sync_en: false              # 软件时间同步（仅在无硬件同步时使用）
    time_offset_lidar_to_imu: 0.0    # LiDAR到IMU的时间偏移
```

#### 5.2.2 预处理参数 (preprocess)
```yaml
preprocess:
    lidar_type: 1                    # 1: Livox, 2: Velodyne, 3: Ouster
    scan_line: 6                     # 扫描线数（Livox默认6）
    blind: 4                         # 盲区距离(m)，小于此距离的点被过滤
```

#### 5.2.3 建图参数 (mapping)
```yaml
mapping:
    acc_cov: 0.1                     # 加速度计噪声协方差
    gyr_cov: 0.1                     # 陀螺仪噪声协方差
    b_acc_cov: 0.0001                # 加速度计偏置协方差
    b_gyr_cov: 0.0001                # 陀螺仪偏置协方差
    fov_degree: 90                   # 视场角度
    det_range: 450.0                 # 检测范围(m)
    extrinsic_est_en: false          # 是否在线估计外参
    extrinsic_T: [0.04165, 0.02326, -0.0284]  # LiDAR到IMU的平移外参
    extrinsic_R: [1, 0, 0,           # LiDAR到IMU的旋转外参(旋转矩阵)
                  0, 1, 0,
                  0, 0, 1]
```

#### 5.2.4 发布参数 (publish)
```yaml
publish:
    path_en: false                   # 是否发布路径
    scan_publish_en: true            # 是否发布点云
    dense_publish_en: true           # 是否发布稠密点云
    scan_bodyframe_pub_en: true      # 是否发布体坐标系点云
```

#### 5.2.5 PCD保存参数 (pcd_save)
```yaml
pcd_save:
    pcd_save_en: true                # 是否保存PCD文件
    interval: -1                     # 保存间隔(-1表示程序结束时保存所有帧)
```

### 5.3 Launch文件参数

```xml
<param name="feature_extract_enable" type="bool" value="0"/>  <!-- 是否启用特征提取 -->
<param name="point_filter_num" type="int" value="3"/>         <!-- 点云降采样间隔 -->
<param name="max_iteration" type="int" value="3"/>            <!-- 最大迭代次数 -->
<param name="filter_size_surf" type="double" value="0.5"/>    <!-- 点云降采样体素大小 -->
<param name="filter_size_map" type="double" value="0.5"/>     <!-- 地图降采样体素大小 -->
<param name="cube_side_length" type="double" value="1000"/>   <!-- 局部地图立方体边长 -->
<param name="runtime_pos_log_enable" type="bool" value="0"/>  <!-- 是否记录运行时日志 -->
```

### 5.4 使用注意事项

1. **时间同步**: 确保LiDAR和IMU硬件时间同步，否则会导致运动畸变补偿错误
2. **外参标定**: 建议使用[LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init)进行离线外参标定，设置`extrinsic_est_en: false`
3. **LiDAR频率**: Livox需使用`livox_lidar_msg.launch`启动以获取点级时间戳
4. **内存管理**: 大场景建图时注意设置`pcd_save/interval`避免内存溢出
5. **平台适配**: 支持x86和ARM平台(如树莓派4B、Jetson TX2)

## 6. 代码结构

### 6.1 目录结构

```
FAST_LIO/
├── CMakeLists.txt                 # CMake构建文件
├── package.xml                    # ROS功能包配置
├── README.md                      # 项目说明文档
├── config/                        # 参数配置文件目录
│   ├── avia.yaml                 # Livox Avia配置
│   ├── velodyne.yaml             # Velodyne配置
│   ├── ouster64.yaml             # Ouster 64配置
│   ├── mid360.yaml               # Livox Mid-360配置
│   ├── horizon.yaml              # Livox Horizon配置
│   └── marsim.yaml               # 仿真器配置
├── launch/                        # ROS启动文件目录
│   ├── mapping_avia.launch       # Avia建图启动文件
│   ├── mapping_velodyne.launch   # Velodyne建图启动文件
│   ├── mapping_ouster64.launch   # Ouster建图启动文件
│   ├── mapping_mid360.launch     # Mid-360建图启动文件
│   ├── mapping_horizon.launch    # Horizon建图启动文件
│   └── mapping_marsim.launch     # 仿真启动文件
├── include/                       # 头文件目录
│   ├── ikd-Tree/                 # ikd-Tree实现
│   │   └── ikd_Tree.h            # ikd-Tree头文件
│   ├── IKFoM_toolkit/            # IKFoM工具包(流形卡尔曼滤波)
│   │   ├── esekfom/              # 误差状态EKF实现
│   │   │   ├── esekfom.hpp      # ESEKF核心算法
│   │   │   └── util.hpp         # 工具函数
│   │   └── mtk/                  # 流形工具包
│   │       ├── types/            # 流形类型定义(SO3, S2等)
│   │       └── build_manifold.hpp
│   ├── common_lib.h              # 公共数据结构定义
│   ├── so3_math.h                # SO(3)李群运算
│   ├── Exp_mat.h                 # 指数映射
│   ├── use-ikfom.hpp             # IKFoM使用接口
│   └── matplotlibcpp.h           # Matplotlib C++接口
├── src/                           # 源文件目录
│   ├── laserMapping.cpp          # 主程序入口，实现建图流程
│   ├── preprocess.cpp            # 点云预处理实现
│   ├── preprocess.h              # 点云预处理头文件
│   └── IMU_Processing.hpp        # IMU处理和状态传播
├── rviz_cfg/                      # RViz配置文件
│   └── loam_livox.rviz           # 可视化配置
├── msg/                           # 自定义消息
│   └── Pose6D.msg                # 6D位姿消息
├── PCD/                           # 点云保存目录(运行时生成)
└── Log/                           # 日志保存目录(运行时生成)
```

### 6.2 核心文件说明

#### 6.2.1 laserMapping.cpp (主程序)
- **功能**: FAST-LIO的主程序入口，实现完整的建图流程
- **主要函数**:
  - `main()`: 初始化ROS节点，订阅话题，进入主循环
  - `sync_packages()`: 同步LiDAR和IMU数据
  - `lasermap_fov_segment()`: 局部地图视野分割
  - `h_share_model()`: 计算观测模型和雅可比矩阵
  - `map_incremental()`: 增量式地图更新
  - `publish_*()`: 发布各类消息（里程计、路径、点云）
- **核心变量**:
  - `kf`: Error-State EKF对象
  - `ikdtree`: ikd-Tree地图管理对象
  - `p_imu`: IMU处理对象
  - `state_point`: 当前状态估计

#### 6.2.2 preprocess.cpp/h (点云预处理)
- **功能**: 处理不同类型LiDAR的原始数据，提取点云并计算时间戳
- **主要类**: `Preprocess`
- **支持的LiDAR类型**:
  - `AVIA`: Livox Avia/Mid-70/Mid-40
  - `VELO16`: Velodyne系列
  - `OUST64`: Ouster系列
  - `MARSIM`: MARSIM仿真器
- **主要函数**:
  - `process()`: 统一的点云处理接口
  - `avia_handler()`: 处理Livox CustomMsg
  - `velodyne_handler()`: 处理Velodyne PointCloud2
  - `oust64_handler()`: 处理Ouster PointCloud2
  - `give_feature()`: 特征提取（可选）

#### 6.2.3 IMU_Processing.hpp (IMU处理)
- **功能**: IMU数据预积分、状态前向传播、点云去畸变
- **主要类**: `ImuProcess`
- **主要函数**:
  - `Process()`: 处理测量组，进行状态传播和点云去畸变
  - `IMU_init()`: IMU初始化，估计初始姿态和重力方向
  - `UndistortPcl()`: 点云运动畸变补偿
  - `Forward()`: IMU前向传播

#### 6.2.4 ikd_Tree.h (增量式KD树)
- **功能**: 实现高效的增量式动态KD树
- **主要类**: `KD_TREE`
- **核心特性**:
  - 增量式点插入/删除
  - 并行k近邻搜索
  - 自动树平衡维护
  - Box删除（删除指定区域内的点）
- **主要函数**:
  - `Add_Points()`: 添加点到树中
  - `Delete_Point_Boxes()`: 删除指定立方体内的点
  - `Nearest_Search()`: k近邻搜索
  - `Build()`: 从点云构建kdTree

#### 6.2.5 use-ikfom.hpp (IKFoM接口)
- **功能**: 定义状态空间和系统模型，为ESEKF提供接口
- **状态定义**: `state_ikfom` 包含位置、旋转、速度、偏置、重力、外参
- **主要函数**:
  - `get_f()`: 系统状态转移函数
  - `df_dx()`: 状态转移雅可比矩阵
  - `df_dw()`: 过程噪声雅可比矩阵

#### 6.2.6 common_lib.h (公共库)
- **功能**: 定义通用数据结构和工具函数
- **核心数据结构**:
  - `MeasureGroup`: LiDAR和IMU数据组
  - `StatesGroup`: 状态向量(位置、姿态、速度等)
  - `PointType`: PCL点类型(XYZI + Normal)
- **工具函数**:
  - `esti_plane()`: 平面拟合
  - `pointBodyToWorld()`: 坐标变换
  - `set_pose6d()`: 位姿设置

### 6.3 数据流

```
LiDAR数据 ──→ preprocess ──→ 点云缓冲区 ──┐
                                        ├──→ sync_packages ──→ MeasureGroup
IMU数据 ──→ imu_cbk ──→ IMU缓冲区 ──────┘

MeasureGroup ──→ IMU_Processing (前向传播+去畸变) ──→ feats_undistort

feats_undistort ──→ 降采样 ──→ feats_down_body ──┐
                                                 │
ikdtree (局部地图) ←──────────────────────────────┤
                                                 │
                                                 ├──→ IEKF更新
                                                 │    (h_share_model)
                                                 │       │
状态估计 ←───────────────────────────────────────┘       │
                                                         │
里程计/路径/点云发布 ←───────────────────────────────────┘
```

### 6.4 关键算法实现位置

| 算法模块 | 实现文件 | 函数/类 |
|---------|---------|---------|
| 主控流程 | `laserMapping.cpp` | `main()` |
| 数据同步 | `laserMapping.cpp` | `sync_packages()` |
| IMU预积分 | `IMU_Processing.hpp` | `ImuProcess::Process()` |
| 去畸变 | `IMU_Processing.hpp` | `ImuProcess::UndistortPcl()` |
| 点云预处理 | `preprocess.cpp` | `Preprocess::process()` |
| 局部地图管理 | `laserMapping.cpp` | `lasermap_fov_segment()` |
| 观测模型 | `laserMapping.cpp` | `h_share_model()` |
| ESEKF更新 | `esekfom.hpp` | `esekf::update_iterated_dyn_share_modified()` |
| 地图更新 | `laserMapping.cpp` | `map_incremental()` |
| ikd-Tree | `ikd_Tree.h/.cpp` | `KD_TREE` |
| 平面拟合 | `common_lib.h` | `esti_plane()` |

---

**生成时间**: 2025-09-30
**分析版本**: FAST-LIO 2.0
**参考论文**:
- Xu, W., Cai, Y., He, D., Lin, J., & Zhang, F. (2022). FAST-LIO2: Fast Direct LiDAR-Inertial Odometry. IEEE Transactions on Robotics.
- Xu, W., & Zhang, F. (2021). FAST-LIO: A Fast, Robust LiDAR-Inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter. IEEE Robotics and Automation Letters.