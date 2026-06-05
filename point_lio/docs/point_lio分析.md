# 功能包: Point-LIO

## 1. 功能概述

*   **一句话总结**: Point-LIO是一个鲁棒的高带宽激光-惯导里程计算法，支持高频输出(4k-8kHz)并且在IMU饱和和剧烈振动等恶劣运动条件下仍能保持稳定。
*   **核心节点**: `pointlio_mapping`
*   **算法类型**: 基于ESIKF(Error State Iterated Kalman Filter)的紧耦合激光-惯导里程计，结合增量体素地图(iVox)实现快速点到面ICP配准

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: C++客户端库
    *   `geometry_msgs`: 几何消息类型(位置、姿态、速度等)
    *   `nav_msgs`: 导航相关消息类型(里程计、路径)
    *   `sensor_msgs`: 传感器消息类型(点云、IMU数据)
    *   `pcl_ros`: PCL点云库的ROS接口
    *   `tf`: 坐标系转换库
    *   `livox_ros_driver`: Livox激光雷达驱动库
    *   `eigen_conversions`: Eigen数学库和ROS消息的转换
    *   `Eigen3`: 线性代数运算库
    *   `PCL`: 点云处理库
    *   `OpenMP`: 并行计算支持

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/livox/lidar` | `livox_ros_driver/CustomMsg` | Livox激光雷达点云数据(带时间戳) |
| `/livox/imu` | `sensor_msgs/Imu` | IMU惯性测量数据(加速度计、陀螺仪) |
| `/points_raw` | `sensor_msgs/PointCloud2` | 标准激光雷达点云数据(Velodyne/Ouster) |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | ~10-50 | 配准后的点云数据(世界坐标系) |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | ~10-50 | 机体坐标系下的点云数据 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | ~1-5 | 累积的激光点云地图 |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 4000-8000 | 高频里程计输出(位置、姿态、速度) |
| `/path` | `nav_msgs/Path` | ~10-50 | 机器人运动轨迹 |

### 3.3 发布/订阅的坐标系 (TF)

*   **需要输入的TF**: 无特殊要求(内部处理IMU与LiDAR外参)
*   **发布的TF**: `camera_init` → `body` (机器人位姿)

### 3.4 提供的服务 (Services)

Point-LIO主要通过话题进行通信，未定义特殊服务接口。

### 3.5 自定义消息类型

| 消息名称 | 描述 |
| :--- | :--- |
| `LocalSensorExternalTrigger.msg` | 本地传感器外部触发消息，包含触发ID、事件ID和时间戳信息 |

## 4. 核心算法原理

Point-LIO是一个基于ESIKF(Error State Iterated Kalman Filter)的紧耦合激光-惯导里程计算法。其核心创新在于结合了增量体素地图(iVox)和高效的点到面ICP配准，实现了高频输出且在恶劣运动条件下保持鲁棒性。

### 4.1 状态估计框架

#### 双重状态系统
Point-LIO采用了独特的双重状态系统设计：

1. **输入状态(state_input)**: 以IMU作为输入的状态空间
2. **输出状态(state_output)**: 直接估计机器人状态的输出空间

#### 状态向量定义

**输入状态向量(24维)**:
$$
\mathbf{x}_{input} = [\mathbf{p}, \mathbf{R}, \mathbf{R}_{LI}, \mathbf{t}_{LI}, \mathbf{v}, \mathbf{b}_g, \mathbf{b}_a, \mathbf{g}]^T
$$

**输出状态向量(30维)**:
$$
\mathbf{x}_{output} = [\mathbf{p}, \mathbf{R}, \mathbf{R}_{LI}, \mathbf{t}_{LI}, \mathbf{v}, \boldsymbol{\omega}, \mathbf{a}, \mathbf{g}, \mathbf{b}_g, \mathbf{b}_a]^T
$$

其中：
- $\mathbf{p}$: 位置
- $\mathbf{R}$: 旋转矩阵
- $\mathbf{R}_{LI}, \mathbf{t}_{LI}$: LiDAR到IMU的外参
- $\mathbf{v}$: 速度
- $\boldsymbol{\omega}$: 角速度
- $\mathbf{a}$: 加速度
- $\mathbf{g}$: 重力向量
- $\mathbf{b}_g, \mathbf{b}_a$: 陀螺仪和加速度计偏置

### 4.2 ESIKF预测步骤

系统动力学模型根据输入类型分为两种：

**输入模式动力学**:
$$
\begin{align}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{R}} &= \mathbf{R}[\boldsymbol{\omega}_m - \mathbf{b}_g]_\times \\
\dot{\mathbf{v}} &= \mathbf{R}(\mathbf{a}_m - \mathbf{b}_a) + \mathbf{g}
\end{align}
$$

**输出模式动力学**:
$$
\begin{align}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{R}} &= \mathbf{R}[\boldsymbol{\omega}]_\times \\
\dot{\mathbf{v}} &= \mathbf{R}\mathbf{a} + \mathbf{g}
\end{align}
$$

其中$[\cdot]_\times$表示反对称矩阵。

### 4.3 增量体素地图(iVox)

Point-LIO采用增量体素地图进行高效的近邻搜索和地图管理：

#### 体素化策略
$$
\text{voxel\_key} = \lfloor \frac{\mathbf{p}_{point}}{\text{resolution}} \rfloor
$$

#### 增量更新
```cpp
void MapIncremental() {
    for (auto& point : new_points) {
        auto center = compute_voxel_center(point);
        if (!exists_nearby_point(center, filter_size)) {
            ivox_->AddPoint(point);
        }
    }
}
```

### 4.4 点到面ICP配准

#### 平面估计
对于每个点$\mathbf{p}_i$，在iVox中搜索最近的$k$个点(通常$k=5$)，然后估计平面参数：

$$
\mathbf{n}^T\mathbf{p} + d = 0
$$

通过最小二乘法求解：
$$
\begin{bmatrix} \mathbf{p}_1^T & 1 \\ \vdots & \vdots \\ \mathbf{p}_k^T & 1 \end{bmatrix} \begin{bmatrix} \mathbf{n} \\ d \end{bmatrix} = \mathbf{0}
$$

#### 观测模型
点到平面的距离作为观测：
$$
z_i = \mathbf{n}_i^T \mathbf{p}_i + d_i
$$

观测雅可比矩阵：
$$
\mathbf{H}_i = \frac{\partial z_i}{\partial \mathbf{x}} = [\mathbf{n}_i^T, \mathbf{n}_i^T[\mathbf{p}_i]_\times, \ldots]
$$

### 4.5 饱和检测与处理

Point-LIO具备IMU饱和检测机制：

```cpp
bool checkSaturation(const sensor_msgs::Imu& imu_msg) {
    return (imu_msg.linear_acceleration.norm() > satu_acc) ||
           (imu_msg.angular_velocity.norm() > satu_gyro);
}
```

在饱和期间，算法会调整卡尔曼滤波器的信任度和处理策略。

### 4.6 时间同步与运动去畸变

#### 时间压缩算法
Point-LIO实现了高效的时间压缩算法，将点云按时间戳分组：

```cpp
template<typename T>
std::vector<int> time_compressing(const PointCloudXYZI::Ptr &point_cloud) {
    std::vector<int> time_seq;
    int j = 0;
    for(int i = 0; i < points_size - 1; i++) {
        j++;
        if (point_cloud->points[i+1].curvature > point_cloud->points[i].curvature) {
            time_seq.emplace_back(j);
            j = 0;
        }
    }
    return time_seq;
}
```

#### 运动补偿
对于每个点，根据其时间戳进行运动补偿：
$$
\mathbf{p}_{compensated} = \mathbf{R}(t) \mathbf{p}_{raw} + \mathbf{t}(t)
$$

## 5. 使用与配置

### 5.1 启动示例

```bash
# 对于Livox Avia雷达
roslaunch point_lio mapping_avia.launch

# 对于Velodyne雷达
roslaunch point_lio mapping_velody16.launch

# 对于Ouster雷达
roslaunch point_lio mapping_ouster64.launch

# 调试模式
roslaunch point_lio gdb_debug_example.launch
```

### 5.2 关键参数配置

#### 传感器配置
```yaml
# 话题配置
lid_topic: "/livox/lidar"    # 激光雷达话题
imu_topic: "/livox/imu"      # IMU话题

# 外参配置
extrinsic_T: [0.04165, 0.02326, -0.0284]  # LiDAR到IMU平移
extrinsic_R: [1, 0, 0,                     # LiDAR到IMU旋转矩阵
              0, 1, 0,
              0, 0, 1]
```

#### 算法参数
```yaml
# IMU饱和阈值
satu_acc: 3.0      # 加速度饱和值
satu_gyro: 35      # 角速度饱和值

# 滤波器协方差
acc_cov_output: 500    # 加速度观测协方差
gyr_cov_output: 1000   # 角速度观测协方差
b_acc_cov: 0.0001      # 加速度偏置协方差
b_gyr_cov: 0.0001      # 陀螺仪偏置协方差

# 地图参数
ivox_grid_resolution: 2.0  # iVox体素分辨率
plane_thr: 0.1            # 平面拟合阈值
```

#### 输出控制
```yaml
# 高频输出配置
publish_odometry_without_downsample: false  # 是否输出未降采样的高频里程计

# 点云发布
scan_publish_en: true         # 是否发布点云
scan_bodyframe_pub_en: false  # 是否发布机体坐标系点云

# PCD保存
pcd_save_en: false   # 是否保存点云地图
interval: -1         # 保存间隔(-1表示全部保存到一个文件)
```

### 5.3 性能优化

#### 多线程配置
```cpp
// CMakeLists.txt中的并行处理配置
if(N GREATER 5)
    add_definitions(-DMP_EN)           # 启用多线程
    add_definitions(-DMP_PROC_NUM=4)   # 使用4个核心
endif()
```

#### 内存优化
```cpp
// 点云降采样
downSizeFilterSurf.setLeafSize(filter_size_surf_min,
                               filter_size_surf_min,
                               filter_size_surf_min);
```

## 6. 算法特色与创新点

### 6.1 高频输出能力
- 支持4000-8000Hz的里程计输出频率
- 实时性能优异，适合高频控制应用

### 6.2 鲁棒性增强
- IMU饱和检测与处理机制
- 支持高达75 rad/s的角速度
- 抗振动设计，适应恶劣环境

### 6.3 增量体素地图
- iVox数据结构提供O(1)的近邻搜索
- 动态地图管理，内存效率高
- 支持大规模环境建图

### 6.4 无运动畸变
- 精确的时间同步机制
- 逐点运动补偿
- 保持点云几何一致性

### 6.5 双状态系统设计
- 灵活的状态表示方法
- 支持不同的传感器输入模式
- 提高了算法的适应性

## 7. 适用场景

### 7.1 高动态应用
- 无人机快速飞行
- 地面车辆高速行驶
- 机器人敏捷运动

### 7.2 恶劣环境
- 强振动环境
- IMU饱和场景
- 传感器噪声较大的情况

### 7.3 实时控制
- 需要高频位置反馈的控制系统
- 路径跟踪应用
- 实时避障系统

Point-LIO通过其创新的算法设计和工程优化，为激光-惯导SLAM领域提供了一个高性能、高鲁棒性的解决方案，特别适合对实时性和鲁棒性要求较高的机器人应用。