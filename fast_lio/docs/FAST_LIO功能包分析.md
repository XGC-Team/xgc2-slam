# 功能包: fast_lio

## 1. 功能概述

*   **一句话总结**: FAST-LIO是一个高效且鲁棒的激光雷达-惯性里程计(LiDAR-Inertial Odometry)系统，通过紧耦合的迭代扩展卡尔曼滤波器融合LiDAR特征点和IMU数据，实现快速运动、噪声或杂乱环境下的鲁棒导航。
*   **核心节点**: `fastlio_mapping`
*   **算法类型**: 基于误差状态迭代扩展卡尔曼滤波(Error-State Iterated Extended Kalman Filter, ESIKF)的紧耦合LiDAR-惯性里程计算法，使用ikd-Tree进行增量式地图构建和快速最近邻搜索。

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: C++ ROS客户端库，用于节点通信
    *   `rospy`: Python ROS客户端库
    *   `std_msgs`: 标准消息类型
    *   `sensor_msgs`: 传感器消息类型，包括PointCloud2和IMU消息
    *   `geometry_msgs`: 几何消息类型，用于位姿和变换表示
    *   `nav_msgs`: 导航消息类型，包括Odometry和Path
    *   `tf`: 坐标系变换库
    *   `pcl_ros`: PCL与ROS的接口库
    *   `livox_ros_driver`: Livox系列激光雷达驱动(必需)
    *   `message_generation/message_runtime`: 自定义消息生成
    *   `eigen_conversions`: Eigen与ROS消息的转换
    *   **外部库**:
        *   `Eigen3 >= 3.3.4`: 线性代数运算库
        *   `PCL >= 1.8`: 点云处理库
        *   `OpenMP`: 并行计算支持
        *   `PythonLibs`: Python库支持(用于可视化)

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/livox/lidar` (可配置) | `livox_ros_driver/CustomMsg` (Livox) 或 `sensor_msgs/PointCloud2` (其他LiDAR) | 激光雷达点云数据，用于建图和定位。Livox使用CustomMsg格式以获取每个点的时间戳；Velodyne/Ouster等使用标准PointCloud2格式。 |
| `/livox/imu` (可配置) | `sensor_msgs/Imu` | IMU数据(加速度计和陀螺仪)，用于状态预测和运动补偿。支持内置和外置IMU，6轴或9轴。 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | ~10-100 | 配准后的点云（世界坐标系），根据配置可发布稠密或降采样点云。 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | ~10-100 | IMU body坐标系下的点云。 |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | ~10-100 | 有效特征点云（用于EKF更新的点）。 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | ~10-100 | 局部地图点云（来自ikd-Tree）。 |
| `/Odometry` | `nav_msgs/Odometry` | ~10-100 | 里程计输出，包含位姿、速度和协方差信息。child_frame_id为"body"，frame_id为"camera_init"。 |
| `/path` | `nav_msgs/Path` | ~1-10 | 机器人运动轨迹路径。 |

### 3.3 发布/订阅的坐标系 (TF)

*   **发布的TF**: `camera_init` → `body`
    *   `camera_init`: 全局地图坐标系（初始化时的世界坐标系）
    *   `body`: IMU body坐标系
*   **坐标系关系**:
    *   LiDAR坐标系通过外参（`extrinsic_T`和`extrinsic_R`）与IMU body坐标系关联
    *   所有输出位姿和点云都在`camera_init`全局坐标系下表示

### 3.4 提供的服务 (Services)

无。FAST-LIO不提供ROS服务接口。

### 3.5 自定义消息类型

*   **Pose6D.msg**: 预积分的LiDAR状态（在IMU测量时刻）
    *   `offset_time`: IMU测量相对于第一个LiDAR点的时间偏移
    *   `acc[3]`: 预积分的总加速度（全局坐标系）
    *   `gyr[3]`: 无偏陀螺仪角速度（body坐标系）
    *   `vel[3]`: 预积分速度（全局坐标系）
    *   `pos[3]`: 预积分位置（全局坐标系）
    *   `rot[9]`: 预积分旋转矩阵（全局坐标系）

## 4. 核心算法原理

FAST-LIO采用**紧耦合的误差状态迭代扩展卡尔曼滤波(Error-State Iterated Extended Kalman Filter)**框架，结合**ikd-Tree增量式动态KD树**进行高效的点云配准和地图更新。

### 4.1 状态定义

系统状态向量包含24维（流形表示为23维）：

$$
\mathbf{x} = [\mathbf{p}^T, \mathbf{R}, \mathbf{R}_{LI}, \mathbf{t}_{LI}^T, \mathbf{v}^T, \mathbf{b}_g^T, \mathbf{b}_a^T, \mathbf{g}^T]^T
$$

其中：
- $\mathbf{p} \in \mathbb{R}^3$: IMU在全局坐标系下的位置
- $\mathbf{R} \in SO(3)$: IMU在全局坐标系下的旋转
- $\mathbf{R}_{LI} \in SO(3)$: LiDAR相对于IMU的旋转外参
- $\mathbf{t}_{LI} \in \mathbb{R}^3$: LiDAR相对于IMU的平移外参
- $\mathbf{v} \in \mathbb{R}^3$: IMU速度（全局坐标系）
- $\mathbf{b}_g \in \mathbb{R}^3$: 陀螺仪偏置
- $\mathbf{b}_a \in \mathbb{R}^3$: 加速度计偏置
- $\mathbf{g} \in S^2$: 重力向量（球面流形表示）

### 4.2 算法流程

#### 步骤1: 前向传播 (Forward Propagation)

使用IMU数据进行状态预测，运动模型为：

$$
\begin{aligned}
\dot{\mathbf{p}} &= \mathbf{v} \\
\dot{\mathbf{R}} &= \mathbf{R} \, [\boldsymbol{\omega} - \mathbf{b}_g]_{\times} \\
\dot{\mathbf{v}} &= \mathbf{R}(\mathbf{a} - \mathbf{b}_a) + \mathbf{g} \\
\dot{\mathbf{b}}_g &= \mathbf{n}_{bg} \\
\dot{\mathbf{b}}_a &= \mathbf{n}_{ba}
\end{aligned}
$$

其中 $\mathbf{a}$ 和 $\boldsymbol{\omega}$ 分别是加速度计和陀螺仪的测量值，$[\cdot]_{\times}$ 表示反对称矩阵。

#### 步骤2: 反向传播 (Backward Propagation)

对LiDAR扫描周期内的每个点进行运动补偿，将点云去畸变到扫描结束时刻：

$$
\mathbf{p}_i^{end} = \mathbf{R}_{end} \mathbf{R}_i^{-1}(\mathbf{R}_{LI}\mathbf{p}_i^L + \mathbf{t}_{LI} - \mathbf{t}_i) + \mathbf{t}_{end}
$$

其中 $\mathbf{p}_i^L$ 是LiDAR坐标系下第i个点，$\mathbf{R}_i, \mathbf{t}_i$ 是该点时刻的预测位姿。

#### 步骤3: 迭代扩展卡尔曼滤波更新

**3.1 最近邻搜索**

使用ikd-Tree对每个去畸变点云进行K近邻搜索（默认K=5），找到地图中最近的点：

$$
\mathcal{N}_i = \text{KNN}(\mathbf{p}_i^{world}, K)
$$

**3.2 平面拟合与残差计算**

对找到的近邻点进行平面拟合（SVD分解），计算点到平面的距离作为观测残差：

$$
ax + by + cz + d = 0
$$

点到平面的距离（残差）为：

$$
r_i = \frac{a x_i + b y_i + c z_i + d}{\sqrt{a^2 + b^2 + c^2}}
$$

**3.3 雅可比矩阵计算**

测量雅可比矩阵 $\mathbf{H}$ 对状态变量求导：

$$
\mathbf{H}_i = \frac{\partial r_i}{\partial \delta \mathbf{x}} = [\mathbf{n}^T, \, (\mathbf{R}[\mathbf{p}_i]_{\times}\mathbf{C})^T, \, \ldots]
$$

其中 $\mathbf{n} = [a, b, c]^T$ 是平面法向量，$\mathbf{C} = \mathbf{R}^T\mathbf{n}$。

**3.4 迭代更新**

通过迭代求解以下优化问题（类似Gauss-Newton）：

$$
\delta \mathbf{x}^* = \arg\min_{\delta \mathbf{x}} \|\mathbf{H}\delta\mathbf{x} - \mathbf{r}\|^2_{\mathbf{R}^{-1}}
$$

卡尔曼增益为：

$$
\mathbf{K} = \mathbf{P}\mathbf{H}^T(\mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R})^{-1}
$$

状态更新：

$$
\mathbf{x} \leftarrow \mathbf{x} \boxplus \mathbf{K}(\mathbf{z} - h(\mathbf{x}))
$$

协方差更新：

$$
\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}
$$

默认迭代次数为3-4次。

#### 步骤4: 地图更新 (ikd-Tree)

使用ikd-Tree进行增量式地图维护：

**4.1 局部地图分割**

根据当前位置动态调整局部地图范围（立方体），移除视野外的点：

$$
\text{LocalMap} = \{\mathbf{p} \mid \|\mathbf{p} - \mathbf{p}_{current}\| < D_{range}\}
$$

**4.2 增量式插入**

将新扫描的点云（经过降采样和重复点检测后）插入ikd-Tree：

```
ikdTree.Add_Points(PointToAdd, downsampled=true)
```

**4.3 动态删除**

使用Box删除操作移除超出范围的点，保持树的平衡：

```
ikdTree.Delete_Point_Boxes(cub_needrm)
```

ikd-Tree支持O(log n)的增删改查操作，适合实时SLAM应用。

### 4.3 关键特性

1. **直接里程计**: 直接在原始LiDAR点上进行scan-to-map配准，无需特征提取（可选）
2. **ikd-Tree**: 增量式动态KD树，支持点的增删改查和降采样，相比PCL KD-Tree快10-100倍
3. **误差状态流形**: 在流形上进行卡尔曼滤波，保证旋转的正交性约束
4. **外参在线估计**: 可选择在线估计LiDAR-IMU外参（建议使用LI-Init离线标定后关闭）
5. **多线程加速**: 使用OpenMP并行化最近邻搜索和残差计算
6. **多传感器支持**: 支持Livox（Avia, Horizon, Mid-360）、Velodyne、Ouster等多种LiDAR

## 5. 使用与配置

### 5.1 启动示例

**Livox Avia示例：**
```bash
# 终端1: 启动FAST-LIO
roslaunch fast_lio mapping_avia.launch

# 终端2: 启动Livox驱动
roslaunch livox_ros_driver livox_lidar_msg.launch

# 或播放rosbag
rosbag play your_dataset.bag
```

**Velodyne示例：**
```bash
# 启动FAST-LIO (Velodyne配置)
roslaunch fast_lio mapping_velodyne.launch

# 启动Velodyne驱动或播放rosbag
roslaunch velodyne_pointcloud VLP16_points.launch
# 或
rosbag play velodyne_dataset.bag
```

**Ouster示例：**
```bash
roslaunch fast_lio mapping_ouster64.launch
```

### 5.2 关键参数配置

在 `config/avia.yaml` (或对应的配置文件) 中：

**通用参数：**
```yaml
common:
    lid_topic: "/livox/lidar"        # LiDAR话题名
    imu_topic: "/livox/imu"          # IMU话题名
    time_sync_en: false              # 软件时间同步（仅在无硬件同步时使用）
    time_offset_lidar_to_imu: 0.0    # LiDAR到IMU的时间偏移
```

**预处理参数：**
```yaml
preprocess:
    lidar_type: 1                    # 1=Livox, 2=Velodyne, 3=Ouster
    scan_line: 6                     # 扫描线数（Velodyne需要）
    blind: 4                         # 盲区距离(m)
    timestamp_unit: 2                # 时间戳单位(0=s, 1=ms, 2=us, 3=ns)
```

**建图参数：**
```yaml
mapping:
    acc_cov: 0.1                     # 加速度计测量噪声协方差
    gyr_cov: 0.1                     # 陀螺仪测量噪声协方差
    b_acc_cov: 0.0001                # 加速度计偏置随机游走噪声
    b_gyr_cov: 0.0001                # 陀螺仪偏置随机游走噪声
    fov_degree: 90                   # LiDAR视场角(度)
    det_range: 450.0                 # 检测范围(m)
    extrinsic_est_en: false          # 是否在线估计外参
    extrinsic_T: [0.04165, 0.02326, -0.0284]  # LiDAR到IMU的平移外参
    extrinsic_R: [1, 0, 0,           # LiDAR到IMU的旋转外参(行优先旋转矩阵)
                  0, 1, 0,
                  0, 0, 1]
```

**发布参数：**
```yaml
publish:
    path_en: false                   # 是否发布路径
    scan_publish_en: true            # 是否发布点云
    dense_publish_en: true           # 发布稠密点云(false则降采样)
    scan_bodyframe_pub_en: true      # 是否发布body坐标系点云
```

**PCD保存：**
```yaml
pcd_save:
    pcd_save_en: true                # 是否保存PCD文件
    interval: -1                     # 保存间隔(-1=所有帧保存在一个文件)
```

**Launch文件参数：**
```xml
<param name="feature_extract_enable" type="bool" value="0"/>  <!-- 0=禁用特征提取 -->
<param name="point_filter_num" type="int" value="3"/>         <!-- 点采样数 -->
<param name="max_iteration" type="int" value="3"/>            <!-- 最大迭代次数 -->
<param name="filter_size_surf" type="double" value="0.5"/>    <!-- 点云降采样体素大小 -->
<param name="filter_size_map" type="double" value="0.5"/>     <!-- 地图降采样体素大小 -->
<param name="cube_side_length" type="double" value="1000"/>   <!-- 局部地图立方体边长 -->
```

### 5.3 重要注意事项

1. **时间同步**: IMU和LiDAR**必须时间同步**，这非常重要。硬件同步优先，仅在无法硬件同步时使用`time_sync_en: true`
2. **Livox驱动**: 必须使用`livox_lidar_msg.launch`启动，因为需要每个点的时间戳
3. **外参标定**: 建议使用[LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init)进行外参标定，然后设置`extrinsic_est_en: false`
4. **帧率调整**: 修改livox_ros_driver的`publish_freq`参数可以改变帧率
5. **PCD保存**: 保存PCD会影响实时性能，注意内存使用

### 5.4 可视化

在RViz中订阅以下话题：
- `/cloud_registered`: 全局配准点云
- `/Odometry`: 里程计可视化
- `/path`: 轨迹路径
- `/Laser_map`: 局部地图

也可以使用自带的rviz配置：
```bash
rviz -d $(rospack find fast_lio)/rviz_cfg/loam_livox.rviz
```

### 5.6 性能基准

- **实时性**: 在Intel i7-8550U上可达100Hz (Livox Avia)
- **精度**: NCLT数据集上ATE < 1m (2km轨迹)
- **地图大小**: 支持超大场景(测试过立方体边长1000m)
- **启动时间**: 自动初始化，通常在0.1秒内完成

## 6. 相关工作

FAST-LIO是香港大学MARS实验室开发的一系列工作之一：

- **ikd-Tree**: 动态KD树数据结构
- **IKFoM**: 流形上的快速卡尔曼滤波工具箱
- **FAST-LIVO/FAST-LIVO2**: LiDAR-惯性-视觉里程计
- **R2LIVE**: LiDAR-惯性-视觉融合SLAM
- **LI-Init**: LiDAR-IMU外参标定和同步

## 7. 参考文献

1. **FAST-LIO2**: Xu, W., et al. "FAST-LIO2: Fast Direct LiDAR-inertial Odometry." IEEE Transactions on Robotics, 2022.
2. **FAST-LIO**: Xu, W., Zhang, F. "FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter." arXiv:2010.08196, 2020.
3. **ikd-Tree**: Cai, Y., et al. "ikd-Tree: An Incremental K-D Tree for Robotic Applications." arXiv:2102.10808, 2021.

## 8. 许可证

BSD License

## 9. 联系方式

- 维护者: claydergc (dev@livoxtech.com)
- GitHub: https://github.com/hku-mars/FAST_LIO
- 实验室: HKU-MARS (Mars Lab, The University of Hong Kong)