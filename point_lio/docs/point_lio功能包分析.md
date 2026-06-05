# 功能包: Point-LIO

## 1. 功能概述

*   **一句话总结**: 高带宽、鲁棒的激光雷达-惯性里程计(LIO)系统，支持高频率输出(4-8kHz)和抗退化、抗IMU饱和的SLAM
*   **核心节点**: `pointlio_mapping`
*   **算法类型**: 基于误差状态迭代卡尔曼滤波器(Error-State Iterative Kalman Filter, ESIKF)的紧耦合激光雷达-惯性里程计算法，采用增量式体素地图(iVox)进行局部地图管理

## 2. 依赖关系

*   **主要依赖项**:
    *   `roscpp`: ROS C++客户端库
    *   `std_msgs`: ROS标准消息类型
    *   `nav_msgs`: 导航消息类型(Odometry, Path)
    *   `sensor_msgs`: 传感器消息类型(PointCloud2, Imu)
    *   `geometry_msgs`: 几何消息类型(PoseStamped)
    *   `tf`: 坐标变换库
    *   `pcl_ros`: PCL与ROS接口库
    *   `livox_ros_driver`: Livox系列激光雷达驱动
    *   `Eigen3`: 线性代数库
    *   `PCL`: 点云处理库
    *   `OpenMP`: 并行计算库

## 3. 接口说明 (API)

### 3.1 订阅的话题 (Inputs)

| 话题名称 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/livox/lidar` | `livox_ros_driver/CustomMsg` | Livox系列雷达的自定义点云数据 |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Velodyne系列雷达的标准点云数据 |
| `/livox/imu` | `sensor_msgs/Imu` | IMU数据 |

### 3.2 发布的话题 (Outputs)

| 话题名称 | 消息类型 | 频率 (Hz) | 描述 |
| :--- | :--- | :--- | :--- |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | ~10 | 配准到世界坐标系的点云地图 |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | ~10 | IMU坐标系下的点云数据 |
| `/aft_mapped_to_init` | `nav_msgs/Odometry` | 4000-8000 | 高频里程计输出 |
| `/path` | `nav_msgs/Path` | ~10 | 机器人运动轨迹 |

### 3.3 发布/订阅的坐标系 (TF)

*   **发布的TF**: `camera_init` → `body`

## 4. 核心算法原理

Point-LIO的核心创新是**点对点处理**和**iVox增量式体素地图**。

### 4.1 点对点处理

与传统的帧级处理不同，Point-LIO以单个点的时间戳为单位进行状态更新：
- 完全消除运动畸变
- 实现4-8kHz的高频里程计输出
- 增量式地图更新

### 4.2 ESIKF状态估计

**状态向量** (IMU-Input模式，24维):
$$
\mathbf{x} = [\mathbf{p}, \mathbf{q}, \mathbf{q}_L, \mathbf{t}_L, \mathbf{v}, \mathbf{b}_g, \mathbf{b}_a, \mathbf{g}]^T
$$

**观测模型**:
- 在iVox中搜索5个最近邻点
- 拟合局部平面
- 点到平面距离作为残差
- 迭代EKF更新

### 4.3 IMU饱和处理

当检测到IMU饱和时（测量值接近饱和值99%），系统会：
- 忽略该轴的IMU测量
- 依赖其他轴和LiDAR信息
- 保持系统稳定运行

### 4.4 iVox数据结构

增量式体素地图的特点：
- 基于哈希表的O(1)最近邻搜索
- 增量式点云添加
- 自适应体素分辨率

## 5. 使用与配置

### 5.1 启动示例

```bash
# Livox Avia
roslaunch livox_ros_driver livox_lidar_msg.launch
roslaunch point_lio mapping_avia.launch

# Velodyne
roslaunch point_lio mapping_velody16.launch
rosbag play your_dataset.bag
```

### 5.2 关键参数

```yaml
# IMU模式
use_imu_as_input: 0          # 0-Output模式, 1-Input模式
check_satu: 1                # 是否检查IMU饱和
satu_acc: 3.0                # 加速度饱和值
satu_gyro: 35                # 陀螺仪饱和值(rad/s)

# 建图参数
plane_thr: 0.1               # 平面拟合阈值
match_s: 81                  # 点到平面匹配阈值
ivox_grid_resolution: 2.0    # iVox体素分辨率

# 降采样
point_filter_num: 1          # 点云降采样系数
filter_size_surf: 0.5        # 体素大小
```

## 6. 代码结构

```
Point-LIO/
├── src/
│   ├── laserMapping.cpp      # 主节点(状态估计与建图)
│   ├── Estimator.cpp         # ESIKF更新函数
│   ├── IMU_Processing.cpp    # IMU处理
│   ├── preprocess.cpp        # 点云预处理
│   └── parameters.cpp        # 参数读取
├── include/
│   ├── common_lib.h          # 通用库
│   ├── so3_math.h            # SO(3)数学
│   └── ivox/                 # iVox数据结构
│       └── ivox3d.h
├── config/
│   ├── avia.yaml
│   └── velody16.yaml
└── launch/
    ├── mapping_avia.launch
    └── mapping_velody16.launch
```

### 核心特性

1. **高带宽输出**: 4-8kHz里程计频率
2. **无运动畸变**: 点对点处理
3. **抗退化**: 鲁棒设计
4. **抗IMU饱和**: 可处理75 rad/s角速度
5. **计算高效**: iVox + 增量处理

## 7. 适用场景

- 高速无人机飞行
- 剧烈振动环境
- 激进运动场景
- 高频控制系统
- IMU性能受限场景

---

**参考论文**: He et al., "Point-LIO: Robust High-Bandwidth Lidar-Inertial Odometry," Advanced Intelligent Systems, 2022
