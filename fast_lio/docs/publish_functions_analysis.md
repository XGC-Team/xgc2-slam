# FAST-LIO2 发布函数详细分析文档

## 文档概述

本文档详细讲解 FAST-LIO2 主循环中步骤10的发布代码，涵盖所有涉及的函数、变量、类和数学原理。

**目标代码段（位于 laserMapping.cpp:1175-1178）：**
```cpp
// 步骤10: 发布点云和路径
if (path_en)                         publish_path(pubPath);
if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
```

---

## 目录
1. [代码逐行分析](#1-代码逐行分析)
2. [publish_path 函数详解](#2-publish_path-函数详解)
3. [publish_frame_world 函数详解](#3-publish_frame_world-函数详解)
4. [publish_frame_body 函数详解](#4-publish_frame_body-函数详解)
5. [相关变量详解](#5-相关变量详解)
6. [相关类和数据结构](#6-相关类和数据结构)
7. [坐标系变换数学原理](#7-坐标系变换数学原理)
8. [ROS消息类型说明](#8-ros消息类型说明)

---

## 1. 代码逐行分析

### 1.1 第1行：路径发布条件判断与调用
```cpp
if (path_en)                         publish_path(pubPath);
```

**逐行分析：**

- **`if (path_en)`**：条件判断
  - `path_en` 是 bool 类型全局变量（定义于 laserMapping.cpp:84）
  - 从 ROS 参数服务器加载：`nh.param<bool>("publish/path_en", path_en, true)`（行923）
  - 默认值为 `true`
  - **作用**：控制是否发布机器人轨迹路径
  - **为什么需要此开关**：路径消息会随时间累积变大，可能导致 RViz 崩溃，通过此开关可禁用

- **`publish_path(pubPath)`**：函数调用
  - 调用路径发布函数
  - 参数 `pubPath` 是 `ros::Publisher` 类型（定义于行1041-1042）
  - 发布到 ROS 话题 `/path`

**控制流程：**
```
path_en == true → 执行 publish_path() → 发布机器人轨迹
path_en == false → 跳过发布
```

---

### 1.2 第2行：世界坐标系点云发布
```cpp
if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
```

**逐行分析：**

- **`if (scan_pub_en || pcd_save_en)`**：复合条件判断

  **条件1：`scan_pub_en`**
  - bool 类型全局变量（定义于行107）
  - 从 ROS 参数加载：`nh.param<bool>("publish/scan_publish_en", scan_pub_en, true)`（行924）
  - 默认值：`true`
  - **作用**：控制是否发布点云用于可视化

  **条件2：`pcd_save_en`**
  - bool 类型全局变量（定义于行84）
  - 从 ROS 参数加载：`nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false)`（行974）
  - 默认值：`false`
  - **作用**：控制是否保存点云到 PCD 文件

  **逻辑运算符 `||`（或）**：
  - 只要任一条件为真，就执行发布函数
  - 即使不发布到 ROS，如果需要保存 PCD，也要执行此函数

- **`publish_frame_world(pubLaserCloudFull)`**：函数调用
  - 发布世界坐标系下的点云
  - 参数 `pubLaserCloudFull` 是 `ros::Publisher` 类型（定义于行1031-1032）
  - 发布到 ROS 话题 `/cloud_registered`

**为什么使用 OR 逻辑？**
```
scan_pub_en = true, pcd_save_en = false  → 发布点云用于可视化
scan_pub_en = false, pcd_save_en = true  → 不发布但保存 PCD
scan_pub_en = true, pcd_save_en = true   → 既发布又保存
scan_pub_en = false, pcd_save_en = false → 跳过（节省计算）
```

---

### 1.3 第3行：机体坐标系点云发布
```cpp
if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
```

**逐行分析：**

- **`if (scan_pub_en && scan_body_pub_en)`**：复合条件判断

  **条件1：`scan_pub_en`**
  - 同上（行1.2）
  - 必须先启用点云发布

  **条件2：`scan_body_pub_en`**
  - bool 类型全局变量（定义于行107）
  - 从 ROS 参数加载：`nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true)`（行926）
  - 默认值：`true`
  - **作用**：控制是否发布机体坐标系（Body Frame）下的点云

  **逻辑运算符 `&&`（且）**：
  - 两个条件都为真才执行
  - 先检查总开关 `scan_pub_en`，再检查机体坐标系开关

- **`publish_frame_body(pubLaserCloudFull_body)`**：函数调用
  - 发布机体（IMU）坐标系下的点云
  - 参数 `pubLaserCloudFull_body` 是 `ros::Publisher` 类型（定义于行1033-1034）
  - 发布到 ROS 话题 `/cloud_registered_body`

**为什么使用 AND 逻辑？**
```
scan_pub_en = true, scan_body_pub_en = true   → 发布机体坐标系点云
scan_pub_en = true, scan_body_pub_en = false  → 不发布机体坐标系
scan_pub_en = false, scan_body_pub_en = true  → 不发布（总开关关闭）
scan_pub_en = false, scan_body_pub_en = false → 不发布
```

---

## 2. publish_path 函数详解

### 2.1 函数签名与位置
**位置**：laserMapping.cpp:750-764

```cpp
void publish_path(const ros::Publisher pubPath)
```

**参数说明：**
- `pubPath`：`ros::Publisher` 类型，按值传递（实际内部是智能指针，复制代价低）
- 发布到话题：`/path`（类型：`nav_msgs::Path`）

---

### 2.2 函数实现逐行分析

#### 行752：设置位姿戳（Pose Stamp）
```cpp
set_posestamp(msg_body_pose);
```

**子函数 `set_posestamp` 详解（laserMapping.cpp:704-715）：**

```cpp
template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}
```

**逐行分析：**

1. **行704：模板定义**
   ```cpp
   template<typename T>
   ```
   - 泛型编程，接受任何有 `pose` 成员的类型
   - 实际使用：`T = geometry_msgs::PoseStamped`

2. **行705：函数参数**
   ```cpp
   void set_posestamp(T & out)
   ```
   - 引用传递，直接修改输入参数
   - `out` 是要填充的位姿消息

3. **行707-709：设置位置（Position）**
   ```cpp
   out.pose.position.x = state_point.pos(0);
   out.pose.position.y = state_point.pos(1);
   out.pose.position.z = state_point.pos(2);
   ```
   - `state_point`：全局状态变量（类型：`state_ikfom`，定义于行143）
   - `state_point.pos`：3D 位置向量（类型：`vect3`，即 `Eigen::Vector3d`）
   - `.pos(0), .pos(1), .pos(2)`：访问 x, y, z 分量
   - **含义**：机器人在世界坐标系中的位置（单位：米）

4. **行710-713：设置姿态（Orientation）**
   ```cpp
   out.pose.orientation.x = geoQuat.x;
   out.pose.orientation.y = geoQuat.y;
   out.pose.orientation.z = geoQuat.z;
   out.pose.orientation.w = geoQuat.w;
   ```
   - `geoQuat`：全局四元数变量（类型：`geometry_msgs::Quaternion`，定义于行148）
   - **四元数表示**：`q = w + xi + yj + zk`
     - `w`：标量部分（实部）
     - `(x, y, z)`：向量部分（虚部）
   - **来源**：在主循环中从状态更新（laserMapping.cpp:1160-1163）：
     ```cpp
     geoQuat.x = state_point.rot.coeffs()[0];
     geoQuat.y = state_point.rot.coeffs()[1];
     geoQuat.z = state_point.rot.coeffs()[2];
     geoQuat.w = state_point.rot.coeffs()[3];
     ```
   - `state_point.rot`：SO3 旋转（类型：`SO3`，定义于 use-ikfom.hpp:7）
   - **含义**：机器人在世界坐标系中的姿态（旋转）

**数学背景：四元数（Quaternion）**

四元数是表示3D旋转的紧凑且无奇异性的方法：

$$
q = w + xi + yj + zk
$$

其中满足：
$$
i^2 = j^2 = k^2 = ijk = -1
$$

**四元数的优势：**
1. 无万向锁（Gimbal Lock）问题
2. 插值平滑（SLERP）
3. 存储高效（4个数 vs 9个数的旋转矩阵）
4. 数值稳定

**单位四元数表示旋转：**
$$
||q|| = \sqrt{w^2 + x^2 + y^2 + z^2} = 1
$$

**旋转向量 $v$ 的公式：**
$$
v' = q \otimes v \otimes q^*
$$
其中 $q^*$ 是 $q$ 的共轭四元数。

---

#### 行753：设置时间戳
```cpp
msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
```

**变量详解：**

- **`msg_body_pose`**：
  - 类型：`geometry_msgs::PoseStamped`（定义于行149）
  - 成员 `.header`：消息头，包含时间戳和坐标系 ID
  - 成员 `.pose`：位姿信息（位置 + 姿态）

- **`lidar_end_time`**：
  - 类型：`double`（定义于行102）
  - 含义：当前帧雷达扫描结束时刻（秒）
  - 来源：在 `sync_packages` 函数中计算（laserMapping.cpp:499）

- **`ros::Time().fromSec(lidar_end_time)`**：
  - 将 double 类型的秒数转换为 ROS 时间戳类型
  - ROS 时间戳精度：纳秒级

**为什么使用雷达结束时间？**
- 点云的最后一个点采集于 `lidar_end_time`
- 状态估计对应于该时刻
- 保证时间戳与数据同步

---

#### 行754：设置坐标系 ID
```cpp
msg_body_pose.header.frame_id = "camera_init";
```

**坐标系说明：**

- **`camera_init`**：世界坐标系
  - FAST-LIO2 沿用 LOAM 的命名传统
  - 实际是 LIO 系统的全局惯性坐标系
  - 原点：系统启动时的位置
  - 方向：ENU（东-北-天）或 NED（北-东-地），取决于 IMU 初始化

**FAST-LIO2 坐标系层级：**
```
camera_init (世界坐标系)
    ↓ (平移 + 旋转)
body (IMU 坐标系)
    ↓ (外参变换)
lidar (激光雷达坐标系)
```

---

#### 行757-762：路径抽稀
```cpp
static int jjj = 0;
jjj++;
if (jjj % 10 == 0)
{
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}
```

**逐行分析：**

1. **行757：静态计数器**
   ```cpp
   static int jjj = 0;
   ```
   - `static`：静态变量，在函数调用间保持值
   - 初始化仅执行一次
   - 用于计数函数调用次数

2. **行758：计数器递增**
   ```cpp
   jjj++;
   ```
   - 每次调用函数时计数器加1

3. **行759：抽稀条件判断**
   ```cpp
   if (jjj % 10 == 0)
   ```
   - `%`：取模运算符
   - 每10次调用只执行一次发布
   - **抽稀比率**：1/10 = 10%

**为什么需要抽稀？**

原因在代码注释中说明（行756）：
```cpp
/*** if path is too large, the rvis will crash ***/
```

- **问题**：路径消息随时间无限增长
- **后果**：RViz 渲染大量路径点会导致：
  1. 内存占用增加
  2. 渲染性能下降
  3. 可能导致 RViz 崩溃
- **解决方案**：降采样（每10个位姿只保存1个）

**数学公式：**
设系统运行频率为 $f$ Hz，抽稀比率为 $r = 0.1$，则路径点增长速率：
$$
\frac{dN}{dt} = f \times r
$$

例如：$f = 10$ Hz，$r = 0.1$，则每秒增加 1 个路径点。

4. **行761：添加位姿到路径**
   ```cpp
   path.poses.push_back(msg_body_pose);
   ```
   - `path`：全局变量（类型：`nav_msgs::Path`，定义于行146）
   - `.poses`：`std::vector<geometry_msgs::PoseStamped>` 类型
   - 将当前位姿追加到路径末尾

5. **行762：发布路径**
   ```cpp
   pubPath.publish(path);
   ```
   - 发布完整路径到 ROS 话题 `/path`
   - 订阅者（如 RViz）可实时显示机器人轨迹

---

### 2.3 publish_path 函数总结

**功能流程图：**
```
┌─────────────────────────────────┐
│  1. 读取当前状态 state_point    │
│     (位置 + 四元数姿态)          │
└──────────────┬──────────────────┘
               ↓
┌─────────────────────────────────┐
│  2. 填充 PoseStamped 消息       │
│     - 设置 position (x,y,z)     │
│     - 设置 orientation (qx,qy,qz,qw) │
│     - 设置时间戳和坐标系 ID      │
└──────────────┬──────────────────┘
               ↓
┌─────────────────────────────────┐
│  3. 抽稀判断 (每10次调用1次)    │
└──────────────┬──────────────────┘
               ↓
┌─────────────────────────────────┐
│  4. 添加到路径 & 发布           │
└─────────────────────────────────┘
```

**关键变量总结：**
| 变量名 | 类型 | 作用 | 来源 |
|--------|------|------|------|
| `state_point` | `state_ikfom` | 当前状态估计 | EKF 输出（行1155） |
| `geoQuat` | `geometry_msgs::Quaternion` | 姿态四元数 | 从 `state_point.rot` 转换 |
| `lidar_end_time` | `double` | 时间戳 | 数据同步模块 |
| `path` | `nav_msgs::Path` | 累积路径 | 全局变量 |
| `msg_body_pose` | `geometry_msgs::PoseStamped` | 当前位姿 | 临时消息 |

---

## 3. publish_frame_world 函数详解

### 3.1 函数签名与位置
**位置**：laserMapping.cpp:606-658

```cpp
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
```

**参数说明：**
- `pubLaserCloudFull`：`ros::Publisher` 引用，发布器对象
- 发布到话题：`/cloud_registered`（类型：`sensor_msgs::PointCloud2`）

---

### 3.2 函数实现逐行分析

#### 第一部分：点云发布逻辑（行608-627）

##### 行608：点云发布条件判断
```cpp
if(scan_pub_en)
```
- 检查点云发布开关
- 只有启用时才执行发布（节省计算资源）

##### 行610：选择点云源
```cpp
PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
```

**三元运算符详解：**
```cpp
条件 ? 值1 : 值2
```
- **条件**：`dense_pub_en`（bool，定义于行107）
  - 从参数加载：`nh.param<bool>("publish/dense_publish_en", dense_pub_en, true)`（行925）
  - 默认值：`true`

- **值1**：`feats_undistort`
  - 类型：`PointCloudXYZI::Ptr`（定义于行120）
  - 含义：去畸变后的完整点云（未降采样）
  - 点数：~10,000-100,000（取决于雷达型号）

- **值2**：`feats_down_body`
  - 类型：`PointCloudXYZI::Ptr`（定义于行121）
  - 含义：降采样后的点云
  - 点数：通常是原始的 10%-20%

**为什么需要选择？**

| 模式 | 点云源 | 优点 | 缺点 | 适用场景 |
|------|--------|------|------|----------|
| Dense | `feats_undistort` | 细节丰富，可视化效果好 | 数据量大，传输慢 | 离线建图、调试 |
| Downsampled | `feats_down_body` | 数据量小，实时性好 | 细节损失 | 在线运行、嵌入式 |

##### 行611：获取点云大小
```cpp
int size = laserCloudFullRes->points.size();
```
- `size`：点云中点的数量
- `.points`：`std::vector<PointType>` 类型
- `PointType`：`pcl::PointXYZINormal`（定义于 common_lib.h:37）

**PointType 结构详解：**
```cpp
typedef pcl::PointXYZINormal PointType;
```
包含字段：
- `x, y, z`：3D 坐标（float）
- `intensity`：强度值（float）
- `normal_x, normal_y, normal_z`：法向量（float）
- `curvature`：曲率/时间戳（float，FAST-LIO2 复用为相对时间）

##### 行612-613：创建输出点云
```cpp
PointCloudXYZI::Ptr laserCloudWorld( \
                new PointCloudXYZI(size, 1));
```

**智能指针构造：**
- `PointCloudXYZI::Ptr`：`boost::shared_ptr<PointCloudXYZI>` 的别名
- `new PointCloudXYZI(size, 1)`：
  - 参数1（`size`）：点云宽度（点数）
  - 参数2（`1`）：点云高度（无组织点云为1）
  - 预分配内存，避免动态扩容

##### 行615-619：坐标变换循环
```cpp
for (int i = 0; i < size; i++)
{
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                        &laserCloudWorld->points[i]);
}
```

**循环分析：**
- 遍历每个点
- 调用坐标变换函数 `RGBpointBodyToWorld`

**子函数 `RGBpointBodyToWorld` 详解（laserMapping.cpp:230-239）：**

```cpp
void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
```

**逐行分析：**

1. **行232：提取输入点坐标**
   ```cpp
   V3D p_body(pi->x, pi->y, pi->z);
   ```
   - `V3D`：`Eigen::Vector3d` 的别名（定义于 common_lib.h:40）
   - 将 PCL 点转换为 Eigen 向量

2. **行233：坐标变换（关键！）**
   ```cpp
   V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
   ```

   **数学推导：**

   设：
   - ${}^L\mathbf{p}$：点在雷达坐标系（Lidar）中的坐标
   - ${}^I\mathbf{p}$：点在 IMU 坐标系中的坐标
   - ${}^W\mathbf{p}$：点在世界坐标系（World）中的坐标
   - ${}^I_L\mathbf{R}$：雷达到 IMU 的旋转矩阵（`state_point.offset_R_L_I`）
   - ${}^I_L\mathbf{t}$：雷达到 IMU 的平移向量（`state_point.offset_T_L_I`）
   - ${}^W_I\mathbf{R}$：IMU 到世界的旋转矩阵（`state_point.rot`）
   - ${}^W_I\mathbf{t}$：IMU 到世界的平移向量（`state_point.pos`）

   **步骤1：雷达系 → IMU 系**
   $$
   {}^I\mathbf{p} = {}^I_L\mathbf{R} \cdot {}^L\mathbf{p} + {}^I_L\mathbf{t}
   $$
   对应代码：
   ```cpp
   state_point.offset_R_L_I * p_body + state_point.offset_T_L_I
   ```

   **步骤2：IMU 系 → 世界系**
   $$
   {}^W\mathbf{p} = {}^W_I\mathbf{R} \cdot {}^I\mathbf{p} + {}^W_I\mathbf{t}
   $$
   对应代码：
   ```cpp
   state_point.rot * (...) + state_point.pos
   ```

   **组合变换：**
   $$
   {}^W\mathbf{p} = {}^W_I\mathbf{R} \cdot ({}^I_L\mathbf{R} \cdot {}^L\mathbf{p} + {}^I_L\mathbf{t}) + {}^W_I\mathbf{t}
   $$

   **矩阵形式（齐次坐标）：**
   $$
   \begin{bmatrix} {}^W\mathbf{p} \\ 1 \end{bmatrix} =
   \begin{bmatrix} {}^W_I\mathbf{R} & {}^W_I\mathbf{t} \\ 0 & 1 \end{bmatrix}
   \begin{bmatrix} {}^I_L\mathbf{R} & {}^I_L\mathbf{t} \\ 0 & 1 \end{bmatrix}
   \begin{bmatrix} {}^L\mathbf{p} \\ 1 \end{bmatrix}
   $$

3. **行235-237：保存结果坐标**
   ```cpp
   po->x = p_global(0);
   po->y = p_global(1);
   po->z = p_global(2);
   ```
   - 将 Eigen 向量转换回 PCL 点格式

4. **行238：保留强度值**
   ```cpp
   po->intensity = pi->intensity;
   ```
   - 强度（反射率）在坐标变换中保持不变

**为什么叫 RGBpointBodyToWorld？**
- 函数名包含 "RGB" 但实际不处理颜色
- 可能是从处理 RGB 点云的代码复制而来
- FAST-LIO2 使用 intensity 而非 RGB

##### 行621-625：转换为 ROS 消息并发布
```cpp
sensor_msgs::PointCloud2 laserCloudmsg;
pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
laserCloudmsg.header.frame_id = "camera_init";
pubLaserCloudFull.publish(laserCloudmsg);
```

**逐行分析：**

1. **行621：创建 ROS 消息**
   ```cpp
   sensor_msgs::PointCloud2 laserCloudmsg;
   ```
   - `sensor_msgs::PointCloud2`：ROS 标准点云消息类型
   - 二进制格式，高效传输

2. **行622：PCL → ROS 转换**
   ```cpp
   pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
   ```
   - `pcl::toROSMsg`：PCL 库函数
   - 将 `pcl::PointCloud` 转换为 `sensor_msgs::PointCloud2`
   - 保留所有字段（x, y, z, intensity, 法向量等）

3. **行623：设置时间戳**
   ```cpp
   laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
   ```
   - 与路径发布相同，使用雷达结束时间

4. **行624：设置坐标系**
   ```cpp
   laserCloudmsg.header.frame_id = "camera_init";
   ```
   - 表明点云在世界坐标系中

5. **行625：发布消息**
   ```cpp
   pubLaserCloudFull.publish(laserCloudmsg);
   ```
   - 发送到 ROS 话题 `/cloud_registered`

6. **行626：发布计数器**
   ```cpp
   publish_count -= PUBFRAME_PERIOD;
   ```
   - `PUBFRAME_PERIOD = 20`（定义于行77）
   - `publish_count`：全局计数器（定义于行103）
   - 用于控制发布频率（详见主循环）

---

#### 第二部分：PCD 保存逻辑（行629-657）

##### 行632：PCD 保存条件判断
```cpp
if (pcd_save_en)
```
- 检查是否启用 PCD 文件保存

##### 行634-642：坐标变换（同上）
```cpp
int size = feats_undistort->points.size();
PointCloudXYZI::Ptr laserCloudWorld( \
                new PointCloudXYZI(size, 1));

for (int i = 0; i < size; i++)
{
    RGBpointBodyToWorld(&feats_undistort->points[i], \
                        &laserCloudWorld->points[i]);
}
```

**注意：**
- 保存时始终使用 `feats_undistort`（完整点云）
- 确保保存的数据最完整

##### 行643：累积点云
```cpp
*pcl_wait_save += *laserCloudWorld;
```

**运算符重载：**
- `operator+=`：PCL 库重载的点云合并运算符
- 将新点云追加到累积点云中

**变量详解：**
- `pcl_wait_save`：
  - 类型：`PointCloudXYZI::Ptr`（定义于行605）
  - 作用：累积多帧点云，批量保存

##### 行645-656：批量保存逻辑
```cpp
static int scan_wait_num = 0;
scan_wait_num ++;
if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
{
    pcd_index ++;
    string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
    pcl::PCDWriter pcd_writer;
    cout << "current scan saved to /PCD/" << all_points_dir << endl;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    pcl_wait_save->clear();
    scan_wait_num = 0;
}
```

**逐行分析：**

1. **行645-646：扫描计数器**
   ```cpp
   static int scan_wait_num = 0;
   scan_wait_num ++;
   ```
   - 累积的扫描帧数

2. **行647：保存条件判断**
   ```cpp
   if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
   ```

   **条件1**：`pcl_wait_save->size() > 0`
   - 确保有点云数据

   **条件2**：`pcd_save_interval > 0`
   - `pcd_save_interval`：保存间隔（帧数）
   - 从参数加载：`nh.param<int>("pcd_save/interval", pcd_save_interval, -1)`（行975）
   - 默认值 `-1` 表示禁用

   **条件3**：`scan_wait_num >= pcd_save_interval`
   - 达到保存间隔

3. **行649：文件索引递增**
   ```cpp
   pcd_index ++;
   ```
   - `pcd_index`：全局变量（定义于行104）
   - 生成唯一文件名

4. **行650：构造文件路径**
   ```cpp
   string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
   ```

   **字符串拼接示例：**
   - `ROOT_DIR = "/home/user/FAST_LIO/"`
   - `pcd_index = 1`
   - 结果：`"/home/user/FAST_LIO/PCD/scans_1.pcd"`

5. **行651-653：保存文件**
   ```cpp
   pcl::PCDWriter pcd_writer;
   cout << "current scan saved to /PCD/" << all_points_dir << endl;
   pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
   ```

   - `PCDWriter`：PCL 库的点云写入器
   - `.writeBinary()`：二进制格式保存（压缩高效）

   **PCD 格式对比：**
   | 格式 | 方法 | 优点 | 缺点 |
   |------|------|------|------|
   | ASCII | `writeASCII()` | 可读，易调试 | 文件大，读写慢 |
   | Binary | `writeBinary()` | 紧凑，快速 | 不可读 |
   | Compressed | `writeBinaryCompressed()` | 最小文件 | 需解压，慢 |

6. **行654-655：清理并重置**
   ```cpp
   pcl_wait_save->clear();
   scan_wait_num = 0;
   ```
   - 清空累积点云
   - 重置计数器，准备下一批

---

### 3.3 publish_frame_world 函数总结

**功能流程图：**
```
┌──────────────────────────────────┐
│ 1. 选择点云源                    │
│    (密集 or 降采样)              │
└────────────┬─────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 2. 遍历所有点                    │
│    调用坐标变换函数              │
└────────────┬─────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 3. RGBpointBodyToWorld           │
│    Lidar → IMU → World           │
└────────────┬─────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 4a. [scan_pub_en] 发布到 ROS     │
│     /cloud_registered            │
└──────────────────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 4b. [pcd_save_en] 累积点云       │
│     达到间隔时保存 PCD 文件      │
└──────────────────────────────────┘
```

**关键变量总结：**
| 变量名 | 类型 | 作用 |
|--------|------|------|
| `feats_undistort` | `PointCloudXYZI::Ptr` | 去畸变完整点云 |
| `feats_down_body` | `PointCloudXYZI::Ptr` | 降采样点云 |
| `dense_pub_en` | `bool` | 发布密集/稀疏点云 |
| `scan_pub_en` | `bool` | 点云发布开关 |
| `pcd_save_en` | `bool` | PCD 保存开关 |
| `pcd_save_interval` | `int` | 保存间隔（帧） |
| `pcl_wait_save` | `PointCloudXYZI::Ptr` | 累积点云缓存 |

---

## 4. publish_frame_body 函数详解

### 4.1 函数签名与位置
**位置**：laserMapping.cpp:660-677

```cpp
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
```

**参数说明：**
- `pubLaserCloudFull_body`：`ros::Publisher` 引用
- 发布到话题：`/cloud_registered_body`（类型：`sensor_msgs::PointCloud2`）

---

### 4.2 函数实现逐行分析

#### 行662：获取点云大小
```cpp
int size = feats_undistort->points.size();
```
- 使用完整的去畸变点云（与 `publish_frame_world` 不同）
- 机体坐标系点云用于调试，需要完整数据

#### 行663：创建输出点云
```cpp
PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
```
- 预分配内存
- 点云名称 "IMUBody" 表明坐标系

#### 行665-669：坐标变换循环
```cpp
for (int i = 0; i < size; i++)
{
    RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                        &laserCloudIMUBody->points[i]);
}
```

**子函数 `RGBpointBodyLidarToIMU` 详解（laserMapping.cpp:241-250）：**

```cpp
void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}
```

**逐行分析：**

1. **行243：提取雷达系坐标**
   ```cpp
   V3D p_body_lidar(pi->x, pi->y, pi->z);
   ```
   - 输入点在雷达坐标系中

2. **行244：仅应用外参变换**
   ```cpp
   V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);
   ```

   **数学公式：**
   $$
   {}^I\mathbf{p} = {}^I_L\mathbf{R} \cdot {}^L\mathbf{p} + {}^I_L\mathbf{t}
   $$

   **与 `RGBpointBodyToWorld` 的区别：**

   | 函数 | 变换 | 输出坐标系 | 用途 |
   |------|------|------------|------|
   | `RGBpointBodyToWorld` | Lidar → IMU → World | 世界系 | 建图、可视化 |
   | `RGBpointBodyLidarToIMU` | Lidar → IMU | IMU/Body 系 | 调试、外参标定 |

   **为什么需要 Body 系点云？**
   1. **外参标定验证**：检查 ${}^I_L\mathbf{R}$ 和 ${}^I_L\mathbf{t}$ 是否正确
   2. **运动补偿调试**：查看去畸变效果
   3. **多传感器融合**：其他传感器（相机）也在 Body 系

3. **行246-248：保存坐标**
   ```cpp
   po->x = p_body_imu(0);
   po->y = p_body_imu(1);
   po->z = p_body_imu(2);
   ```

4. **行249：保留强度**
   ```cpp
   po->intensity = pi->intensity;
   ```

#### 行671-676：发布消息
```cpp
sensor_msgs::PointCloud2 laserCloudmsg;
pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
laserCloudmsg.header.frame_id = "body";
pubLaserCloudFull_body.publish(laserCloudmsg);
publish_count -= PUBFRAME_PERIOD;
```

**与 `publish_frame_world` 的唯一区别：**

**行674：坐标系 ID**
```cpp
laserCloudmsg.header.frame_id = "body";
```
- 使用 `"body"` 而非 `"camera_init"`
- 表明点云在 IMU/Body 坐标系中

---

### 4.3 publish_frame_body 函数总结

**功能流程图：**
```
┌──────────────────────────────────┐
│ 1. 获取去畸变点云               │
│    (feats_undistort)             │
└────────────┬─────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 2. 遍历所有点                    │
│    RGBpointBodyLidarToIMU        │
└────────────┬─────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 3. 外参变换                      │
│    Lidar → IMU (仅外参)          │
└────────────┬─────────────────────┘
             ↓
┌──────────────────────────────────┐
│ 4. 发布到 ROS                    │
│    /cloud_registered_body        │
│    frame_id = "body"             │
└──────────────────────────────────┘
```

---

## 5. 相关变量详解

### 5.1 全局开关变量

| 变量名 | 类型 | 定义位置 | 默认值 | 作用 | 参数路径 |
|--------|------|----------|--------|------|----------|
| `path_en` | `bool` | 84 | `true` | 路径发布开关 | `publish/path_en` |
| `scan_pub_en` | `bool` | 107 | `true` | 点云发布总开关 | `publish/scan_publish_en` |
| `dense_pub_en` | `bool` | 107 | `true` | 密集点云开关 | `publish/dense_publish_en` |
| `scan_body_pub_en` | `bool` | 107 | `true` | Body系点云开关 | `publish/scan_bodyframe_pub_en` |
| `pcd_save_en` | `bool` | 84 | `false` | PCD保存开关 | `pcd_save/pcd_save_en` |

**参数加载示例（laserMapping.cpp:923-926）：**
```cpp
nh.param<bool>("publish/path_en", path_en, true);
nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
```

**YAML 配置示例：**
```yaml
publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: false  # 节省带宽
  scan_bodyframe_pub_en: false  # 通常不需要
```

---

### 5.2 状态变量

#### 5.2.1 state_point（全局状态）
**定义位置**：laserMapping.cpp:143
```cpp
state_ikfom state_point;
```

**类型**：`state_ikfom`（定义于 use-ikfom.hpp:12-21）

**结构定义：**
```cpp
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))           // 位置 (3D)
((SO3, rot))             // 旋转 (SO3)
((SO3, offset_R_L_I))    // 外参旋转 (SO3)
((vect3, offset_T_L_I))  // 外参平移 (3D)
((vect3, vel))           // 速度 (3D)
((vect3, bg))            // 陀螺仪偏差 (3D)
((vect3, ba))            // 加速度计偏差 (3D)
((S2, grav))             // 重力方向 (S2流形)
);
```

**状态维度：23维（流形维度）**
- 位置：3维
- 旋转：3维（SO3的李代数维度）
- 外参旋转：3维
- 外参平移：3维
- 速度：3维
- 陀螺仪偏差：3维
- 加速度计偏差：3维
- 重力：2维（S2流形，单位球面）

**状态更新**：
```cpp
state_point = kf.get_x();  // 从EKF获取最新状态（行1155）
```

**访问成员：**
```cpp
state_point.pos         // Eigen::Vector3d，位置
state_point.rot         // SO3，旋转（四元数表示）
state_point.offset_R_L_I // SO3，雷达到IMU旋转
state_point.offset_T_L_I // Eigen::Vector3d，雷达到IMU平移
state_point.vel         // Eigen::Vector3d，速度
state_point.bg          // Eigen::Vector3d，陀螺仪偏差
state_point.ba          // Eigen::Vector3d，加速度计偏差
state_point.grav        // S2，重力方向
```

---

#### 5.2.2 geoQuat（姿态四元数）
**定义位置**：laserMapping.cpp:148
```cpp
geometry_msgs::Quaternion geoQuat;
```

**类型**：ROS 消息类型 `geometry_msgs::Quaternion`

**成员：**
```cpp
double x, y, z, w;  // 四元数分量
```

**更新来源（laserMapping.cpp:1160-1163）：**
```cpp
geoQuat.x = state_point.rot.coeffs()[0];
geoQuat.y = state_point.rot.coeffs()[1];
geoQuat.z = state_point.rot.coeffs()[2];
geoQuat.w = state_point.rot.coeffs()[3];
```

**注意**：Eigen 四元数系数顺序
```cpp
state_point.rot.coeffs() = [x, y, z, w]  // Eigen
geoQuat = [x, y, z, w]                   // ROS
```

---

### 5.3 点云变量

| 变量名 | 类型 | 定义位置 | 坐标系 | 作用 |
|--------|------|----------|--------|------|
| `feats_undistort` | `PointCloudXYZI::Ptr` | 120 | Lidar | 去畸变完整点云 |
| `feats_down_body` | `PointCloudXYZI::Ptr` | 121 | Lidar | 降采样点云 |
| `feats_down_world` | `PointCloudXYZI::Ptr` | 122 | World | 世界系点云 |
| `pcl_wait_save` | `PointCloudXYZI::Ptr` | 605 | World | PCD保存缓存 |

**点云指针类型：**
```cpp
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef PointCloudXYZI::Ptr        (boost::shared_ptr<PointCloudXYZI>)
```

**点云来源：**
1. **`feats_undistort`**：
   - 由 `p_imu->Process()` 生成（行1076）
   - 去除运动畸变后的原始点云
   - 点数：10,000-100,000

2. **`feats_down_body`**：
   - 由体素滤波器生成（行1094-1095）
   ```cpp
   downSizeFilterSurf.setInputCloud(feats_undistort);
   downSizeFilterSurf.filter(*feats_down_body);
   ```
   - 降采样比率：由 `filter_size_surf` 决定（默认0.5m）
   - 点数：通常是原始的10%-20%

3. **`feats_down_world`**：
   - 在各发布函数内部生成
   - 通过坐标变换得到

---

### 5.4 时间变量

#### 5.4.1 lidar_end_time
**定义位置**：laserMapping.cpp:102
```cpp
double lidar_end_time = 0;
```

**含义**：当前帧雷达扫描结束时刻（秒）

**设置位置**：`sync_packages()` 函数（行499）
```cpp
meas.lidar_end_time = lidar_end_time;
```

**计算逻辑（行486-495）：**
```cpp
if (meas.lidar->points.size() <= 1)
{
    lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
}
else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
{
    lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
}
else
{
    lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
}
```

**说明：**
- 点云最后一个点的 `curvature` 字段存储相对时间戳（毫秒）
- 除以1000转换为秒
- 如果时间戳异常，使用平均扫描时间

---

### 5.5 发布器变量

| 变量名 | 定义位置 | 话题 | 消息类型 | 队列大小 |
|--------|----------|------|----------|----------|
| `pubPath` | 1041 | `/path` | `nav_msgs::Path` | 100000 |
| `pubLaserCloudFull` | 1031 | `/cloud_registered` | `sensor_msgs::PointCloud2` | 100000 |
| `pubLaserCloudFull_body` | 1033 | `/cloud_registered_body` | `sensor_msgs::PointCloud2` | 100000 |

**创建示例（laserMapping.cpp:1031-1042）：**
```cpp
ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_registered", 100000);
ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
        ("/cloud_registered_body", 100000);
ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
        ("/path", 100000);
```

**队列大小 100000 的原因：**
- FAST-LIO2 高频运行（~10Hz）
- 防止消息丢失
- 实际使用中通常不会满

---

### 5.6 PCD 保存相关变量

| 变量名 | 类型 | 定义位置 | 作用 |
|--------|------|----------|------|
| `pcd_save_en` | `bool` | 84 | 保存开关 |
| `pcd_save_interval` | `int` | 104 | 保存间隔（帧） |
| `pcd_index` | `int` | 104 | 文件索引 |
| `pcl_wait_save` | `PointCloudXYZI::Ptr` | 605 | 累积点云 |

**保存逻辑：**
```cpp
每 pcd_save_interval 帧保存一次
文件名：ROOT_DIR/PCD/scans_<pcd_index>.pcd
```

---

## 6. 相关类和数据结构

### 6.1 state_ikfom（状态流形）

**定义位置**：use-ikfom.hpp:12-21

**完整定义：**
```cpp
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_L_I))
((vect3, offset_T_L_I))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);
```

**流形类型说明：**

#### 6.1.1 vect3（欧几里得空间）
```cpp
typedef MTK::vect<3, double> vect3;
```
- 3维欧几里得向量
- 流形：$\mathbb{R}^3$
- 李代数维度：3

#### 6.1.2 SO3（特殊正交群）
```cpp
typedef MTK::SO3<double> SO3;
```
- 3D旋转群
- 流形：$SO(3) = \{R \in \mathbb{R}^{3\times3} | R^T R = I, \det(R) = 1\}$
- 李代数：$\mathfrak{so}(3)$，维度3
- 参数化：单位四元数 $(w, x, y, z)$，满足 $w^2+x^2+y^2+z^2=1$

**SO3 的李代数（Lie Algebra）：**

旋转向量 $\boldsymbol{\phi} = [\phi_x, \phi_y, \phi_z]^T \in \mathbb{R}^3$ 的指数映射：
$$
\mathbf{R} = \exp(\boldsymbol{\phi}^\wedge) = \exp\left(\begin{bmatrix} 0 & -\phi_z & \phi_y \\ \phi_z & 0 & -\phi_x \\ -\phi_y & \phi_x & 0 \end{bmatrix}\right)
$$

**Rodrigues 公式：**
$$
\mathbf{R} = \mathbf{I} + \frac{\sin\theta}{\theta}\boldsymbol{\phi}^\wedge + \frac{1-\cos\theta}{\theta^2}(\boldsymbol{\phi}^\wedge)^2
$$
其中 $\theta = ||\boldsymbol{\phi}||$

#### 6.1.3 S2（单位球面）
```cpp
typedef MTK::S2<double, 98090, 10000, 1> S2;
```
- 2维单位球面（3D空间中）
- 流形：$S^2 = \{\mathbf{v} \in \mathbb{R}^3 | ||\mathbf{v}|| = 1\}$
- 李代数维度：2（切空间维度）
- 用于表示重力方向（单位向量）

**为什么重力用 S2？**
- 重力大小固定（$g = 9.81 m/s^2$）
- 只需估计方向（2个自由度）
- 避免过参数化

**S2 的参数化：**

使用球面坐标：
$$
\mathbf{g} = g \begin{bmatrix} \sin\theta\cos\phi \\ \sin\theta\sin\phi \\ \cos\theta \end{bmatrix}
$$
状态只包含 $(\theta, \phi)$，2个参数。

---

### 6.2 PointType（点云点类型）

**定义位置**：common_lib.h:37
```cpp
typedef pcl::PointXYZINormal PointType;
```

**PCL 原始定义：**
```cpp
struct PointXYZINormal {
    float x, y, z;           // 3D 坐标
    float intensity;         // 强度（反射率）
    float normal_x, normal_y, normal_z;  // 法向量
    float curvature;         // 曲率
};
```

**FAST-LIO2 的使用：**
| 字段 | 标准用途 | FAST-LIO2 用途 |
|------|----------|----------------|
| `x, y, z` | 坐标 | 坐标 |
| `intensity` | 反射率 | 反射率 |
| `normal_x, normal_y, normal_z` | 法向量 | **未使用** |
| `curvature` | 曲率 | **相对时间戳（毫秒）** |

**为什么复用 curvature？**
- 避免定义新点类型
- PCL 兼容性好
- 字段足够存储时间戳

**时间戳计算示例：**
```cpp
// 点的绝对时间戳
double point_timestamp = lidar_beg_time + point.curvature / 1000.0;
```

---

### 6.3 MeasureGroup（测量组）

**定义位置**：common_lib.h:55-66

```cpp
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar;
    deque<sensor_msgs::Imu::ConstPtr> imu;
};
```

**成员详解：**

| 成员 | 类型 | 作用 |
|------|------|------|
| `lidar_beg_time` | `double` | 点云扫描开始时间（秒） |
| `lidar_end_time` | `double` | 点云扫描结束时间（秒） |
| `lidar` | `PointCloudXYZI::Ptr` | 点云数据 |
| `imu` | `deque<sensor_msgs::Imu::ConstPtr>` | IMU 数据队列 |

**用途：**
- 将同步的 LiDAR 和 IMU 数据打包
- 传递给 `ImuProcess::Process()` 进行去畸变
- 保证时间对齐：IMU 数据覆盖整个点云扫描周期

**时间关系：**
```
lidar_beg_time         lidar_end_time
    |                       |
    v                       v
    [========点云扫描========]
    [======IMU数据==========]
    ^                       ^
imu[0].stamp          imu[N].stamp
```

---

### 6.4 ROS 消息类型

#### 6.4.1 nav_msgs::Path
**文档**：http://docs.ros.org/api/nav_msgs/html/msg/Path.html

```cpp
std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
geometry_msgs/PoseStamped[] poses
```

**用途**：表示机器人轨迹

**FAST-LIO2 使用：**
```cpp
path.header.frame_id = "camera_init";
path.poses.push_back(msg_body_pose);  // 累积位姿
```

---

#### 6.4.2 geometry_msgs::PoseStamped
**文档**：http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html

```cpp
std_msgs/Header header
geometry_msgs/Pose pose
    Point position
        float64 x, y, z
    Quaternion orientation
        float64 x, y, z, w
```

**用途**：带时间戳的位姿

---

#### 6.4.3 sensor_msgs::PointCloud2
**文档**：http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html

```cpp
std_msgs/Header header
uint32 height
uint32 width
PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
```

**用途**：二进制点云格式

**FAST-LIO2 使用：**
```cpp
pcl::toROSMsg(*cloud_ptr, ros_msg);  // PCL → ROS 转换
ros_msg.header.frame_id = "camera_init";
```

---

## 7. 坐标系变换数学原理

### 7.1 坐标系定义

FAST-LIO2 涉及三个主要坐标系：

#### 7.1.1 世界坐标系（World Frame）`W`
- **名称**：`camera_init`
- **原点**：系统启动时的位置
- **方向**：由 IMU 初始化决定
  - 水平面：由初始加速度计测量确定（假设静止）
  - Z 轴：重力方向（向上或向下）
  - X, Y 轴：在水平面内，由初始姿态确定

#### 7.1.2 机体坐标系（Body Frame）`I`
- **名称**：`body`
- **原点**：IMU 中心
- **方向**：固定在 IMU 上
  - 通常遵循 FRD（前-右-下）或 FLU（前-左-上）

#### 7.1.3 雷达坐标系（Lidar Frame）`L`
- **名称**：`lidar` / `laser`
- **原点**：雷达中心
- **方向**：雷达自身坐标系
  - Livox Avia: X 前，Y 左，Z 上

---

### 7.2 外参标定

外参描述 Lidar 和 IMU 之间的固定变换：

**旋转矩阵**：${}^I_L\mathbf{R} \in SO(3)$
- 代码：`state_point.offset_R_L_I`
- 将 Lidar 系向量旋转到 IMU 系

**平移向量**：${}^I_L\mathbf{t} \in \mathbb{R}^3$
- 代码：`state_point.offset_T_L_I`
- Lidar 原点在 IMU 系中的位置

**齐次变换矩阵**：
$$
{}^I_LT = \begin{bmatrix} {}^I_L\mathbf{R} & {}^I_L\mathbf{t} \\ 0 & 1 \end{bmatrix}
$$

**参数来源：**
1. **初始值**：从配置文件加载（laserMapping.cpp:970-971）
   ```cpp
   nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
   nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
   ```

2. **在线估计**：FAST-LIO2 支持在线外参标定
   - 由 `extrinsic_est_en` 控制（默认 `true`）
   - EKF 状态包含外参
   - 运行过程中持续优化

---

### 7.3 点云坐标变换推导

#### 7.3.1 Lidar → IMU（Body 系）

**变换公式：**
$$
{}^I\mathbf{p} = {}^I_L\mathbf{R} \cdot {}^L\mathbf{p} + {}^I_L\mathbf{t}
$$

**代码实现**（`RGBpointBodyLidarToIMU`，行244）：
```cpp
V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I);
```

**矩阵形式：**
$$
\begin{bmatrix} {}^I\mathbf{p} \\ 1 \end{bmatrix} =
\begin{bmatrix} {}^I_L\mathbf{R} & {}^I_L\mathbf{t} \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} {}^L\mathbf{p} \\ 1 \end{bmatrix}
$$

---

#### 7.3.2 IMU → World（世界系）

**变换公式：**
$$
{}^W\mathbf{p} = {}^W_I\mathbf{R} \cdot {}^I\mathbf{p} + {}^W_I\mathbf{t}
$$

其中：
- ${}^W_I\mathbf{R}$：IMU 当前姿态（`state_point.rot`）
- ${}^W_I\mathbf{t}$：IMU 当前位置（`state_point.pos`）

**来源**：
- 由 IEKF 估计
- 融合 IMU 预积分和点云匹配

---

#### 7.3.3 Lidar → World（组合变换）

**变换公式：**
$$
{}^W\mathbf{p} = {}^W_I\mathbf{R} \cdot ({}^I_L\mathbf{R} \cdot {}^L\mathbf{p} + {}^I_L\mathbf{t}) + {}^W_I\mathbf{t}
$$

**展开：**
$$
{}^W\mathbf{p} = {}^W_I\mathbf{R} \cdot {}^I_L\mathbf{R} \cdot {}^L\mathbf{p} + {}^W_I\mathbf{R} \cdot {}^I_L\mathbf{t} + {}^W_I\mathbf{t}
$$

**代码实现**（`RGBpointBodyToWorld`，行233）：
```cpp
V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);
```

**运算顺序：**
```
1. offset_R_L_I * p_body          → 旋转到 IMU 系
2. + offset_T_L_I                  → 平移到 IMU 系
3. state_point.rot * (...)         → 旋转到世界系
4. + state_point.pos               → 平移到世界系
```

**矩阵形式（齐次坐标）：**
$$
\begin{bmatrix} {}^W\mathbf{p} \\ 1 \end{bmatrix} =
\begin{bmatrix} {}^W_I\mathbf{R} & {}^W_I\mathbf{t} \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} {}^I_L\mathbf{R} & {}^I_L\mathbf{t} \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} {}^L\mathbf{p} \\ 1 \end{bmatrix}
$$

---

### 7.4 旋转表示转换

FAST-LIO2 使用多种旋转表示：

#### 7.4.1 旋转矩阵（Rotation Matrix）
$$
\mathbf{R} \in SO(3), \quad \mathbf{R}^T\mathbf{R} = \mathbf{I}, \quad \det(\mathbf{R}) = 1
$$

**优点**：
- 坐标变换直接（$\mathbf{v}' = \mathbf{R}\mathbf{v}$）
- 组合旋转简单（$\mathbf{R}_3 = \mathbf{R}_2\mathbf{R}_1$）

**缺点**：
- 存储冗余（9个数，只有3自由度）
- 数值误差累积可能破坏正交性

---

#### 7.4.2 单位四元数（Unit Quaternion）
$$
\mathbf{q} = w + xi + yj + zk, \quad w^2+x^2+y^2+z^2=1
$$

**优点**：
- 紧凑（4个数）
- 无奇异性（无万向锁）
- 插值平滑（SLERP）
- 数值稳定

**缺点**：
- 不直观
- 坐标变换需要转换为矩阵

**四元数 → 旋转矩阵：**
$$
\mathbf{R} = \begin{bmatrix}
1-2(y^2+z^2) & 2(xy-wz) & 2(xz+wy) \\
2(xy+wz) & 1-2(x^2+z^2) & 2(yz-wx) \\
2(xz-wy) & 2(yz+wx) & 1-2(x^2+y^2)
\end{bmatrix}
$$

**代码实现**（SO3 内部）：
```cpp
state_point.rot.toRotationMatrix()  // SO3 → 旋转矩阵
```

---

#### 7.4.3 旋转向量/轴角（Axis-Angle）
$$
\boldsymbol{\phi} = \theta \mathbf{n}
$$
其中：
- $\mathbf{n}$：单位旋转轴
- $\theta = ||\boldsymbol{\phi}||$：旋转角

**优点**：
- 最紧凑（3个数）
- 李代数表示（便于优化）

**缺点**：
- 有奇异性（$\theta = \pi$）
- 坐标变换需要转换为矩阵

**旋转向量 → 旋转矩阵（Rodrigues 公式）：**
$$
\mathbf{R} = \mathbf{I} + \frac{\sin\theta}{\theta}\boldsymbol{\phi}^\wedge + \frac{1-\cos\theta}{\theta^2}(\boldsymbol{\phi}^\wedge)^2
$$

**代码实现**（so3_math.h:18-34）：
```cpp
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);
        /// Rodrigues Transformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}
```

---

#### 7.4.4 欧拉角（Euler Angles）
常见顺序：ZYX（Yaw-Pitch-Roll）

**优点**：
- 直观（航向、俯仰、横滚）

**缺点**：
- 万向锁（Gimbal Lock）
- 不适合插值
- 不适合优化

**FAST-LIO2 仅用于调试输出：**
```cpp
euler_cur = SO3ToEuler(state_point.rot);  // 用于打印日志
```

---

### 7.5 变换的逆

#### 7.5.1 旋转矩阵的逆
$$
\mathbf{R}^{-1} = \mathbf{R}^T
$$

#### 7.5.2 四元数的逆
$$
\mathbf{q}^{-1} = \mathbf{q}^* = w - xi - yj - zk
$$

#### 7.5.3 齐次变换的逆
$$
\mathbf{T}^{-1} = \begin{bmatrix} \mathbf{R} & \mathbf{t} \\ 0 & 1 \end{bmatrix}^{-1}
= \begin{bmatrix} \mathbf{R}^T & -\mathbf{R}^T\mathbf{t} \\ 0 & 1 \end{bmatrix}
$$

**应用示例**：World → Lidar
$$
{}^L\mathbf{p} = {}^I_L\mathbf{R}^T ({}^W_I\mathbf{R}^T ({}^W\mathbf{p} - {}^W_I\mathbf{t}) - {}^I_L\mathbf{t})
$$

---

## 8. ROS消息类型说明

### 8.1 std_msgs/Header

**定义：**
```cpp
uint32 seq        // 序列号（自动递增）
time stamp        // 时间戳
string frame_id   // 坐标系ID
```

**FAST-LIO2 使用：**
```cpp
msg.header.stamp = ros::Time().fromSec(lidar_end_time);
msg.header.frame_id = "camera_init";  // 或 "body"
```

---

### 8.2 geometry_msgs/Point

**定义：**
```cpp
float64 x
float64 y
float64 z
```

---

### 8.3 geometry_msgs/Quaternion

**定义：**
```cpp
float64 x
float64 y
float64 z
float64 w
```

**归一化检查：**
```cpp
// ROS 不强制，但推荐满足
x^2 + y^2 + z^2 + w^2 = 1
```

---

### 8.4 geometry_msgs/Pose

**定义：**
```cpp
Point position
Quaternion orientation
```

---

### 8.5 geometry_msgs/PoseStamped

**定义：**
```cpp
Header header
Pose pose
```

---

### 8.6 nav_msgs/Path

**定义：**
```cpp
Header header
PoseStamped[] poses
```

**注意事项：**
- `poses` 数组无限增长
- 大规模路径可能导致性能问题
- FAST-LIO2 采用抽稀策略（每10帧1个）

---

### 8.7 sensor_msgs/PointCloud2

**定义：**
```cpp
Header header
uint32 height       // 点云高度（无组织为1）
uint32 width        // 点云宽度（点数）
PointField[] fields // 字段描述（x,y,z,intensity等）
bool is_bigendian
uint32 point_step   // 单个点字节数
uint32 row_step     // 单行字节数
uint8[] data        // 二进制数据
bool is_dense       // 是否包含无效点
```

**字段示例：**
```cpp
fields[0]: name="x", offset=0, datatype=FLOAT32, count=1
fields[1]: name="y", offset=4, datatype=FLOAT32, count=1
fields[2]: name="z", offset=8, datatype=FLOAT32, count=1
fields[3]: name="intensity", offset=12, datatype=FLOAT32, count=1
...
```

---

## 9. 总结

### 9.1 核心流程总结

**步骤10 的完整数据流：**
```
                    ┌──────────────┐
                    │  EKF 状态     │
                    │ state_point  │
                    └──────┬───────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
           v               v               v
    ┌──────────┐   ┌───────────┐  ┌──────────────┐
    │路径发布   │   │世界系点云 │  │机体系点云    │
    │publish_  │   │publish_   │  │publish_      │
    │path      │   │frame_world│  │frame_body    │
    └──────────┘   └───────────┘  └──────────────┘
         │               │               │
         v               v               v
    /path话题     /cloud_registered  /cloud_registered_body
```

---

### 9.2 关键技术要点

1. **坐标变换**：Lidar → IMU → World，两级变换
2. **外参标定**：在线估计，EKF 状态包含
3. **旋转表示**：SO3（四元数）→ 旋转矩阵 → 欧拉角
4. **数据抽稀**：路径1/10，PCD按间隔
5. **点云选择**：密集/降采样可切换

---

### 9.3 常见参数配置

**高性能模式（低带宽）：**
```yaml
publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: false        # 降采样点云
  scan_bodyframe_pub_en: false   # 不发布body系
pcd_save:
  pcd_save_en: false
```

**调试模式（完整数据）：**
```yaml
publish:
  path_en: true
  scan_publish_en: true
  dense_publish_en: true         # 密集点云
  scan_bodyframe_pub_en: true    # 发布body系
pcd_save:
  pcd_save_en: true
  interval: 10                   # 每10帧保存
```

**建图模式（离线处理）：**
```yaml
publish:
  path_en: false                 # 不发布路径（节省资源）
  scan_publish_en: false         # 不发布点云
pcd_save:
  pcd_save_en: true
  interval: 1                    # 每帧保存
```

---

### 9.4 性能优化建议

1. **禁用不需要的发布**：减少 CPU 和网络负载
2. **使用降采样点云**：`dense_pub_en = false`
3. **增大 PCD 保存间隔**：减少磁盘 I/O
4. **路径抽稀比率**：可修改行759的模数（当前10）
5. **点云降采样参数**：调整 `filter_size_surf`（当前0.5m）

---

### 9.5 数学工具箱

| 工具 | 文件 | 关键函数 |
|------|------|----------|
| SO3 旋转 | so3_math.h | `Exp()`, `Log()` |
| 坐标变换 | laserMapping.cpp | `pointBodyToWorld()`, `RGBpointBodyLidarToIMU()` |
| 平面拟合 | common_lib.h | `esti_plane()` |
| 四元数转换 | use-ikfom.hpp | `SO3ToEuler()` |

---

## 10. 参考文献

1. **FAST-LIO2 论文**：
   - Xu, W., et al. "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry." IEEE Transactions on Robotics, 2022.

2. **李群与李代数**：
   - Sola, J. "Quaternion kinematics for the error-state Kalman filter." arXiv:1711.02508, 2017.

3. **IKFoM 框架**：
   - https://github.com/hku-mars/IKFoM

4. **ikd-Tree**：
   - Cai, Y., et al. "ikd-Tree: An Incremental KD Tree for Robotic Applications." arXiv:2102.10808, 2021.

---

## 附录 A：完整代码索引

| 函数/变量 | 位置 | 行号 |
|-----------|------|------|
| `publish_path()` | laserMapping.cpp | 750-764 |
| `publish_frame_world()` | laserMapping.cpp | 606-658 |
| `publish_frame_body()` | laserMapping.cpp | 660-677 |
| `RGBpointBodyToWorld()` | laserMapping.cpp | 230-239 |
| `RGBpointBodyLidarToIMU()` | laserMapping.cpp | 241-250 |
| `set_posestamp()` | laserMapping.cpp | 704-715 |
| `state_ikfom` 定义 | use-ikfom.hpp | 12-21 |
| `state_point` 声明 | laserMapping.cpp | 143 |
| `geoQuat` 声明 | laserMapping.cpp | 148 |

---

## 附录 B：常见问题 FAQ

**Q1: 为什么世界坐标系叫 "camera_init"？**
A: 继承自 LOAM 的命名传统，实际是 LIO 的全局惯性坐标系，与相机无关。

**Q2: 路径为什么抽稀1/10？**
A: 防止 RViz 崩溃。路径消息无限增长，全发布会导致内存和渲染性能问题。

**Q3: `feats_undistort` 和 `feats_down_body` 的区别？**
A: `feats_undistort` 是去畸变完整点云，`feats_down_body` 是降采样后的点云（通常10%-20%）。

**Q4: 外参标定是离线还是在线？**
A: FAST-LIO2 支持在线外参标定（`extrinsic_est_en = true`），运行中持续优化外参。

**Q5: 如何保存完整地图？**
A: 设置 `pcd_save_en = true` 和 `interval = 1`，程序结束时会在 `ROOT_DIR/PCD/` 保存所有帧。

---

**文档版本**：1.0
**最后更新**：2025-10-05
**作者**：Claude (Anthropic)
**项目**：FAST-LIO2
**许可**：MIT License
