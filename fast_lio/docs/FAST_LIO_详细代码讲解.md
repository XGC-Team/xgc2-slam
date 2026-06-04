# FAST-LIO2 完整代码详解

## 目录
- [1. 系统概述](#1-系统概述)
- [2. 核心数据结构](#2-核心数据结构)
- [3. 数学原理](#3-数学原理)
- [4. publish_odometry函数详解](#4-publish_odometry函数详解)
- [5. 主函数完整流程](#5-主函数完整流程)
- [6. 关键子函数详解](#6-关键子函数详解)

---

## 1. 系统概述

### 1.1 FAST-LIO2简介

FAST-LIO2 (Fast LiDAR-Inertial Odometry) 是一个计算高效且鲁棒的激光雷达惯性里程计系统。

**核心特点：**
- 使用**迭代扩展卡尔曼滤波器 (IEKF)** 进行状态估计
- 采用**增量式KD树 (ikd-Tree)** 进行高效地图管理
- 支持多种激光雷达类型（Livox Avia、Velodyne等）
- 实时性能优异，适用于无人机、机器人等移动平台

**算法流程：**
```
传感器数据 → 数据同步 → IMU预积分 → 点云去畸变 →
点云配准 → IEKF更新 → 地图更新 → 发布里程计
```

---

## 2. 核心数据结构

### 2.1 状态向量 `state_ikfom`

FAST-LIO2使用流形空间上的状态表示（定义在 `use-ikfom.hpp`）：

```cpp
MTK_BUILD_MANIFOLD(state_ikfom,
    ((vect3, pos))           // 位置 (3D)
    ((SO3, rot))             // 旋转 (SO(3)流形, 3自由度)
    ((SO3, offset_R_L_I))    // LiDAR到IMU外参旋转
    ((vect3, offset_T_L_I))  // LiDAR到IMU外参平移
    ((vect3, vel))           // 速度 (3D)
    ((vect3, bg))            // 陀螺仪偏差
    ((vect3, ba))            // 加速度计偏差
    ((S2, grav))             // 重力方向 (S2流形, 2自由度)
);
```

**状态维度分析：**
- **总维度**: 23维
  - 位置: 3维
  - 旋转: 3维（SO(3)的切空间）
  - 外参旋转: 3维
  - 外参平移: 3维
  - 速度: 3维
  - 陀螺仪偏差: 3维
  - 加速度计偏差: 3维
  - 重力: 2维（S2球面，受约束）

**为什么使用流形？**
- 旋转不是欧几里得空间，直接加法无意义
- SO(3)保证旋转矩阵的正交性和单位行列式
- S2保证重力向量的模长恒定

### 2.2 测量组 `MeasureGroup`

```cpp
struct MeasureGroup {
    double lidar_beg_time;              // 激光扫描起始时间
    double lidar_end_time;              // 激光扫描结束时间
    PointCloudXYZI::Ptr lidar;          // 点云数据
    deque<sensor_msgs::Imu::ConstPtr> imu;  // 时间段内的IMU数据队列
};
```

**作用：** 将LiDAR和IMU数据打包，确保时间同步

### 2.3 点云类型定义

```cpp
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
```

**字段说明：**
- `x, y, z`: 3D坐标
- `intensity`: 反射强度
- `normal_x, normal_y, normal_z`: 法向量（用于存储平面法向量）
- `curvature`: 曲率（用于存储时间戳，单位：毫秒）

---

## 3. 数学原理

### 3.1 误差状态卡尔曼滤波器 (ESKF)

FAST-LIO2使用**误差状态**而非**标称状态**进行滤波。

#### 3.1.1 状态定义

**标称状态 (Nominal State):**
```
x = [p, R, R_LI, t_LI, v, b_g, b_a, g]^T
```

**误差状态 (Error State):**
```
δx = [δp, δθ, δθ_LI, δt_LI, δv, δb_g, δb_a, δg]^T
```

其中 `δθ` 是旋转的切空间表示（李代数）

#### 3.1.2 状态预测方程

**运动学模型：**

```cpp
// 位置: p_dot = v
res(i) = s.vel[i];

// 旋转: R_dot = R * skew(ω)  (其中 ω = gyro - b_g)
res(i + 3) = omega[i];

// 速度: v_dot = R * (a - b_a) + g
res(i + 12) = a_inertial[i] + s.grav[i];
```

**数学形式：**
$$
\begin{aligned}
\dot{p} &= v \\
\dot{R} &= R \cdot [\omega - b_g]_\times \\
\dot{v} &= R \cdot (a - b_a) + g \\
\dot{b}_g &= n_{bg} \\
\dot{b}_a &= n_{ba}
\end{aligned}
$$

其中 $[\cdot]_\times$ 表示反对称矩阵（叉乘矩阵）。

#### 3.1.3 状态转移雅可比矩阵

**F矩阵（24×23维）：**

```cpp
Eigen::Matrix<double, 24, 23> df_dx(state_ikfom &s, const input_ikfom &in) {
    // ∂p_dot/∂v = I
    cov.block<3, 3>(0, 12) = I;

    // ∂v_dot/∂R = -R * [a - b_a]_×
    cov.block<3, 3>(12, 3) = -s.rot * hat(acc_);

    // ∂v_dot/∂b_a = -R
    cov.block<3, 3>(12, 18) = -s.rot;

    // ∂v_dot/∂g (S2流形的切空间)
    cov.block<3, 2>(12, 21) = grav_matrix;

    // ∂R_dot/∂b_g = -I
    cov.block<3, 3>(3, 15) = -I;
}
```

#### 3.1.4 过程噪声协方差

**Q矩阵（12×12维）：**

```cpp
MTK::get_cov<process_noise_ikfom>::type process_noise_cov() {
    cov = Zero;
    setDiagonal(cov, ng,  0.0001);  // 陀螺仪噪声
    setDiagonal(cov, na,  0.0001);  // 加速度计噪声
    setDiagonal(cov, nbg, 0.00001); // 陀螺仪偏差随机游走
    setDiagonal(cov, nba, 0.00001); // 加速度计偏差随机游走
    return cov;
}
```

### 3.2 点云配准（点到面ICP）

#### 3.2.1 观测模型

对于第 $i$ 个点：

1. **最近邻搜索：** 在地图中找到 5 个最近点
2. **平面拟合：** 用最小二乘法拟合平面 $ax + by + cz + d = 0$
3. **残差计算：** 点到平面的距离

**平面拟合数学：**

```
给定5个点 {p_j}, j=1,...,5
平面方程: n^T * p + d = 0  (归一化: ||n|| = 1)
```

求解方程组：
$$
\begin{bmatrix}
x_1 & y_1 & z_1 \\
x_2 & y_2 & z_2 \\
\vdots & \vdots & \vdots \\
x_5 & y_5 & z_5
\end{bmatrix}
\begin{bmatrix}
a/d \\ b/d \\ c/d
\end{bmatrix}
=
\begin{bmatrix}
-1 \\ -1 \\ \vdots \\ -1
\end{bmatrix}
$$

代码实现：
```cpp
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point) {
    Matrix<T, 5, 3> A;
    Matrix<T, 5, 1> b = -ones();

    // 构建方程组
    for (int j = 0; j < 5; j++) {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    // QR分解求解
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    // 归一化
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;  // n_x
    pca_result(1) = normvec(1) / n;  // n_y
    pca_result(2) = normvec(2) / n;  // n_z
    pca_result(3) = 1.0 / n;         // d
}
```

#### 3.2.2 观测雅可比矩阵

**观测方程：**
$$
h(x) = n^T \cdot (R \cdot (R_{LI} \cdot p_L + t_{LI}) + t) + d
$$

其中：
- $p_L$: 点在LiDAR坐标系下的坐标
- $R_{LI}, t_{LI}$: LiDAR到IMU的外参
- $R, t$: IMU在世界坐标系的位姿
- $n, d$: 平面参数

**H矩阵（1×12维，对于单个点）：**

```cpp
// 对位置的导数: ∂h/∂t = n^T
ekfom_data.h_x.block<1, 3>(i, 0) << norm_p.x, norm_p.y, norm_p.z;

// 对旋转的导数: ∂h/∂θ = n^T * [R*(R_LI*p_L + t_LI)]_×
V3D point_this = s.offset_R_L_I * point_body + s.offset_T_L_I;
V3D A = point_crossmat * (s.rot.conjugate() * norm_vec);
ekfom_data.h_x.block<1, 3>(i, 3) << A(0), A(1), A(2);

// 对外参旋转的导数: ∂h/∂θ_LI
V3D B = point_be_crossmat * s.offset_R_L_I.conjugate() * C;
ekfom_data.h_x.block<1, 3>(i, 6) << B(0), B(1), B(2);

// 对外参平移的导数: ∂h/∂t_LI = n^T * R
V3D C = s.rot.conjugate() * norm_vec;
ekfom_data.h_x.block<1, 3>(i, 9) << C(0), C(1), C(2);
```

### 3.3 迭代扩展卡尔曼滤波 (IEKF)

**标准EKF问题：** 观测方程高度非线性时，一次线性化误差大

**IEKF解决方案：** 在同一组观测上多次迭代

**迭代过程（第k次迭代）：**

```
1. 线性化观测方程: H_k = ∂h/∂x |_{x_k}
2. 计算卡尔曼增益: K_k = P * H_k^T * (H_k * P * H_k^T + R)^(-1)
3. 更新状态: x_{k+1} = x_k + K_k * (z - h(x_k))
4. 检查收敛: ||x_{k+1} - x_k|| < ε ?
```

代码调用：
```cpp
kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
```

**收敛准则：**
- 达到最大迭代次数（NUM_MAX_ITERATIONS = 4）
- 状态变化量小于阈值（epsi = 0.001）

---

## 4. publish_odometry函数详解

### 4.1 函数签名与位置

**文件位置：** `src/FAST_LIO/src/laserMapping.cpp:717`

**函数原型：**
```cpp
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
```

**调用位置：** `laserMapping.cpp:1168` (主循环的步骤8)

### 4.2 逐行代码讲解

#### 第719-721行：设置消息头

```cpp
odomAftMapped.header.frame_id = "camera_init";
odomAftMapped.child_frame_id = "body";
odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
```

**详细说明：**

**`odomAftMapped`变量：**
- **类型：** `nav_msgs::Odometry`（定义于 line 147）
- **作用：** 存储优化后的里程计信息

**`header.frame_id = "camera_init"`：**
- **含义：** 父坐标系（世界坐标系）
- **命名原因：** 继承自LOAM的命名惯例
- **实际意义：** 第一帧LiDAR的坐标系，作为全局参考系

**`child_frame_id = "body"`：**
- **含义：** 子坐标系（机体坐标系，通常是IMU坐标系）
- **作用：** 表示里程计描述的是body相对于camera_init的位姿

**`header.stamp`：**
- **时间戳：** `lidar_end_time`（当前帧点云扫描结束时刻）
- **重要性：** 用于TF树的时间同步和多传感器融合

**坐标系关系图：**
```
camera_init (世界系)
    └─> body (IMU系)
          └─> lidar (LiDAR系, 通过外参offset_R_L_I, offset_T_L_I连接)
```

#### 第722行：填充位姿信息

```cpp
set_posestamp(odomAftMapped.pose);
```

**`set_posestamp`函数定义（line 704-715）：**

```cpp
template<typename T>
void set_posestamp(T & out) {
    // 设置位置 (从EKF状态中获取)
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    // 设置姿态四元数 (已在line 1160-1163转换好)
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}
```

**涉及的全局变量：**

**`state_point`（line 143）：**
- **类型：** `state_ikfom`（23维状态向量）
- **来源：** `state_point = kf.get_x();` (line 1155)
- **内容：** EKF估计的当前状态（位置、旋转、速度、偏差等）

**`geoQuat`（line 148）：**
- **类型：** `geometry_msgs::Quaternion`
- **转换过程（line 1160-1163）：**
  ```cpp
  // state_point.rot 是 SO3 类型（四元数内部表示）
  geoQuat.x = state_point.rot.coeffs()[0];  // q_x
  geoQuat.y = state_point.rot.coeffs()[1];  // q_y
  geoQuat.z = state_point.rot.coeffs()[2];  // q_z
  geoQuat.w = state_point.rot.coeffs()[3];  // q_w (标量部分)
  ```

**四元数表示法：**
- **定义：** $q = w + xi + yj + zk$
- **旋转公式：** $v' = q \otimes v \otimes q^*$
- **优点：** 无奇异性、计算效率高、易于插值

#### 第723行：发布里程计消息

```cpp
pubOdomAftMapped.publish(odomAftMapped);
```

**ROS发布流程：**
1. 将消息序列化为字节流
2. 通过TCP/UDP发送给所有订阅者
3. 订阅者反序列化并处理

**话题名称：** `/Odometry`（定义于 line 1039-1040）

#### 第724-734行：填充协方差矩阵

```cpp
auto P = kf.get_P();  // 获取EKF的协方差矩阵 (23×23)
for (int i = 0; i < 6; i ++)
{
    int k = i < 3 ? i + 3 : i - 3;
    odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
    odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
    odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
    odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
    odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
    odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
}
```

**详细解析：**

**`P` 矩阵：**
- **维度：** 23×23
- **内容：** 状态估计的不确定度
- **物理意义：** $P_{ij}$ 表示状态 $i$ 和状态 $j$ 的协方差

**状态顺序（0-22）：**
```
0-2:   位置 (p_x, p_y, p_z)
3-5:   旋转 (θ_x, θ_y, θ_z)
6-8:   外参旋转
9-11:  外参平移
12-14: 速度
15-17: 陀螺仪偏差
18-20: 加速度计偏差
21-22: 重力
```

**`odomAftMapped.pose.covariance`：**
- **类型：** `boost::array<double, 36>` (6×6矩阵，行优先)
- **ROS标准：** 前3行3列是位置协方差，后3行3列是姿态协方差

**索引映射关系：**

**当 i = 0（位置x）：**
```cpp
k = 0 + 3 = 3  // 对应旋转的索引
covariance[0*6 + 0] = P(3, 3)  // 旋转x与位置x的协方差
covariance[0*6 + 1] = P(3, 4)  // 旋转x与位置y的协方差
covariance[0*6 + 2] = P(3, 5)  // 旋转x与位置z的协方差
covariance[0*6 + 3] = P(3, 0)  // 旋转x与旋转x的协方差
covariance[0*6 + 4] = P(3, 1)  // 旋转x与旋转y的协方差
covariance[0*6 + 5] = P(3, 2)  // 旋转x与旋转z的协方差
```

**映射逻辑：**
```
ROS协方差顺序: [x, y, z, roll, pitch, yaw]
EKF状态顺序:   [p_x, p_y, p_z, θ_x, θ_y, θ_z, ...]

目标: 构建6×6协方差矩阵
前3行(i=0,1,2): 填充旋转(k=3,4,5)与位置/旋转的协方差
后3行(i=3,4,5): 填充位置(k=0,1,2)与位置/旋转的协方差
```

**为什么这样映射？**
- ROS标准：位姿协方差矩阵应该描述6自由度的不确定度
- EKF输出：位置和旋转是解耦的
- 交叉协方差：反映位置和姿态估计的相关性

#### 第736-747行：发布TF变换

```cpp
static tf::TransformBroadcaster br;      // TF广播器（静态变量，避免重复构造）
tf::Transform                   transform;
tf::Quaternion                  q;

// 设置平移部分
transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                odomAftMapped.pose.pose.position.y,
                                odomAftMapped.pose.pose.position.z));

// 设置旋转部分
q.setW(odomAftMapped.pose.pose.orientation.w);
q.setX(odomAftMapped.pose.pose.orientation.x);
q.setY(odomAftMapped.pose.pose.orientation.y);
q.setZ(odomAftMapped.pose.pose.orientation.z);
transform.setRotation(q);

// 发布TF树
br.sendTransform(tf::StampedTransform(transform,
                                      odomAftMapped.header.stamp,
                                      "camera_init",
                                      "body"));
```

**TF（Transform）系统：**

**作用：**
- 维护坐标系之间的变换关系
- 支持任意两个坐标系之间的查询
- 自动处理时间插值

**发布内容：**
- **父系：** `camera_init`（世界坐标系）
- **子系：** `body`（机体坐标系）
- **变换：** 4×4齐次变换矩阵 $T = [R | t; 0 | 1]$
- **时间戳：** `lidar_end_time`

**TF树结构：**
```
camera_init (世界系，固定)
    └─> body (IMU系，随时间变化)
          └─> lidar (由URDF或静态发布器定义)
```

**应用场景：**
1. **传感器融合：** 将不同传感器的数据转换到统一坐标系
2. **可视化：** RViz根据TF树正确显示各个坐标系
3. **路径规划：** 需要知道机器人相对于地图的位姿

### 4.3 函数总结

**`publish_odometry` 完整功能：**

1. **构建里程计消息：** 从EKF状态提取位置和姿态
2. **填充协方差：** 提供状态估计的不确定度信息
3. **发布话题：** 供下游节点（如路径规划、控制器）使用
4. **广播TF：** 维护全局坐标系树

**数据流图：**
```
EKF状态 (state_point, P矩阵)
    │
    ├─> 位置 + 四元数 ──> nav_msgs::Odometry ──> /Odometry话题
    │
    ├─> 协方差矩阵 ──────┘
    │
    └─> TF变换 ──────────> tf::TransformBroadcaster ──> /tf话题
```

---

## 5. 主函数完整流程

### 5.1 初始化阶段（line 916-1042）

#### 步骤1：ROS节点初始化

```cpp
ros::init(argc, argv, "laserMapping");
ros::NodeHandle nh;
```

#### 步骤2：参数加载

**发布选项：**
```cpp
nh.param<bool>("publish/path_en", path_en, true);
nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
```

**优化参数：**
```cpp
nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);  // IEKF最大迭代次数
```

**IMU噪声参数：**
```cpp
nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);      // 陀螺仪协方差
nh.param<double>("mapping/acc_cov", acc_cov, 0.1);      // 加速度计协方差
nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);  // 陀螺仪偏差
nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);  // 加速度计偏差
```

**外参标定：**
```cpp
nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

// 转换为Eigen矩阵
Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);  // 展开为 extrinT[0], extrinT[1], extrinT[2]
Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);  // 展开为3×3矩阵
```

#### 步骤3：配置预处理器和IMU处理器

```cpp
p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
```

#### 步骤4：初始化EKF

```cpp
double epsi[23] = {0.001};  // 收敛阈值
fill(epsi, epsi+23, 0.001);
kf.init_dyn_share(get_f,           // 状态转移函数
                  df_dx,           // 状态雅可比
                  df_dw,           // 噪声雅可比
                  h_share_model,   // 观测模型
                  NUM_MAX_ITERATIONS,
                  epsi);
```

**EKF函数说明：**

**`get_f`（line 47-59）：**
```cpp
// 返回状态导数 dx/dt
Eigen::Matrix<double, 24, 1> get_f(state_ikfom &s, const input_ikfom &in) {
    res(0:2)   = s.vel;                        // p_dot = v
    res(3:5)   = gyro - s.bg;                  // θ_dot = ω
    res(12:14) = R*(acc - s.ba) + s.grav;      // v_dot = a
    return res;
}
```

**`df_dx`（line 61-77）：**状态转移的雅可比矩阵 $F = \frac{\partial f}{\partial x}$

**`df_dw`（line 80-88）：**噪声雅可比矩阵 $G = \frac{\partial f}{\partial w}$

**`h_share_model`（line 779-906）：**观测模型（点到面距离）

#### 步骤5：订阅和发布器

```cpp
// 订阅器
ros::Subscriber sub_pcl = (p_pre->lidar_type == AVIA) ?
    nh.subscribe(lid_topic, 200000, livox_pcl_cbk) :
    nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

// 发布器
ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
```

### 5.2 主循环（line 1044-1215）

#### 循环框架

```cpp
signal(SIGINT, SigHandle);  // 注册信号处理器
ros::Rate rate(5000);       // 5000 Hz循环频率
while (ros::ok()) {
    ros::spinOnce();        // 处理回调函数

    if (sync_packages(Measures)) {
        // 主处理逻辑
    }

    rate.sleep();
}
```

#### 步骤1：数据同步（line 1054）

```cpp
bool sync_packages(MeasureGroup &meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) return false;

    // 取出一帧LiDAR
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    meas.lidar_end_time = lidar_beg_time + last_point.curvature/1000.0;

    // 等待足够的IMU数据覆盖整个LiDAR扫描周期
    if (last_timestamp_imu < lidar_end_time) return false;

    // 提取时间范围内的所有IMU
    while (imu_time < lidar_end_time) {
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    return true;
}
```

**同步策略图：**
```
时间轴: ─────────────────────────────────>
IMU:    ●●●●●●●●●●●●●●●●●●●●●●●●● (200Hz)
LiDAR:  [━━━━━━扫描━━━━━━] (10Hz)
        ↑                 ↑
     beg_time        end_time

取出: 所有在 [beg_time, end_time] 内的IMU数据
```

#### 步骤2：IMU预积分和点云去畸变（line 1076）

```cpp
p_imu->Process(Measures, kf, feats_undistort);
```

**ImuProcess::Process 详细流程：**

**2.1 IMU初始化（首次调用）：**
```cpp
void IMU_init(const MeasureGroup &meas, esekfom::esekf &kf_state, int &N) {
    // 增量式计算均值
    for (auto &imu : meas.imu) {
        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        // Welford算法计算方差
        cov_acc = cov_acc*(N-1)/N + (cur_acc-mean_acc)^2 * (N-1)/(N*N);
        N++;
    }

    // 初始化状态
    init_state.grav = -mean_acc / ||mean_acc|| * 9.81;  // 重力方向
    init_state.bg = mean_gyr;                            // 陀螺仪偏差
}
```

**为什么需要初始化？**
- 估计静止时的重力方向
- 标定陀螺仪零偏
- 计算传感器噪声水平

**2.2 点云去畸变：**

```cpp
void UndistortPcl(const MeasureGroup &meas, esekfom::esekf &kf_state, PointCloudXYZI &pcl_out) {
    // 步骤A: IMU前向传播
    for (auto it = imu.begin(); it < imu.end()-1; it++) {
        // 中值积分
        angvel_avr = 0.5 * (head->gyro + tail->gyro);
        acc_avr = 0.5 * (head->acc + tail->acc);
        dt = tail->time - head->time;

        // EKF预测
        kf_state.predict(dt, Q, in);

        // 保存每个IMU时刻的位姿
        IMUpose.push_back(pose_at_time);
    }

    // 步骤B: 点云反向传播去畸变
    for (auto it_pcl = pcl.end()-1; it_pcl >= pcl.begin(); it_pcl--) {
        // 找到点对应的IMU时刻
        t_point = it_pcl->curvature / 1000.0;

        // 插值得到该时刻的位姿
        R_i = R_head * Exp(angvel_avr * dt);
        T_i = pos_head + vel_head*dt + 0.5*acc*dt^2;

        // 将点转换到扫描结束时刻
        P_compensate = R_end^T * (R_i*P_i + T_i - T_end);

        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
    }
}
```

**去畸变数学原理：**

假设点 $p_i$ 在时刻 $t_i$ 采集，需要转换到扫描结束时刻 $t_e$：

$$
\begin{aligned}
p_e &= R_e^T \cdot (R_i \cdot p_i + (T_i - T_e)) \\
\text{其中:} \\
R_i &= R_{t_i \rightarrow \text{world}} \text{ (IMU预积分得到)} \\
T_i &= T_{t_i \rightarrow \text{world}} \\
R_e, T_e &= \text{扫描结束时刻的位姿}
\end{aligned}
$$

#### 步骤3：局部地图FOV分割（line 1091）

```cpp
void lasermap_fov_segment() {
    // 首次初始化
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len/2;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len/2;
        }
        return;
    }

    // 检查是否需要移动地图
    for (int i = 0; i < 3; i++) {
        dist_to_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

        if (dist_to_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            need_move = true;
        }
    }

    // 移动地图
    if (need_move) {
        mov_dist = max((cube_len - 2*MOV_THRESHOLD*DET_RANGE)*0.5*0.9,
                       DET_RANGE*(MOV_THRESHOLD-1));

        // 更新边界并记录删除区域
        for (int i = 0; i < 3; i++) {
            if (dist_to_edge[i][0] <= threshold) {
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                cub_needrm.push_back(old_region);  // 记录右侧需删除的区域
            }
        }

        // 从ikd-tree删除点
        ikdtree.Delete_Point_Boxes(cub_needrm);
    }
}
```

**地图移动示意图（1维）：**
```
初始状态:
|───────────cube_len=200m───────────|
min                 pos                max
●━━━━━━━━━━━━━━━━━●━━━━━━━━━━━━━━━━━●
        100m            100m

当 pos 接近 min 时 (距离 < MOV_THRESHOLD*DET_RANGE):
移动后:
|───────────cube_len=200m───────────|
   new_min          pos            new_max
   ●━━━━━━━━━━━━━━━●━━━━━━━━━━━━━━━━━●
    [删除这部分]●━━━━━●

保持 pos 始终在地图中心附近
```

#### 步骤4：点云降采样（line 1094-1097）

```cpp
downSizeFilterSurf.setInputCloud(feats_undistort);
downSizeFilterSurf.filter(*feats_down_body);
feats_down_size = feats_down_body->points.size();
```

**VoxelGrid降采样：**
- **原理：** 将空间划分为体素网格，每个体素内只保留一个点（通常是质心）
- **体素大小：** `filter_size_surf_min`（例如0.5m）
- **作用：** 减少计算量，同时保持点云的几何特征

#### 步骤5：初始化ikd-tree（line 1100-1113）

```cpp
if (ikdtree.Root_Node == nullptr) {
    if (feats_down_size > 5) {
        ikdtree.set_downsample_param(filter_size_map_min);

        // 将点转换到世界坐标系
        for (int i = 0; i < feats_down_size; i++) {
            pointBodyToWorld(&feats_down_body->points[i],
                           &feats_down_world->points[i]);
        }

        // 构建初始树
        ikdtree.Build(feats_down_world->points);
    }
    continue;  // 跳过第一帧
}
```

#### 步骤6：IEKF迭代更新（line 1154）

```cpp
kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
```

**IEKF内部流程：**

```cpp
void update_iterated_dyn_share_modified(double R, double &solve_H_time) {
    int iteration = 0;
    state_ikfom x_propagated = get_x();

    for (iteration = 0; iteration < maximum_iter; iteration++) {
        // 步骤1: 调用观测模型，计算H和h
        dyn_share_datastruct<double> dyn_share;
        h_dyn_share_in(x_propagated, dyn_share);

        if (!dyn_share.valid) break;  // 无有效观测

        // 步骤2: 构建观测协方差矩阵 R (对角阵)
        Eigen::MatrixXd R_mat = R * I(effct_feat_num, effct_feat_num);

        // 步骤3: 计算卡尔曼增益
        // K = P * H^T * (H*P*H^T + R)^(-1}
        Eigen::MatrixXd K = P * H.transpose() *
                           (H * P * H.transpose() + R_mat).inverse();

        // 步骤4: 状态更新
        // Δx = K * (h - H*x)  (残差)
        Eigen::MatrixXd delta_x = K * (dyn_share.h - H * x_propagated);

        // 步骤5: 流形空间的状态更新
        x_propagated = x_propagated + delta_x;  // 使用重载的+运算符

        // 步骤6: 检查收敛
        if (delta_x.norm() < epsi[0]) {
            dyn_share.converge = true;
            break;
        }
    }

    // 步骤7: 协方差更新 (Joseph形式，数值稳定)
    // P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    Eigen::MatrixXd I_KH = I - K * H;
    P = I_KH * P * I_KH.transpose() + K * R_mat * K.transpose();

    // 步骤8: 保存更新后的状态
    change_x(x_propagated);
    change_P(P);
}
```

**观测模型 `h_share_model` 详解（line 779-906）：**

```cpp
void h_share_model(state_ikfom &s, dyn_share_datastruct &ekfom_data) {
    // 步骤A: 最近邻搜索和平面拟合（并行）
    #pragma omp parallel for
    for (int i = 0; i < feats_down_size; i++) {
        // A1: 将点转换到世界坐标系
        V3D p_global = s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos;

        // A2: 在ikd-tree中搜索最近的5个点
        if (ekfom_data.converge) {
            ikdtree.Nearest_Search(p_global, 5, points_near, distances);
            point_selected[i] = (points_near.size()==5 && distances[4]<5);
        }

        if (!point_selected[i]) continue;

        // A3: 拟合平面 ax+by+cz+d=0
        if (esti_plane(pabcd, points_near, 0.1)) {
            // A4: 计算点到平面距离
            float pd2 = pabcd(0)*p.x + pabcd(1)*p.y + pabcd(2)*p.z + pabcd(3);

            // A5: 质量检查（距离阈值）
            if (s > 0.9) {  // s = 1 - 0.9*|pd2|/||p||
                point_selected[i] = true;
                normvec[i] = pabcd(0:2);  // 法向量
                normvec[i].intensity = pd2;  // 距离（残差）
            }
        }
    }

    // 步骤B: 收集有效特征
    effct_feat_num = 0;
    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body[i];
            corr_normvect->points[effct_feat_num] = normvec[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        return;
    }

    // 步骤C: 计算雅可比矩阵H和残差h
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        V3D p_body(laser_p.x, laser_p.y, laser_p.z);
        V3D p_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        // ∂h/∂p = n^T
        h_x.block<1,3>(i,0) = norm_vec.transpose();

        // ∂h/∂θ = n^T * [R*p_imu]_×
        h_x.block<1,3>(i,3) = norm_vec.transpose() * skew(s.rot*p_imu);

        // ∂h/∂θ_LI = n^T * R * [R_LI*p_body]_×
        h_x.block<1,3>(i,6) = norm_vec.transpose() * s.rot * skew(s.offset_R_L_I*p_body);

        // ∂h/∂t_LI = n^T * R
        h_x.block<1,3>(i,9) = norm_vec.transpose() * s.rot.toRotationMatrix();

        // 残差
        h(i) = -norm_p.intensity;  // 点到面距离（取负号）
    }
}
```

#### 步骤7：获取更新后的状态（line 1155-1163）

```cpp
state_point = kf.get_x();
euler_cur = SO3ToEuler(state_point.rot);
pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

// 转换为四元数
geoQuat.x = state_point.rot.coeffs()[0];
geoQuat.y = state_point.rot.coeffs()[1];
geoQuat.z = state_point.rot.coeffs()[2];
geoQuat.w = state_point.rot.coeffs()[3];
```

#### 步骤8：发布里程计（line 1168）

```cpp
publish_odometry(pubOdomAftMapped);
```
（详见第4章）

#### 步骤9：地图增量更新（line 1172）

```cpp
void map_incremental() {
    PointVector PointToAdd;

    for (int i = 0; i < feats_down_size; i++) {
        // 转换到世界系
        pointBodyToWorld(&feats_down_body[i], &feats_down_world[i]);

        // 降采样判断
        if (!Nearest_Points[i].empty()) {
            // 计算体素中心
            mid_point.x = floor(p.x/voxel_size)*voxel_size + 0.5*voxel_size;
            mid_point.y = floor(p.y/voxel_size)*voxel_size + 0.5*voxel_size;
            mid_point.z = floor(p.z/voxel_size)*voxel_size + 0.5*voxel_size;

            // 检查最近邻是否在同一体素
            bool need_add = true;
            for (auto &np : Nearest_Points[i]) {
                if (calc_dist(np, mid_point) < calc_dist(p, mid_point)) {
                    need_add = false;  // 已有更好的点
                    break;
                }
            }

            if (need_add) PointToAdd.push_back(feats_down_world[i]);
        }
    }

    // 批量添加
    ikdtree.Add_Points(PointToAdd, true);
}
```

**智能降采样策略：**
```
3D空间体素化:
┌─────┬─────┬─────┐
│  ●  │     │  ●  │  ●: 已有点
├─────┼─────┼─────┤  ○: 新点
│     │  ○  │     │  ✓: 添加
│     │  ✓  │     │  ✗: 跳过
├─────┼─────┼─────┤
│  ●  │  ○  │     │
│     │  ✗  │     │
└─────┴─────┴─────┘

规则: 每个体素内只保留最接近中心的点
```

#### 步骤10：发布可视化信息（line 1176-1180）

```cpp
if (path_en)                         publish_path(pubPath);
if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
```

---

## 6. 关键子函数详解

### 6.1 IMU初始化 `IMU_init`

**位置：** `IMU_Processing.hpp:184`

**目标：** 在静止假设下标定传感器

**算法：Welford在线方差算法**

$$
\begin{aligned}
\mu_n &= \mu_{n-1} + \frac{x_n - \mu_{n-1}}{n} \\
\sigma^2_n &= \sigma^2_{n-1} \cdot \frac{n-1}{n} + \frac{(x_n - \mu_n)^2(n-1)}{n^2}
\end{aligned}
$$

**优点：**
- 单次遍历
- 内存占用常数
- 数值稳定

### 6.2 平面拟合 `esti_plane`

**位置：** `common_lib.h:226`

**数学模型：**

给定5个点 $\{p_i\}$，拟合平面 $n^T p + d = 0$

构建超定方程组：
$$
\begin{bmatrix}
x_1 & y_1 & z_1 \\
\vdots & \vdots & \vdots \\
x_5 & y_5 & z_5
\end{bmatrix}
\begin{bmatrix}
n_x/d \\ n_y/d \\ n_z/d
\end{bmatrix}
=
\begin{bmatrix}
-1 \\ \vdots \\ -1
\end{bmatrix}
$$

**求解方法：QR分解**
```cpp
normvec = A.colPivHouseholderQr().solve(b);
```

**归一化：**
$$
n = \frac{\text{normvec}}{||\text{normvec}||}, \quad d = \frac{1}{||\text{normvec}||}
$$

### 6.3 坐标变换 `pointBodyToWorld`

**位置：** `laserMapping.cpp:208`

**变换链：**
```
LiDAR → IMU → World
```

**数学公式：**
$$
p_w = R_w^i \cdot (R_i^l \cdot p_l + t_i^l) + t_w^i
$$

其中：
- $p_l$: LiDAR系坐标
- $R_i^l, t_i^l$: LiDAR到IMU外参
- $R_w^i, t_w^i$: IMU在世界系的位姿

**代码实现：**
```cpp
V3D p_body(pi->x, pi->y, pi->z);
V3D p_global = state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos;
```

### 6.4 ikd-Tree操作

**增量式KD树特点：**
- **动态平衡：** 自动重平衡
- **删除操作：** 懒删除+定期重建
- **并发安全：** 读写锁

**关键操作：**

**最近邻搜索：**
```cpp
ikdtree.Nearest_Search(point, 5, points_near, distances);
```

**批量添加：**
```cpp
ikdtree.Add_Points(PointToAdd, downsample=true);
```

**区域删除：**
```cpp
ikdtree.Delete_Point_Boxes(cub_needrm);
```

---

## 7. 完整数据流图

```
┌─────────────────────────────────────────────────────────────┐
│                        传感器输入                            │
└───────────────┬─────────────────┬───────────────────────────┘
                │                 │
         ┌──────▼──────┐   ┌──────▼──────┐
         │ IMU (200Hz) │   │LiDAR (10Hz) │
         └──────┬──────┘   └──────┬───────┘
                │                 │
         ┌──────▼──────┐   ┌──────▼──────┐
         │ imu_buffer  │   │lidar_buffer │
         └──────┬──────┘   └──────┬───────┘
                │                 │
                └────────┬────────┘
                         │
                  ┌──────▼──────┐
                  │sync_packages│ ← 数据同步
                  └──────┬──────┘
                         │
                  ┌──────▼──────┐
                  │ MeasureGroup│
                  └──────┬──────┘
                         │
          ┌──────────────┴──────────────┐
          │                             │
   ┌──────▼──────┐             ┌────────▼────────┐
   │ IMU_init    │(首次)       │  UndistortPcl   │
   │ ·估计重力   │             │  ·IMU预积分     │
   │ ·标定偏差   │             │  ·点云去畸变    │
   └──────┬──────┘             └────────┬────────┘
          │                             │
          └──────────────┬──────────────┘
                         │
                  ┌──────▼──────────┐
                  │ feats_undistort │ ← 去畸变点云
                  └──────┬──────────┘
                         │
                  ┌──────▼──────────┐
                  │   VoxelGrid     │ ← 降采样
                  │   downsample    │
                  └──────┬──────────┘
                         │
                  ┌──────▼──────────┐
                  │ feats_down_body │
                  └──────┬──────────┘
                         │
          ┌──────────────┴──────────────┐
          │                             │
   ┌──────▼──────┐             ┌────────▼────────┐
   │lasermap_fov │             │   ikd-Tree      │
   │  segment    │             │  ·最近邻搜索    │
   │  ·地图移动  │◄───────────►│  ·平面拟合      │
   │  ·删除点    │             │  ·增量更新      │
   └─────────────┘             └────────┬────────┘
                                        │
                                 ┌──────▼──────┐
                                 │h_share_model│ ← 观测模型
                                 │ ·计算H矩阵  │
                                 │ ·计算残差h  │
                                 └──────┬──────┘
                                        │
                                 ┌──────▼──────┐
                                 │    IEKF     │ ← 状态估计
                                 │  迭代更新   │
                                 └──────┬──────┘
                                        │
                         ┌──────────────┼──────────────┐
                         │              │              │
                  ┌──────▼──────┐┌──────▼──────┐┌─────▼─────┐
                  │state_point  ││   P矩阵     ││   位姿    │
                  └──────┬──────┘└──────┬──────┘└─────┬─────┘
                         │              │             │
                         └──────┬───────┴─────────────┘
                                │
                         ┌──────▼──────────┐
                         │publish_odometry │ ← 发布结果
                         │ ·Odometry话题   │
                         │ ·TF广播         │
                         │ ·Path轨迹       │
                         └─────────────────┘
```

---

## 8. 性能优化技巧

### 8.1 并行计算

```cpp
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif
for (int i = 0; i < feats_down_size; i++) {
    // 最近邻搜索和平面拟合（相互独立，可并行）
}
```

### 8.2 内存管理

**智能指针：**
```cpp
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());  // 自动释放
```

**预分配：**
```cpp
normvec->resize(feats_down_size);  // 避免多次重新分配
```

### 8.3 数值稳定性

**Joseph形式协方差更新：**
```cpp
P = (I - K*H) * P * (I - K*H)^T + K*R*K^T;  // 确保P对称正定
```

**QR分解求解：**
```cpp
normvec = A.colPivHouseholderQr().solve(b);  // 比直接求逆更稳定
```

---

## 9. 常见问题与解决

### 9.1 系统发散

**症状：** 位置或姿态估计出现跳变

**可能原因：**
1. IMU未正确初始化
2. 外参标定不准确
3. 特征点太少

**解决方法：**
```cpp
// 检查有效特征点数量
if (effct_feat_num < 50) {
    ROS_WARN("Too few effective points!");
}

// 增加迭代次数
NUM_MAX_ITERATIONS = 8;  // 默认4
```

### 9.2 地图质量差

**症状：** 点云重影、模糊

**原因：** 去畸变不准确

**解决：**
- 检查IMU频率是否足够（推荐>200Hz）
- 验证时间同步是否正确
- 调整外参

### 9.3 实时性不足

**症状：** 帧率低、延迟大

**优化：**
```cpp
// 减小地图范围
cube_len = 100;  // 默认200

// 增大降采样体素
filter_size_surf_min = 0.8;  // 默认0.5

// 减少最大迭代
NUM_MAX_ITERATIONS = 2;
```

---

## 10. 总结

### 10.1 FAST-LIO2核心创新

1. **紧耦合IEKF：** 直接在原始点上优化，避免特征提取误差
2. **ikd-Tree：** 动态地图管理，支持高效的增删改查
3. **并行加速：** 最近邻搜索和平面拟合并行化

### 10.2 适用场景

✅ **适合：**
- 快速运动场景（无人机、车辆）
- 纹理缺失环境（走廊、隧道）
- 实时性要求高的应用

❌ **不适合：**
- 完全退化环境（长直走廊）
- IMU质量很差
- 需要全局一致性的建图

### 10.3 参数调优指南

| 参数 | 默认值 | 调优建议 |
|------|--------|----------|
| `gyr_cov` | 0.1 | IMU噪声大时增大 |
| `acc_cov` | 0.1 | 同上 |
| `NUM_MAX_ITERATIONS` | 4 | 收敛慢时增大 |
| `filter_size_surf_min` | 0.5 | 速度慢时增大 |
| `cube_len` | 200 | 内存不足时减小 |

---

**文档生成时间：** 2025-10-05
**FAST-LIO版本：** 2.0
**作者：** Claude Code Analysis
