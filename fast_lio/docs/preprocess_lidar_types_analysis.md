# Preprocess类LiDAR类型支持与特征提取全面分析

> 基于 `/home/lxk/Paper/motion-planning/src/FAST_LIO/src/preprocess.h` 和 `preprocess.cpp` 的深度解析

---

## 目录

1. [支持的LiDAR类型概览](#一支持的lidar类型概览)
2. [数据结构与枚举定义](#二数据结构与枚举定义)
3. [各类LiDAR处理流程详解](#三各类lidar处理流程详解)
4. [特征提取算法深入剖析](#四特征提取算法深入剖析)
5. [时间戳处理机制](#五时间戳处理机制)
6. [使用建议与参数调优](#六使用建议与参数调优)

---

## 一、支持的LiDAR类型概览

### 1.1 LiDAR类型枚举定义

**位置**: `preprocess.h:17`

```cpp
enum LID_TYPE {
    AVIA = 1,    // Livox Avia / Mid-360 / Horizon (非重复扫描)
    VELO16,      // Velodyne VLP-16 (机械旋转式)
    OUST64,      // Ouster OS1-64/128 (机械旋转式)
    MARSIM       // 仿真雷达 (MARS仿真器)
};
```

### 1.2 各类型特点对比

| LiDAR类型 | 扫描模式 | 线束数 | 帧率 | 消息格式 | 时间戳来源 |
|-----------|---------|--------|------|---------|-----------|
| **Livox Avia/Mid-360** | 非重复扫描 | 6/4 | 10-100Hz | `CustomMsg` | 点级offset_time |
| **Velodyne VLP-16** | 360°旋转 | 16 | 10-20Hz | `PointCloud2` | 自带/计算 |
| **Ouster OS1-64** | 360°旋转 | 64-128 | 10-20Hz | `PointCloud2` | 点级时间戳 |
| **仿真雷达** | 理想模型 | 可变 | 可变 | `PointCloud2` | 固定时刻 |

### 1.3 处理函数映射

```cpp
// preprocess.cpp:44-89
void process(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    → avia_handler(msg);  // Livox系列

void process(const sensor_msgs::PointCloud2::ConstPtr &msg)
    → switch (lidar_type):
        case OUST64:  → oust64_handler(msg);
        case VELO16:  → velodyne_handler(msg);
        case MARSIM:  → sim_handler(msg);
```

---

## 二、数据结构与枚举定义

### 2.1 特征类型 (Feature)

**位置**: `preprocess.h:19`

```cpp
enum Feature {
    Nor,         // 普通点（无显著几何特征）
    Poss_Plane,  // 可能的平面点（平面边界）
    Real_Plane,  // 确定的平面点（平面内部）
    Edge_Jump,   // 深度跳变边缘（物体边缘）
    Edge_Plane,  // 平面交线边缘（两平面交界）
    Wire,        // 线状结构（孤立边缘）
    ZeroPoint    // 零点/无效点
};
```

**几何意义**：

```
Real_Plane:   ●―――●―――●―――●―――●    (平面内部，点分布均匀)
Poss_Plane:   ●                  ●    (平面边界点)
Edge_Jump:         ●●
                     ＼
                      ●――――●        (深度突变)
Edge_Plane:   ●―――●
                    ╱
              ●―――●                (两平面交线)
Wire:         ●        ●        ●    (孤立点，无邻域)
```

### 2.2 跳变类型 (E_jump)

```cpp
enum E_jump {
    Nr_nor,    // 正常（无跳变）
    Nr_zero,   // 零度跳变（向前跳变，距离突然增大）
    Nr_180,    // 180度跳变（向后跳变，距离突然减小）
    Nr_inf,    // 无穷远跳变（邻点在盲区外）
    Nr_blind   // 盲区跳变（邻点在盲区内）
};
```

**角度判据**（`preprocess.cpp:649-657`）：

```cpp
// 计算当前点与邻点的夹角余弦值
angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();

if (angle[j] < jump_up_limit)    // < cos(170°) ≈ -0.985
    edj[j] = Nr_180;              // 向后跳变
else if (angle[j] > jump_down_limit)  // > cos(8°) ≈ 0.990
    edj[j] = Nr_zero;             // 向前跳变
```

### 2.3 点云几何属性结构 (orgtype)

**位置**: `preprocess.h:23-39`

```cpp
struct orgtype {
    double range;        // 点到LiDAR的距离
    double dista;        // 点到下一点的距离
    double angle[2];     // 与前/后点的夹角余弦值
    double intersect;    // 前后方向向量的夹角余弦值
    E_jump edj[2];       // 前/后跳变类型
    Feature ftype;       // 特征类型
};
```

**用途**：
- `range`: 盲区过滤、距离加权
- `dista`: 平面连续性判断
- `angle`: 边缘检测
- `intersect`: 线状结构检测

### 2.4 自定义点云类型

#### Velodyne点云格式

```cpp
namespace velodyne_ros {
    struct Point {
        float x, y, z;
        float intensity;
        float time;          // 点级时间戳（相对帧起始时间）
        uint16_t ring;       // 线束编号 (0-15)
    };
}
```

#### Ouster点云格式

```cpp
namespace ouster_ros {
    struct Point {
        float x, y, z;
        float intensity;
        uint32_t t;          // 时间戳（纳秒级）
        uint16_t reflectivity; // 反射率
        uint8_t ring;        // 线束编号
        uint16_t ambient;    // 环境光
        uint32_t range;      // 原始距离测量
    };
}
```

---

## 三、各类LiDAR处理流程详解

### 3.1 Livox系列 (AVIA Handler)

#### 3.1.1 扫描模式特点

**非重复扫描**（Non-Repetitive Scanning）：
- 每帧点云覆盖不同区域
- FOV内采样密度均匀
- 不存在固定扫描线

**Livox CustomMsg格式**：
```cpp
struct CustomPoint {
    float x, y, z;
    uint8_t reflectivity;      // 反射率 (0-255)
    uint8_t tag;               // 点状态标签
    uint8_t line;              // 伪线束编号
    uint32_t offset_time;      // 微秒级时间偏移
};
```

#### 3.1.2 处理流程（无特征提取模式）

**位置**: `preprocess.cpp:162-186`

```cpp
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    pl_surf.clear();
    int plsize = msg->point_num;
    pl_full.resize(plsize);
    uint valid_num = 0;

    for (uint i=1; i<plsize; i++)
    {
        // ===== 步骤1: 质量检查 =====
        // tag位检查：
        // 0x00: 正常点
        // 0x10: 可能有遮挡但可用
        // 0x20: 低置信度（丢弃）
        // 0x30: 噪声点（丢弃）
        if ((msg->points[i].line < N_SCANS) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num++;

            // ===== 步骤2: 降采样 =====
            if (valid_num % point_filter_num == 0)  // MID360默认: 3
            {
                // ===== 步骤3: 数据转换 =====
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature = msg->points[i].offset_time / 1000000.0; // 微秒→秒

                // ===== 步骤4: 去重 + 盲区过滤 =====
                if (((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) ||
                     (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7) ||
                     (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)) &&
                    (pl_full[i].x*pl_full[i].x + pl_full[i].y*pl_full[i].y +
                     pl_full[i].z*pl_full[i].z > (blind * blind)))
                {
                    pl_surf.push_back(pl_full[i]);
                }
            }
        }
    }
}
```

**关键设计**：

1. **质量标签解析**：
   ```
   tag字段（8位）：
   |  高4位  |  低4位  |
   | 置信度  | 回波序号 |

   质量等级：
   0x00: 单回波正常点
   0x10: 多回波第一个（可能有遮挡）
   0x20: 低置信度
   0x30: 噪声点
   ```

2. **去重逻辑**：
   - 静止时Livox可能输出重复点
   - 阈值 `1e-7` 米 ≈ 0.1微米（数值精度级别）

3. **盲区处理**：
   - 球形盲区：`range² > blind²`
   - MID360默认 `blind=0.5m`

#### 3.1.3 处理流程（特征提取模式）

**位置**: `preprocess.cpp:112-159`

```cpp
if (feature_enabled)
{
    // ===== 步骤1: 按伪线束分组 =====
    for (int i=0; i<N_SCANS; i++)
        pl_buff[i].clear();

    for (uint i=1; i<plsize; i++)
    {
        if (valid_quality)
        {
            // 转换格式并存入对应线束缓冲区
            pl_buff[msg->points[i].line].push_back(point);
        }
    }

    // ===== 步骤2: 逐线束提取特征 =====
    for (int j=0; j<N_SCANS; j++)
    {
        if (pl_buff[j].size() <= 5) continue;

        // 计算几何属性（距离、相邻点间距）
        for (uint i=0; i<plsize-1; i++)
        {
            types[i].range = sqrt(pl[i].x² + pl[i].y²);
            vx = pl[i].x - pl[i+1].x;
            vy = pl[i].y - pl[i+1].y;
            vz = pl[i].z - pl[i+1].z;
            types[i].dista = sqrt(vx² + vy² + vz²);
        }

        // ===== 步骤3: 特征分类 =====
        give_feature(pl_buff[j], typess[j]);
    }
}
```

**Livox特殊处理**：
- 虽然无真实扫描线，但用 `line` 字段模拟
- 特征提取按伪线束进行，效果次于机械式雷达
- 建议关闭特征提取，使用全点云

---

### 3.2 Velodyne系列 (Velodyne Handler)

#### 3.2.1 扫描模式特点

**机械旋转扫描**：
- 垂直16线束固定
- 水平360°连续旋转
- 扫描线呈螺旋分布

**扫描参数**：
```cpp
N_SCANS = 16;           // VLP-16
SCAN_RATE = 10;         // 10Hz (可调: 5/10/20Hz)
omega_l = 0.361 * SCAN_RATE;  // 角速度 (rad/s)
```

#### 3.2.2 时间戳处理策略

**问题**：部分Velodyne驱动不提供点级时间戳

**解决方案**（`preprocess.cpp:296-322`）：

```cpp
// 检测是否有时间戳
if (pl_orig.points[plsize-1].time > 0)
    given_offset_time = true;  // 使用驱动提供的时间
else
    given_offset_time = false; // 需要自行计算
```

**计算时间戳**（基于角度）：

```cpp
// 记录每条扫描线的首个点角度
if (is_first[layer])
{
    yaw_fp[layer] = atan2(y, x) * 57.2957;  // 弧度→度
    is_first[layer] = false;
}

// 根据角度差计算时间偏移
if (yaw_angle <= yaw_fp[layer])
    curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
else
    curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;

// 防止时间倒退
if (curvature < time_last[layer])
    curvature += 360.0 / omega_l;
```

**原理图解**：

```
                    yaw_fp[layer] (首点角度)
                          ↓
      90°             0° ●―→ 旋转方向
       |               /
       |              /
180° ――+―― 0°    当前点●
       |          yaw_angle
       |
     270°

时间偏移 = (角度差) / (角速度)
```

#### 3.2.3 处理流程（无特征提取）

**位置**: `preprocess.cpp:399-455`

```cpp
void Preprocess::velodyne_handler(...)
{
    for (int i=0; i<plsize; i++)
    {
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        // ===== 时间戳处理 =====
        if (given_offset_time)
            added_pt.curvature = pl_orig.points[i].time * time_unit_scale;
        else
            added_pt.curvature = 计算值;  // 基于角度

        // ===== 降采样 + 盲区过滤 =====
        if (i % point_filter_num == 0)
        {
            if (range² > blind²)
                pl_surf.push_back(added_pt);
        }
    }
}
```

**优势**：
- 扫描线固定，便于特征提取
- 点云分布规律，适合线性插值
- 广泛使用，驱动成熟

**劣势**：
- 垂直分辨率低（16线 → 2.0°间隔）
- 远距离稀疏
- 帧率受限（最高20Hz）

---

### 3.3 Ouster系列 (Ouster Handler)

#### 3.3.1 扫描模式特点

**高线束机械旋转**：
- OS1-64: 64线
- OS1-128: 128线
- 高垂直分辨率（0.35° - 0.7°）

**数据特点**：
- 原生支持点级时间戳（纳秒精度）
- 提供反射率和环境光信息
- 高帧率支持（10-20Hz）

#### 3.3.2 处理流程（无特征提取）

**位置**: `preprocess.cpp:254-279`

```cpp
void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    for (int i=0; i<pl_orig.points.size(); i++)
    {
        // ===== 降采样 =====
        if (i % point_filter_num != 0) continue;

        // ===== 盲区过滤 =====
        double range = x² + y² + z²;
        if (range < blind²) continue;

        // ===== 数据转换 =====
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        // ===== 时间戳转换 =====
        // time_unit: NS (纳秒)
        // time_unit_scale: 1e-6 (纳秒→毫秒)
        added_pt.curvature = pl_orig.points[i].t * time_unit_scale;

        pl_surf.push_back(added_pt);
    }
}
```

**时间戳转换表**（`preprocess.cpp:52-69`）：

```cpp
switch (time_unit) {
    case SEC:  time_unit_scale = 1.e3f;   break;  // 秒 → 毫秒
    case MS:   time_unit_scale = 1.f;     break;  // 毫秒 → 毫秒
    case US:   time_unit_scale = 1.e-3f;  break;  // 微秒 → 毫秒
    case NS:   time_unit_scale = 1.e-6f;  break;  // 纳秒 → 毫秒
}
```

**优势**：
- 高垂直分辨率，细节丰富
- 精确时间戳，去畸变准确
- 多信息通道（反射率、环境光）

**劣势**：
- 数据量大（128线@20Hz → 250万点/秒）
- 计算资源需求高
- 价格昂贵

---

### 3.4 仿真雷达 (Simulation Handler)

#### 3.4.1 应用场景

- **MARS仿真器**：多智能体仿真环境
- **Gazebo仿真**：机器人操作系统标准仿真
- **算法验证**：无需实际硬件的测试

#### 3.4.2 处理流程

**位置**: `preprocess.cpp:458-481`

```cpp
void Preprocess::sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    for (int i=0; i<pl_orig.points.size(); i++)
    {
        // ===== 盲区过滤 =====
        double range = x² + y² + z²;
        if (range < blind²) continue;

        // ===== 简单转换 =====
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.curvature = 0.0;  // 仿真中无运动畸变

        pl_surf.push_back(added_pt);
    }
}
```

**特点**：
- **无时间戳**：curvature固定为0（理想瞬时采样）
- **无降采样**：仿真点云已经是理想密度
- **无质量检查**：仿真数据无噪声标签
- **简化处理**：专注算法验证而非数据预处理

---

## 四、特征提取算法深入剖析

### 4.1 特征提取总流程

**入口函数**: `give_feature()` (`preprocess.cpp:483-795`)

```
输入: 单条扫描线点云 + 几何属性数组
      ↓
[1] 平面区域识别
      ↓
[2] 平面交线检测
      ↓
[3] 深度跳变边缘检测
      ↓
[4] 线状结构检测
      ↓
[5] 小平面补充识别
      ↓
[6] 特征点降采样输出
      ↓
输出: pl_surf (平面点) + pl_corn (边缘点)
```

---

### 4.2 平面区域识别

#### 4.2.1 算法原理

**函数**: `plane_judge()` (`preprocess.cpp:806-918`)

**核心思想**: 判断一组连续点是否共面，通过**长宽比**判据

**步骤1: 选取点群**

```cpp
// 从当前点i_cur开始，选择group_size=8个连续点作为候选
for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
    disarr.push_back(types[i_nex].dista);  // 记录相邻点间距

// 继续扩展，直到首尾距离达到阈值
double group_dis = disA * types[i_cur].range + disB;
while (两点距离 < group_dis)
{
    i_nex++;
    disarr.push_back(types[i_nex].dista);
}
```

**自适应窗口大小**：
```
group_dis = 0.1 * range + 0.1
→ 近距离: 小窗口 (精细检测)
→ 远距离: 大窗口 (容忍稀疏)
```

**步骤2: 计算长宽比**

```cpp
// 长度: 首尾点欧氏距离
vx = pl[i_nex].x - pl[i_cur].x;
vy = pl[i_nex].y - pl[i_cur].y;
vz = pl[i_nex].z - pl[i_cur].z;
double two_dis = vx² + vy² + vz²;

// 宽度: 所有中间点到首尾连线的最大距离
double leng_wid = 0;
for (uint j = i_cur+1; j < i_nex; j++)
{
    v1 = pl[j] - pl[i_cur];        // 中间点向量
    v2 = v1 × (vx, vy, vz);        // 叉乘 = 垂直距离向量
    double lw = ||v2||²;
    leng_wid = max(leng_wid, lw);
}
```

**几何意义**：

```
首点●――――――――――――――――●尾点  (长度 = two_dis)
      ＼        ↑       ／
        ＼      宽度    ／
          ●●●●●●●       (中间点偏离连线的最大距离)
```

**步骤3: 平面判据**

```cpp
// 判据1: 长宽比检查
if ((two_dis² / leng_wid) < p2l_ratio)  // p2l_ratio = 225
    return 0;  // 非平面（宽度太大）

// 判据2: 点分布均匀性检查（Livox特有）
if (lidar_type == AVIA)
{
    // 对相邻点距离数组排序（降序）
    sort(disarr, descending);

    double dismax_mid = disarr[0] / disarr[size/2];
    double dismid_min = disarr[size/2] / disarr[size-2];

    // 最大间距不能过度大于中位数
    if (dismax_mid >= 6.25 || dismid_min >= 6.25)
        return 0;  // 点分布不均，非平面
}
else  // 机械式雷达
{
    double dismax_min = disarr[0] / disarr[size-2];
    if (dismax_min >= 3.24)
        return 0;
}

// 通过检查，返回平面法向量
curr_direct = (vx, vy, vz).normalized();
return 1;  // 确定为平面
```

**参数含义**：

```
p2l_ratio = 225:
    two_dis² / leng_wid ≥ 225
    → two_dis / √leng_wid ≥ 15
    → 长度至少是宽度的15倍

limit_maxmid = 6.25 (Livox):
    最大间距 / 中位间距 < 6.25
    → 防止存在孤立点或断裂

limit_maxmin = 3.24 (机械式):
    最大间距 / 最小间距 < 3.24
    → 点分布相对均匀
```

#### 4.2.2 平面点分类

```cpp
// preprocess.cpp:521-547
if (plane_type == 1)  // 检测到平面
{
    // 内部点标记为 Real_Plane
    for (uint j = i; j <= i_nex; j++)
    {
        if (j != i && j != i_nex)
            types[j].ftype = Real_Plane;
        else
            types[j].ftype = Poss_Plane;  // 边界点
    }

    // 检测平面交线
    if (last_state == 1 && last_direct.norm() > 0.1)
    {
        // 当前平面方向与上一平面方向的夹角
        double mod = last_direct.transpose() * curr_direct;

        if (mod > -0.707 && mod < 0.707)  // 45° < θ < 135°
            types[i].ftype = Edge_Plane;  // 两平面交线
        else
            types[i].ftype = Real_Plane;
    }
}
```

**平面交线检测原理**：

```
上一平面法向量: n1
当前平面法向量: n2

cos(θ) = n1·n2 / (||n1|| ||n2||)

if -0.707 < cos(θ) < 0.707:
    45° < θ < 135°  → 两平面近似垂直 → 交线
```

---

### 4.3 深度跳变边缘检测

#### 4.3.1 跳变类型判定

**函数**: `give_feature()` 第二阶段 (`preprocess.cpp:609-703`)

```cpp
for (uint i = head+3; i < plsize-3; i++)
{
    if (已是平面点 || 距离<盲区) continue;

    // ===== 计算前后点的夹角 =====
    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);  // 当前点
    Eigen::Vector3d vecs[2];  // 前后邻点

    for (int j=0; j<2; j++)  // j=0: 前点, j=1: 后点
    {
        int m = (j == 1) ? 1 : -1;
        vecs[j] = Eigen::Vector3d(pl[i+m]) - vec_a;

        // 计算夹角余弦值
        types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();

        if (types[i].angle[j] < jump_up_limit)      // < cos(170°)
            types[i].edj[j] = Nr_180;               // 向后跳变
        else if (types[i].angle[j] > jump_down_limit)  // > cos(8°)
            types[i].edj[j] = Nr_zero;              // 向前跳变
    }

    // 计算前后向量夹角
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / ...;
}
```

**几何解释**：

```
向前跳变 (Nr_zero):
    LiDAR ●――――――●i
                  ＼
                   ●i+1 (突然远离)
    angle[Next] > cos(8°) ≈ 0.99

向后跳变 (Nr_180):
    LiDAR ●―――――――――●i+1
             ●i (突然靠近)
            ／
    angle[Next] < cos(170°) ≈ -0.985
```

#### 4.3.2 边缘跳变判定

```cpp
// preprocess.cpp:661-695
// 模式1: 前正常 + 后跳变
if (types[i].edj[Prev] == Nr_nor &&
    types[i].edj[Next] == Nr_zero &&
    types[i].dista > 0.0225 &&           // 间距 > 15cm
    types[i].dista > 4*types[i-1].dista) // 后点远4倍
{
    if (types[i].intersect > cos160)     // 前后向量夹角 < 160°
    {
        if (edge_jump_judge(pl, types, i, Prev))
            types[i].ftype = Edge_Jump;
    }
}

// 模式2: 前跳变 + 后正常
else if (types[i].edj[Prev] == Nr_zero &&
         types[i].edj[Next] == Nr_nor &&
         types[i-1].dista > 0.0225 &&
         types[i-1].dista > 4*types[i].dista)
{
    if (types[i].intersect > cos160)
    {
        if (edge_jump_judge(pl, types, i, Next))
            types[i].ftype = Edge_Jump;
    }
}

// 模式3: 边界到无穷远
else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
{
    if (edge_jump_judge(pl, types, i, Prev))
        types[i].ftype = Edge_Jump;
}
```

**边缘精细化判定** (`edge_jump_judge`):

```cpp
bool Preprocess::edge_jump_judge(...)
{
    // 检查更远邻域的稳定性
    if (nor_dir == Prev)
    {
        if (types[i-1].range < blind || types[i-2].range < blind)
            return false;  // 邻域在盲区，不可靠
    }

    double d1 = types[i + nor_dir - 1].dista;
    double d2 = types[i + 3*nor_dir - 2].dista;

    // 确保跳变方向一致
    if (d1 > edgea*d2 || (d1-d2) > edgeb)  // edgea=2, edgeb=0.1
        return false;

    return true;  // 确认为边缘
}
```

---

### 4.4 线状结构检测

**位置**: `preprocess.cpp:696-702`

```cpp
// 如果前后都是异常跳变
else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
{
    if (types[i].ftype == Nor)
        types[i].ftype = Wire;  // 孤立点，标记为线状
}
```

**应用场景**：
- 电线、栏杆等细长结构
- 远距离单点反射
- 孤立噪声点（需进一步过滤）

---

### 4.5 小平面补充识别

**位置**: `preprocess.cpp:705-743`

```cpp
for (uint i = head+1; i < plsize-2; i++)
{
    if (已有特征) continue;

    // 计算前后点距离比
    double ratio;
    if (types[i-1].dista > types[i].dista)
        ratio = types[i-1].dista / types[i].dista;
    else
        ratio = types[i].dista / types[i-1].dista;

    // 小平面判据:
    // 1. 前后向量夹角小 (< 172.5°)
    // 2. 距离比接近 (< 1.2)
    if (types[i].intersect < smallp_intersect &&  // cos(172.5°) ≈ -0.991
        ratio < smallp_ratio)                      // < 1.2
    {
        // 将邻域标记为平面
        if (types[i-1].ftype == Nor)
            types[i-1].ftype = Real_Plane;
        if (types[i+1].ftype == Nor)
            types[i+1].ftype = Real_Plane;
        types[i].ftype = Real_Plane;
    }
}
```

**作用**：
- 捕获平面检测遗漏的小区域
- 增强平面连续性
- 降低边缘误检率

---

### 4.6 特征点输出与降采样

**位置**: `preprocess.cpp:745-794`

```cpp
int last_surface = -1;
for (uint j = head; j < plsize; j++)
{
    // ===== 平面点处理 =====
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
        if (last_surface == -1)
            last_surface = j;  // 记录平面起点

        // 每 point_filter_num 个平面点输出一个
        if (j == uint(last_surface + point_filter_num - 1))
        {
            pl_surf.push_back(pl[j]);
            last_surface = -1;
        }
    }
    // ===== 边缘点处理 =====
    else
    {
        if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
        {
            pl_corn.push_back(pl[j]);  // 边缘点直接输出
        }

        // 平面中断，输出平均点
        if (last_surface != -1)
        {
            PointType ap;
            for (uint k = last_surface; k < j; k++)
            {
                ap.x += pl[k].x;
                ap.y += pl[k].y;
                ap.z += pl[k].z;
                ap.intensity += pl[k].intensity;
                ap.curvature += pl[k].curvature;
            }
            ap /= (j - last_surface);  // 平均值
            pl_surf.push_back(ap);
            last_surface = -1;
        }
    }
}
```

**降采样策略**：
1. **平面点**：每N个取1个（`point_filter_num`）
2. **边缘点**：全部保留（边缘稀疏且重要）
3. **平面中断**：输出区域平均点

---

## 五、时间戳处理机制

### 5.1 时间戳来源对比

| LiDAR类型 | 时间戳来源 | 精度 | 单位 | 存储字段 |
|-----------|-----------|------|------|---------|
| Livox | `offset_time` | 微秒 | 相对帧起始 | `curvature` |
| Velodyne | `time` 或计算 | 毫秒 | 相对帧起始 | `curvature` |
| Ouster | `t` | 纳秒 | 绝对时间 | `curvature` |
| 仿真 | 无 | - | - | 0.0 |

### 5.2 运动畸变去除原理

**问题**：LiDAR扫描期间，平台在运动

```
t=0ms     ●―――――――――――――→  平台运动方向
          扫描起始

t=50ms                  ●  扫描中途
                       点云畸变

t=100ms                              ●  扫描结束
```

**解决**：利用点级时间戳，将点变换到统一时刻

```cpp
// laserMapping.cpp中的应用
for (int i=0; i<points.size(); i++)
{
    double dt = points[i].curvature;  // 时间偏移

    // 插值获得该时刻的位姿
    Pose pose_t = interpolate(pose_start, pose_end, dt);

    // 去畸变变换
    points_undistorted[i] = pose_t.inverse() * points[i];
}
```

### 5.3 时间单位统一

**所有时间戳最终转换为毫秒** (`curvature`字段)：

```cpp
// Livox: 微秒 → 毫秒
curvature = offset_time / 1000000.0 * 1000.0 = offset_time / 1000.0;

// Velodyne: 已是毫秒或秒
curvature = time * time_unit_scale;

// Ouster: 纳秒 → 毫秒
curvature = t * 1e-6;

// 仿真: 固定0
curvature = 0.0;
```

---

## 六、使用建议与参数调优

### 6.1 LiDAR类型选择建议

#### Livox Avia/Mid-360

```yaml
# 推荐配置
lidar_type: 1               # AVIA
feature_extract_enable: false   # 关闭特征提取
point_filter_num: 3         # 1/3降采样
blind: 0.5                  # 盲区0.5m
N_SCANS: 4                  # Mid-360为4线
```

**适用场景**：
- 无人机、移动机器人
- 高帧率需求（100Hz）
- 预算有限

#### Velodyne VLP-16

```yaml
# 推荐配置
lidar_type: 2               # VELO16
feature_extract_enable: true    # 建议开启
point_filter_num: 1         # 特征点已稀疏
blind: 0.1
N_SCANS: 16
SCAN_RATE: 10
```

**适用场景**：
- 结构化环境（建筑、走廊）
- 中低速运动
- 需要与LOAM对比

#### Ouster OS1-64/128

```yaml
# 推荐配置
lidar_type: 3               # OUST64
feature_extract_enable: false   # 点云已足够密集
point_filter_num: 4         # 高线束需更强降采样
blind: 0.5
N_SCANS: 64                 # 或128
time_unit: 3                # NS (纳秒)
```

**适用场景**：
- 高精度建图
- 复杂环境感知
- 充足计算资源

### 6.2 参数调优指南

#### 6.2.1 盲区 (blind)

```
规则：
- 室内: 0.1 - 0.5m
- 室外: 0.5 - 1.0m  (避免地面点)
- 高速: 1.0 - 2.0m  (运动模糊)

调试方法：
1. 观察RViz中近距离点云
2. 如有密集地面点 → 增大blind
3. 如丢失近处特征 → 减小blind
```

#### 6.2.2 降采样率 (point_filter_num)

```
选择依据：
- 点云密度: 密集 → 大值, 稀疏 → 小值
- 计算能力: 弱 → 大值, 强 → 小值
- 精度需求: 高 → 小值, 低 → 大值

推荐值：
- Livox Mid-360: 2-4
- Velodyne 16:   1-2
- Ouster 64:     3-5
- Ouster 128:    4-8
```

#### 6.2.3 扫描线数 (N_SCANS)

```
必须与实际硬件匹配：
- Livox Avia:     6
- Livox Mid-360:  4
- Velodyne VLP-16: 16
- Velodyne VLP-32: 32
- Ouster OS1-64:  64
- Ouster OS1-128: 128
```

#### 6.2.4 特征提取参数

**平面检测灵敏度**：

```cpp
// preprocess.cpp:15-18
p2l_ratio = 225;        // 减小 → 更严格 (fewer planes)
                        // 增大 → 更宽松 (more planes)

limit_maxmid = 6.25;    // Livox: 减小 → 更均匀
limit_maxmin = 3.24;    // 机械式: 减小 → 更均匀
```

**边缘检测灵敏度**：

```cpp
jump_up_limit = 170.0;   // 增大 → 更少边缘 (更严格)
jump_down_limit = 8.0;   // 减小 → 更少边缘

edgea = 2;               // 边缘一致性阈值
edgeb = 0.1;             // 距离容忍度
```

### 6.3 故障排查

#### 问题1: 点云过于稀疏

**原因**：
- `point_filter_num` 过大
- `blind` 范围过大
- 特征提取过滤过多

**解决**：
```yaml
point_filter_num: 1-2    # 降低降采样率
blind: 0.1-0.3           # 缩小盲区
feature_extract_enable: false  # 关闭特征提取
```

#### 问题2: 计算负载过高

**原因**：
- 点云密度过大
- 特征提取开启且慢

**解决**：
```yaml
point_filter_num: 4-8    # 增大降采样
feature_extract_enable: false  # 关闭特征提取
```

#### 问题3: 时间戳异常

**现象**：点云严重畸变

**检查**：
```bash
# 查看curvature字段范围
rostopic echo /cloud_registered | grep curvature
```

**解决**：
- Velodyne: 检查驱动是否提供time字段
- Ouster: 确认 `time_unit: 3` (NS)
- Livox: 检查offset_time是否为0

---

## 七、总结

### 7.1 核心设计思想

1. **模块化设计**：
   - 不同LiDAR → 不同Handler
   - 统一输出格式 → `PointCloudXYZI`

2. **特征提取的权衡**：
   - 开启：降低计算量，增强几何约束
   - 关闭：保留完整信息，适配ikd-Tree

3. **时间戳统一**：
   - 所有雷达输出毫秒级时间偏移
   - 存储在 `curvature` 字段
   - 支持IMU预积分去畸变

### 7.2 LiDAR选型建议

| 应用场景 | 推荐LiDAR | 理由 |
|---------|----------|------|
| **无人机** | Livox Mid-360 | 轻量、高帧率、性价比 |
| **室内移动机器人** | Velodyne VLP-16 | 成熟、稳定、扫描线固定 |
| **自动驾驶** | Ouster OS1-128 | 高分辨率、远距离 |
| **算法研究** | 仿真雷达 | 无成本、可控参数 |

### 7.3 关键代码位置索引

| 功能 | 文件 | 行号 |
|------|------|------|
| LiDAR类型定义 | `preprocess.h` | 17 |
| 特征类型定义 | `preprocess.h` | 19 |
| Livox处理 | `preprocess.cpp` | 92-187 |
| Velodyne处理 | `preprocess.cpp` | 284-456 |
| Ouster处理 | `preprocess.cpp` | 189-282 |
| 仿真处理 | `preprocess.cpp` | 458-481 |
| 平面检测 | `preprocess.cpp` | 806-918 |
| 边缘检测 | `preprocess.cpp` | 920-957 |
| 特征分类 | `preprocess.cpp` | 483-795 |

---

*文档完成于 2025-01-03*
*基于 FAST-LIO v2.0 源码分析*
