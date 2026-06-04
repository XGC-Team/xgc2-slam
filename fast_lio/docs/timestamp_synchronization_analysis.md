# FAST-LIO 时间戳同步机制分析

> 创建日期: 2025-01-03
> 文档说明: 深入分析FAST-LIO中LiDAR与IMU时间戳同步机制

---

## 1. 问题背景

### 1.1 为什么需要时间同步？

在多传感器融合系统中，**时间戳同步**是确保数据正确关联的关键：

- **LiDAR** 和 **IMU** 是两个独立的传感器
- 每个传感器都有自己的时钟源
- 即使物理上同时采集数据，**时间戳可能完全不同**

### 1.2 时间戳不一致的典型情况

**情况1: 使用同一时间基准（已同步）**
```
LiDAR时间戳: 1000.500秒  ← ROS系统时间
IMU时间戳:   1000.500秒  ← ROS系统时间
差值: 0秒 ✅ 无需处理
```

**情况2: 使用不同时间基准（需同步）**
```
LiDAR时间戳: 1000.500秒  ← 传感器内部时钟A
IMU时间戳:    500.200秒  ← 传感器内部时钟B
差值: 500.3秒 ❌ 需要校正！
```

### 1.3 时间不同步的后果

如果不处理时间戳偏移：
- ❌ IMU预积分时间段错误
- ❌ 点云去畸变失败
- ❌ 状态估计发散
- ❌ 定位精度严重下降

---

## 2. FAST-LIO的两种时间同步方案

FAST-LIO提供了两种处理时间戳不一致的方案：

### 2.1 方案对比表

| 方案 | 配置 | 适用场景 | 偏移量来源 |
|------|------|---------|-----------|
| **方案1** | `time_sync_en: false` | 同一时钟源或已标定 | 手动配置 |
| **方案2** | `time_sync_en: true` | 不同时钟源 | 自动计算 |

### 2.2 方案1: 手动配置偏移（推荐用于MID360）

**配置文件** (`mid360.yaml`):
```yaml
common:
    time_sync_en: false                      # 关闭自动同步
    time_offset_lidar_to_imu: 0.0            # 手动设置偏移为0
```

**适用场景**:
- ✅ LiDAR和IMU使用相同的时间源（如ROS系统时间）
- ✅ 已通过外部工具（如LI-Init）标定好固定偏移量
- ✅ 时间戳天然对齐，无需额外处理

**工作原理**:
```cpp
// imu_cbk中直接应用手动配置的偏移
msg->header.stamp = ros::Time().fromSec(
    msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
// time_diff_lidar_to_imu = 0.0 (从配置读取)
```

**MID360为什么用这个方案？**
- Livox驱动将LiDAR和IMU数据都打上ROS系统时间戳
- 两者时间戳天然对齐
- 设置 `offset=0.0` 即可

### 2.3 方案2: 自动计算偏移

**配置**:
```yaml
common:
    time_sync_en: true                       # 启用自动同步
```

**适用场景**:
- ✅ LiDAR和IMU使用不同的硬件时钟
- ✅ 无法预先知道时间偏移量
- ✅ 需要程序自动检测和校正

**工作原理**: 见下节详细代码分析

---

## 3. 代码实现详解

### 3.1 全局变量定义

```cpp
// 文件: laserMapping.cpp

// 手动配置的时间偏移（从ROS参数服务器读取）
double time_diff_lidar_to_imu = 0.0;

// 自动计算的时间偏移
double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;         // 标记：是否已计算过偏移

// 时间同步开关（从配置文件读取）
bool time_sync_en = false;
```

### 3.2 参数读取（main函数）

```cpp
// 读取配置参数
nh.param<bool>("common/time_sync_en", time_sync_en, false);
nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
```

### 3.3 LiDAR回调中的自动同步（首次触发）

```cpp
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    // ... 前置代码 ...

    // ===== 诊断：检测未同步情况 =====
    if (!time_sync_en
        && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0
        && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        // 警告：关闭了自动同步，但时间差超过10秒
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
               last_timestamp_imu, last_timestamp_lidar);
    }

    // ===== 自动计算时间偏移（仅执行一次） =====
    if (time_sync_en                                      // 条件1: 启用自动同步
        && !timediff_set_flg                              // 条件2: 尚未计算过
        && abs(last_timestamp_lidar - last_timestamp_imu) > 1  // 条件3: 时间差>1秒
        && !imu_buffer.empty())                           // 条件4: 已有IMU数据
    {
        timediff_set_flg = true;                          // 标记已计算

        // 计算偏移量
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;

        printf("Self sync IMU and LiDAR, time diff is %.10lf \n",
               timediff_lidar_wrt_imu);
    }

    // ... 后续代码 ...
}
```

**计算公式解析**:
```
timediff_lidar_wrt_imu = t_lidar + 0.1 - t_imu

假设:
- t_lidar = 1000.5秒 (LiDAR当前时间戳)
- t_imu   = 500.2秒  (IMU最新时间戳)

计算:
timediff = 1000.5 + 0.1 - 500.2 = 500.4秒

物理含义:
将IMU时间基准向前偏移500.4秒，使其与LiDAR对齐
+0.1是一个小补偿值（可能考虑传输延迟）
```

### 3.4 IMU回调中的时间戳校正

```cpp
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // ===== 方案1: 使用手动配置的偏移 =====
    msg->header.stamp = ros::Time().fromSec(
        msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

    // ===== 方案2: 如果启用自动同步，覆盖上面的结果 =====
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = ros::Time().fromSec(
            timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    // ... 将校正后的IMU数据存入缓冲区 ...
}
```

**执行逻辑**:
```
如果 time_sync_en == false:
    → 使用 time_diff_lidar_to_imu (手动配置，如0.0)
    → 校正后时间戳 = 原始时间戳 - 0.0

如果 time_sync_en == true 且已计算偏移:
    → 使用 timediff_lidar_wrt_imu (自动计算，如500.4)
    → 校正后时间戳 = 原始时间戳 + 500.4
```

---

## 4. 完整执行流程示例

### 4.1 场景A: MID360配置（time_sync_en=false）

**配置**:
```yaml
time_sync_en: false
time_offset_lidar_to_imu: 0.0
```

**数据流**:
```
t=0时刻:
├─ LiDAR发布: timestamp = 1000.5秒 (ROS time)
├─ IMU发布:   timestamp = 1000.5秒 (ROS time)
│
├─ livox_pcl_cbk执行:
│  ├─ time_sync_en=false → 跳过自动同步代码
│  ├─ 时间差 = |1000.5-1000.5| = 0 < 10秒 → 无警告
│  └─ 存入lidar_buffer: 1000.5秒
│
└─ imu_cbk执行:
   ├─ 校正: 1000.5 - 0.0 = 1000.5秒
   ├─ time_sync_en=false → 不覆盖
   └─ 存入imu_buffer: 1000.5秒

结果: ✅ 时间戳完美对齐
```

### 4.2 场景B: 不同时钟源（time_sync_en=true）

**配置**:
```yaml
time_sync_en: true
```

**首次执行（自动计算偏移）**:
```
t=0时刻:
├─ LiDAR发布: timestamp = 1000.5秒 (传感器时钟A)
├─ IMU发布:   timestamp = 500.2秒  (传感器时钟B)
│
├─ livox_pcl_cbk执行:
│  ├─ time_sync_en=true 且 !timediff_set_flg
│  ├─ 时间差 = |1000.5-500.2| = 500.3 > 1秒 → 触发
│  ├─ 计算: timediff_lidar_wrt_imu = 1000.5 + 0.1 - 500.2 = 500.4秒
│  ├─ timediff_set_flg = true (标记已计算)
│  └─ 打印: "Self sync IMU and LiDAR, time diff is 500.4000000000"
│
└─ 后续所有imu_cbk执行:
   ├─ 原始IMU时间戳500.2秒 → 校正后 500.4 + 500.2 = 1000.6秒
   ├─ 原始IMU时间戳500.3秒 → 校正后 500.4 + 500.3 = 1000.7秒
   └─ 所有IMU数据被调整到LiDAR时间基准

结果: ✅ 时间戳成功同步
```

**后续执行（使用已计算的偏移）**:
```
t=1时刻:
├─ LiDAR发布: 1001.5秒
├─ IMU发布:   501.2秒
│
├─ livox_pcl_cbk:
│  └─ timediff_set_flg=true → 跳过计算（不再重复）
│
└─ imu_cbk:
   └─ 校正: 500.4 + 501.2 = 1001.6秒

结果: ✅ 持续使用首次计算的偏移量
```

---

## 5. 配置参数详解

### 5.1 time_sync_en（时间同步开关）

| 值 | 含义 | 使用场景 |
|----|------|---------|
| `false` | 关闭自动同步 | LiDAR和IMU时间戳已对齐，或使用外部标定值 |
| `true` | 启用自动同步 | 传感器使用不同时钟，需自动检测偏移 |

### 5.2 time_offset_lidar_to_imu（手动偏移量）

**单位**: 秒
**默认值**: 0.0
**作用**:
```cpp
校正后的IMU时间戳 = 原始时间戳 - time_offset_lidar_to_imu
```

**使用建议**:
- 如果LiDAR和IMU已同步 → 设为 `0.0`
- 如果通过LI-Init等工具标定 → 设为标定值（如 `0.005`）

### 5.3 参数组合效果

| time_sync_en | time_offset | 实际使用的偏移量 | 备注 |
|--------------|-------------|------------------|------|
| `false` | `0.0` | `0.0` | MID360默认配置 |
| `false` | `0.005` | `0.005` | 使用外部标定值 |
| `true` | 任意值 | 自动计算值 | 忽略手动配置 |

---

## 6. 诊断与调试

### 6.1 如何判断是否需要时间同步？

**步骤1**: 运行FAST-LIO，观察终端输出

**情况A**: 看到此警告
```
IMU and LiDAR not Synced, IMU time: 500.200000, lidar header time: 1000.500000
```
**诊断**:
- 时间差超过10秒
- 当前配置: `time_sync_en=false`
- **建议**: 修改为 `time_sync_en=true`

**情况B**: 看到此消息
```
Self sync IMU and LiDAR, time diff is 500.4000000000
```
**诊断**:
- 自动同步已触发
- 计算出偏移量500.4秒
- **结果**: 正常工作

**情况C**: 无任何同步相关消息
**诊断**:
- 时间戳已对齐
- **结果**: 正常工作

### 6.2 验证同步效果

**方法1: 检查日志**
- 查看 `mat_out.txt` 中的状态估计
- 如果位姿连续变化 → 同步正常
- 如果数值异常或发散 → 检查时间同步

**方法2: 可视化**
```bash
rostopic echo /Odometry | grep stamp
```
检查发布的里程计时间戳是否连续

### 6.3 常见问题

**问题1**: 设置了`time_sync_en=true`但没有看到"Self sync"消息

**原因**:
- 时间差 < 1秒，不满足触发条件
- IMU缓冲区为空

**解决**: 可能本来就不需要同步，改为`time_sync_en=false`

**问题2**: 偏移量计算不准确

**原因**:
- 首次计算时LiDAR和IMU数据并非严格同时采集
- `+0.1`补偿值可能不适合所有场景

**解决**:
- 使用LI-Init等专业工具精确标定
- 使用`time_offset_lidar_to_imu`手动设置

---

## 7. 总结

### 7.1 核心要点

1. **两种方案，各有适用场景**:
   - 已同步 → `time_sync_en=false`
   - 需自动检测 → `time_sync_en=true`

2. **MID360推荐配置**: `time_sync_en=false, offset=0.0`
   - Livox驱动保证时间戳一致性

3. **自动同步仅执行一次**:
   - 首次检测到大偏移时计算
   - 后续持续使用该值

4. **时间同步是多传感器融合的基础**:
   - 错误的时间戳会导致算法完全失败

### 7.2 决策流程图

```
开始
  ↓
LiDAR和IMU使用同一时间源?
  ├─ 是 → time_sync_en=false, offset=0.0 ✅
  └─ 否 → 已知固定偏移量?
           ├─ 是 → time_sync_en=false, offset=已知值 ✅
           └─ 否 → time_sync_en=true ✅
```

---

> 文档结束
> 如有疑问，可对照源代码 `laserMapping.cpp` 第300-364行