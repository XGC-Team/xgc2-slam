/**
 * @file preprocess.cpp
 * @brief Point-LIO点云预处理模块实现
 *
 * 本文件实现了针对不同类型激光雷达（Livox AVIA、Velodyne、Oust64、HESAI等）的点云预处理功能。
 * 主要功能包括：
 * 1. 点云数据格式转换和时间戳处理
 * 2. 点云滤波（距离滤波、降采样等）
 * 3. 特征提取（平面点、边缘点识别）
 * 4. 点云帧切分（用于分段处理）
 * 5. 几何特征判断（平面判断、边缘跳变检测等）
 *
 * @author Point-LIO开发团队
 */

#include "preprocess.h"

// 特征提取返回值定义
#define RETURN0     0x00      // 不返回任何特征
#define RETURN0AND1 0x10      // 返回平面和边缘特征

/**
 * @brief 点云按时间戳排序的比较函数
 * @param x 第一个点
 * @param y 第二个点
 * @return 如果x的时间戳小于y返回true，用于升序排序
 * @note 这里使用curvature字段存储时间戳信息（单位：ms）
 */
const bool time_list_cut_frame(PointType &x, PointType &y) {
    return (x.curvature < y.curvature);
}

/**
 * @brief Preprocess类构造函数，初始化点云预处理参数
 *
 * 初始化列表：
 * @param lidar_type 雷达类型，默认为AVIA（Livox AVIA）
 * @param blind 最小检测距离，默认0.01m，小于此距离的点被过滤
 * @param point_filter_num 点云降采样系数，默认为1（不降采样）
 * @param det_range 最大检测距离，默认1000m
 */
Preprocess::Preprocess()
  :lidar_type(AVIA), blind(0.01), point_filter_num(1), det_range(1000)
{
  // 特征提取相关参数
  inf_bound = 10;              // 无穷远边界阈值
  N_SCANS   = 6;               // 激光雷达扫描线数
  SCAN_RATE = 10;              // 扫描频率（Hz）
  group_size = 8;              // 平面判断时的点群大小
  disA = 0.01;                 // 距离阈值系数A（用于自适应距离阈值）
  disA = 0.1; // B?            // 距离阈值系数B（注：这里可能是disB的笔误）
  p2l_ratio = 225;             // 点到线距离比率阈值，用于平面判断
  limit_maxmid = 6.25;         // 最大距离与中值距离比率的上限（AVIA专用）
  limit_midmin = 6.25;         // 中值距离与最小距离比率的上限（AVIA专用）
  limit_maxmin = 3.24;         // 最大距离与最小距离比率的上限（其他雷达）
  jump_up_limit = 170.0;       // 向上跳变角度阈值（度）
  jump_down_limit = 8.0;       // 向下跳变角度阈值（度）
  cos160 = 160.0;              // 160度角阈值
  edgea = 2;                   // 边缘判断距离比率系数a
  edgeb = 0.1;                 // 边缘判断距离差值系数b
  smallp_intersect = 172.5;    // 小平面夹角阈值（度）
  smallp_ratio = 1.2;          // 小平面距离比率阈值
  given_offset_time = false;   // 是否提供了时间戳偏移（false表示需要计算）

  // 将角度阈值转换为余弦值，提高后续计算效率
  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

/**
 * @brief 析构函数
 */
Preprocess::~Preprocess() {}

/**
 * @brief 设置预处理参数
 * @param feat_en 是否启用特征提取（当前未使用）
 * @param lid_type 雷达类型（AVIA/VELO16/OUST64/HESAIxt32等）
 * @param bld 盲区距离，小于此距离的点被过滤
 * @param pfilt_num 点云降采样系数，每pfilt_num个点取一个
 */
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

/**
 * @brief 处理Livox自定义消息格式的点云数据
 * @param msg Livox雷达的CustomMsg消息指针
 * @param pcl_out 输出的处理后点云
 */
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

/**
 * @brief 处理标准ROS PointCloud2消息格式的点云数据
 * @param msg ROS标准PointCloud2消息指针
 * @param pcl_out 输出的处理后点云
 *
 * 根据不同的雷达类型调用对应的处理函数：
 * - OUST64: Ouster 64线雷达
 * - VELO16: Velodyne 16线雷达
 * - HESAIxt32: 禾赛XT32雷达
 */
void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  // 根据时间单位设置时间缩放因子，统一转换为毫秒(ms)
  switch (time_unit)
  {
    case SEC:  // 秒转毫秒
      time_unit_scale = 1.e3f;
      break;
    case MS:   // 毫秒保持不变
      time_unit_scale = 1.f;
      break;
    case US:   // 微秒转毫秒
      time_unit_scale = 1.e-3f;
      break;
    case NS:   // 纳秒转毫秒
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  // 根据雷达类型调用对应的处理函数
  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case HESAIxt32:
    hesai_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

/**
 * @brief 处理Livox雷达点云并切分成多个子帧
 * @param msg Livox雷达的CustomMsg消息指针
 * @param pcl_out 输出的点云队列，每个元素是一个子帧
 * @param time_lidar 输出的时间戳队列，对应每个子帧的起始时间
 * @param required_frame_num 需要切分的子帧数量
 * @param scan_count 当前扫描计数（用于初始化阶段判断）
 *
 * 该函数将一帧Livox点云按时间均匀切分成多个子帧，用于提高处理频率。
 * 处理流程：
 * 1. 点云滤波（距离滤波、降采样、去重）
 * 2. 按时间戳排序
 * 3. 均匀切分成多个子帧
 */
void Preprocess::process_cut_frame_livox(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                                         deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar,
                                         const int required_frame_num, int scan_count) {
    int plsize = msg->point_num;
    pl_surf.clear();
    pl_surf.reserve(plsize);
    pl_full.clear();
    pl_full.resize(plsize);
    int valid_point_num = 0;

    // 遍历所有点进行预处理
    for (uint i = 1; i < plsize; i++) {
        // 检查点的有效性：扫描线号有效 && 点的质量标签正常
        // tag & 0x30：提取质量标签位
        // 0x10: 正常反射点, 0x00: 高质量点
        if ((msg->points[i].line < N_SCANS) &&
        ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_point_num++;
            // 降采样：每point_filter_num个有效点取一个
            if (valid_point_num % point_filter_num == 0) {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                // 使用curvature字段存储每个点的时间戳，单位：ms
                pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

                // 距离滤波：过滤盲区和超出检测范围的点
                double dist = pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z;
                if (dist < blind * blind || dist > det_range * det_range) continue;

                // 去重：过滤与前一个点位置完全相同的点
                if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7)
                    || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7)
                    || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
                    pl_surf.push_back(pl_full[i]);
                }
            }
        }
    }

    // 按时间戳对点云排序
    sort(pl_surf.points.begin(), pl_surf.points.end(), time_list_cut_frame);

    // 上一帧结束时间，单位：ms
    double last_frame_end_time = msg->header.stamp.toSec() * 1000;
    uint valid_num = 0;
    uint cut_num = 0;
    uint valid_pcl_size = pl_surf.points.size();

    // 前5帧扫描时不进行切分，用于初始化
    int required_cut_num = required_frame_num;
    if (scan_count < 5)
        required_cut_num = 1;

    PointCloudXYZI pcl_cut;
    // 将点云均匀切分成多个子帧
    for (uint i = 1; i < valid_pcl_size; i++) {
        valid_num++;
        // 计算每个点相对于当前帧起始时间的新偏移时间（ms）
        pl_surf[i].curvature += msg->header.stamp.toSec() * 1000 - last_frame_end_time;
        pcl_cut.push_back(pl_surf[i]);

        // 判断是否达到一个子帧的结束位置
        if (valid_num == (int((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
            cut_num++;
            time_lidar.push_back(last_frame_end_time);
            PointCloudXYZI::Ptr pcl_temp(new PointCloudXYZI()); // 初始化shared_ptr
            *pcl_temp = pcl_cut;
            pcl_out.push_back(pcl_temp);
            // 更新下一子帧的起始时间
            last_frame_end_time += pl_surf[i].curvature;
            pcl_cut.clear();
            pcl_cut.reserve(valid_pcl_size * 2 / required_frame_num);
        }
    }
}
#define MAX_LINE_NUM 128  // 最大扫描线数

/**
 * @brief 处理标准PointCloud2格式的点云并切分成多个子帧
 * @param msg ROS标准PointCloud2消息指针
 * @param pcl_out 输出的点云队列，每个元素是一个子帧
 * @param time_lidar 输出的时间戳队列，对应每个子帧的起始时间
 * @param required_frame_num 需要切分的子帧数量
 * @param scan_count 当前扫描计数（用于初始化阶段判断）
 *
 * 支持多种雷达类型（VELO16、OUST64、HESAIxt32）的点云切分处理。
 * 对于没有提供时间戳的雷达，会基于恒定旋转模型计算每个点的时间偏移。
 */
void
Preprocess::process_cut_frame_pcl2(const sensor_msgs::PointCloud2::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out,
                                   deque<double> &time_lidar, const int required_frame_num, int scan_count) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    // ========== Velodyne 16线雷达处理 ==========
    if (lidar_type == VELO16) {
        pcl::PointCloud<velodyne_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        pl_surf.reserve(plsize);

        // 时间戳计算相关变量（用于没有提供时间戳的情况）
        bool is_first[MAX_LINE_NUM];
        double yaw_fp[MAX_LINE_NUM] = {0};     // 每条扫描线第一个点的偏航角
        double omega_l = 3.61;                 // 扫描角速度 (度/毫秒)
        float yaw_last[MAX_LINE_NUM] = {0.0};  // 每条扫描线上一个点的偏航角
        float time_last[MAX_LINE_NUM] = {0.0}; // 每条扫描线上一个点的时间偏移

        // 判断是否提供了时间戳
        if (pl_orig.points[plsize - 1].time > 0) {
            given_offset_time = true;
        } else {
            cout << "Compute offset time using constant rotation model." << endl;
            given_offset_time = false;
            memset(is_first, true, sizeof(is_first));
        }

        // 遍历所有点进行处理
        for (int i = 0; i < plsize; i++) {
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time * 1000.0;  // 秒转毫秒

            // 距离滤波：过滤盲区、超出范围和无效点
            double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if ( dist < blind * blind || dist > det_range * det_range || isnan(added_pt.x) || isnan(added_pt.y) || isnan(added_pt.z))
                continue;

            // 如果没有提供时间戳，基于恒定旋转模型计算
            if (!given_offset_time) {
                int layer = pl_orig.points[i].ring;  // 扫描线编号
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // 弧度转角度

                // 记录每条扫描线的第一个点
                if (is_first[layer]) {
                    yaw_fp[layer] = yaw_angle;
                    is_first[layer] = false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer] = yaw_angle;
                    time_last[layer] = added_pt.curvature;
                    continue;
                }
                // 计算时间偏移：基于角度差和角速度
                if (yaw_angle <= yaw_fp[layer]) {
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
                } else {
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
                }
                // 处理时间回绕情况
                if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
            }

            // 降采样并限制扫描线范围
            if (i % point_filter_num == 0 && pl_orig.points[i].ring < N_SCANS) {
                pl_surf.points.push_back(added_pt);
            }
        }
    // ========== Ouster 64线雷达处理 ==========
    } else if (lidar_type == OUST64) {
        pcl::PointCloud<ouster_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        pl_surf.reserve(plsize);
        for (int i = 0; i < plsize; i++) {
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].t / 1e6;  // 纳秒转毫秒

            // 距离滤波
            double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if ( dist < blind * blind || dist > det_range * det_range || isnan(added_pt.x) || isnan(added_pt.y) || isnan(added_pt.z))
                continue;

            // 降采样并限制扫描线范围
            if (i % point_filter_num == 0 && pl_orig.points[i].ring < N_SCANS) {
                pl_surf.points.push_back(added_pt);
            }
        }
    // ========== HESAI XT32雷达处理 ==========
    } else if(lidar_type == HESAIxt32) {
        pcl::PointCloud<hesai_ros::Point> pl_orig;
        pcl::fromROSMsg(*msg, pl_orig);
        int plsize = pl_orig.points.size();
        pl_surf.reserve(plsize);
        for (int i = 0; i < plsize; i++) {
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = (pl_orig.points[i].timestamp - msg->header.stamp.toSec()) * 1000;  // 秒转毫秒

            // 距离滤波
            double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if ( dist < blind * blind || dist > det_range * det_range || isnan(added_pt.x) || isnan(added_pt.y) || isnan(added_pt.z))
                continue;

            // 降采样并限制扫描线范围
            if (i % point_filter_num == 0 && pl_orig.points[i].ring < N_SCANS) {
                pl_surf.points.push_back(added_pt);
            }
        }
    }else{
        cout << "Wrong LiDAR Type!!!" << endl;
        return;
    }

    // 按时间戳对点云排序
    sort(pl_surf.points.begin(), pl_surf.points.end(), time_list_cut_frame);

    // 初始化切分相关变量
    double last_frame_end_time = msg->header.stamp.toSec() * 1000;  // 上一帧结束时间（ms）
    uint valid_num = 0;
    uint cut_num = 0;
    uint valid_pcl_size = pl_surf.points.size();

    // 前20帧扫描时不进行切分，用于初始化
    int required_cut_num = required_frame_num;
    if (scan_count < 20)
        required_cut_num = 1;

    // 将点云均匀切分成多个子帧
    PointCloudXYZI pcl_cut;
    for (uint i = 1; i < valid_pcl_size; i++) {
        valid_num++;
        // 计算每个点相对于当前帧起始时间的新偏移时间
        pl_surf[i].curvature += msg->header.stamp.toSec() * 1000 - last_frame_end_time;
        pcl_cut.push_back(pl_surf[i]);

        // 判断是否达到一个子帧的结束位置
        if (valid_num == (int((cut_num + 1) * valid_pcl_size / required_cut_num) - 1)) {
            cut_num++;
            time_lidar.push_back(last_frame_end_time);
            PointCloudXYZI::Ptr pcl_temp(new PointCloudXYZI());
            *pcl_temp = pcl_cut;
            pcl_out.push_back(pcl_temp);
            // 更新下一子帧的起始时间
            last_frame_end_time += pl_surf[i].curvature;
            pcl_cut.clear();
            pcl_cut.reserve(valid_pcl_size * 2 / required_frame_num);
        }
    }
}

/**
 * @brief 处理Livox AVIA雷达的点云数据
 * @param msg Livox雷达的CustomMsg消息指针
 *
 * 专门针对Livox AVIA雷达的点云处理函数，进行滤波和预处理。
 * 包括：降采样、距离滤波、去重等操作。
 */
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();  // 记录处理开始时间
  int plsize = msg->point_num;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  // 清空并预分配缓冲区
  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  // 遍历所有点进行处理
  for(uint i=1; i<plsize; i++)
  {
    // 检查点的有效性：扫描线号有效 && 点的质量标签正常
    // tag & 0x30：提取质量标签位，0x10为正常反射点，0x00为高质量点
    if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
    {
      valid_num ++;
      // 降采样：每point_filter_num个有效点取一个
      if (valid_num % point_filter_num == 0)
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        // 使用curvature字段存储每个点的时间戳，单位：ms
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000);

        // 距离滤波：过滤盲区和超出检测范围的点
        double dist = pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z;
        if (dist < blind * blind || dist > det_range * det_range) continue;

        // 去重：过滤与前一个点位置完全相同的点
        if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)))
        {
          pl_surf.push_back(pl_full[i]);
        }
      }
    }
  }

}

/**
 * @brief 处理Ouster 64线雷达的点云数据
 * @param msg ROS标准PointCloud2消息指针
 *
 * 针对Ouster 64线雷达的点云处理函数。
 * Ouster雷达提供完整的时间戳信息，因此不需要额外计算时间偏移。
 */
void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = msg->header.stamp.toSec();

  // 遍历所有点进行处理
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    // 降采样
    if (i % point_filter_num != 0) continue;

    // 计算点到原点的距离
    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    // 距离滤波：过滤盲区、超出范围和无效点
    if (range < (blind * blind) || range > det_range * det_range || isnan(pl_orig.points[i].x) || isnan(pl_orig.points[i].y) || isnan(pl_orig.points[i].z)) continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    // 使用curvature字段存储时间戳，转换为毫秒
    added_pt.curvature = pl_orig.points[i].t * time_unit_scale;

    pl_surf.points.push_back(added_pt);
  }
}

/**
 * @brief 处理Velodyne 16线雷达的点云数据
 * @param msg ROS标准PointCloud2消息指针
 *
 * 针对Velodyne 16线雷达的点云处理函数。
 * 如果雷达没有提供时间戳，会基于恒定旋转模型计算每个点的时间偏移。
 */
void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** 这些变量仅在没有提供时间戳时使用 ***/
    double omega_l = 0.361 * SCAN_RATE;              // 扫描角速度（弧度/秒）
    std::vector<bool> is_first(N_SCANS,true);        // 标记每条扫描线是否为第一个点
    std::vector<double> yaw_fp(N_SCANS, 0.0);        // 每条扫描线第一个点的偏航角
    std::vector<float> yaw_last(N_SCANS, 0.0);       // 每条扫描线上一个点的偏航角
    std::vector<float> time_last(N_SCANS, 0.0);      // 每条扫描线上一个点的时间偏移
    /*****************************************************************/

    // 判断是否提供了时间戳
    if (pl_orig.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      // 以下代码用于计算扫描角度范围（已注释）
      // double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      // double yaw_end  = yaw_first;
      // int layer_first = pl_orig.points[0].ring;
      // for (uint i = plsize - 1; i > 0; i--)
      // {
      //   if (pl_orig.points[i].ring == layer_first)
      //   {
      //     yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
      //     break;
      //   }
      // }
    }

    // 遍历所有点进行处理
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      // 使用curvature字段存储时间戳，单位：ms
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale;

      // 降采样和NaN值过滤
      if (i % point_filter_num != 0 || std::isnan(added_pt.x) || std::isnan(added_pt.y) || std::isnan(added_pt.z)) continue;

      // 如果没有提供时间戳，基于恒定旋转模型计算
      if (!given_offset_time)
      {
        // 以下代码用于根据点索引计算扫描线（已注释）
        // int unit_size = plsize / 16;
        // int layer = i / unit_size;

        int layer = 0;  // 简化处理，使用单一层
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // 弧度转角度

        // 记录每条扫描线的第一个点
        if (is_first[layer])
        {
            yaw_fp[layer]=yaw_angle;
            is_first[layer]=false;
            added_pt.curvature = 0.0;
            yaw_last[layer]=yaw_angle;
            time_last[layer]=added_pt.curvature;
            continue;
        }

        // 计算时间偏移：基于角度差和角速度
        if (yaw_angle < yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
        }

        // 以下代码用于处理时间回绕（已注释）
        // if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;
        // yaw_last[layer] = yaw_angle;
        // time_last[layer]=added_pt.curvature;
      }

      // 距离滤波：保留盲区和检测范围之间的点
      double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
      {
        if(dist > (blind * blind)
        && dist < (det_range * det_range))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }

}

/**
 * @brief 处理HESAI（禾赛）XT32雷达的点云数据
 * @param msg ROS标准PointCloud2消息指针
 *
 * 针对禾赛XT32雷达的点云处理函数。
 * 如果雷达没有提供时间戳，会基于恒定旋转模型计算每个点的时间偏移。
 */
void Preprocess::hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** 这些变量仅在没有提供时间戳时使用 ***/
    double omega_l = 0.361 * SCAN_RATE;              // 扫描角速度（弧度/秒）
    std::vector<bool> is_first(N_SCANS,true);        // 标记每条扫描线是否为第一个点
    std::vector<double> yaw_fp(N_SCANS, 0.0);        // 每条扫描线第一个点的偏航角
    std::vector<float> yaw_last(N_SCANS, 0.0);       // 每条扫描线上一个点的偏航角
    std::vector<float> time_last(N_SCANS, 0.0);      // 每条扫描线上一个点的时间偏移
    /*****************************************************************/

    // 判断是否提供了时间戳
    if (pl_orig.points[plsize - 1].timestamp > 0)
    {
      given_offset_time = true;
    }
    else
    {
      given_offset_time = false;
      // 计算第一条扫描线的角度范围
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end  = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        if (pl_orig.points[i].ring == layer_first)
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
          break;
        }
      }
    }

    double time_head = pl_orig.points[0].timestamp;  // 第一个点的时间戳

    // 遍历所有点进行处理
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      // 使用curvature字段存储时间戳，单位：ms
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * time_unit_scale;

      // 如果没有提供时间戳，基于恒定旋转模型计算
      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;  // 扫描线编号
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // 弧度转角度

        // 记录每条扫描线的第一个点
        if (is_first[layer])
        {
            yaw_fp[layer]=yaw_angle;
            is_first[layer]=false;
            added_pt.curvature = 0.0;
            yaw_last[layer]=yaw_angle;
            time_last[layer]=added_pt.curvature;
            continue;
        }

        // 计算时间偏移：基于角度差和角速度
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
        }

        // 处理时间回绕情况
        if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer]=added_pt.curvature;
      }

      // 降采样、NaN值过滤和距离滤波
      if (i % point_filter_num == 0 && !std::isnan(added_pt.x) && !std::isnan(added_pt.y) && !std::isnan(added_pt.z))
      {
        if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind)
        &&added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z < (det_range * det_range))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }

}

/**
 * @brief 提取点云特征（平面点、边缘点等）
 * @param pl 输入点云
 * @param types 每个点的几何类型信息
 *
 * 该函数对点云进行几何特征分析，识别平面点、边缘点、跳变边缘等特征。
 * 特征类型包括：
 * - Nor: 普通点
 * - Poss_Plane: 可能的平面点
 * - Real_Plane: 确定的平面点
 * - Edge_Plane: 平面边缘点
 * - Edge_Jump: 跳变边缘点
 * - Wire: 线状特征点
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  // 跳过盲区内的点，找到第一个有效点
  while(types[head].range < blind)
  {
    head++;
  }

  // ========== 第一步：平面特征提取 ==========
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());  // 当前平面法向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());  // 上一个平面法向量

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;  // 上一次平面判断的状态
  int plane_type;

  // 遍历点云进行平面识别
  for(uint i=head; i<plsize2; i++)
  {
    // 跳过盲区内的点
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    // 判断当前点是否属于平面
    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if(plane_type == 1)  // 识别到平面
    {
      // 标记平面内的点
      for(uint j=i; j<=i_nex; j++)
      {
        if(j!=i && j!=i_nex)  // 平面内部点标记为确定平面点
        {
          types[j].ftype = Real_Plane;
        }
        else  // 平面边界点标记为可能平面点
        {
          types[j].ftype = Poss_Plane;
        }
      }

      // 检测平面边缘：如果当前平面与上一个平面法向量夹角较大，则为边缘
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;  // 计算法向量点积
        if(mod>-0.707 && mod<0.707)  // 夹角在45度到135度之间
        {
          types[i].ftype = Edge_Plane;  // 标记为平面边缘
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // 未识别到平面
    {
      i = i_nex;
      last_state = 0;
    }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  // ========== 第二步：边缘跳变检测 ==========
  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    // 跳过盲区点和已识别为平面的点
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    // 跳过距离过小的点（避免数值不稳定）
    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);  // 当前点向量
    Eigen::Vector3d vecs[2];  // 前后相邻点的方向向量

    // 计算当前点与前后相邻点的几何关系
    for(int j=0; j<2; j++)
    {
      int m = -1;  // j=0时检查前一个点
      if(j == 1)
      {
        m = 1;     // j=1时检查后一个点
      }

      // 检查相邻点是否在盲区
      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;    // 无穷远边界
        }
        else
        {
          types[i].edj[j] = Nr_blind;  // 盲区边界
        }
        continue;
      }

      // 计算相邻点相对于当前点的方向向量
      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;

      // 计算夹角余弦值
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)      // 夹角接近180度
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)  // 夹角接近0度
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    // 计算前后相邻点方向向量的夹角
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

    // 各种边缘跳变模式检测
    // 模式1：前方正常，后方0度跳变（向外跳变）
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)  // 夹角小于160度
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    // 模式2：前方0度跳变，后方正常（向内跳变）
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    // 模式3：前方正常，后方无穷远
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    // 模式4：前方无穷远，后方正常
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }

    }
    // 模式5：前后都不正常，可能是线状特征
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;  // 标记为线状特征
      }
    }
  }

  // ========== 第三步：小平面检测 ==========
  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    // 跳过盲区点
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }

    // 跳过距离过小的点
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    // 检测小平面：未分类的普通点
    if(types[i].ftype == Nor)
    {
      // 计算与前一个点的距离比率
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      // 如果夹角小且距离比率小，识别为小平面
      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  // ========== 第四步：特征点输出 ==========
  int last_surface = -1;  // 上一个平面点的索引
  for(uint j=head; j<plsize; j++)
  {
    // 处理平面点
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;  // 记录平面点序列的起始位置
      }

      // 每point_filter_num个平面点输出一个
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else  // 处理非平面点
    {
      // 输出边缘点和平面边缘点到角点云
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }

      // 如果前面有未处理的平面点序列，计算平均值并输出
      if(last_surface != -1)
      {
        PointType ap;
        // 计算平面点序列的平均值
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

/**
 * @brief 发布点云的辅助函数（将点云转换为ROS消息格式）
 * @param pl 输入点云
 * @param ct 时间戳
 */
void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}

/**
 * @brief 判断一组点是否构成平面
 * @param pl 输入点云
 * @param types 每个点的几何类型信息
 * @param i_cur 当前点索引
 * @param i_nex 输出参数，平面最后一个点的索引
 * @param curr_direct 输出参数，平面的法向量
 * @return 1表示是平面，0表示不是平面，2表示包含盲区点
 *
 * 平面判断算法：
 * 1. 从当前点开始收集一组相邻点（group_size个）
 * 2. 计算点群的长度和宽度
 * 3. 如果长宽比大于阈值p2l_ratio，则认为是平面
 * 4. 检查点群内距离分布的一致性
 */
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  // 计算自适应的点群距离阈值
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;

  double two_dis;
  vector<double> disarr;  // 存储点群内各点到激光原点的距离
  disarr.reserve(20);

  // 收集初始的group_size个点
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;  // 包含盲区点
    }
    disarr.push_back(types[i_nex].dista);
  }

  // 继续向后收集点，直到距离超过阈值
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }

    // 计算当前点到起始点的距离
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;

    if(two_dis >= group_dis)  // 超出距离阈值，停止收集
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  // 计算点群的"宽度"：所有点到起始点-终止点连线的最大距离
  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;

    // v1: 从起始点到当前点的向量
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    // v2: v1与(vx,vy,vz)的叉积，即点到直线的距离向量
    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;  // 记录最大宽度的平方
    }
  }

  // 判断长宽比：如果长度的平方/宽度的平方 < p2l_ratio，则不是平面
  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  // 对距离数组进行降序排序（冒泡排序）
  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  // 检查距离是否过小
  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  // 根据雷达类型检查距离分布的一致性
  if(lidar_type==AVIA)
  {
    // AVIA雷达：检查最大、中值、最小距离的比率
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;  // 距离分布不一致，不是平面
    }
  }
  else
  {
    // 其他雷达：只检查最大与最小距离的比率
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  // 设置平面法向量（起始点到终止点的方向）
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;  // 确认是平面
}

/**
 * @brief 判断是否为边缘跳变点
 * @param pl 输入点云
 * @param types 每个点的几何类型信息
 * @param i 当前点索引
 * @param nor_dir 检查方向（Prev=0：向前检查，Next=1：向后检查）
 * @return true表示是边缘跳变点，false表示不是
 *
 * 边缘跳变判断：检查当前点附近的距离分布是否符合边缘特征。
 * 如果相邻点之间的距离差异过大，则认为不是边缘跳变。
 */
bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)  // 向前检查
  {
    // 检查前面的点是否在盲区
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)  // 向后检查
  {
    // 检查后面的点是否在盲区
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }

  // 获取相邻两个点到激光原点的距离
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  // 确保d1是较大的距离
  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  // 检查距离比率和距离差值
  // 如果距离差异过大，则不是边缘跳变（可能是噪声或遮挡）
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }

  return true;
}
