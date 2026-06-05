/**
 * @file laserMapping.cpp
 * @brief Point-LIO激光雷达里程计与建图主程序
 *
 * 本文件实现了基于点云的激光雷达惯性里程计（LIO）系统的核心功能。
 * 主要功能包括：
 * - 激光雷达点云数据处理与去畸变
 * - IMU数据融合与状态估计
 * - 基于iVox的增量式地图构建
 * - 迭代卡尔曼滤波器（IEKF）的状态更新
 * - 实时里程计发布与轨迹可视化
 *
 * Point-LIO特点：
 * - 支持点到点（Point-to-Point）残差模型
 * - 使用iVox进行高效近邻搜索
 * - 支持两种滤波器模式：以IMU为输入或以雷达为输入
 * - 实现了紧耦合的传感器融合框架
 */

// #include <so3_math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "li_initialization.h"
#include <malloc.h>
// #include <cv_bridge/cv_bridge.h>
// #include "matplotlibcpp.h"
// #include <ros/console.h>

using namespace std;     

// ===== 常量定义 =====
#define PUBFRAME_PERIOD     (20)  // 发布帧周期

const float MOV_THRESHOLD = 1.5f;  // 运动阈值

// ===== 全局变量 =====
string root_dir = ROOT_DIR;  // 根目录路径

int time_log_counter = 0;  // 时间日志计数器

bool init_map = false;          // 地图初始化标志
bool flg_first_scan = true;     // 首帧扫描标志

// 时间日志变量
double match_time = 0;      // 匹配耗时
double solve_time = 0;      // 求解耗时
double propag_time = 0;     // 传播耗时
double update_time = 0;     // 更新耗时

bool flg_reset = false;     // 重置标志
bool flg_exit = false;      // 退出标志

// ===== 点云相关变量 =====
// 去畸变后的特征点云
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
// 机体坐标系下降采样后的点云
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
// 世界坐标系下的初始化特征点云
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
// 深度特征点云队列
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
// PCL降采样滤波器
pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 表面点降采样滤波器
pcl::VoxelGrid<PointType> downSizeFilterMap;   // 地图点降采样滤波器

// ===== 位姿与路径相关变量 =====
V3D euler_cur;  // 当前欧拉角

nav_msgs::Path path;                           // 轨迹路径
nav_msgs::Odometry odomAftMapped;              // 里程计消息
geometry_msgs::PoseStamped msg_body_pose;      // 机体位姿消息

/**
 * @brief 信号处理函数
 * @param sig 信号类型
 *
 * 捕获系统信号（如Ctrl+C），设置退出标志并通知所有等待线程
 */
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

/**
 * @brief 将LIO状态信息输出到日志文件
 * @param fp 文件指针
 *
 * 将当前时刻的姿态、位置、速度、角速度、加速度、偏差等状态信息写入日志文件
 * 根据use_imu_as_input标志选择使用kf_input或kf_output的状态
 */
inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang;
    // 根据滤波器类型选择不同的状态源
    if (!use_imu_as_input)
    {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    }
    else
    {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }
    
    // 输出时间戳（相对于第一帧雷达时间）
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    // 输出欧拉角（roll, pitch, yaw）
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));
    if (use_imu_as_input)
    {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    }
    else
    {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2)); // Bias_a  
    }
    fprintf(fp, "\r\n");  
    fflush(fp);
}

/**
 * @brief 将点从激光雷达坐标系转换到IMU坐标系
 * @param pi 输入点（激光雷达坐标系）
 * @param po 输出点（IMU坐标系）
 *
 * 根据外参标定结果，将激光雷达坐标系下的点转换到IMU坐标系
 * 如果启用外参估计，则使用卡尔曼滤波器估计的外参；否则使用配置文件中的外参
 */
void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    // 根据是否启用外参估计选择不同的转换方式
    if (extrinsic_est_en)
    {
        // 使用卡尔曼滤波器估计的外参
        if (!use_imu_as_input)
        {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        }
        else
        {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }
    else
    {
        // 使用配置文件中的外参
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

/**
 * @brief 增量式地图更新
 *
 * 将当前帧的点云增量添加到全局地图中
 * 采用体素化策略避免重复添加：
 * - 计算每个点所属的体素中心
 * - 检查该体素是否已有点存在
 * - 仅添加未被占据体素中的点
 * 最终通过iVox数据结构高效地添加新点
 */
void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i) {
        // 判断是否需要将点添加到地图
        PointType &point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector &points_near = Nearest_Points[i];

            // 计算点所属体素的中心位置
            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
            bool need_add = true;
            // 检查近邻点中是否已有点在该体素中心附近
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                // 如果近邻点距离体素中心足够近，则无需添加新点
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
                    fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            // 如果没有近邻点，直接添加
            points_to_add.emplace_back(point_world);
        }
    }
    // 批量添加点到iVox地图
    ivox_->AddPoints(points_to_add);
}

/**
 * @brief 发布初始化地图
 * @param pubLaserCloudFullRes ROS发布器
 *
 * 在地图初始化完成后，发布初始化阶段收集的点云地图
 */
void publish_init_map(const ros::Publisher & pubLaserCloudFullRes)
{
    int size_init_map = init_feats_world->size();

    sensor_msgs::PointCloud2 laserCloudmsg;

    // 将PCL点云转换为ROS消息格式
    pcl::toROSMsg(*init_feats_world, laserCloudmsg);

    // 设置时间戳和坐标系
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
}

// 用于发布和保存的点云缓存
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

/**
 * @brief 发布世界坐标系下的点云帧
 * @param pubLaserCloudFullRes ROS发布器
 *
 * 主要功能：
 * 1. 将当前帧点云转换到世界坐标系并发布（如果启用）
 * 2. 保存点云到PCD文件（如果启用）
 */
void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    // 发布点云
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; //
        }
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        // publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** 保存地图 ****************/
    /* 注意事项：
    /* 1. 确保有足够的内存空间
    /* 2. 注意PCD保存会影响实时性能 **/
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        // 累积点云
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        // 达到保存间隔后写入PCD文件
        if (pcl_wait_save->size() > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

/**
 * @brief 发布机体坐标系下的点云帧
 * @param pubLaserCloudFull_body ROS发布器
 *
 * 将去畸变后的点云从激光雷达坐标系转换到IMU坐标系（机体坐标系）并发布
 */
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    // 将每个点从激光雷达坐标系转换到IMU坐标系
    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
}

/**
 * @brief 设置位姿信息
 * @param out 输出的位姿消息（模板类型）
 *
 * 根据滤波器类型选择相应的状态源，将位置和姿态信息填充到消息中
 * 姿态以四元数形式表示
 */
template<typename T>
void set_posestamp(T & out)
{
    if (!use_imu_as_input)
    {
        // 使用输出滤波器的状态
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        Eigen::Quaterniond q(kf_output.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
    else
    {
        // 使用输入滤波器的状态
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        Eigen::Quaterniond q(kf_input.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

/**
 * @brief 发布里程计信息
 * @param pubOdomAftMapped ROS发布器
 *
 * 发布当前的里程计估计，包括：
 * - 位姿信息（位置和姿态）
 * - TF变换（从camera_init到body坐标系）
 */
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    // 根据配置选择时间戳
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped.publish(odomAftMapped);

    // 发布TF变换
    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body") );
}

/**
 * @brief 发布轨迹路径
 * @param pubPath ROS发布器
 *
 * 将当前位姿添加到轨迹路径中并发布，用于在RViz中可视化机器人运动轨迹
 */
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";
    static int jjj = 0;
    jjj++;
    // 注意：如果路径过长可能导致RViz崩溃，可以通过取模减少发布频率
    {
        path.poses.emplace_back(msg_body_pose);
        pubPath.publish(path);
    }
}

/**
 * @brief 主函数 - Point-LIO系统入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 *
 * 主要流程：
 * 1. 初始化ROS节点和参数
 * 2. 初始化iVox地图结构
 * 3. 设置卡尔曼滤波器
 * 4. 订阅传感器话题（激光雷达、IMU）
 * 5. 创建发布器（点云、里程计、路径等）
 * 6. 进入主循环处理传感器数据
 * 7. 执行点云去畸变、滤波更新、地图构建等操作
 */
int main(int argc, char** argv)
{
    // ===== ROS初始化 =====
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // 读取参数配置
    readParameters(nh);
    cout<<"lidar_type: "<<lidar_type<<endl;

    // 初始化iVox地图数据结构
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    path.header.stamp    = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id ="camera_init";

    // ===== 统计变量定义 =====
    int frame_num = 0;
    double aver_time_consu = 0;    // 平均总耗时
    double aver_time_icp = 0;      // 平均ICP耗时
    double aver_time_match = 0;    // 平均匹配耗时
    double aver_time_incre = 0;    // 平均增量耗时
    double aver_time_solve = 0;    // 平均求解耗时
    double aver_time_propag = 0;   // 平均传播耗时

    // ===== 初始化降采样滤波器 =====
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    // ===== 设置激光雷达到IMU的外参 =====
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);

    // 如果启用外参估计，将初始外参设置到卡尔曼滤波器状态中
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
        else
        {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    // ===== 设置IMU处理器参数 =====
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    // ===== 初始化卡尔曼滤波器 =====
    // 初始化输入滤波器（24维状态，2个观测模型）
    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    // 初始化输出滤波器（30维状态，3个观测模型）
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);

    // 设置初始协方差矩阵
    Eigen::Matrix<double, 24, 24> P_init;
    reset_cov(P_init);
    kf_input.change_P(P_init);

    Eigen::Matrix<double, 30, 30> P_init_output;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);

    // 设置过程噪声协方差矩阵
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    // ===== 打开调试日志文件 =====
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");
    open_file();

    // ===== ROS订阅器初始化 =====
    // 根据雷达类型选择不同的点云回调函数
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    // ===== ROS发布器初始化 =====
    // 发布配准后的点云（世界坐标系）
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 1000);
    // 发布配准后的点云（机体坐标系）
    ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 1000);
    // 发布地图点云
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 1000);
    // 发布里程计
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 1000);
    // 发布轨迹路径
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
            ("/path", 1000);
//------------------------------------------------------------------------------------------------------
    // ===== 主循环 =====
    signal(SIGINT, SigHandle);
    ros::Rate loop_rate(500);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        // 同步传感器数据包
        if(sync_packages(Measures))
        {
            // ----- 系统重置处理 -----
            if (flg_reset)
            {
                ROS_WARN("reset when rosbag play back");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                if (use_imu_as_input)
                {
                    // state_in = kf_input.get_x();
                    state_in = state_input();
                    kf_input.change_P(P_init);
                }
                else
                {
                    // state_out = kf_output.get_x();
                    state_out = state_output();
                    kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;
                
                {
                    ivox_.reset(new IVoxType(ivox_options_));
                }
            }

            // ----- 首帧处理 -----
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1)
                {
                    first_imu_time = imu_next.header.stamp.toSec();
                    printf("first imu time: %f\n", first_imu_time);
                }
                time_current = 0.0;
                if(imu_en)
                {
                    // imu_next = *(imu_deque.front());
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
                    // kf_output.x_.acc *= -1; 

                    {
                        while (Measures.lidar_beg_time > imu_next.header.stamp.toSec()) // if it is needed for the new map?
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty())
                            {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            // imu_deque.pop();
                        }
                    }
                }
                else
                {
                    kf_input.x_.gravity << VEC_FROM_ARRAY(gravity); // _init);
                    kf_output.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc << VEC_FROM_ARRAY(gravity); //_init);
                    kf_output.x_.acc *= -1; 
                    p_imu->imu_need_init_ = false;
                    // p_imu->after_imu_init_ = true;
                }     
                G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
            }

            // ----- 计时变量初始化 -----
            double t0,t1,t2,t3,t4,t5,match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();

            // ----- 点云预处理与降采样 -----
            t1 = omp_get_wtime();
            // IMU处理：去畸变等
            p_imu->Process(Measures, feats_undistort);
            // 空间降采样（如果启用）
            if(space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                // 按时间排序点云
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            {
                // 时间压缩：将点云按时间分组
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }

            // ----- IMU初始化 -----
            if (!p_imu->after_imu_init_)
            {
                if (!p_imu->imu_need_init_)
                {
                    // 计算初始重力方向
                    V3D tmp_gravity;
                    if (imu_en)
                    {
                        tmp_gravity = - p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    }
                    else
                    {
                        tmp_gravity << VEC_FROM_ARRAY(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    // 根据重力方向初始化旋转
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                    kf_output.x_.acc = - rot_init.transpose() * kf_output.x_.gravity;
                }
                else{
                continue;}
            }

            // ----- 地图初始化 -----
            if(!init_map)
            {
                // 将点云转换到世界坐标系
                feats_down_world->resize(feats_undistort->size());
                for(int i = 0; i < feats_undistort->size(); i++)
                {
                    {
                        pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                    }
                }
                // 累积初始化点云
                for (size_t i = 0; i < feats_down_world->size(); i++)
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                // 检查是否收集足够的点用于初始化
                if(init_feats_world->size() < init_map_size)
                {init_map = false;}
                else
                {
                    // 将初始化点云添加到iVox地图
                    ivox_->AddPoints(init_feats_world->points);
                    publish_init_map(pubLaserCloudMap);

                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                }
                continue;
            }

            // ----- ICP匹配与卡尔曼滤波更新 -----
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            t2 = omp_get_wtime();

            // 迭代状态估计准备
            // 预留空间以提高效率
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);

            // 计算点云的叉乘矩阵（用于雅可比计算）
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                if (!extrinsic_est_en)
                // {
                //     if (!use_imu_as_input)
                //     {
                //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                //     }
                //     else
                //     {
                //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
                //     }
                // }
                // else
                {
                    // 将点转换到IMU坐标系
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    // 计算点的反对称矩阵（用于叉乘运算）
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);
                    crossmat_list[i]=point_crossmat;
                }
            }

            // ===== 滤波器更新：两种模式 =====
            // 模式1：不使用IMU作为输入（输出模式）
            if (!use_imu_as_input)
            {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                // 逐点更新策略
                if (time_seq.size() > 0)
                {
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                // 遍历时间序列组
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

                    // 计算当前点的时间戳
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                    if (is_first_frame)
                    {
                        if(imu_en)
                        {
                            while (time_current > imu_next.header.stamp.toSec())
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        imu_upda_cov = true;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if(imu_en && !imu_deque.empty())
                    {
                        bool last_imu = imu_next.header.stamp.toSec() == imu_deque.front()->header.stamp.toSec();
                        while (imu_next.header.stamp.toSec() < time_predict_last_const && !imu_deque.empty())
                        {
                            if (!last_imu)
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                break;
                            }
                            else
                            {
                                imu_deque.pop_front();
                                if (imu_deque.empty()) break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                        bool imu_comes = time_current > imu_next.header.stamp.toSec();
                        while (imu_comes) 
                        {
                            imu_upda_cov = true;
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            // 协方差预测更新
                            double dt = imu_next.header.stamp.toSec() - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = imu_next.header.stamp.toSec();
                            
                            {
                                double dt_cov = imu_next.header.stamp.toSec() - time_update_last;

                                if (dt_cov > 0.0)
                                {
                                    time_update_last = imu_next.header.stamp.toSec();
                                    double propag_imu_start = omp_get_wtime();

                                    // 状态传播
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    // IMU观测更新
                                    kf_output.update_iterated_dyn_share_IMU();
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_comes = time_current > imu_next.header.stamp.toSec();
                        }
                    }
                    if (flg_reset)
                    {
                        break;
                    }

                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1)
                    {
                        ROS_WARN("No point, skip this scan!\n");
                        idx += time_seq[k];
                        continue;
                    }
                    // 迭代卡尔曼滤波更新（点云观测）
                    if (!kf_output.update_iterated_dyn_share_modified())
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    solve_start = omp_get_wtime();

                    // 发布里程计（无降采样模式）
                    if (publish_odometry_without_downsample)
                    {
                        /******* 发布里程计 *******/

                        publish_odometry(pubOdomAftMapped);
                        if (runtime_pos_log)
                        {
                            euler_cur = SO3ToEuler(kf_output.x_.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " " << kf_output.x_.vel.transpose() \
                            <<" "<<kf_output.x_.omg.transpose()<<" "<<kf_output.x_.acc.transpose()<<" "<<kf_output.x_.gravity.transpose()<<" "<<kf_output.x_.bg.transpose()<<" "<<kf_output.x_.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    // 将当前组内的点转换到世界坐标系
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                
                    solve_time += omp_get_wtime() - solve_start;
    
                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                    // cout << "pbp output effect feat num:" << effct_feat_num << endl;
                }
                }
                else
                {
                    if (!imu_deque.empty())
                    { 
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                    while (imu_next.header.stamp.toSec() > time_current && ((imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte )))
                    { // >= ?
                        if (is_first_frame)
                        {
                            {
                                {
                                    while (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front());
                                    }
                                }
                                break;
                            }
                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                                            
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                            imu_upda_cov = true;
                            time_update_last = time_current;
                            time_predict_last_const = time_current;

                                is_first_frame = false;
                        }
                        time_current = imu_next.header.stamp.toSec();

                        if (!is_first_frame)
                        {
                        double dt = time_current - time_predict_last_const;
                        {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0)
                            {
                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                            kf_output.predict(dt, Q_output, input_in, true, false);
                        }

                        time_predict_last_const = time_current;

                        angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                        kf_output.update_iterated_dyn_share_IMU();
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    else
                    {
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                    }
                    }
                    }
                }
            }
            // 模式2：使用IMU作为输入（输入模式）
            else
            {
                bool imu_prop_cov = false;
                effct_feat_num = 0;
                if (time_seq.size() > 0)
                {
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame)
                    {
                        while (time_current > imu_next.header.stamp.toSec()) 
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                        }
                        imu_prop_cov = true;

                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current; 
                        {
                            input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;                 
                            input_in.acc<<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }
                    
                    while (time_current > imu_next.header.stamp.toSec()) // && !imu_deque.empty())
                    {
                        imu_deque.pop_front();
                        
                        input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z; 
                        input_in.acc    = input_in.acc * G_m_s2 / acc_norm; 
                        double dt = imu_last.header.stamp.toSec() - t_last;

                        double dt_cov = imu_last.header.stamp.toSec() - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = imu_last.header.stamp.toSec(); //time_current;
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false); 
                        t_last = imu_last.header.stamp.toSec();
                        imu_prop_cov = true;

                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        // imu_upda_cov = true;
                    }     
                    if (flg_reset)
                    {
                        break;
                    }     
                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime();
                    
                    if(!prop_at_freq_of_imu)
                    {   
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {    
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = time_current; 
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false); 

                    propag_time += omp_get_wtime() - propag_start;

                    double t_update_start = omp_get_wtime();
                    
                    if (feats_down_size < 1)
                    {
                        ROS_WARN("No point, skip this scan!\n");

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified()) 
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();

                    if (publish_odometry_without_downsample)
                    {
                        /******* Publish odometry *******/

                        publish_odometry(pubOdomAftMapped);
                        if (runtime_pos_log)
                        {
                            euler_cur = SO3ToEuler(kf_input.x_.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " " << kf_input.x_.vel.transpose() \
                            <<" "<<kf_input.x_.bg.transpose()<<" "<<kf_input.x_.ba.transpose()<<" "<<kf_input.x_.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j); 
                    }
                    solve_time += omp_get_wtime() - solve_start;
                
                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }  
                }
                else
                {
                    if (!imu_deque.empty())
                    { 
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    while (imu_next.header.stamp.toSec() > time_current && ((imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)))
                    { // >= ?
                        if (is_first_frame)
                        {
                            {
                                {
                                    while (imu_next.header.stamp.toSec() < Measures.lidar_beg_time + lidar_time_inte)
                                    {
                                        imu_deque.pop_front();
                                        if(imu_deque.empty()) break;
                                        imu_last = imu_next;
                                        imu_next = *(imu_deque.front());
                                    }
                                }
                                
                                break;
                            }
                            imu_prop_cov = true;
                            
                            t_last = time_current;
                            time_update_last = time_current; 
                            input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            input_in.acc   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            
                                is_first_frame = false;
                            
                        }
                        time_current = imu_next.header.stamp.toSec();

                        if (!is_first_frame)
                        {
                        double dt = time_current - t_last;

                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {        
                            // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = imu_next.header.stamp.toSec(); //time_current;
                        }
                        // kf_input.predict(dt, Q_input, input_in, true, false);

                        t_last = imu_next.header.stamp.toSec();
                    
                        input_in.gyro<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        input_in.acc<<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z; 
                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        imu_deque.pop_front();
                        if (imu_deque.empty()) break;
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        }
                        else
                        {
                            imu_deque.pop_front();
                            if (imu_deque.empty()) break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                        }
                    }
                    }
                }
            }
            // M3D rot_cur_lidar;
            // {
            //     rot_cur_lidar = state.rot_end;
            // }
            // euler_cur = RotMtoEuler(rot_cur_lidar);
            // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            //                     (euler_cur(0), euler_cur(1), euler_cur(2));
            // ----- 发布里程计（降采样模式） -----
            if (!publish_odometry_without_downsample)
            {
                publish_odometry(pubOdomAftMapped);
            }

            // ----- 增量式地图更新 -----
            t3 = omp_get_wtime();

            if(feats_down_size > 4)
            {
                MapIncremental();
            }

            t5 = omp_get_wtime();

            // ----- 发布可视化信息 -----
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFullRes);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
            
            // ----- 调试信息记录 -----
            if (runtime_pos_log)
            {
                // 更新统计变量
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                {aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + update_time/frame_num;}
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + solve_time/frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1)/frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                // 输出性能统计信息
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu, aver_time_icp, aver_time_propag); 
                if (!publish_odometry_without_downsample)
                {
                    if (!use_imu_as_input)
                    {
                        euler_cur = SO3ToEuler(kf_output.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " " << kf_output.x_.vel.transpose() \
                        <<" "<<kf_output.x_.omg.transpose()<<" "<<kf_output.x_.acc.transpose()<<" "<<kf_output.x_.gravity.transpose()<<" "<<kf_output.x_.bg.transpose()<<" "<<kf_output.x_.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                    else
                    {
                        euler_cur = SO3ToEuler(kf_input.x_.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " " << kf_input.x_.vel.transpose() \
                        <<" "<<kf_input.x_.bg.transpose()<<" "<<kf_input.x_.ba.transpose()<<" "<<kf_input.x_.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
        status = ros::ok();
        loop_rate.sleep();
    }

    // ===== 程序退出时保存地图 =====
    /* 注意事项：
    /* 1. 确保有足够的内存
    /* 2. 注意PCD保存会影响实时性能 **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "Saving final map to " << all_points_dir << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    // 关闭日志文件
    fout_out.close();
    fout_imu_pbp.close();
    return 0;
}
