// LIO-SAM地图优化模块主文件
#include "utility.h"
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"

// GTSAM库头文件：用于因子图优化
#include <gtsam/geometry/Rot3.h>  // 3D旋转表示
#include <gtsam/geometry/Pose3.h>  // 3D位姿表示
#include <gtsam/slam/PriorFactor.h>  // 先验因子
#include <gtsam/slam/BetweenFactor.h>  // 帧间约束因子
#include <gtsam/navigation/GPSFactor.h>  // GPS因子
#include <gtsam/navigation/ImuFactor.h>  // IMU因子
#include <gtsam/navigation/CombinedImuFactor.h>  // 组合IMU因子
#include <gtsam/nonlinear/NonlinearFactorGraph.h>  // 非线性因子图
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>  // LM优化器
#include <gtsam/nonlinear/Marginals.h>  // 边缘化
#include <gtsam/nonlinear/Values.h>  // 变量值容器
#include <gtsam/inference/Symbol.h>  // 符号表示

#include <gtsam/nonlinear/ISAM2.h>  // 增量式平滑与建图算法

using namespace gtsam;

// 符号简写定义
using symbol_shorthand::X; // Pose3 位姿 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   速度 (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  偏置 (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose GPS位姿

/*
 * 自定义6D位姿点云类型
 * 包含6D位姿信息：[x,y,z,roll,pitch,yaw]
 * intensity字段存储时间戳
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D  // 添加XYZ坐标和padding
    PCL_ADD_INTENSITY;  // 添加intensity字段（用于存储时间戳）
    float roll;   // 翻滚角
    float pitch;  // 俯仰角
    float yaw;    // 偏航角
    double time;  // 时间戳
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐
} EIGEN_ALIGN16;  // 强制SSE对齐以保证内存正确对齐

// 注册自定义点云类型到PCL
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

// 定义位姿点类型别名
typedef PointXYZIRPYT  PointTypePose;


/**
 * @class mapOptimization
 * @brief 地图优化类 - LIO-SAM核心建图与优化模块
 *
 * 主要功能：
 * 1. 基于GTSAM的因子图优化
 * 2. 关键帧管理和局部地图构建
 * 3. 回环检测与闭环优化
 * 4. GPS融合
 * 5. 地图发布与保存
 */
class mapOptimization : public ParamServer
{

public:

    // ===== GTSAM因子图优化相关变量 =====
    NonlinearFactorGraph gtSAMgraph;  // GTSAM因子图
    Values initialEstimate;  // 初始估计值
    Values optimizedEstimate;  // 优化后的估计值
    ISAM2 *isam;  // iSAM2增量优化器
    Values isamCurrentEstimate;  // iSAM2当前估计值
    Eigen::MatrixXd poseCovariance;  // 位姿协方差矩阵

    // ===== ROS发布器 =====
    ros::Publisher pubLaserCloudSurround;  // 发布周围点云地图
    ros::Publisher pubLaserOdometryGlobal;  // 发布全局里程计
    ros::Publisher pubLaserOdometryIncremental;  // 发布增量里程计
    ros::Publisher pubKeyPoses;  // 发布关键帧位姿
    ros::Publisher pubPath;  // 发布路径

    ros::Publisher pubHistoryKeyFrames;  // 发布历史关键帧（用于回环检测）
    ros::Publisher pubIcpKeyFrames;  // 发布ICP配准后的关键帧
    ros::Publisher pubRecentKeyFrames;  // 发布最近的关键帧
    ros::Publisher pubRecentKeyFrame;  // 发布最新关键帧
    ros::Publisher pubCloudRegisteredRaw;  // 发布配准后的原始点云
    ros::Publisher pubLoopConstraintEdge;  // 发布回环约束边

    ros::Publisher pubSLAMInfo;  // 发布SLAM信息（供第三方使用）

    // ===== ROS订阅器 =====
    ros::Subscriber subCloud;  // 订阅特征点云信息
    ros::Subscriber subGPS;  // 订阅GPS数据
    ros::Subscriber subLoop;  // 订阅回环检测信息

    // ===== ROS服务 =====
    ros::ServiceServer srvSaveMap;  // 保存地图服务

    // ===== 数据队列 =====
    std::deque<nav_msgs::Odometry> gpsQueue;  // GPS数据队列
    lio_sam::cloud_info cloudInfo;  // 当前帧点云信息

    // ===== 关键帧点云存储 =====
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;  // 存储所有关键帧的角点点云
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;  // 存储所有关键帧的平面点云

    // ===== 关键帧位姿存储 =====
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;  // 关键帧3D位置（x,y,z）
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;  // 关键帧6D位姿（x,y,z,roll,pitch,yaw）
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;  // 关键帧3D位置的副本（用于回环检测线程）
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;  // 关键帧6D位姿的副本（用于回环检测线程）

    // ===== 当前帧特征点云 =====
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;  // 当前帧角点特征（来自里程计）
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;  // 当前帧平面点特征（来自里程计）
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;  // 降采样后的当前帧角点
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;  // 降采样后的当前帧平面点

    // ===== 优化相关点云 =====
    pcl::PointCloud<PointType>::Ptr laserCloudOri;  // 用于优化的原始点云
    pcl::PointCloud<PointType>::Ptr coeffSel;  // 对应点的残差系数

    // ===== 并行计算临时存储 =====
    std::vector<PointType> laserCloudOriCornerVec;  // 角点原始点（并行计算用）
    std::vector<PointType> coeffSelCornerVec;  // 角点残差系数（并行计算用）
    std::vector<bool> laserCloudOriCornerFlag;  // 角点有效性标志
    std::vector<PointType> laserCloudOriSurfVec;  // 平面点原始点（并行计算用）
    std::vector<PointType> coeffSelSurfVec;  // 平面点残差系数（并行计算用）
    std::vector<bool> laserCloudOriSurfFlag;  // 平面点有效性标志

    // ===== 局部地图点云 =====
    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;  // 局部地图容器（缓存）
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;  // 从地图提取的角点
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;  // 从地图提取的平面点
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;  // 降采样后的地图角点
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;  // 降采样后的地图平面点

    // ===== KD树索引 =====
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;  // 地图角点KD树
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;  // 地图平面点KD树

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;  // 周围关键帧位姿KD树
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;  // 历史关键帧位姿KD树（用于回环检测）

    // ===== 降采样滤波器 =====
    pcl::VoxelGrid<PointType> downSizeFilterCorner;  // 角点降采样滤波器
    pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 平面点降采样滤波器
    pcl::VoxelGrid<PointType> downSizeFilterICP;  // ICP降采样滤波器
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;  // 周围关键帧降采样滤波器

    // ===== 时间戳 =====
    ros::Time timeLaserInfoStamp;  // 激光数据时间戳
    double timeLaserInfoCur;  // 当前激光数据时间（秒）

    // ===== 待优化的位姿变换 =====
    float transformTobeMapped[6];  // 待映射的6D位姿变换 [roll, pitch, yaw, x, y, z]

    // ===== 线程互斥锁 =====
    std::mutex mtx;  // 主线程互斥锁
    std::mutex mtxLoopInfo;  // 回环信息互斥锁

    // ===== 退化检测 =====
    bool isDegenerate = false;  // 是否退化标志
    cv::Mat matP;  // 退化方向矩阵

    // ===== 点云数量 =====
    int laserCloudCornerFromMapDSNum = 0;  // 地图角点数量
    int laserCloudSurfFromMapDSNum = 0;  // 地图平面点数量
    int laserCloudCornerLastDSNum = 0;  // 当前帧角点数量
    int laserCloudSurfLastDSNum = 0;  // 当前帧平面点数量

    // ===== 回环检测相关 =====
    bool aLoopIsClosed = false;  // 是否检测到回环
    map<int, int> loopIndexContainer;  // 回环索引容器（新->旧）
    vector<pair<int, int>> loopIndexQueue;  // 回环索引队列
    vector<gtsam::Pose3> loopPoseQueue;  // 回环位姿队列
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;  // 回环噪声模型队列
    deque<std_msgs::Float64MultiArray> loopInfoVec;  // 回环信息向量

    // ===== 路径 =====
    nav_msgs::Path globalPath;  // 全局路径

    // ===== 变换矩阵 =====
    Eigen::Affine3f transPointAssociateToMap;  // 点关联到地图的变换矩阵
    Eigen::Affine3f incrementalOdometryAffineFront;  // 增量里程计前端变换
    Eigen::Affine3f incrementalOdometryAffineBack;  // 增量里程计后端变换


    /**
     * @brief 构造函数 - 初始化地图优化模块
     *
     * 主要初始化内容：
     * 1. ISAM2优化器参数配置
     * 2. ROS话题发布器和订阅器
     * 3. 降采样滤波器参数
     * 4. 内存分配
     */
    mapOptimization()
    {
        // 配置ISAM2参数
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;  // 重新线性化阈值
        parameters.relinearizeSkip = 1;  // 重新线性化跳过次数
        isam = new ISAM2(parameters);

        // 发布关键位姿和地图相关话题
        pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1);
        pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

        // 订阅特征点云、GPS和回环信息
        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 保存地图服务
        srvSaveMap  = nh.advertiseService("lio_sam/save_map", &mapOptimization::saveMapService, this);

        // 发布回环检测相关话题
        pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        // 发布局部地图相关话题
        pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        // 发布SLAM信息（供第三方使用）
        pubSLAMInfo           = nh.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);

        // 设置降采样滤波器参数
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity);

        // 分配内存
        allocateMemory();
    }

    /**
     * @brief 分配内存 - 为所有点云和数据结构分配内存
     *
     * 初始化内容：
     * 1. 关键帧位姿点云
     * 2. KD树索引结构
     * 3. 特征点云存储
     * 4. 并行计算数组
     * 5. 地图点云
     * 6. 初始变换矩阵
     */
    void allocateMemory()
    {
        // 初始化关键帧位姿点云
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        // 初始化KD树
        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        // 初始化当前帧特征点云
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());  // 来自里程计的角点
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());  // 来自里程计的平面点
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());  // 降采样后的角点
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());  // 降采样后的平面点

        // 初始化优化用点云
        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        // 调整并行计算数组大小（基于激光扫描线数和水平分辨率）
        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        // 初始化标志位为false
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        // 初始化地图点云
        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        // 初始化地图KD树
        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        // 初始化待优化的变换矩阵为0
        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        // 初始化退化检测矩阵
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    /**
     * @brief 激光点云信息回调函数 - 处理来自特征提取模块的点云数据
     *
     * @param msgIn 输入的点云信息消息
     *
     * 处理流程：
     * 1. 提取时间戳和特征点云
     * 2. 更新初始位姿估计
     * 3. 提取周围关键帧构建局部地图
     * 4. 降采样当前扫描
     * 5. 执行扫描到地图的配准优化
     * 6. 保存关键帧和因子图
     * 7. 修正位姿（回环后）
     * 8. 发布里程计和点云
     */
    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        // 提取时间戳
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        // 提取点云信息和特征点云
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        std::lock_guard<std::mutex> lock(mtx);

        // 控制处理频率，避免过于频繁处理
        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            // 步骤1: 更新初始位姿猜测（使用IMU预积分或增量估计）
            updateInitialGuess();

            // 步骤2: 提取周围关键帧，构建局部地图
            extractSurroundingKeyFrames();

            // 步骤3: 降采样当前扫描的特征点
            downsampleCurrentScan();

            // 步骤4: 执行扫描到地图的配准优化
            scan2MapOptimization();

            // 步骤5: 保存关键帧和添加因子到图优化
            saveKeyFramesAndFactor();

            // 步骤6: 修正所有历史位姿（回环检测后）
            correctPoses();

            // 步骤7: 发布里程计信息
            publishOdometry();

            // 步骤8: 发布可视化点云和路径
            publishFrames();
        }
    }

    /**
     * @brief GPS数据回调函数 - 接收GPS数据并加入队列
     * @param gpsMsg GPS里程计消息
     */
    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    /**
     * @brief 将点从激光雷达坐标系变换到地图坐标系
     * @param pi 输入点（激光雷达坐标系）
     * @param po 输出点（地图坐标系）
     */
    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    /**
     * @brief 使用给定位姿变换点云
     * @param cloudIn 输入点云
     * @param transformIn 变换位姿（6D）
     * @return 变换后的点云
     *
     * 使用OpenMP并行加速点云变换
     */
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        // 构建变换矩阵
        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

        // 并行变换所有点
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    /**
     * @brief 将PCL位姿点转换为GTSAM Pose3格式
     * @param thisPoint PCL位姿点
     * @return GTSAM Pose3对象
     */
    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    /**
     * @brief 将变换数组转换为GTSAM Pose3格式
     * @param transformIn 变换数组 [roll, pitch, yaw, x, y, z]
     * @return GTSAM Pose3对象
     */
    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    /**
     * @brief 将PCL位姿点转换为Eigen Affine3f格式
     * @param thisPoint PCL位姿点
     * @return Eigen Affine3f变换矩阵
     */
    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    /**
     * @brief 将变换数组转换为Eigen Affine3f格式
     * @param transformIn 变换数组 [roll, pitch, yaw, x, y, z]
     * @return Eigen Affine3f变换矩阵
     */
    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    /**
     * @brief 将变换数组转换为PCL位姿点格式
     * @param transformIn 变换数组 [roll, pitch, yaw, x, y, z]
     * @return PCL位姿点
     */
    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    













    /**
     * @brief 保存地图服务 - 将全局地图保存为PCD文件
     *
     * @param req 保存地图请求（包含目标路径和分辨率）
     * @param res 保存地图响应（是否成功）
     * @return true 保存成功
     *
     * 保存内容：
     * 1. 关键帧轨迹（trajectory.pcd）
     * 2. 关键帧变换（transformations.pcd）
     * 3. 角点地图（CornerMap.pcd）
     * 4. 平面点地图（SurfMap.pcd）
     * 5. 全局地图（GlobalMap.pcd）
     */
    bool saveMapService(lio_sam::save_mapRequest& req, lio_sam::save_mapResponse& res)
    {
      string saveMapDirectory;

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files ..." << endl;
      // 确定保存路径
      if(req.destination.empty()) saveMapDirectory = std::getenv("HOME") + savePCDDirectory;
      else saveMapDirectory = std::getenv("HOME") + req.destination;
      cout << "Save destination: " << saveMapDirectory << endl;

      // 创建目录并删除旧文件
      int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
      unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());

      // 保存关键帧轨迹和变换
      pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
      pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

      // 提取全局点云地图
      pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

      // 将所有关键帧点云变换到全局坐标系并融合
      for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++) {
          *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i],  &cloudKeyPoses6D->points[i]);
          *globalSurfCloud   += *transformPointCloud(surfCloudKeyFrames[i],    &cloudKeyPoses6D->points[i]);
          cout << "\r" << std::flush << "Processing feature cloud " << i << " of " << cloudKeyPoses6D->size() << " ...";
      }

      // 根据请求的分辨率降采样
      if(req.resolution != 0)
      {
        cout << "\n\nSave resolution: " << req.resolution << endl;

        // 降采样并保存角点地图
        downSizeFilterCorner.setInputCloud(globalCornerCloud);
        downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterCorner.filter(*globalCornerCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);

        // 降采样并保存平面点地图
        downSizeFilterSurf.setInputCloud(globalSurfCloud);
        downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
        downSizeFilterSurf.filter(*globalSurfCloudDS);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
      }
      else
      {
        // 不降采样直接保存
        pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
      }

      // 保存全局点云地图（角点+平面点）
      *globalMapCloud += *globalCornerCloud;
      *globalMapCloud += *globalSurfCloud;

      int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
      res.success = ret == 0;

      // 恢复降采样滤波器参数
      downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
      downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

      cout << "****************************************************" << endl;
      cout << "Saving map to pcd files completed\n" << endl;

      return true;
    }

    /**
     * @brief 可视化全局地图线程 - 定期发布全局地图用于可视化
     *
     * 功能：
     * 1. 以0.2Hz频率发布全局地图
     * 2. 程序结束时自动保存地图（如果启用）
     */
    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok()){
            rate.sleep();
            publishGlobalMap();
        }

        // 如果启用了保存PCD，程序结束时保存地图
        if (savePCD == false)
            return;

        lio_sam::save_mapRequest  req;
        lio_sam::save_mapResponse res;

        if(!saveMapService(req, res)){
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());;
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses; // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for(auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i){
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }












    /**
     * @brief 回环检测线程 - 定期执行回环检测和可视化
     *
     * 功能：
     * 1. 以指定频率执行回环检测
     * 2. 可视化回环约束边
     */
    void loopClosureThread()
    {
        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performLoopClosure();  // 执行回环检测
            visualizeLoopClosure();  // 可视化回环约束
        }
    }

    /**
     * @brief 回环信息回调函数 - 接收外部回环检测模块的回环信息
     *
     * @param loopMsg 回环消息（包含当前帧和历史帧的时间戳）
     *
     * 注意：此函数当前未使用，保留用于外部回环检测接口
     */
    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr& loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        // 保持队列大小不超过5
        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;
        
        visualization_msgs::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }







    



    /**
     * @brief 更新初始位姿估计 - 使用IMU预积分或增量估计更新位姿初值
     *
     * 位姿初始化策略（优先级从高到低）：
     * 1. 首帧：使用IMU姿态初始化
     * 2. IMU预积分可用：使用IMU预积分结果预测位姿
     * 3. IMU增量可用：使用IMU增量更新位姿（仅旋转）
     * 4. 其他：保持上一帧位姿
     *
     * 这个初始估计将作为后续scan-to-map优化的初值
     */
    void updateInitialGuess()
    {
        // 保存当前变换（用于计算增量里程计）
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;

        // 情况1: 首帧初始化
        if (cloudKeyPoses3D->points.empty())
        {
            // 使用IMU提供的初始姿态
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;

            // 如果不使用IMU航向初始化，则yaw设为0
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            return;
        }

        // 情况2: 使用IMU预积分估计位姿（包含位置和姿态）
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            // 从IMU预积分获取当前帧位姿估计
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ,
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                // 首次使用IMU预积分，保存当前值
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                // 计算IMU预积分的增量变换
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                // 应用增量变换到上一帧优化后的位姿
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
                return;
            }
        }

        // 情况3: 使用IMU增量估计位姿（仅旋转，不含位置）
        if (cloudInfo.imuAvailable == true)
        {
            // 获取当前IMU姿态
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            // 计算姿态增量
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            // 应用姿态增量到上一帧位姿
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (loopClosureEnableFlag == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    /**
     * @brief 降采样当前扫描 - 对当前帧的角点和平面点进行降采样
     *
     * 功能：
     * 1. 降采样角点特征
     * 2. 降采样平面点特征
     * 3. 记录降采样后的点数
     *
     * 降采样可以减少计算量，提高scan-to-map优化效率
     */
    void downsampleCurrentScan()
    {
        // 降采样角点
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        // 降采样平面点
        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    /**
     * @brief 更新点到地图的变换矩阵
     *
     * 将当前的位姿估计转换为Affine3f格式，用于点云变换
     */
    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    /**
     * @brief 角点优化 - 计算当前帧角点到地图角点的点到线距离残差
     *
     * 算法流程：
     * 1. 将当前帧角点变换到地图坐标系
     * 2. 在地图中搜索最近的5个角点
     * 3. 通过PCA（主成分分析）拟合直线
     * 4. 计算点到直线的距离作为残差
     * 5. 构建优化系数（雅可比矩阵的一部分）
     *
     * 使用OpenMP并行加速
     */
    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            // 将当前点变换到地图坐标系
            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);

            // 在地图中搜索最近的5个角点
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            // PCA拟合直线所需的矩阵
            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            // 如果最远点距离小于1m，认为找到了有效的对应关系
            if (pointSearchSqDis[4] < 1.0) {
                // 计算5个最近点的中心点
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                // 计算协方差矩阵
                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                // 构建协方差矩阵
                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                // 特征值分解（PCA）
                cv::eigen(matA1, matD1, matV1);

                // 如果最大特征值远大于其他特征值，说明这些点构成一条直线
                // matD1存储特征值（降序），matV1存储对应的特征向量
                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    // 当前点坐标
                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    // 沿主方向（最大特征向量方向）的两个点，用于定义直线
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    // 点到直线距离的分子部分（叉乘的模）
                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    // 直线的长度
                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    // 点到直线距离关于x, y, z的偏导数（雅可比）
                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    // 点到直线的距离
                    float ld2 = a012 / l12;

                    // 权重因子s，距离越大权重越小
                    float s = 1 - 0.9 * fabs(ld2);

                    // 存储优化系数（加权的雅可比和残差）
                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    // 只有当权重足够大时才加入优化
                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    /**
     * @brief 平面点优化 - 计算当前帧平面点到地图平面的点到面距离残差
     *
     * 算法流程：
     * 1. 将当前帧平面点变换到地图坐标系
     * 2. 在地图中搜索最近的5个平面点
     * 3. 通过最小二乘法拟合平面
     * 4. 计算点到平面的距离作为残差
     * 5. 构建优化系数（雅可比矩阵的一部分）
     *
     * 使用OpenMP并行加速
     */
    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            // 将当前点变换到地图坐标系
            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);

            // 在地图中搜索最近的5个平面点
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            // 平面拟合：Ax + By + Cz + D = 0
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            // 如果最远点距离小于1m，认为找到了有效的对应关系
            if (pointSearchSqDis[4] < 1.0) {
                // 构建平面方程的超定方程组
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                // 求解平面参数 [A, B, C]，平面方程为 Ax + By + Cz + 1 = 0
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                // 归一化平面参数，使法向量为单位向量
                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                // 检查平面是否有效：所有点到平面的距离应该都很小
                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    // 计算当前点到平面的距离（点到面残差）
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    // 权重因子s，距离越大权重越小
                    // 同时考虑点到原点的距离（距离越远，权重越小）
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    // 存储优化系数（法向量 * 权重 作为雅可比，距离 * 权重 作为残差）
                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    // 只有当权重足够大时才加入优化
                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    /**
     * @brief 合并优化系数 - 将角点和平面点的优化系数合并到一起
     *
     * 功能：
     * 1. 收集所有有效的角点及其系数
     * 2. 收集所有有效的平面点及其系数
     * 3. 重置标志位，准备下一次迭代
     */
    void combineOptimizationCoeffs()
    {
        // 合并角点系数
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // 合并平面点系数
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // 重置标志位，准备下一次迭代
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // camera -> lidar
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    /**
     * @brief 扫描到地图的配准优化 - LIO-SAM的核心优化函数
     *
     * 功能：
     * 1. 构建角点和平面点的KD树
     * 2. 迭代优化位姿（最多30次）
     *    - 计算角点残差和雅可比
     *    - 计算平面点残差和雅可比
     *    - 使用LM算法求解位姿增量
     * 3. 更新优化后的位姿
     *
     * 退出条件：
     * - 达到最大迭代次数（30次）
     * - LM优化收敛（位姿增量很小）
     */
    void scan2MapOptimization()
    {
        // 首帧不进行优化
        if (cloudKeyPoses3D->points.empty())
            return;

        // 检查特征点数量是否足够
        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            // 为地图点云构建KD树，用于快速最近邻搜索
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            // 迭代优化位姿（最多30次）
            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                // 步骤1: 计算角点匹配和残差
                cornerOptimization();

                // 步骤2: 计算平面点匹配和残差
                surfOptimization();

                // 步骤3: 合并角点和平面点的优化系数
                combineOptimizationCoeffs();

                // 步骤4: 使用LM算法求解位姿增量并更新位姿
                if (LMOptimization(iterCount) == true)
                    break;  // 如果收敛，提前退出
            }

            // 步骤5: 使用IMU约束更新位姿
            transformUpdate();
        } else {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        if (sensor == SensorType::LIVOX)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
                return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold && 
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);
        
        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
        // publish SLAM infomation for 3rd-party usage
        static int lastSLAMInfoPubSize = -1;
        if (pubSLAMInfo.getNumSubscribers() != 0)
        {
            if (lastSLAMInfoPubSize != cloudKeyPoses6D->size())
            {
                lio_sam::cloud_info slamInfo;
                slamInfo.header.stamp = timeLaserInfoStamp;
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                *cloudOut += *laserCloudCornerLastDS;
                *cloudOut += *laserCloudSurfLastDS;
                slamInfo.key_frame_cloud = publishCloud(ros::Publisher(), cloudOut, timeLaserInfoStamp, lidarFrame);
                slamInfo.key_frame_poses = publishCloud(ros::Publisher(), cloudKeyPoses6D, timeLaserInfoStamp, odometryFrame);
                pcl::PointCloud<PointType>::Ptr localMapOut(new pcl::PointCloud<PointType>());
                *localMapOut += *laserCloudCornerFromMapDS;
                *localMapOut += *laserCloudSurfFromMapDS;
                slamInfo.key_frame_map = publishCloud(ros::Publisher(), localMapOut, timeLaserInfoStamp, odometryFrame);
                pubSLAMInfo.publish(slamInfo);
                lastSLAMInfoPubSize = cloudKeyPoses6D->size();
            }
        }
    }
};


/**
 * @brief 主函数 - LIO-SAM地图优化模块入口
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 *
 * 功能：
 * 1. 初始化ROS节点
 * 2. 创建地图优化对象
 * 3. 启动回环检测线程
 * 4. 启动全局地图可视化线程
 * 5. 进入ROS事件循环
 *
 * 线程架构：
 * - 主线程：处理点云数据，执行scan-to-map优化
 * - 回环检测线程：定期检测回环并添加回环约束
 * - 可视化线程：定期发布全局地图用于可视化
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lio_sam");

    // 创建地图优化对象
    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    // 启动回环检测线程
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);

    // 启动全局地图可视化线程
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    // 进入ROS事件循环（主线程处理回调函数）
    ros::spin();

    // 等待子线程结束
    loopthread.join();
    visualizeMapThread.join();

    return 0;
}
