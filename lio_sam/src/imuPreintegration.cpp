// 包含工具函数头文件
#include "utility.h"

// GTSAM几何相关头文件
#include <gtsam/geometry/Rot3.h>         // 3D旋转表示
#include <gtsam/geometry/Pose3.h>        // 3D位姿表示(旋转+平移)
#include <gtsam/slam/PriorFactor.h>      // 先验因子
#include <gtsam/slam/BetweenFactor.h>    // 相对位姿因子
#include <gtsam/navigation/GPSFactor.h>  // GPS因子
#include <gtsam/navigation/ImuFactor.h>  // IMU预积分因子
#include <gtsam/navigation/CombinedImuFactor.h>  // 组合IMU因子
#include <gtsam/nonlinear/NonlinearFactorGraph.h> // 非线性因子图
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h> // LM优化器
#include <gtsam/nonlinear/Marginals.h>   // 边缘化相关
#include <gtsam/nonlinear/Values.h>      // 变量值容器
#include <gtsam/inference/Symbol.h>      // 符号表示

#include <gtsam/nonlinear/ISAM2.h>       // 增量式平滑与建图优化器
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h> // 固定时延平滑器

using gtsam::symbol_shorthand::X; // Pose3 位姿 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   速度 (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  偏置 (ax,ay,az,gx,gy,gz)

/**
 * @class TransformFusion
 * @brief 变换融合类：融合IMU里程计和激光里程计，发布融合后的里程计信息
 *
 * 该类订阅IMU增量里程计和激光里程计，通过计算IMU的增量变换，
 * 将其叠加到激光里程计上，实现高频率的里程计输出
 */
class TransformFusion : public ParamServer
{
public:
    std::mutex mtx; // 互斥锁，保护数据访问

    // ROS订阅器
    ros::Subscriber subImuOdometry;   // 订阅IMU里程计
    ros::Subscriber subLaserOdometry; // 订阅激光里程计

    // ROS发布器
    ros::Publisher pubImuOdometry;    // 发布融合后的IMU里程计
    ros::Publisher pubImuPath;        // 发布IMU轨迹路径

    // 位姿变换矩阵
    Eigen::Affine3f lidarOdomAffine;      // 激光里程计位姿
    Eigen::Affine3f imuOdomAffineFront;   // IMU里程计队列前端位姿
    Eigen::Affine3f imuOdomAffineBack;    // IMU里程计队列后端位姿

    // TF变换相关
    tf::TransformListener tfListener;     // TF监听器
    tf::StampedTransform lidar2Baselink;  // 激光坐标系到基座标系的变换

    // 时间戳和数据队列
    double lidarOdomTime = -1;            // 最新激光里程计时间戳
    deque<nav_msgs::Odometry> imuOdomQueue; // IMU里程计队列

    /**
     * @brief TransformFusion构造函数
     *
     * 初始化TF变换监听器，订阅激光和IMU里程计，发布融合后的里程计和路径
     */
    TransformFusion()
    {
        // 如果激光坐标系和基座标系不同，则获取它们之间的变换
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                // 等待TF变换可用，最多等待3秒
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                // 查找激光坐标系到基座标系的变换
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        // 订阅激光里程计话题（来自建图模块）
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 订阅IMU增量里程计话题（来自IMU预积分模块）
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        // 发布融合后的IMU里程计
        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        // 发布IMU路径用于可视化
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
    }

    /**
     * @brief 将里程计消息转换为仿射变换矩阵
     * @param odom 里程计消息
     * @return Eigen::Affine3f 仿射变换矩阵（4x4）
     */
    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        // 提取位置信息
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        // 提取旋转信息（四元数转欧拉角）
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // 构造并返回仿射变换矩阵
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    /**
     * @brief 激光里程计回调函数
     * @param odomMsg 激光里程计消息
     *
     * 接收并保存最新的激光里程计位姿和时间戳
     */
    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // 将里程计消息转换为仿射变换矩阵并保存
        lidarOdomAffine = odom2affine(*odomMsg);

        // 保存激光里程计时间戳
        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    /**
     * @brief IMU里程计回调函数
     * @param odomMsg IMU增量里程计消息
     *
     * 融合IMU增量里程计和激光里程计，发布高频融合里程计、TF变换和轨迹路径
     * 核心思路：lidarOdom * (imuOdomFront^-1 * imuOdomBack) = 融合后的位姿
     */
    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // 发布静态TF变换：map -> odom (单位变换)
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        // 将当前IMU里程计加入队列
        imuOdomQueue.push_back(*odomMsg);

        // 如果还没有收到激光里程计，直接返回
        if (lidarOdomTime == -1)
            return;

        // 清除IMU里程计队列中早于最新激光里程计的数据
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }

        // 获取IMU里程计队列的前后端位姿
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());

        // 计算IMU增量变换：从激光里程计时刻到当前时刻的相对变换
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;

        // 将IMU增量变换叠加到激光里程计上，得到融合后的位姿
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

        // 发布融合后的最新里程计
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // 发布TF变换：odom -> base_link
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        // 如果激光坐标系和基座标系不同，需要额外变换
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // 发布IMU路径用于可视化
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        // 每隔0.1秒添加一个路径点
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            // 删除1秒前的路径点，保持路径长度
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            // 如果有订阅者，发布路径
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

/**
 * @class IMUPreintegration
 * @brief IMU预积分类：使用GTSAM进行IMU预积分和位姿优化
 *
 * 该类实现基于因子图优化的IMU预积分，融合激光里程计作为位姿约束，
 * 通过ISAM2增量优化估计位姿、速度和IMU偏置，并发布高频IMU里程计
 */
class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx; // 互斥锁

    // ROS订阅器和发布器
    ros::Subscriber subImu;           // 订阅IMU原始数据
    ros::Subscriber subOdometry;      // 订阅激光里程计（用于优化校正）
    ros::Publisher pubImuOdometry;    // 发布IMU增量里程计

    bool systemInitialized = false;   // 系统初始化标志

    // GTSAM噪声模型：用于定义各类因子的不确定性
    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;   // 初始位姿先验噪声
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;    // 初始速度先验噪声
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;   // 初始偏置先验噪声
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;  // 激光里程计校正噪声（正常情况）
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2; // 激光里程计校正噪声（退化情况）
    gtsam::Vector noiseModelBetweenBias;                      // IMU偏置变化噪声

    // IMU预积分器：分别用于优化和里程计发布
    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;   // 用于因子图优化的预积分器
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;   // 用于发布IMU里程计的预积分器

    // IMU数据队列
    std::deque<sensor_msgs::Imu> imuQueOpt; // 用于优化的IMU队列
    std::deque<sensor_msgs::Imu> imuQueImu; // 用于里程计的IMU队列

    // 优化后的上一时刻状态（用于因子图优化）
    gtsam::Pose3 prevPose_;                      // 上一时刻位姿
    gtsam::Vector3 prevVel_;                     // 上一时刻速度
    gtsam::NavState prevState_;                  // 上一时刻导航状态（位姿+速度）
    gtsam::imuBias::ConstantBias prevBias_;      // 上一时刻IMU偏置

    // 用于IMU里程计发布的状态
    gtsam::NavState prevStateOdom;               // 里程计上一时刻导航状态
    gtsam::imuBias::ConstantBias prevBiasOdom;   // 里程计上一时刻IMU偏置

    // 时间戳管理
    bool doneFirstOpt = false;   // 是否完成第一次优化
    double lastImuT_imu = -1;    // 上一次用于里程计的IMU时间戳
    double lastImuT_opt = -1;    // 上一次用于优化的IMU时间戳

    // GTSAM优化相关
    gtsam::ISAM2 optimizer;                      // ISAM2增量优化器
    gtsam::NonlinearFactorGraph graphFactors;    // 因子图
    gtsam::Values graphValues;                   // 变量初始值

    const double delta_t = 0; // 时间偏移量（未使用）

    int key = 1; // 当前关键帧索引

    // T_bl: 从激光坐标系到IMU坐标系的变换
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: 从IMU坐标系到激光坐标系的变换
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    /**
     * @brief IMUPreintegration构造函数
     *
     * 初始化订阅器、发布器、噪声模型和IMU预积分器
     */
    IMUPreintegration()
    {
        // 订阅IMU原始数据
        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        // 订阅激光里程计增量数据（用于优化校正）
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布IMU增量里程计
        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

        // 配置IMU预积分参数
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // 加速度计连续时间白噪声协方差
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // 陀螺仪连续时间白噪声协方差
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);        // 从速度积分位置时产生的误差
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished()); // 假设初始偏置为零

        // 配置噪声模型
        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // 初始位姿噪声 (rad,rad,rad,m,m,m)
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);  // 初始速度噪声 (m/s) 设置较大表示不确定
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 初始偏置噪声 1e-2 ~ 1e-3 效果较好
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // 激光里程计校正噪声（正常）
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // 激光里程计校正噪声（退化）
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished(); // 偏置变化噪声

        // 创建IMU预积分器
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // 用于IMU里程计发布的预积分器
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // 用于因子图优化的预积分器
    }

    /**
     * @brief 重置优化器
     *
     * 清空ISAM2优化器、因子图和变量值，用于系统初始化或定期重置
     */
    void resetOptimization()
    {
        // 配置ISAM2优化器参数
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1; // 重新线性化阈值
        optParameters.relinearizeSkip = 1;        // 每次都进行重新线性化
        optimizer = gtsam::ISAM2(optParameters);

        // 清空因子图
        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        // 清空变量值
        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    /**
     * @brief 重置参数
     *
     * 重置系统状态标志，用于故障检测后的恢复
     */
    void resetParams()
    {
        lastImuT_imu = -1;              // 重置IMU时间戳
        doneFirstOpt = false;           // 重置优化完成标志
        systemInitialized = false;      // 重置系统初始化标志
    }

    /**
     * @brief 激光里程计回调函数（核心优化函数）
     * @param odomMsg 激光里程计消息
     *
     * 执行三个主要步骤：
     * 1. 系统初始化：添加先验因子
     * 2. IMU预积分与优化：添加IMU因子、偏置因子和激光位姿因子，执行优化
     * 3. 重新传播：用优化后的偏置重新预积分IMU数据，用于高频里程计发布
     */
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // 获取当前激光里程计时间戳
        double currentCorrectionTime = ROS_TIME(odomMsg);

        // 确保有IMU数据可用于积分
        if (imuQueOpt.empty())
            return;

        // 提取激光里程计位姿
        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false; // 检查是否退化（协方差第一个元素为1表示退化）
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // ========== 步骤0: 系统初始化 ==========
        if (systemInitialized == false)
        {
            // 重置优化器
            resetOptimization();

            // 清除早于当前激光里程计的旧IMU数据
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            // 添加初始位姿先验因子（激光坐标系位姿转换到IMU坐标系）
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);

            // 添加初始速度先验因子（假设初始静止）
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);

            // 添加初始偏置先验因子（假设零偏置）
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);

            // 将初始值插入变量值容器
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);

            // 执行一次优化
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            // 用初始偏置重置两个预积分器
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            // 设置关键帧索引为1，标记系统已初始化
            key = 1;
            systemInitialized = true;
            return;
        }


        // ========== 定期重置因子图以提升速度 ==========
        // 当关键帧数量达到100时，重置因子图，避免图过大导致优化变慢
        if (key == 100)
        {
            // 在重置前获取最新的协方差（噪声模型）
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));

            // 重置因子图
            resetOptimization();

            // 用更新后的噪声模型添加位姿先验因子
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);

            // 添加速度先验因子
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);

            // 添加偏置先验因子
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);

            // 插入变量初始值
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);

            // 执行一次优化
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            // 重置关键帧索引
            key = 1;
        }


        // ========== 步骤1: 积分IMU数据并执行优化 ==========
        // 积分从上次优化到当前激光里程计时刻之间的所有IMU数据
        while (!imuQueOpt.empty())
        {
            // 取出并积分两次优化之间的IMU数据
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                // 计算时间间隔（第一次假设500Hz）
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                // 积分IMU测量值（加速度和角速度）
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }

        // 添加IMU预积分因子到因子图
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);

        // 添加IMU偏置between因子（假设偏置缓慢变化）
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        // 添加激光里程计位姿因子（激光坐标系转IMU坐标系）
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);

        // 插入预测的初始值（用IMU预积分结果作为初值）
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        // 执行优化
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();

        // 获取优化后的结果，作为下一次优化的起点
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));

        // 用优化后的偏置重置预积分器
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        // 检查优化结果是否异常（速度或偏置过大）
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // ========== 步骤2: 优化后，用新偏置重新传播IMU里程计 ==========
        // 保存优化后的状态和偏置，用于IMU里程计发布
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;

        // 首先清除早于当前激光里程计的IMU数据
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }

        // 用优化后的偏置重新积分IMU数据
        if (!imuQueImu.empty())
        {
            // 用新优化的偏置重置预积分器
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);

            // 从本次优化开始重新积分所有IMU数据
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        // 增加关键帧索引，标记已完成第一次优化
        ++key;
        doneFirstOpt = true;
    }

    /**
     * @brief 故障检测：检查优化结果是否异常
     * @param velCur 当前速度
     * @param biasCur 当前IMU偏置
     * @return true表示检测到异常，false表示正常
     *
     * 检测速度和偏置是否超过合理范围，若异常则需要重置系统
     */
    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        // 检查速度是否过大（>30m/s）
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        // 检查加速度计和陀螺仪偏置是否过大（>1.0）
        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    /**
     * @brief IMU原始数据回调函数
     * @param imu_raw IMU原始数据
     *
     * 接收IMU数据，执行预积分，并发布高频IMU增量里程计
     * 这个里程计基于上次激光里程计优化后的状态和偏置进行预测
     */
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // 转换IMU数据（坐标系对齐等）
        sensor_msgs::Imu thisImu = imuConverter(*imu_raw);

        // 将IMU数据同时加入两个队列
        imuQueOpt.push_back(thisImu);  // 用于优化的队列
        imuQueImu.push_back(thisImu);  // 用于里程计发布的队列

        // 如果还没完成第一次优化，则不发布里程计
        if (doneFirstOpt == false)
            return;

        // 计算时间间隔
        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // 积分这一帧IMU数据
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // 用预积分器预测当前状态
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // 准备发布里程计消息
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // 将IMU位姿转换到激光坐标系
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        // 填充位姿信息
        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        // 填充速度信息（线性速度和角速度）
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();

        // 发布IMU增量里程计
        pubImuOdometry.publish(odometry);
    }
};


/**
 * @brief 主函数
 *
 * 初始化ROS节点，创建IMU预积分和变换融合对象，启动多线程处理
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "roboat_loam");

    // 创建IMU预积分对象（订阅IMU和激光里程计，执行因子图优化）
    IMUPreintegration ImuP;

    // 创建变换融合对象（融合IMU和激光里程计，发布高频里程计和路径）
    TransformFusion TF;

    // 输出启动信息
    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

    // 启动多线程spinner（4个线程）处理回调函数
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
