#include "utility.h"
#include "lio_sam/cloud_info.h"

// Velodyne激光雷达点云数据结构
// 包含XYZ坐标、强度、线束编号和时间戳
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D  // 添加XYZ坐标和齐次坐标
    PCL_ADD_INTENSITY;  // 添加强度信息
    uint16_t ring;  // 激光雷达线束编号(扫描线ID)
    float time;  // 点的相对时间戳
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保Eigen内存对齐
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Ouster激光雷达点云数据结构
// 包含XYZ坐标、强度、时间戳、反射率、线束、噪声和距离信息
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;  // 添加XYZ坐标和齐次坐标
    float intensity;  // 强度信息
    uint32_t t;  // 时间戳(纳秒)
    uint16_t reflectivity;  // 反射率
    uint8_t ring;  // 线束编号
    uint16_t noise;  // 噪声值
    uint32_t range;  // 距离信息
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保Eigen内存对齐
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// 使用Velodyne点云格式作为通用表示
using PointXYZIRT = VelodynePointXYZIRT;

// 队列最大长度常量
const int queueLength = 2000;

/**
 * @brief 图像投影类 - 负责将3D激光点云投影到距离图像并进行运动畸变校正
 *
 * 主要功能：
 * 1. 接收原始激光点云数据
 * 2. 使用IMU和里程计数据进行运动畸变校正(deskew)
 * 3. 将点云投影到距离图像(range image)
 * 4. 提取有效点云并发布
 */
class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;  // IMU数据队列互斥锁
    std::mutex odoLock;  // 里程计数据队列互斥锁

    ros::Subscriber subLaserCloud;  // 激光点云订阅器
    ros::Publisher  pubLaserCloud;  // 激光点云发布器

    ros::Publisher pubExtractedCloud;  // 去畸变后点云发布器
    ros::Publisher pubLaserCloudInfo;  // 点云信息发布器

    ros::Subscriber subImu;  // IMU数据订阅器
    std::deque<sensor_msgs::Imu> imuQueue;  // IMU数据队列

    ros::Subscriber subOdom;  // 里程计数据订阅器
    std::deque<nav_msgs::Odometry> odomQueue;  // 里程计数据队列

    std::deque<sensor_msgs::PointCloud2> cloudQueue;  // 点云数据队列
    sensor_msgs::PointCloud2 currentCloudMsg;  // 当前处理的点云消息

    // IMU旋转和时间数据数组
    double *imuTime = new double[queueLength];  // IMU时间戳数组
    double *imuRotX = new double[queueLength];  // IMU绕X轴旋转角度数组
    double *imuRotY = new double[queueLength];  // IMU绕Y轴旋转角度数组
    double *imuRotZ = new double[queueLength];  // IMU绕Z轴旋转角度数组

    int imuPointerCur;  // 当前IMU指针索引
    bool firstPointFlag;  // 第一个点标志位
    Eigen::Affine3f transStartInverse;  // 起始变换矩阵的逆

    // 点云指针
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;  // 输入激光点云
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;  // Ouster临时点云
    pcl::PointCloud<PointType>::Ptr   fullCloud;  // 完整点云(投影后)
    pcl::PointCloud<PointType>::Ptr   extractedCloud;  // 提取的有效点云

    int deskewFlag;  // 去畸变标志(-1:不可用, 0:未检查, 1:可用)
    cv::Mat rangeMat;  // 距离图像矩阵

    // 里程计去畸变相关变量
    bool odomDeskewFlag;  // 里程计去畸变标志
    float odomIncreX;  // 里程计X方向增量
    float odomIncreY;  // 里程计Y方向增量
    float odomIncreZ;  // 里程计Z方向增量

    lio_sam::cloud_info cloudInfo;  // 点云信息结构体
    double timeScanCur;  // 当前扫描开始时间
    double timeScanEnd;  // 当前扫描结束时间
    std_msgs::Header cloudHeader;  // 点云消息头

    vector<int> columnIdnCountVec;  // 每条扫描线的列索引计数向量(用于Livox)


public:
    /**
     * @brief 构造函数 - 初始化订阅器、发布器和数据结构
     */
    ImageProjection():
    deskewFlag(0)
    {
        // 订阅IMU数据，使用tcpNoDelay提高实时性
        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // 订阅增量式里程计数据
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        // 订阅激光点云数据
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布去畸变后的点云
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        // 发布点云附加信息
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

        allocateMemory();  // 分配内存
        resetParameters();  // 重置参数

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);  // 设置PCL日志级别为ERROR
    }

    /**
     * @brief 分配内存 - 为点云数据结构分配内存空间
     */
    void allocateMemory()
    {
        // 初始化各个点云指针
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());  // 输入点云
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());  // Ouster临时点云
        fullCloud.reset(new pcl::PointCloud<PointType>());  // 完整投影点云
        extractedCloud.reset(new pcl::PointCloud<PointType>());  // 提取的有效点云

        // 预分配完整点云大小(扫描线数 × 水平分辨率)
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        // 初始化每条扫描线的起始和结束索引
        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        // 初始化点的列索引和距离信息
        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();  // 重置参数
    }

    /**
     * @brief 重置参数 - 清空数据并重置所有标志位
     */
    void resetParameters()
    {
        laserCloudIn->clear();  // 清空输入点云
        extractedCloud->clear();  // 清空提取的点云
        // 重置距离图像矩阵，初始化为最大浮点数
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;  // 重置IMU指针
        firstPointFlag = true;  // 重置第一个点标志
        odomDeskewFlag = false;  // 重置里程计去畸变标志

        // 清空IMU数据数组
        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);  // 重置列索引计数向量
    }

    /**
     * @brief 析构函数
     */
    ~ImageProjection(){}

    /**
     * @brief IMU数据回调函数 - 接收并存储IMU数据
     * @param imuMsg IMU消息指针
     */
    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        // 转换IMU坐标系(根据配置文件中的外参)
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        // 加锁保护IMU队列
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);  // 将IMU数据加入队列

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x <<
        //       ", y: " << thisImu.linear_acceleration.y <<
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x <<
        //       ", y: " << thisImu.angular_velocity.y <<
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    /**
     * @brief 里程计数据回调函数 - 接收并存储里程计数据
     * @param odometryMsg 里程计消息指针
     */
    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        // 加锁保护里程计队列
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);  // 将里程计数据加入队列
    }

    /**
     * @brief 点云数据回调函数 - 点云处理的主流程
     * @param laserCloudMsg 激光点云消息指针
     *
     * 处理流程：
     * 1. 缓存点云数据
     * 2. 获取去畸变所需的IMU和里程计信息
     * 3. 将点云投影到距离图像
     * 4. 提取有效点云
     * 5. 发布处理后的点云
     */
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // 1. 缓存点云并转换格式
        if (!cachePointCloud(laserCloudMsg))
            return;

        // 2. 获取去畸变信息(IMU和里程计)
        if (!deskewInfo())
            return;

        // 3. 将点云投影到距离图像
        projectPointCloud();

        // 4. 从距离图像中提取有效点云
        cloudExtraction();

        // 5. 发布去畸变后的点云和信息
        publishClouds();

        // 6. 重置参数，准备处理下一帧
        resetParameters();
    }

    /**
     * @brief 缓存点云数据并转换格式
     * @param laserCloudMsg 激光点云消息指针
     * @return 是否成功缓存和转换点云
     */
    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // 将点云加入缓存队列
        cloudQueue.push_back(*laserCloudMsg);
        // 确保队列中至少有2帧点云(用于时间同步)
        if (cloudQueue.size() <= 2)
            return false;

        // 取出队列最前端的点云进行处理
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        // 根据传感器类型转换点云格式
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            // Velodyne和Livox格式直接转换
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Ouster格式需要转换为Velodyne格式
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            // 逐点转换格式
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;  // 将纳秒转换为秒
            }
        }
        else
        {
            // 未知传感器类型，报错并关闭节点
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }

        // 获取时间戳信息
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();  // 当前扫描开始时间
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;  // 当前扫描结束时间

        // 检查点云是否为稠密格式(不包含NaN点)
        if (laserCloudIn->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // 检查点云是否包含ring通道(线束信息)
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;  // 找到ring通道
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        // 检查点云是否包含时间戳通道
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;  // 找到时间戳通道
                    break;
                }
            }
            if (deskewFlag == -1)
                ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    /**
     * @brief 获取去畸变所需的IMU和里程计信息
     * @return 是否成功获取去畸变信息
     */
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);  // 锁定IMU队列
        std::lock_guard<std::mutex> lock2(odoLock);  // 锁定里程计队列

        // 确保IMU数据覆盖整个点云扫描时间段
        // 需要满足：IMU数据在点云开始之前已有，且在点云结束之后仍有
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        // 处理IMU数据，获取旋转信息
        imuDeskewInfo();

        // 处理里程计数据，获取位置信息
        odomDeskewInfo();

        return true;
    }

    /**
     * @brief 处理IMU数据用于去畸变
     *
     * 功能：
     * 1. 移除过时的IMU数据
     * 2. 获取扫描开始时刻的IMU姿态(Roll, Pitch, Yaw)
     * 3. 通过角速度积分计算扫描期间的旋转量
     */
    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        // 移除过时的IMU数据(早于当前扫描0.01秒的数据)
        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        // 遍历IMU队列，处理扫描期间的所有IMU数据
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            // 获取扫描开始时刻的初始姿态角(Roll, Pitch, Yaw)
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            // 超过扫描结束时间，停止处理
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            // 初始化第一个IMU数据
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // 获取角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // 通过角速度积分计算旋转增量
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        // 至少需要2个IMU数据点才能进行插值
        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    /**
     * @brief 处理里程计数据用于去畸变
     *
     * 功能：
     * 1. 移除过时的里程计数据
     * 2. 获取扫描开始和结束时刻的里程计位姿
     * 3. 计算扫描期间的位姿变化(增量)
     * 4. 为后续的地图优化提供初始位姿估计
     */
    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        // 移除过时的里程计数据(早于当前扫描0.01秒的数据)
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        // 确保有扫描开始时刻之前的里程计数据
        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // 获取扫描开始时刻的里程计数据
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        // 将四元数转换为欧拉角
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // 保存初始位姿估计，用于地图优化模块
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // 获取扫描结束时刻的里程计数据
        odomDeskewFlag = false;

        // 确保有扫描结束时刻的里程计数据
        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        // 检查起始和结束里程计是否来自同一轨迹(通过协方差矩阵第一个元素判断)
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        // 计算扫描开始时刻的变换矩阵
        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // 计算扫描结束时刻的变换矩阵
        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        // 计算相对变换(增量)
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        // 提取平移和旋转增量
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;  // 标记里程计去畸变数据已准备好
    }

    /**
     * @brief 根据点的时间戳查找对应的旋转量
     * @param pointTime 点的时间戳
     * @param rotXCur 输出：绕X轴的旋转角度
     * @param rotYCur 输出：绕Y轴的旋转角度
     * @param rotZCur 输出：绕Z轴的旋转角度
     *
     * 通过线性插值计算指定时刻的旋转量
     */
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        // 找到第一个时间戳大于点时间戳的IMU数据索引
        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        // 如果点时间戳超出范围或在第一个IMU数据之前，直接使用该索引的旋转量
        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            // 线性插值计算旋转量
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    /**
     * @brief 根据相对时间查找对应的位置增量
     * @param relTime 相对于扫描开始的时间
     * @param posXCur 输出：X方向位置增量
     * @param posYCur 输出：Y方向位置增量
     * @param posZCur 输出：Z方向位置增量
     *
     * 注意：如果传感器移动较慢(如步行速度)，位置去畸变效果不明显，因此代码被注释
     */
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // 如果传感器移动较慢(如步行速度)，位置去畸变效果不明显，因此下面的代码被注释

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    /**
     * @brief 对单个点进行运动畸变校正
     * @param point 输入点
     * @param relTime 点相对于扫描开始的时间
     * @return 校正后的点
     *
     * 通过IMU和里程计数据，将点转换到扫描起始时刻的坐标系下
     */
    PointType deskewPoint(PointType *point, double relTime)
    {
        // 如果没有时间戳或IMU数据不可用，直接返回原点
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        // 计算点的绝对时间戳
        double pointTime = timeScanCur + relTime;

        // 查找该时刻的旋转量
        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        // 查找该时刻的位置增量
        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        // 对于第一个点，计算并保存起始变换矩阵的逆
        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // 将点变换到扫描起始时刻的坐标系
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        // 应用变换矩阵
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    /**
     * @brief 将点云投影到距离图像
     *
     * 功能：
     * 1. 遍历所有点云
     * 2. 过滤距离超出范围的点
     * 3. 根据线束和水平角度计算点在距离图像中的位置
     * 4. 对点进行运动畸变校正
     * 5. 将校正后的点存储到距离图像中
     */
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // 距离图像投影
        for (int i = 0; i < cloudSize; ++i)
        {
            // 提取点的坐标和强度
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            // 计算点的距离，过滤距离超出范围的点
            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            // 获取点的线束ID(行索引)
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // 根据降采样率过滤点
            if (rowIdn % downsampleRate != 0)
                continue;

            // 计算点在距离图像中的列索引
            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
            {
                // 对于Velodyne和Ouster，根据水平角度计算列索引
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LIVOX)
            {
                // 对于Livox，使用顺序计数作为列索引
                columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
            }

            // 检查列索引是否有效
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            // 如果该位置已有点，跳过(保留第一个点)
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // 对点进行运动畸变校正
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            // 在距离图像中记录距离值
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 将校正后的点存储到完整点云中
            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    /**
     * @brief 从距离图像中提取有效点云
     *
     * 功能：
     * 1. 遍历距离图像
     * 2. 提取所有有效点(距离不为FLT_MAX)
     * 3. 记录每条扫描线的起始和结束索引
     * 4. 保存点的列索引和距离信息
     */
    void cloudExtraction()
    {
        int count = 0;
        // 提取分割后的点云用于激光里程计
        for (int i = 0; i < N_SCAN; ++i)
        {
            // 记录当前扫描线的起始索引(预留5个点的边界)
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // 标记点的列索引，用于后续遮挡检测
                    cloudInfo.pointColInd[count] = j;
                    // 保存距离信息
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // 保存提取的点云
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // 更新提取点云的大小
                    ++count;
                }
            }
            // 记录当前扫描线的结束索引(预留5个点的边界)
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }

    /**
     * @brief 发布处理后的点云和信息
     *
     * 发布去畸变后的点云和附加信息(线束索引、距离等)
     */
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

/**
 * @brief 主函数 - 初始化ROS节点并启动图像投影模块
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "lio_sam");

    // 创建图像投影对象
    ImageProjection IP;

    // 打印启动信息(绿色高亮)
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    // 使用多线程spinner处理回调(3个线程)
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
