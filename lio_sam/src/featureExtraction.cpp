// LIO-SAM 特征提取模块
// 功能：从去畸变后的点云中提取角点和平面点特征
#include "utility.h"
#include "lio_sam/cloud_info.h"

// 平滑度结构体，用于存储点的平滑度值及其索引
struct smoothness_t{
    float value;  // 平滑度数值
    size_t ind;   // 点的索引
};

// 平滑度比较结构体，用于按平滑度值排序
struct by_value{
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

/**
 * @brief 特征提取类
 * 该类负责从去畸变后的点云中提取角点特征和平面点特征
 * 特征提取基于点云的曲率计算，并进行遮挡点标记和特征筛选
 */
class FeatureExtraction : public ParamServer
{

public:

    // ROS订阅器：订阅去畸变后的点云信息
    ros::Subscriber subLaserCloudInfo;

    // ROS发布器：发布提取后的特征信息
    ros::Publisher pubLaserCloudInfo;   // 发布包含特征点的点云信息
    ros::Publisher pubCornerPoints;     // 发布角点点云
    ros::Publisher pubSurfacePoints;    // 发布平面点点云

    // 点云数据存储
    pcl::PointCloud<PointType>::Ptr extractedCloud;  // 提取的原始点云
    pcl::PointCloud<PointType>::Ptr cornerCloud;     // 角点点云
    pcl::PointCloud<PointType>::Ptr surfaceCloud;    // 平面点点云

    // 体素滤波器，用于平面点的降采样
    pcl::VoxelGrid<PointType> downSizeFilter;

    // 点云信息和消息头
    lio_sam::cloud_info cloudInfo;  // 点云信息消息
    std_msgs::Header cloudHeader;   // 消息头

    // 特征提取相关数据结构
    std::vector<smoothness_t> cloudSmoothness;  // 点云平滑度向量
    float *cloudCurvature;                       // 点云曲率数组
    int *cloudNeighborPicked;                    // 邻域点是否被选择的标记数组
    int *cloudLabel;                             // 点云标签数组（1:角点, -1:平面点, 0:未标记）

    /**
     * @brief 构造函数
     * 初始化ROS订阅器和发布器，并调用参数初始化函数
     */
    FeatureExtraction()
    {
        // 订阅去畸变后的点云信息话题，使用TCP无延迟传输
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布特征提取后的点云信息和特征点云
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);

        // 初始化数据结构
        initializationValue();
    }

    /**
     * @brief 初始化所有数据结构
     * 为点云处理分配内存空间，设置滤波器参数
     */
    void initializationValue()
    {
        // 分配平滑度向量空间，大小为扫描线数×水平分辨率
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        // 设置体素滤波器的叶子大小（降采样网格大小）
        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        // 初始化点云指针
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        // 为特征提取相关数组分配内存
        cloudCurvature = new float[N_SCAN*Horizon_SCAN];      // 曲率数组
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];   // 邻域点选择标记
        cloudLabel = new int[N_SCAN*Horizon_SCAN];            // 点标签数组
    }

    /**
     * @brief 点云信息处理回调函数
     * @param msgIn 输入的去畸变点云信息消息
     *
     * 处理流程：
     * 1. 计算点云平滑度
     * 2. 标记遮挡点
     * 3. 提取特征点
     * 4. 发布特征点云
     */
    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn; // 保存点云信息
        cloudHeader = msgIn->header; // 保存消息头
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // 将ROS消息转换为PCL点云

        // 计算每个点的平滑度（曲率）
        calculateSmoothness();

        // 标记遮挡点和平行光束点，这些点不适合作为特征点
        markOccludedPoints();

        // 提取角点和平面点特征
        extractFeatures();

        // 发布特征点云
        publishFeatureCloud();
    }

    /**
     * @brief 计算点云平滑度（曲率）
     *
     * 使用当前点前后各5个点的距离值来计算曲率
     * 曲率计算公式：c = |Σ(r[i-5:i+5]) - 10*r[i]|^2
     * 其中r表示点的距离值
     *
     * 曲率大的点可能是边缘点（角点），曲率小的点可能是平面点
     */
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        // 遍历每个点（跳过前后各5个点，因为需要邻域点）
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 计算距离差值：前5个点+后5个点的距离和 减去 当前点距离的10倍
            // 这个值反映了当前点与其邻域点的距离变化程度
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];

            // 曲率为距离差值的平方
            cloudCurvature[i] = diffRange*diffRange;

            // 初始化邻域点选择标记为0（未被选择）
            cloudNeighborPicked[i] = 0;
            // 初始化点标签为0（未分类）
            cloudLabel[i] = 0;
            // 保存平滑度值和索引，用于后续排序
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    /**
     * @brief 标记遮挡点和平行光束点
     *
     * 遮挡点：两个相邻点的距离差异过大时，较近点附近的点可能被遮挡
     * 平行光束点：激光束几乎平行于物体表面，测量不准确
     * 这些点不适合作为特征点，需要标记并排除
     */
    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // 标记遮挡点和平行光束点
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // 遮挡点检测
            float depth1 = cloudInfo.pointRange[i];      // 当前点深度
            float depth2 = cloudInfo.pointRange[i+1];    // 下一个点深度
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));  // 列索引差

            // 如果两点在距离图像中相邻（列差小于10）
            if (columnDiff < 10){
                // 如果当前点比下一个点远很多（深度差>0.3m），说明当前点附近可能被遮挡
                if (depth1 - depth2 > 0.3){
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                // 如果下一个点比当前点远很多，说明下一个点附近可能被遮挡
                else if (depth2 - depth1 > 0.3){
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // 平行光束检测
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));

            // 如果当前点与前后两点的距离差都大于距离的2%，可能是平行光束
            // 这种情况下测量不准确，标记为不可用
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    /**
     * @brief 提取角点和平面点特征
     *
     * 处理流程：
     * 1. 将每条扫描线分成6个区域
     * 2. 在每个区域内按曲率排序
     * 3. 提取曲率最大的点作为角点（最多20个）
     * 4. 提取曲率最小的点作为平面点
     * 5. 对平面点进行降采样
     */
    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        // 临时存储每条扫描线的平面点
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        // 遍历每条扫描线
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();

            // 将每条扫描线分成6个区域，分别提取特征
            for (int j = 0; j < 6; j++)
            {
                // 计算当前区域的起始点和结束点索引
                // 使用线性插值将扫描线分成6段
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                // 如果区域无效，跳过
                if (sp >= ep)
                    continue;

                // 按曲率值对当前区域内的点进行排序（从小到大）
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                // 提取角点特征（曲率最大的点）
                int largestPickedNum = 0;
                // 从曲率最大的点开始遍历（因为已经排序）
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    // 如果该点未被标记且曲率大于边缘阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        // 每个区域最多提取20个角点
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;  // 标记为角点
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        // 标记该点已被选择
                        cloudNeighborPicked[ind] = 1;
                        // 标记该点后续的5个邻域点为已选择，避免特征点过于集中
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 标记该点前面的5个邻域点为已选择
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 提取平面点特征（曲率最小的点）
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    // 如果该点未被标记且曲率小于平面阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        // 标记为平面点
                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        // 标记该点后续的5个邻域点为已选择
                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        // 标记该点前面的5个邻域点为已选择
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 收集所有非角点（平面点或未标记点）
                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            // 对当前扫描线的平面点进行降采样
            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            // 累加到总的平面点云中
            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    /**
     * @brief 释放点云信息内存
     * 清空不再需要的点云信息数据，释放内存空间
     */
    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();  // 清空扫描线起始索引
        cloudInfo.endRingIndex.clear();    // 清空扫描线结束索引
        cloudInfo.pointColInd.clear();     // 清空点列索引
        cloudInfo.pointRange.clear();      // 清空点距离信息
    }

    /**
     * @brief 发布特征点云
     *
     * 功能：
     * 1. 释放点云信息内存
     * 2. 发布角点和平面点点云
     * 3. 将特征点云信息发布给地图优化模块
     */
    void publishFeatureCloud()
    {
        // 释放点云信息内存
        freeCloudInfoMemory();
        // 保存并发布新提取的特征点云
        cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // 发布点云信息给地图优化模块
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


/**
 * @brief 主函数
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出状态码
 *
 * 功能：
 * 1. 初始化ROS节点
 * 2. 创建特征提取对象
 * 3. 启动ROS事件循环
 */
int main(int argc, char** argv)
{
    // 初始化ROS节点，节点名称为"lio_sam"
    ros::init(argc, argv, "lio_sam");

    // 创建特征提取对象
    FeatureExtraction FE;

    // 打印启动信息（绿色文本）
    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");

    // 进入ROS事件循环，处理回调函数
    ros::spin();

    return 0;
}