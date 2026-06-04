//
// Created by fangcheng on 2023/7/16.
//

#ifndef MULTIUAV_H
#define MULTIUAV_H

/**
 * 多无人机协同定位模块头文件
 * 实现多无人机间的相对定位、全局外参估计、轨迹匹配和可视化功能
 */

#include <omp.h>
#include <unistd.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <swarm_msgs/QuadStatePub.h>
#include <swarm_msgs/ObserveTeammate.h>
#include <swarm_msgs/GlobalExtrinsicStatus.h>
#include <swarm_msgs/GlobalExtrinsic.h>
#include <swarm_msgs/ConnectedTeammateList.h>
#include <common_lib.h>
#include <pcl/filters/filter.h>
#include "esikf_tracker.hpp"
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include "ExtrinsicInfection.hpp"
#include <algorithm>
#include <unordered_map>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/Int8.h>
#include "FEC.h"

// 无人机ID到扩展卡尔曼滤波器的映射表类型定义
typedef unordered_map<int, ESIKF> id_esikf_map;
typedef id_esikf_map::value_type id2ekf;

using namespace std;
using namespace Eigen;

/**
 * 多无人机管理类
 * 负责处理多架无人机之间的通信、相对定位、全局外参估计和可视化
 */
class Multi_UAV {
public:
    // 将参数转换为字符串的模板函数
    template<class T>
    string SetString(T &param_in);

    // 构造函数：初始化多无人机系统
    Multi_UAV(const ros::NodeHandle &nh, const int & drone_id_);

    // 析构函数：清理资源
    ~Multi_UAV();

    /**
     * 队友无人机状态订阅数据结构
     * 存储从其他无人机接收到的状态信息
     */
    struct QuadStateSub {
        // 默认构造函数：初始化所有成员变量
        QuadStateSub() {
            this->sub_time = 0.0;
            this->swarmlio_start_time = 0.0;
            this->rot_cov.setZero();
            this->pos_cov.setZero();
            this->rot = M3D::Identity();
            this->pos = Zero3d;
            this->gyr = Zero3d;
            this->vel = Zero3d;
            this->rot_end = M3D::Identity();
            this->pos_end = Zero3d;
            this->my_pos_in_teammate = Zero3d;
            this->degenerated = false;
            this->is_observed = false;
        };

        // 拷贝构造函数
        QuadStateSub(const QuadStateSub &b) {
            this->sub_time = b.sub_time;
            this->swarmlio_start_time = b.swarmlio_start_time;
            this->rot_cov = b.rot_cov;
            this->pos_cov = b.pos_cov;
            this->rot = b.rot;
            this->pos = b.pos;
            this->gyr = b.gyr;
            this->vel = b.vel;
            this->rot_end = b.rot_end;
            this->pos_end = b.pos_end;
            this->my_pos_in_teammate = b.my_pos_in_teammate;
            this->degenerated = b.degenerated;
            this->is_observed = b.is_observed;
        };

        // 赋值运算符重载
        QuadStateSub& operator=(const QuadStateSub& b){
            this->sub_time = b.sub_time;
            this->swarmlio_start_time = b.swarmlio_start_time;
            this->rot_cov = b.rot_cov;
            this->pos_cov = b.pos_cov;
            this->rot = b.rot;
            this->pos = b.pos;
            this->gyr = b.gyr;
            this->vel = b.vel;
            this->rot_end = b.rot_end;
            this->pos_end = b.pos_end;
            this->my_pos_in_teammate = b.my_pos_in_teammate;
            this->degenerated = b.degenerated;
            this->is_observed = b.is_observed;
            return *this;
        };

        double sub_time; // 接收到队友状态的时间戳
        double swarmlio_start_time; // 队友无人机Swarm-LIO系统的启动时间
        M3D rot_cov; // 接收到的队友姿态协方差矩阵
        M3D pos_cov; // 接收到的队友位置协方差矩阵
        M3D rot;         // 队友在时刻sub_time的姿态旋转矩阵
        V3D pos;         // 队友在时刻sub_time的位置
        V3D gyr;         // 队友在时刻sub_time的去偏角速度（在其自身机体坐标系下）
        V3D vel;         // 队友在时刻sub_time的线速度（在其世界坐标系下）
        M3D rot_end;     // 队友在时刻t_k（帧结束时刻）的姿态旋转矩阵
        V3D pos_end;     // 队友在时刻t_k（帧结束时刻）的位置
        V3D my_pos_in_teammate; // 队友观测到的我在其坐标系下的相对位置
        bool degenerated; // 队友是否处于退化状态
        bool is_observed; // 我是否被队友观测到
    };

    /**
     * 临时跟踪器结构
     * 用于跟踪未知ID的动态物体，直到通过轨迹匹配确认其身份
     */
    struct TemporaryTracker {
        // 构造函数：初始化临时跟踪器
        TemporaryTracker(const ESIKF &tracker, const double &timestamp, const int &id) {
            this->dyn_tracker = tracker;
            Vector4d pos_and_time;
            pos_and_time.block<3, 1>(0, 0) = this->dyn_tracker.get_state_pos();
            pos_and_time(3) = timestamp;
            this->dyn_pos_time.push_back(pos_and_time);
            this->exist_meas = false;
            this->meas_of_tracker = Zero3d;
            this->create_time = timestamp;
            this->id = id;
        };

        // 拷贝构造函数
        TemporaryTracker(const TemporaryTracker &b) {
            this->dyn_tracker = b.dyn_tracker;
            this->dyn_pos_time = b.dyn_pos_time;
            this->exist_meas = b.exist_meas;
            this->meas_of_tracker = b.meas_of_tracker;
            this->create_time = b.create_time;
            this->id = b.id;
        };
        ESIKF dyn_tracker; // 动态物体的卡尔曼滤波跟踪器
        deque<Vector4d> dyn_pos_time; // 动态物体的位置和时间戳历史队列（前3维是位置，第4维是时间）
        bool exist_meas; // 当前帧是否存在对该物体的观测
        V3D meas_of_tracker; // 当前帧对该物体的观测值（在机体坐标系下）
        double create_time; // 跟踪器创建时间
        int id; // 临时跟踪器的ID
    };

    /**
     * 聚类结果结构
     * 存储点云聚类提取的物体信息
     */
    struct Cluster {
        // 构造函数：初始化聚类信息
        Cluster(const V3D &pos, const bool &is_high_inten, const double &max_dist) {
            this->pos_in_body = pos;
            this->is_high_intensity = is_high_inten;
            this->max_dist = max_dist;
        };

        // 拷贝构造函数
        Cluster(const Cluster &b) {
            this->pos_in_body = b.pos_in_body;
            this->is_high_intensity = b.is_high_intensity;
            this->max_dist = b.max_dist;
        };
        V3D pos_in_body; // 聚类中心在机体坐标系下的位置
        bool is_high_intensity; // 是否是高反射强度聚类
        double max_dist; // 聚类的最大尺寸（包围盒对角线长度）
    };

    /**
     * 队友无人机信息结构
     * 存储每个队友的完整状态和连接信息
     */
    struct Teammate {
        // 默认构造函数
        Teammate() {
            teammate_pos_in_body = Zero3d;
            is_observe_teammate = false;
            last_connect_time = 0.0;
            first_connect_time = - 1.0;
            teammate_odom_time.clear();
            total_dist = 0.0;
            last_position = Zero3d;
            world_to_gravity_deg = Zero3d;
        };
        // 拷贝构造函数
        Teammate(const Teammate &b) {
            this->teammate_state = b.teammate_state;
            this->teammate_state_temp = b.teammate_state_temp;
            this->teammate_pos_in_body = b.teammate_pos_in_body;
            this->is_observe_teammate = b.is_observe_teammate;
            this->last_connect_time = b.last_connect_time;
            this->first_connect_time = b.first_connect_time;
            this->teammate_odom_time = b.teammate_odom_time;
            this->total_dist = b.total_dist;
            this->last_position = b.last_position;
            this->world_to_gravity_deg = b.world_to_gravity_deg;
        };
        QuadStateSub teammate_state; // 队友的状态信息（已确认）
        QuadStateSub teammate_state_temp; // 队友的状态信息（临时缓存）
        V3D teammate_pos_in_body; // 观测到的队友在本机机体坐标系下的位置
        bool is_observe_teammate; // 本机是否观测到该队友
        double last_connect_time; // 最后一次与队友建立通信连接的时间
        double first_connect_time; // 首次与队友建立通信连接的时间
        deque<Vector4d> teammate_odom_time; // 队友里程计轨迹历史（位置+时间戳）
        double total_dist; // 队友累计飞行距离
        V3D last_position; // 队友上一次的位置（用于计算累计距离）
        V3D world_to_gravity_deg; // 队友世界坐标系到重力对齐坐标系的旋转角度（欧拉角，单位：度）
    };

    // 从上三角向量构建姿态和位置协方差矩阵
    void BuildMatrixWithUpperTriangular(const VD(12) &vec, M3D &rot_cov, M3D &pos_cov);

    // 队友状态消息回调函数：接收并处理来自其他无人机的状态信息
    void QuadstateCbk(const swarm_msgs::QuadStatePub::ConstPtr &msg);

    // 全局外参消息回调函数：接收并处理来自其他无人机的全局外参信息
    void GlobalExtrinsicCbk(const swarm_msgs::GlobalExtrinsicStatus::ConstPtr &msg);

    // 重置重连队友的全局外参：当队友重启时，清除旧的外参信息
    void ResetReconnectedGlobalExtrinsic(StatesGroup &state_in, const double &lidar_end_time);

    // 更新因子图：使用ISAM2优化全局外参
    void UpdateFactorGraph(const bool &print_log);

    // 更新全局外参并创建新的队友跟踪器：基于外参传染模型
    void UpdateGlobalExtrinsicAndCreateNewTeammateTracker(StatesGroup &state_in, const double &lidar_end_time);

    // 将临时缓存的队友状态复制到正式状态
    void CopyTeammateState(Teammate &teammate);

    // 重置每个队友的观测标志位
    void ResetTeammateState();

    // 发布本机状态：包含位姿、速度、协方差和对队友的观测信息
    void PublishQuadstate(const V3D &unbiased_gyr, const double &lidar_end_time, const double &first_lidar_time);

    // 发布全局外参：向其他无人机广播本机与队友之间的外参关系
    void PublishGlobalExtrinsic(const double &lidar_end_time);


    // 从ROS参数服务器加载单个参数
    template<class T>
    bool LoadParam(string param_name, T &param_value, T default_value);

    // 从ROS参数服务器加载向量类型参数
    template<class T>
    bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);

    // 设置位姿消息：将状态转换为ROS消息格式
    template<typename T>
    void SetPosestamp(T &out);


    // 判断本机是否被指定队友观察到
    bool IsObservedByTeammate(const Teammate &teammate);

    // 判断本机是否观察到指定队友
    bool IsObserveTeammate(const Teammate &teammate);

    // 判断队友消息时间间隔是否足够短：检查是否满足匀速模型假设
    bool IsDurationShort(const double &lidar_end_time, Teammate &teammate, const int &id);

    // 传播队友状态：使用匀速模型将队友状态从接收时刻传播到当前帧结束时刻
    void PropagateTeammateState(const double &lidar_end_time, Teammate &teammate);

    // 计算主动观测的雅可比矩阵：本机观测到队友的情况
    Matrix<double, (3), (12)>
    JacobianActiveObserve(V3D &active_observation_meas, const int &id, const Teammate &teammate);

    // 计算被动观测的雅可比矩阵（情况2）：队友观测到本机，在t_k时刻的计算
    Matrix<double, (3), (12)>
    JacobComputeCase2(V3D &passive_observation_meas, const int &id, const Teammate &teammate);

    // 计算被动观测的雅可比矩阵：队友观测到本机，在t_s时刻的计算
    Matrix<double, (3), (12)>
    JacobianPassiveObserve(V3D &passive_observation_meas, const int &id, const Teammate &teammate,
                           const double &lidar_end_time);

    // 获取主动互观测的测量噪声协方差矩阵
    M3D GetActiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double &mutual_observe_noise);

    // 获取被动互观测的测量噪声协方差矩阵
    M3D GetPassiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double & lidar_end_time, const double &mutual_observe_noise);

    // 在预测区域内进行欧几里得聚类提取：基于跟踪器预测位置提取点云聚类
    void ClusterExtractPredictRegion(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort);

    // 基于高反射强度进行聚类提取：提取激光雷达高反射强度点的聚类
    void ClusterExtractHighIntensity(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort);

    // 检查聚类有效性：将聚类结果与队友跟踪器预测位置匹配
    void CheckClusterValidation(const int &id, Teammate &teammate);

    // 预测队友跟踪器状态：使用EKF进行状态预测
    void PredictTracker(const double &lidar_end_time, const int &id);

    // 更新队友跟踪器：使用聚类观测或队友里程计更新EKF
    void UpdateTracker(const double &lidar_end_time, const int &id, const Teammate &teammate,
                       const bool &cluster_meas, const bool &print_log);

    // 删除无效的临时跟踪器：长时间未更新或与已知队友重复的跟踪器
    bool DeleteInvalidTemporaryTracker(const double &lidar_end_time, const int &index);

    // 轨迹匹配：使用SVD分解计算两条轨迹之间的旋转和平移关系
    bool TrajMatching(vector<Vector3d> &dyn_positions,
                      vector<Vector3d> &uav_positions,
                      Matrix3d &Rot,
                      Vector3d &trans,
                      double &Coeff,
                      const int &id);

    // 创建队友跟踪器：通过轨迹匹配确认临时跟踪器为已知队友
    bool CreateTeammateTracker(const double &lidar_end_time, const int &index, StatesGroup &state_in,
                               StatesGroup &state_prop, const bool &print_log);

    // 基于高反射强度创建临时跟踪器：为新检测到的高反物体创建跟踪器
    void CreateTempTrackerByHighIntensity(const double &lidar_end_time);

    // 检查临时跟踪器的聚类有效性：匹配聚类观测与临时跟踪器预测
    void CheckTempClusterValidation(const int &index);

    // 预测临时跟踪器状态：使用EKF进行状态预测
    void PredictTemporaryTracker(const double &lidar_end_time, const int &index);

    // 更新临时跟踪器：使用聚类观测更新EKF
    void UpdateTemporaryTracker(const double &lidar_end_time, const int &index, const bool &print_log);

    // 可视化文本标签：在RViz中显示文字信息
    void VisualizeText(const ros::Publisher &pub, const double &time, const int &id, const double &scale, const V3D &pos, const string &text, const V3D &color);

    // 可视化边界框：在RViz中显示立方体边界框
    void VisualizeBoundingBox(const ros::Publisher &pub, const double &time, const int &id, const V3D &color, const V3D &pos, const double &size);

    // 删除所有聚类可视化标记
    void VisualizeDeleteAllCluster(const double &time);

    // 发布队友里程计信息：用于可视化和下游模块使用
    void PublishTeammateOdom(const double &lidar_end_time);

    // 发布已连接的队友列表
    void PublishConnectedTeammateList(const double &lidar_end_time);

    // 可视化队友跟踪器：显示队友的位置和状态
    void VisualizeTeammateTracker(const double &lidar_end_time, const int &teammate_id);

    // 可视化失联的队友跟踪器：显示失去连接的队友
    void VisualizeDisconnectedTracker(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate);

    // 可视化无人机网格模型：显示3D无人机模型
    void VisualizeMeshUAV(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate);

    // 可视化临时跟踪器：显示所有临时跟踪器的位置
    void VisualizeTempTracker(const double &lidar_end_time);

    // 可视化临时跟踪器的预测区域：显示搜索范围
    void VisualizeTemporaryPredictRegion(const double &lidar_end_time);

    // 删除临时跟踪器的可视化标记
    void VisualizeTempTrackerDelete(const double &lidar_end_time, const int &index);

    // 可视化聚类结果：显示点云聚类的位置和大小
    void VisualizeCluster(const double &lidar_end_time, const V3D &pos, const int &cluster_index, const V3D &size);

    // 可视化队友预测区域：显示基于EKF预测的搜索范围
    void VisualizePredictRegion(const double &lidar_end_time, const int &id);

    // 可视化矩形框：显示长方体边界框
    void VisualizeRectangle(const ros::Publisher &pub_rect, const double &lidar_end_time, const int &rect_id, const V3D &position, const V3D rect_size);

    // 可视化队友轨迹（箭头形式）：显示队友的运动轨迹
    void VisualizeTeammateTrajectory(const ros::Publisher pub_, deque<Vector4d> &traj,
                                     const V3D position,
                                     const string namespace_,
                                     const double mkr_size,
                                     const double &timestamp,
                                     const int teammate_id,
                                     const int has_observation
    );

    // 可视化队友轨迹（球形式）：以球体显示队友的运动轨迹点
    void VisualizeTeammateTrajectorySphere(const ros::Publisher pub_, deque<Vector4d> &traj,
                                     const V3D position,
                                     const string namespace_,
                                     const double mkr_size,
                                     const double &timestamp,
                                     const int teammate_id,
                                     const int has_observation
    );

    // === 公共成员变量 ===
    id_esikf_map teammate_tracker; // 队友EKF跟踪器映射表（ID -> 跟踪器）
    int pos_num_in_traj; // 轨迹中保存的位置点数量
    vector<TemporaryTracker> temp_tracker; // 临时跟踪器向量
    typedef unordered_map<int, Teammate> id_teammate_map;
    typedef id_teammate_map::value_type id2teammate;
    id_teammate_map teammates; // 队友信息映射表（ID -> 队友信息）
    string topic_name_prefix; // ROS话题名称前缀（用于仿真时区分不同无人机）
    M3D rot_world_to_gravity; // 世界坐标系到重力对齐坐标系的旋转矩阵
    StatesGroup state; // 本机的状态信息
    ExtrinsicInfection extrinsic_infection; // 外参传染模型管理器
    mutex mtx_buffer_reconnectID; // 重连ID缓冲区的互斥锁
    bool degenerated; // 本机是否处于退化状态
    deque<Vector4d> teammate_traj[MAX_UAV_NUM]; // 队友轨迹历史数组

private:
    // === ROS接口相关 ===
    ros::NodeHandle nh_; // ROS节点句柄
    ros::Subscriber QuadState_subscriber, GlobalExtrinsic_subscriber; // 队友状态和全局外参订阅器
    ros::Subscriber QuadState_subscriber_sim, GlobalExtrinsic_subscriber_sim; // 仿真专用订阅器
    ros::Publisher QuadState_publisher, GlobalExtrinsic_publisher; // 本机状态和全局外参发布器
    ros::Publisher pubUAV, pubMeshUAV, pubCluster; // 可视化发布器（无人机、网格模型、聚类）
    ros::Publisher pubPredictRegionInput, pubHighIntenInput; // 点云输入可视化发布器
    ros::Publisher pubPredictRegion, pubTempTracker, pubTeammateList, pubTeammateNum; // 预测区域、临时跟踪器、队友列表发布器
    ros::Publisher* pubTeammateOdom = new ros::Publisher[MAX_UAV_NUM]; // 队友里程计发布器数组
    ros::Publisher* pubTeammateTraj = new ros::Publisher[MAX_UAV_NUM]; // 队友轨迹发布器数组
    swarm_msgs::QuadStatePub quadstate_msg_pub; // 本机状态消息
    swarm_msgs::GlobalExtrinsicStatus global_extrinsic_msg; // 全局外参状态消息

    // === 参数配置 ===
    int drone_id; // 本机无人机ID
    int inten_threshold; // 高反射强度阈值
    int min_high_inten_cluster_size; // 高反射聚类的最小点数
    int min_cluster_size; // 普通聚类的最小点数
    int cluster_id{0}; // 聚类ID计数器
    int lidar_type; // 激光雷达类型
    int actual_uav_num{4}; // 实际无人机数量

    // === 聚类和跟踪相关 ===
    vector<Cluster> cluster_pos_tag; // 当前帧的聚类结果
    vector<int> reconnected_id; // 重连的队友ID列表
    vector<int> teammate_id_by_traj_matching; // 通过轨迹匹配识别的队友ID列表
    ros::Publisher pubTeammateIdTrajMatching; // 轨迹匹配结果发布器

    // === 阈值参数 ===
    double predict_region_radius; // 队友预测区域半径
    double valid_cluster_dist_thresh; // 有效聚类距离阈值
    double valid_cluster_size_thresh; // 有效聚类尺寸阈值
    double reset_tracker_thresh; // 重置跟踪器的时间阈值
    double temp_predict_region_radius; // 临时跟踪器预测区域半径
    double valid_temp_cluster_dist_thresh; // 临时跟踪器有效聚类距离阈值
    double same_obj_thresh; // 判定为同一物体的距离阈值
    double traj_matching_start_thresh; // 开始轨迹匹配的激励程度阈值
    double ave_match_error_thresh; // 轨迹匹配平均误差阈值

    // === 其他 ===
    nav_msgs::Odometry TeammateOdom; // 队友里程计消息缓存
    bool found_all_teammates{false}; // 是否已找到所有队友
    bool cluster_extraction_in_predict_region; // 是否在预测区域内进行聚类提取
    double text_scale, mesh_scale; // 可视化文本和网格模型的缩放比例
};
#endif //MULTIUAV_H