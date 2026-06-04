/**
 * @file ExtrinsicInfection.hpp
 * @brief 外参传播类头文件，用于多机协同SLAM中的外参估计和优化
 * @author fangcheng
 * @date 2022/11/06
 * @details 通过因子图优化估计多UAV系统中的全局外参
 */
#ifndef ExtrinsicInfection_HPP
#define ExtrinsicInfection_HPP

#include <omp.h>
#include <unistd.h>
#include <Eigen/Core>
#include <common_lib.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <unordered_map>
#include "mars_simple_graph.hpp"
#include <vector>
using namespace std;
using namespace Eigen;

/**
 * @class ExtrinsicInfection
 * @brief 外参传播类，用于通过图优化估计多UAV系统的全局外参
 * @details 使用GTSAM库进行因子图优化，通过iSAM2增量式求解器优化外参
 */
class ExtrinsicInfection{
public:
    /**
     * @brief 比较两个向量是否相等
     * @tparam T 向量元素类型
     * @param v1 第一个向量
     * @param v2 第二个向量
     * @return 如果两个向量大小相同且元素全部相等则返回true
     */
    template<typename T>
    bool isEqual(std::vector<T> const &v1, std::vector<T> const &v2)
    {
        return (v1.size() == v2.size() &&
                std::equal(v1.begin(), v1.end(), v2.begin()));
    }

    /**
     * @brief 构造函数
     * @details 初始化简单图数据结构，用于存储UAV之间的外参约束
     */
    ExtrinsicInfection(){
        //Insert initials (initial values of global extrinsic)
        simple_graph = mars::MarsSimpleGraph<mars::EdgeData>(MAX_UAV_NUM);
    }

    /**
     * @brief 析构函数
     */
    ~ExtrinsicInfection() = default;



    /**
     * @brief 向因子图中插入先验因子
     * @param key UAV的索引
     * @param rot 旋转矩阵（此处固定为单位矩阵）
     * @param trans 平移向量（此处固定为零向量）
     * @param noise 噪声标准差
     * @param graph 因子图
     * @details 先验因子后面不会再被优化，用于固定本机的外参为单位变换
     */
    void InsertPriorFactor(const int &key, const M3D &rot, const V3D &trans, const double &noise, gtsam::NonlinearFactorGraph &graph){
        gtsam::Vector Vector6(6);
        Vector6 << noise, noise, noise, noise, noise, noise;  // 6维协方差（3个旋转+3个平移）
        gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Vector6);
        // 添加先验因子，将本机外参固定为单位变换
        graph.add(gtsam::PriorFactor<gtsam::Pose3>
                                (key, gtsam::Pose3(gtsam::Rot3(Eye3d),gtsam::Point3(Zero3d)), priorModel));
    }

    /**
     * @brief 向因子图中插入边（相对外参约束）
     * @param index_from 起始UAV索引
     * @param index_to 目标UAV索引
     * @param rot 相对旋转矩阵
     * @param trans 相对平移向量
     * @param noise 噪声标准差
     * @param graph 因子图
     * @details 添加两个UAV之间的相对外参约束
     */
    void InsertEdge(const int &index_from, const int &index_to, const M3D &rot, const V3D &trans, const double &noise, gtsam::NonlinearFactorGraph &graph){
        //edge 1_T_2 的噪声矩阵
        gtsam::Vector Vector6(6);
        Vector6 << noise, noise, noise, noise, noise, noise;
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        //添加edge 1_T_2
        gtsam::Rot3 R_sam(rot);
        gtsam::Point3 t_sam(trans);
        gtsam::NonlinearFactor::shared_ptr factor(
                new gtsam::BetweenFactor<gtsam::Pose3>(index_from, index_to, gtsam::Pose3(R_sam, t_sam),
                                                       odometryNoise)); //添加边约束
        graph.push_back(factor);
    }

    /**
     * @brief 使用iSAM2增量式优化求解外参因子图
     * @param state 状态组，包含所有UAV的外参
     * @details 主要步骤：
     *          1. 从简单图中获取与当前UAV连接的边和节点
     *          2. 检查连接节点是否变化，若无变化则跳过优化
     *          3. 构建因子图并使用iSAM2求解
     *          4. 更新状态中的全局外参
     */
    void SolveGraphIsam2(StatesGroup &state){
        vector<mars::EdgeData> edges;
        edges.clear();
        vector<int> connected_nodes;
        connected_nodes.clear();
        // 获取与当前UAV连接的所有边和节点
        if(!simple_graph.GetConnectedEdgesAndNodes(drone_id, edges, connected_nodes))
            return;

        // 检查连接节点是否发生变化
        static vector<int> last_connected_nodes;
        bool same_nodes = isEqual(last_connected_nodes, connected_nodes);
        last_connected_nodes.assign(connected_nodes.begin(), connected_nodes.end());
        if(same_nodes){
//            cout << "Same Connected Nodes, Skip Optimization" << endl;
            return;  // 如果连接节点未变化，跳过优化
        }


        //顶点，即优化变量
        gtsam::Values initial;
        cout << "Connected Nodes: " << endl;
        // 初始化所有连接节点的外参值
        for (int i = 0; i < connected_nodes.size(); ++i) {
            int id = connected_nodes[i];
            cout << connected_nodes[i] << ", ";
            // 如果外参尚未初始化，使用单位变换作为初始值
            if(state.global_extrinsic_trans[id].norm() < 0.001)
                initial.insert(id, gtsam::Pose3(gtsam::Rot3(Eye3d), gtsam::Point3(Zero3d)));
            else
                initial.insert(id, gtsam::Pose3(gtsam::Rot3(state.global_extrinsic_rot[id]), gtsam::Point3(state.global_extrinsic_trans[id])));
        }
        cout << endl;



        //定义因子图
        gtsam::NonlinearFactorGraph GTSAM_graph;
        //计算结果
        gtsam::Values result;

        //添加先验因子，本机和本机的外参，噪声很小（固定本机外参）
        InsertPriorFactor(drone_id, Eye3d, Zero3d, 1e-6, GTSAM_graph);

        //添加边，本机和其他机器人的外参，噪声略大
        for (int i = 0; i < edges.size(); ++i) {
            InsertEdge(edges[i].from, edges[i].to, edges[i].rotation, edges[i].translation, 1e-4, GTSAM_graph);
        }

        //打印边信息
        cout << "Edges: " << endl;
        for(auto e : edges){
            cout<<e << ",   ";
        }
        cout << endl;

        // 配置iSAM2参数
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;  // 重新线性化阈值
        parameters.relinearizeSkip = 1;          // 重新线性化间隔
        gtsam::ISAM2 isam(parameters);


//        try{
            // 增量式更新因子图
            isam.update(GTSAM_graph, initial);
            isam.update();
            result = isam.calculateEstimate();  // 计算优化结果


            // 更新状态中的全局外参（仅更新未初始化的外参）
            for (int i = 0; i < connected_nodes.size(); ++i) {
                int id = connected_nodes[i];
                if (state.global_extrinsic_trans[id].norm() < 0.001) {
                    state.global_extrinsic_rot[id] = GetGlobalExtrinsic(id, result).block<3, 3>(0, 0);
                    state.global_extrinsic_trans[id] = GetGlobalExtrinsic(id, result).block<3, 1>(0, 3);
                }
            }
//        }
//        catch(gtsam::IndeterminantLinearSystemException &e){
//            cout << "Graph is not connected!" << endl;
//        }


    }



    /**
     * @brief 打印因子图到文件
     * @param index 图索引
     * @param graph 因子图
     * @details 将因子图保存为.dot格式文件，便于可视化调试
     */
    void PrintGraph(const int &index, gtsam::NonlinearFactorGraph &graph){
        string root_dir = ROOT_DIR;
        graph.saveGraph(string(root_dir + "/Log/Graph" + SetString(drone_id)) + "_" + SetString(index) + string(".dot"));
    }

//
//    void SolveGraphLM(){
//        gtsam::LevenbergMarquardtParams parameters;
//        parameters.relativeErrorTol = 1e-5;
//        parameters.maxIterations = 5;
//        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
//        result = optimizer.optimize();
//    }
//
//    void SolveGraphGaussNewton(){
//        gtsam::GaussNewtonParams parameters;
//        parameters.relativeErrorTol = 1e-5;
//        parameters.maxIterations = 5;
//        gtsam::GaussNewtonOptimizer optimizer(graph, initial, parameters);
//        result = optimizer.optimize();
//    }

    /**
     * @brief 从优化结果中提取全局外参
     * @param key UAV索引
     * @param result GTSAM优化结果
     * @return 4x4齐次变换矩阵
     */
    Matrix4d GetGlobalExtrinsic(const int &key, const gtsam::Values &result){
        Matrix4d extrinsic;
        extrinsic.setIdentity();
        gtsam::Pose3 pose = result.at(key).cast<gtsam::Pose3>();
        extrinsic.block<3,3>(0,0) = pose.rotation().matrix();  // 旋转部分
        extrinsic.block<3,1>(0,3) = pose.translation();        // 平移部分
        return extrinsic;
    }

    /**
     * @brief 打印优化结果
     * @param result GTSAM优化结果
     * @details 以欧拉角和平移向量的形式打印所有UAV的外参
     */
    void PrintResult(const gtsam::Values &result){
        for (uint i = 0; i < result.size(); i++) {
            gtsam::Pose3 pose = result.at(i).cast<gtsam::Pose3>();
            cout << drone_id << "^T_" << i << " = " << RotMtoEuler(pose.rotation().matrix()).transpose() * 57.3 << " "
                 << pose.translation().transpose() << endl;
        }
    }

    /**
     * @brief 重置因子图
     * @param graph 因子图
     * @details 清空因子图中的所有因子
     */
    void ResetGraph(gtsam::NonlinearFactorGraph &graph){
        graph.resize(0);
    }

    /**
     * @brief 获取因子图大小
     * @param graph 因子图
     * @return 因子图中因子的数量
     */
    int GetGraphSize(const gtsam::NonlinearFactorGraph &graph){
        return graph.size();
    }

public:
    int drone_id;                                      // 当前UAV的ID
    mars::MarsSimpleGraph<mars::EdgeData> simple_graph; // 简单图结构，存储UAV间的外参约束
};


#endif