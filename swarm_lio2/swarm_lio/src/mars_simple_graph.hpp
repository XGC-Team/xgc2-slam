//
// Created by yunfan on 16/8/2023.
//

#ifndef MARS_SIMPLE_GRAPH_MARS_SIMPLE_GRAPH_HPP
#define MARS_SIMPLE_GRAPH_MARS_SIMPLE_GRAPH_HPP
#include "vector"
#include "iostream"
#include "cstdlib"
#include "cmath"
#include "queue"
#include "Eigen/Eigen"
namespace mars{
    using namespace std;
    using namespace Eigen;

    /**
     * @brief 基础边数据结构
     * 定义了图中边的基本属性，包括起点、终点和有效性标志
     */
    struct BaseEdgeData{
        /**
         * @brief 构造函数，初始化边数据
         * 将起点、终点设为-1，表示未连接状态，有效性标志设为false
         */
        BaseEdgeData(){
            from = -1;  // 起点节点ID，初始化为-1表示未设置
            to = -1;    // 终点节点ID，初始化为-1表示未设置
            is_valid = false;  // 边的有效性标志，初始化为无效
        };
        int from,to;        // 边的起点和终点节点ID
        bool is_valid;      // 边是否有效的标志位
    };

    /**
     * @brief 边数据类，继承自BaseEdgeData
     * 扩展了基础边数据，添加了3D空间中的旋转和平移变换信息
     * 用于表示两个节点之间的相对位姿关系
     */
    class EdgeData:public mars::BaseEdgeData{
    public:
        Matrix3d rotation;      // 3x3旋转矩阵，表示从起点到终点的旋转变换
        Vector3d translation;   // 3D平移向量，表示从起点到终点的平移变换

        /**
         * @brief 计算边的逆变换
         * @return EdgeData 返回逆向边，即从终点到起点的变换
         *
         * 逆变换计算：
         * - 旋转矩阵的逆 = 旋转矩阵的转置 (正交矩阵性质)
         * - 平移向量的逆 = -R^T * t
         * - 交换起点和终点
         */
        EdgeData inverse() const {
            EdgeData out;
            out.rotation = this->rotation.transpose();  // 旋转矩阵求逆（转置）
            out.translation = - out.rotation * this->translation;  // 平移向量求逆
            out.from = this->to;  // 交换起点和终点
            out.to = this->from;
            return out;
        }

        /**
         * @brief 重载输出流操作符，用于打印边信息
         * @param os 输出流对象
         * @param cc 边数据对象
         * @return ostream& 输出流引用
         */
        friend ostream& operator << (ostream& os, EdgeData& cc)
        {
            os<<cc.from<<" --> "<<cc.to;  // 输出格式：起点 --> 终点
            return os;
        }
    };

    /**
     * @brief Mars简单图类模板
     * @tparam T 边数据类型，必须继承自BaseEdgeData
     *
     * 这是一个基于邻接矩阵实现的简单图结构，用于表示节点之间的连接关系
     * 使用一维数组存储二维邻接矩阵，支持添加/删除边、节点，以及查找连通子图
     */
    template<typename T> // T should be inherited from BaseEdgeData
    class MarsSimpleGraph {
    private:
        vector<T> nodes;  // 存储所有边的一维数组，实际上是邻接矩阵的线性化表示

        int max_node_size_{0};  // 图中允许的最大节点数量

        /**
         * @brief 将二维索引转换为一维地址
         * @param idx 行索引（起点节点ID）
         * @param idy 列索引（终点节点ID）
         * @return int 一维数组中的地址
         *
         * 将邻接矩阵的二维坐标(idx, idy)映射到一维数组索引
         */
        int toAddress(const int & idx, const int idy){
            return idx*max_node_size_ + idy;
        }

        /**
         * @brief 将一维地址转换为二维索引
         * @param idx 一维数组索引
         * @return pair<int,int> 二维坐标(行索引, 列索引)
         *
         * 将一维数组索引反向映射为邻接矩阵的二维坐标
         */
        pair<int,int> toIndex(const int & idx){
            return make_pair(idx/max_node_size_, idx%max_node_size_);
        }

        vector<bool> use_as_root_;  // 标记节点是否已被用作根节点（用于图遍历）


    public:
        /**
         * @brief 默认构造函数
         */
        MarsSimpleGraph() = default;

        /**
         * @brief 构造函数，初始化指定大小的图
         * @param max_node_num 图中允许的最大节点数量
         *
         * 初始化邻接矩阵和辅助数据结构：
         * - 邻接矩阵大小为 max_node_num × max_node_num
         * - 使用一维数组存储，总大小为 max_node_num²
         */
        MarsSimpleGraph(int max_node_num){
            max_node_size_ = max_node_num;
            nodes.resize(max_node_num*max_node_num);  // 分配邻接矩阵空间
            use_as_root_.resize(max_node_num);        // 分配根节点标记数组空间
        }

        /**
         * @brief 向图中添加一条边及其逆向边
         * @param edge 要添加的边数据
         * @return bool 添加成功返回true，失败返回false
         *
         * 这个函数会同时添加两条边：
         * 1. 从起点到终点的正向边
         * 2. 从终点到起点的逆向边（使用inverse()方法计算）
         * 这保证了图的对称性，使其成为无向图
         */
        bool AddEdge(const T&edge){
            // 添加正向边
            int address = toAddress(edge.from, edge.to);
            if(address >= nodes.size()){  // 检查地址是否越界
                return false;
            }
            nodes[address] = edge;
            nodes[address].is_valid = true;  // 标记为有效边

            // 添加逆向边
            T edge_reverse = edge.inverse();  // 计算逆变换
            address = toAddress(edge_reverse.from, edge_reverse.to);
            if(address >= nodes.size()){  // 检查地址是否越界
                return false;
            }
            nodes[address] = edge_reverse;
            nodes[address].is_valid = true;  // 标记为有效边
            return true;
        }

        /**
         * @brief 从图中移除一条边
         * @param edge 要移除的边数据
         * @return bool 移除成功返回true，失败返回false
         *
         * 将指定的边重置为默认值（无效边）
         */
        bool RemoveEdge(const T&edge){
            int address = toAddress(edge.from, edge.to);
            if(address >= nodes.size()){  // 检查地址是否越界
                return false;
            }
            nodes[address] = T();  // 重置为默认构造的边（无效边）
            return true;
        }

        /**
         * @brief 从图中移除一个节点及其所有相关的边
         * @param node_id 要移除的节点ID
         * @return bool 移除成功返回true，失败返回false
         *
         * 删除与该节点相关的所有边（包括入边和出边）：
         * - 从该节点出发的所有边
         * - 指向该节点的所有边
         */
        bool RemoveNode(const int&node_id){
            if(node_id > max_node_size_){  // 检查节点ID是否有效
                return false;
            }
            // 遍历所有可能的连接
            for(int i = 0 ; i < max_node_size_ ;i++){
                int address = toAddress(node_id, i);  // 移除从node_id指向i的边
                nodes[address] = T();
                address = toAddress(i, node_id);      // 移除从i指向node_id的边
                nodes[address] = T();
            }
            return true;
        }

        /**
         * @brief 获取与指定节点连通的所有边和节点
         * @param node_id 起始节点ID
         * @param edges 输出参数，存储连通子图中的所有边
         * @param connect_nodes 输出参数，存储连通子图中的所有节点ID
         * @return bool 成功返回true，失败返回false
         *
         * 使用BFS（广度优先搜索）算法遍历图，找出从node_id可达的所有节点和边
         * 这个函数用于提取图中的连通分量
         *
         * 算法流程：
         * 1. 从指定节点开始BFS遍历
         * 2. 标记已访问的节点（use_as_root_）
         * 3. 收集所有有效的边（避免重复：只保留i >= idx的边）
         * 4. 记录所有连通的节点
         */
        bool GetConnectedEdgesAndNodes(const int & node_id, vector<T>& edges, vector<int> & connect_nodes){
            if(node_id > max_node_size_){  // 检查节点ID是否有效
                return false;
            }
            // 静态变量用于记录节点连通性，避免重复分配内存
            static vector<bool> is_connected(max_node_size_, false);
            std::fill(is_connected.begin(), is_connected.end(), false);  // 重置连通性标记
            std::fill(use_as_root_.begin(), use_as_root_.end(), false);  // 重置根节点标记
            edges.clear();          // 清空输出边列表
            connect_nodes.clear();  // 清空输出节点列表

            // 基于BFS的边搜索算法
            queue<int> search_queue;
            search_queue.push(node_id);  // 将起始节点加入队列

            while(!search_queue.empty()){
                int idx = search_queue.front();
                search_queue.pop();

                if(use_as_root_[idx]){  // 如果节点已被访问过，跳过
                    continue;
                }
                // 标记该节点已被访问
                use_as_root_[idx] = true;

                // 查找所有连接到idx的边
                for(int i = 0 ; i < max_node_size_ ;i++){
                    int address = toAddress(idx, i);

                    if(address >= nodes.size()){  // 检查地址是否越界
                        return false;
                    }

                    if(!(nodes[address].is_valid)){  // 如果边无效，跳过
                        continue;
                    }

                    // 有效边：将目标节点加入搜索队列
                    search_queue.push(i);
                    is_connected[i] = true;  // 标记节点i是连通的

                    // 避免重复添加边：只添加i >= idx的边
                    // 因为无向图中每条边会被存储两次（正向和反向）
                    if(i < idx)
                        continue;
                    edges.push_back(nodes[address]);  // 添加边到结果列表
                }
            }

            if(edges.empty()){  // 如果没有找到任何边，返回失败
                return false;
            }

            // 收集所有连通的节点ID
            for(int i = 0 ; i < is_connected.size();i++){
                if(is_connected[i] ){
                    connect_nodes.push_back(i);
                }
            }

            return true;
        }

        /**
         * @brief 析构函数
         */
        ~MarsSimpleGraph()=default;

        /**
         * @brief 打印图的邻接矩阵
         *
         * 以矩阵形式打印图的结构：
         * - 1 表示该位置有有效边
         * - 0 表示该位置无边
         * 用于调试和可视化图的连接关系
         */
        void PrintGraph(){
            for(int i = 0 ; i < max_node_size_; i++){
                for(int j = 0 ; j < max_node_size_; j++){
                    int address = toAddress(i, j);
                    if(address >= nodes.size()){  // 检查地址是否越界
                        return;
                    }
                    if(nodes[address].is_valid){  // 有效边显示为1
                        cout<<"1 ";
                    }else{                         // 无效边显示为0
                        cout<<"0 ";
                    }
                }
                cout<<endl;  // 每行结束换行
            }
        }
    private:

    };
}


#endif //MARS_SIMPLE_GRAPH_MARS_SIMPLE_GRAPH_HPP
