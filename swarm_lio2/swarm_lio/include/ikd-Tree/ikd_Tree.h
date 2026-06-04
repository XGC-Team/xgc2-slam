#pragma once
#include <stdio.h>
#include <queue>
#include <pthread.h>
#include <chrono>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <algorithm>
#include <memory.h>
#include <pcl/point_types.h>

// 全局常量定义
#define EPSS 1e-6                               // 浮点数比较精度阈值
#define Minimal_Unbalanced_Tree_Size 10         // 触发树平衡检查的最小树大小
#define Multi_Thread_Rebuild_Point_Num 1500     // 启用多线程重建的最小点数阈值
#define DOWNSAMPLE_SWITCH true                  // 降采样功能开关
#define ForceRebuildPercentage 0.2              // 强制重建的百分比阈值
#define Q_LEN 1000000                           // 操作日志队列的最大长度

using namespace std;

// typedef pcl::PointXYZINormal PointType;
// typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

// 盒子点类型：定义一个三维轴对齐包围盒（AABB）
struct BoxPointType
{
    float vertex_min[3];  // 包围盒最小顶点坐标 [x_min, y_min, z_min]
    float vertex_max[3];  // 包围盒最大顶点坐标 [x_max, y_max, z_max]
};

// 操作类型枚举：定义对树的各种操作
enum operation_set
{
    ADD_POINT,          // 添加单个点
    DELETE_POINT,       // 删除单个点
    DELETE_BOX,         // 删除包围盒内的所有点
    ADD_BOX,            // 添加包围盒
    DOWNSAMPLE_DELETE,  // 降采样删除
    PUSH_DOWN           // 向下推送删除标记（懒惰删除）
};

// 删除点存储方式枚举：控制删除点的记录方式
enum delete_point_storage_set
{
    NOT_RECORD,           // 不记录删除的点
    DELETE_POINTS_REC,    // 记录删除的点到主删除列表
    MULTI_THREAD_REC      // 记录删除的点到多线程删除列表
};

/**
 * @brief 增量式KD树类：支持动态插入、删除和搜索的KD树实现
 * @tparam PointType 点云类型（如 pcl::PointXYZ, pcl::PointXYZI 等）
 */
template <typename PointType>
class KD_TREE
{
    // using MANUAL_Q_ = MANUAL_Q<typename PointType>;
    // using PointVector = std::vector<PointType>;

    // using MANUAL_Q_ = MANUAL_Q<typename PointType>;
public:
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;  // 点云向量类型（使用Eigen内存对齐）
    using Ptr = std::shared_ptr<KD_TREE<PointType>>;  // 智能指针类型

    /**
     * @brief KD树节点结构：存储节点的所有信息
     */
    struct KD_TREE_NODE
    {
        PointType point;                        // 节点存储的点
        int division_axis;                      // 分割轴（0=x, 1=y, 2=z）
        int TreeSize = 1;                       // 以该节点为根的子树大小（节点总数）
        int invalid_point_num = 0;              // 无效点数量（已删除的点）
        int down_del_num = 0;                   // 降采样删除的点数量
        bool point_deleted = false;             // 该节点的点是否已删除
        bool tree_deleted = false;              // 整棵子树是否已删除
        bool point_downsample_deleted = false;  // 该点是否被降采样删除
        bool tree_downsample_deleted = false;   // 整棵子树是否被降采样删除
        bool need_push_down_to_left = false;    // 是否需要向左子节点下推删除标记
        bool need_push_down_to_right = false;   // 是否需要向右子节点下推删除标记
        bool working_flag = false;              // 工作标志：指示节点是否正在被操作
        pthread_mutex_t push_down_mutex_lock;   // 下推操作的互斥锁
        float node_range_x[2], node_range_y[2], node_range_z[2];  // 节点包围盒范围 [min, max]
        KD_TREE_NODE *left_son_ptr = nullptr;   // 左子节点指针
        KD_TREE_NODE *right_son_ptr = nullptr;  // 右子节点指针
        KD_TREE_NODE *father_ptr = nullptr;     // 父节点指针
        // For paper data record（用于论文数据记录）
        float alpha_del;  // 删除率（无效点数/总节点数）
        float alpha_bal;  // 平衡因子（较大子树大小/总节点数-1）
    };

    /**
     * @brief 操作日志类型：记录树操作以支持多线程重建
     */
    struct Operation_Logger_Type
    {
        PointType point;                        // 操作涉及的点
        BoxPointType boxpoint;                  // 操作涉及的包围盒
        bool tree_deleted, tree_downsample_deleted;  // 树和降采样删除标志
        operation_set op;                       // 操作类型
    };
    // static const PointType zeroP;

    /**
     * @brief 点类型比较结构：用于K近邻搜索的优先队列
     */
    struct PointType_CMP
    {
        PointType point;  // 点
        float dist = 0.0;  // 到查询点的距离

        // 构造函数
        PointType_CMP(PointType p = PointType(), float d = INFINITY)
        {
            this->point = p;
            this->dist = d;
        };

        // 比较操作符：距离小的优先级高，距离相同时x坐标小的优先级高
        bool operator<(const PointType_CMP &a) const
        {
            if (fabs(dist - a.dist) < 1e-10)
                return point.x < a.point.x;
            else
                return dist < a.dist;
        }
    };

    /**
     * @brief 手动实现的最大堆：用于K近邻搜索
     * 使用最大堆存储候选点，堆顶是距离最大的点
     */
    class MANUAL_HEAP
    {

    public:
        // 构造函数：创建指定容量的堆
        MANUAL_HEAP(int max_capacity = 100)

        {
            cap = max_capacity;
            heap = new PointType_CMP[max_capacity];
            heap_size = 0;
        }

        // 析构函数：释放堆内存
        ~MANUAL_HEAP()
        {
            delete[] heap;
        }

        // 弹出堆顶元素（距离最大的点）
        void pop()
        {
            if (heap_size == 0)
                return;
            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);  // 向下调整堆
            return;
        }

        // 获取堆顶元素（不删除）
        PointType_CMP top()
        {
            return heap[0];
        }

        // 向堆中插入新元素
        void push(PointType_CMP point)
        {
            if (heap_size >= cap)
                return;
            heap[heap_size] = point;
            FloatUp(heap_size);  // 向上调整堆
            heap_size++;
            return;
        }

        // 获取堆的大小
        int size()
        {
            return heap_size;
        }

        // 清空堆
        void clear()
        {
            heap_size = 0;
            return;
        }

    private:
        PointType_CMP *heap;  // 堆数组

        // 向下调整堆：维护最大堆性质
        void MoveDown(int heap_index)
        {
            int l = heap_index * 2 + 1;  // 左子节点索引
            PointType_CMP tmp = heap[heap_index];
            while (l < heap_size)
            {
                // 选择左右子节点中较大的
                if (l + 1 < heap_size && heap[l] < heap[l + 1])
                    l++;
                // 如果子节点比父节点大，交换
                if (tmp < heap[l])
                {
                    heap[heap_index] = heap[l];
                    heap_index = l;
                    l = heap_index * 2 + 1;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }

        // 向上调整堆：维护最大堆性质
        void FloatUp(int heap_index)
        {
            int ancestor = (heap_index - 1) / 2;  // 父节点索引
            PointType_CMP tmp = heap[heap_index];
            while (heap_index > 0)
            {
                // 如果子节点比父节点大，交换
                if (heap[ancestor] < tmp)
                {
                    heap[heap_index] = heap[ancestor];
                    heap_index = ancestor;
                    ancestor = (heap_index - 1) / 2;
                }
                else
                    break;
            }
            heap[heap_index] = tmp;
            return;
        }
        int heap_size = 0;  // 当前堆大小
        int cap = 0;        // 堆容量
    };

    /**
     * @brief 手动实现的循环队列：用于存储操作日志
     * 支持多线程重建时记录操作历史
     */
    class MANUAL_Q
    {
    private:
        int head = 0, tail = 0, counter = 0;  // 队列头、尾索引和元素计数
        Operation_Logger_Type q[Q_LEN];       // 固定大小的循环队列数组
        bool is_empty;                         // 队列是否为空

    public:
        // 弹出队首元素
        void pop()
        {
            if (counter == 0)
                return;
            head++;
            head %= Q_LEN;  // 循环队列：头指针回绕
            counter--;
            if (counter == 0)
                is_empty = true;
            return;
        }

        // 获取队首元素
        Operation_Logger_Type front()
        {
            return q[head];
        }

        // 获取队尾元素
        Operation_Logger_Type back()
        {
            return q[tail];
        }

        // 清空队列
        void clear()
        {
            head = 0;
            tail = 0;
            counter = 0;
            is_empty = true;
            return;
        }

        // 向队尾添加元素
        void push(Operation_Logger_Type op)
        {
            q[tail] = op;
            counter++;
            if (is_empty)
                is_empty = false;
            tail++;
            tail %= Q_LEN;  // 循环队列：尾指针回绕
        }

        // 判断队列是否为空
        bool empty()
        {
            return is_empty;
        }

        // 获取队列大小
        int size()
        {
            return counter;
        }
    };

private:
    // ========== 多线程树重建相关变量 ==========
    bool termination_flag = false;   // 线程终止标志
    bool rebuild_flag = false;       // 重建标志
    pthread_t rebuild_thread;        // 重建线程句柄
    // 各种互斥锁
    pthread_mutex_t termination_flag_mutex_lock;        // 终止标志锁
    pthread_mutex_t rebuild_ptr_mutex_lock;             // 重建指针锁
    pthread_mutex_t working_flag_mutex;                 // 工作标志锁
    pthread_mutex_t search_flag_mutex;                  // 搜索标志锁
    pthread_mutex_t rebuild_logger_mutex_lock;          // 重建日志锁
    pthread_mutex_t points_deleted_rebuild_mutex_lock;  // 删除点重建锁
    // queue<Operation_Logger_Type> Rebuild_Logger;
    MANUAL_Q Rebuild_Logger;              // 操作日志队列：记录重建期间的操作
    PointVector Rebuild_PCL_Storage;      // 重建时的点云存储
    KD_TREE_NODE **Rebuild_Ptr = nullptr; // 指向正在重建的子树根节点
    int search_mutex_counter = 0;         // 搜索互斥计数器

    // 多线程相关函数
    static void *multi_thread_ptr(void *arg);  // 线程函数指针
    void multi_thread_rebuild();               // 多线程重建主函数
    void start_thread();                       // 启动重建线程
    void stop_thread();                        // 停止重建线程
    void run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation);  // 执行记录的操作

    // ========== KD树功能函数和增强变量 ==========
    int Treesize_tmp = 0, Validnum_tmp = 0;  // 临时存储树大小和有效点数（重建期间使用）
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;  // 临时平衡因子和删除率
    float delete_criterion_param = 0.5f;     // 删除判据参数：触发重建的删除率阈值
    float balance_criterion_param = 0.7f;    // 平衡判据参数：触发重建的不平衡度阈值
    float downsample_size = 0.2f;            // 降采样体素大小
    bool Delete_Storage_Disabled = false;    // 是否禁用删除点存储
    KD_TREE_NODE *STATIC_ROOT_NODE = nullptr;  // 静态根节点（虚拟节点）
    PointVector Points_deleted;              // 已删除的点集合
    PointVector Downsample_Storage;          // 降采样临时存储
    PointVector Multithread_Points_deleted;  // 多线程删除的点集合

    // 内部核心函数
    void InitTreeNode(KD_TREE_NODE *root);  // 初始化树节点
    void Test_Lock_States(KD_TREE_NODE *root);  // 测试锁状态（调试用）
    void BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage);  // 构建KD树
    void Rebuild(KD_TREE_NODE **root);  // 重建子树
    int Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample);  // 按范围删除
    void Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild);  // 按点删除
    void Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis);  // 添加点
    void Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild);  // 按范围添加
    void Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist);  // K近邻搜索
    void Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage);  // 范围搜索
    bool Criterion_Check(KD_TREE_NODE *root);  // 检查是否需要重建
    void Push_Down(KD_TREE_NODE *root);        // 向下推送删除标记（懒惰删除）
    void Update(KD_TREE_NODE *root);           // 更新节点信息
    void delete_tree_nodes(KD_TREE_NODE **root);  // 删除树节点
    void downsample(KD_TREE_NODE **root);         // 降采样（未使用）

    // 工具函数
    bool same_point(PointType a, PointType b);  // 判断两点是否相同
    float calc_dist(PointType a, PointType b);  // 计算两点间距离的平方
    float calc_box_dist(KD_TREE_NODE *node, PointType point);  // 计算点到包围盒的最小距离平方
    static bool point_cmp_x(PointType a, PointType b);  // 按x坐标比较
    static bool point_cmp_y(PointType a, PointType b);  // 按y坐标比较
    static bool point_cmp_z(PointType a, PointType b);  // 按z坐标比较

public:
    // ========== 构造和析构函数 ==========
    /**
     * @brief 构造函数：初始化KD树
     * @param delete_param 删除判据参数（默认0.5）
     * @param balance_param 平衡判据参数（默认0.6）
     * @param box_length 降采样盒子长度（默认0.2）
     */
    KD_TREE(float delete_param = 0.5, float balance_param = 0.6, float box_length = 0.2);

    /**
     * @brief 析构函数：清理资源
     */
    ~KD_TREE();

    // ========== 参数设置函数 ==========
    /**
     * @brief 设置删除判据参数
     * @param delete_param 当删除率超过此值时触发重建
     */
    void Set_delete_criterion_param(float delete_param)
    {
        delete_criterion_param = delete_param;
    }

    /**
     * @brief 设置平衡判据参数
     * @param balance_param 当不平衡度超过此值时触发重建
     */
    void Set_balance_criterion_param(float balance_param)
    {
        balance_criterion_param = balance_param;
    }

    /**
     * @brief 设置降采样参数
     * @param downsample_param 降采样体素大小
     */
    void set_downsample_param(float downsample_param)
    {
        downsample_size = downsample_param;
    }

    /**
     * @brief 初始化KD树参数
     * @param delete_param 删除判据参数
     * @param balance_param 平衡判据参数
     * @param box_length 降采样盒子长度
     */
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7, float box_length = 0.2);

    // ========== 查询函数 ==========
    /**
     * @brief 获取树的大小（节点总数）
     * @return 节点数量
     */
    int size();

    /**
     * @brief 获取有效点数量（未删除的点）
     * @return 有效点数量
     */
    int validnum();

    /**
     * @brief 获取根节点的平衡因子和删除率
     * @param alpha_bal 输出：平衡因子
     * @param alpha_del 输出：删除率
     */
    void root_alpha(float &alpha_bal, float &alpha_del);

    /**
     * @brief 获取树的包围盒范围
     * @return 包围盒
     */
    BoxPointType tree_range();

    // ========== 核心操作函数 ==========
    /**
     * @brief 从点云构建KD树
     * @param point_cloud 输入点云
     */
    void Build(PointVector point_cloud);

    /**
     * @brief K近邻搜索
     * @param point 查询点
     * @param k_nearest K值（搜索的最近邻点数）
     * @param Nearest_Points 输出：最近邻点集合
     * @param Point_Distance 输出：对应的距离
     * @param max_dist 最大搜索距离（默认无穷大）
     */
    void Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, double max_dist = INFINITY);

    /**
     * @brief 添加点到树中
     * @param PointToAdd 待添加的点集合
     * @param downsample_on 是否开启降采样
     * @return 实际添加的点数
     */
    int Add_Points(PointVector &PointToAdd, bool downsample_on);

    /**
     * @brief 添加包围盒（恢复之前删除的点）
     * @param BoxPoints 包围盒集合
     */
    void Add_Point_Boxes(vector<BoxPointType> &BoxPoints);

    /**
     * @brief 删除指定点
     * @param PointToDel 待删除的点集合
     */
    void Delete_Points(PointVector &PointToDel);

    /**
     * @brief 删除包围盒内的点
     * @param BoxPoints 包围盒集合
     * @return 删除的点数
     */
    int Delete_Point_Boxes(vector<BoxPointType> &BoxPoints);

    /**
     * @brief 将树展平为点云
     * @param root 根节点
     * @param Storage 输出：点云存储
     * @param storage_type 存储类型（是否记录删除点）
     */
    void flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type);

    /**
     * @brief 获取已删除的点
     * @param removed_points 输出：已删除的点集合
     */
    void acquire_removed_points(PointVector &removed_points);

    // ========== 公共成员变量 ==========
    PointVector PCL_Storage;           // 点云存储
    KD_TREE_NODE *Root_Node = nullptr; // 树的根节点
    int max_queue_size = 0;            // 最大队列大小（统计用）
};

// template <typename PointType>
// PointType KD_TREE<PointType>::zeroP = PointType(0,0,0);
