#include "ikd_Tree.h"

/*
Description: ikd-Tree: an incremental k-d tree for robotic applications
             增量式KD树：用于机器人应用的动态KD树实现
Author: Yixi Cai
email: yixicai@connect.hku.hk
*/

/**
 * @brief 构造函数：初始化KD树
 * @param delete_param 删除判据参数
 * @param balance_param 平衡判据参数
 * @param box_length 降采样盒子大小
 */
template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length)
{
    delete_criterion_param = delete_param;    // 设置删除率阈值
    balance_criterion_param = balance_param;  // 设置平衡度阈值
    downsample_size = box_length;             // 设置降采样体素大小
    Rebuild_Logger.clear();                   // 清空重建日志
    termination_flag = false;                 // 设置线程未终止
    start_thread();                           // 启动重建线程
}

/**
 * @brief 析构函数：清理所有资源
 */
template <typename PointType>
KD_TREE<PointType>::~KD_TREE()
{
    stop_thread();                          // 停止重建线程
    Delete_Storage_Disabled = true;         // 禁用删除点存储
    delete_tree_nodes(&Root_Node);          // 删除所有树节点
    PointVector().swap(PCL_Storage);        // 清空点云存储
    Rebuild_Logger.clear();                 // 清空重建日志
}



/**
 * @brief 初始化KD树参数
 * @param delete_param 删除判据参数
 * @param balance_param 平衡判据参数
 * @param box_length 降采样盒子大小
 */
template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length)
{
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
}

/**
 * @brief 初始化树节点：将节点的所有成员变量设为默认值
 * @param root 待初始化的节点指针
 */
template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE *root)
{
    // 初始化点坐标为零
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;
    // 初始化节点包围盒范围为零
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;
    // 初始化分割轴为x轴
    root->division_axis = 0;
    // 初始化指针为空
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    // 初始化树统计信息
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    // 初始化删除标志
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->working_flag = false;
    // 初始化互斥锁
    pthread_mutex_init(&(root->push_down_mutex_lock), NULL);
}

/**
 * @brief 获取树的大小（总节点数）
 * @return 树中的节点总数（包括已删除但未物理移除的节点）
 */
template <typename PointType>
int KD_TREE<PointType>::size()
{
    int s = 0;
    // 如果当前没有在重建，或者重建的不是根节点
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
        {
            return Root_Node->TreeSize;
        }
        else
        {
            return 0;
        }
    }
    else  // 如果正在重建根节点
    {
        // 尝试获取锁
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            s = Root_Node->TreeSize;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        }
        else  // 获取锁失败，返回临时值
        {
            return Treesize_tmp;
        }
    }
}

template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range()
{
    BoxPointType range;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
        {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        }
        else
        {
            memset(&range, 0, sizeof(range));
        }
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            pthread_mutex_unlock(&working_flag_mutex);
        }
        else
        {
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

template <typename PointType>
int KD_TREE<PointType>::validnum()
{
    int s = 0;
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        if (Root_Node != nullptr)
            return (Root_Node->TreeSize - Root_Node->invalid_point_num);
        else
            return 0;
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            s = Root_Node->TreeSize - Root_Node->invalid_point_num;
            pthread_mutex_unlock(&working_flag_mutex);
            return s;
        }
        else
        {
            return -1;
        }
    }
}

template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del)
{
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    }
    else
    {
        if (!pthread_mutex_trylock(&working_flag_mutex))
        {
            alpha_bal = Root_Node->alpha_bal;
            alpha_del = Root_Node->alpha_del;
            pthread_mutex_unlock(&working_flag_mutex);
            return;
        }
        else
        {
            alpha_bal = alpha_bal_tmp;
            alpha_del = alpha_del_tmp;
            return;
        }
    }
}

template <typename PointType>
void KD_TREE<PointType>::start_thread()
{
    pthread_mutex_init(&termination_flag_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_ptr_mutex_lock, NULL);
    pthread_mutex_init(&rebuild_logger_mutex_lock, NULL);
    pthread_mutex_init(&points_deleted_rebuild_mutex_lock, NULL);
    pthread_mutex_init(&working_flag_mutex, NULL);
    pthread_mutex_init(&search_flag_mutex, NULL);
    pthread_create(&rebuild_thread, NULL, multi_thread_ptr, (void *)this);
    printf("Multi thread started \n");
}

template <typename PointType>
void KD_TREE<PointType>::stop_thread()
{
    pthread_mutex_lock(&termination_flag_mutex_lock);
    termination_flag = true;
    pthread_mutex_unlock(&termination_flag_mutex_lock);
    if (rebuild_thread)
        pthread_join(rebuild_thread, NULL);
    pthread_mutex_destroy(&termination_flag_mutex_lock);
    pthread_mutex_destroy(&rebuild_logger_mutex_lock);
    pthread_mutex_destroy(&rebuild_ptr_mutex_lock);
    pthread_mutex_destroy(&points_deleted_rebuild_mutex_lock);
    pthread_mutex_destroy(&working_flag_mutex);
    pthread_mutex_destroy(&search_flag_mutex);
}

template <typename PointType>
void *KD_TREE<PointType>::multi_thread_ptr(void *arg)
{
    KD_TREE *handle = (KD_TREE *)arg;
    handle->multi_thread_rebuild();
    return nullptr;
}

/**
 * @brief 多线程重建的主循环函数
 *
 * 工作流程：
 * 1. 检查是否有子树需要重建（Rebuild_Ptr非空）
 * 2. 遍历旧子树，提取所有有效点
 * 3. 用提取的点重新构建平衡的KD树
 * 4. 将重建期间的操作从日志中重放到新树
 * 5. 用新树替换旧树
 * 6. 删除旧树节点
 *
 * 该函数在后台线程中运行，与主线程并发执行，提高了树的动态性能。
 */
template <typename PointType>
void KD_TREE<PointType>::multi_thread_rebuild()
{
    bool terminated = false;
    KD_TREE_NODE *father_ptr, **new_node_ptr;

    // 检查线程是否应该终止
    pthread_mutex_lock(&termination_flag_mutex_lock);
    terminated = termination_flag;
    pthread_mutex_unlock(&termination_flag_mutex_lock);

    // 主循环：持续检查是否有重建任务
    while (!terminated)
    {
        pthread_mutex_lock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&working_flag_mutex);

        if (Rebuild_Ptr != nullptr)  // 有子树需要重建
        {
            /* 第1步：遍历并复制旧树 */
            if (!Rebuild_Logger.empty())
            {
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;

            // 如果重建的是根节点，保存临时信息
            if (*Rebuild_Ptr == Root_Node)
            {
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp = Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }
            KD_TREE_NODE *old_root_node = (*Rebuild_Ptr);
            father_ptr = (*Rebuild_Ptr)->father_ptr;
            PointVector().swap(Rebuild_PCL_Storage);
            // Lock Search
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0)
            {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;
            pthread_mutex_unlock(&search_flag_mutex);
            // Lock deleted points cache
            pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
            flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);
            // Unlock deleted points cache
            pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
            // Unlock Search
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            pthread_mutex_unlock(&working_flag_mutex);
            /* Rebuild and update missed operations*/
            Operation_Logger_Type Operation;
            KD_TREE_NODE *new_root_node = nullptr;
            if (int(Rebuild_PCL_Storage.size()) > 0)
            {
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size() - 1, Rebuild_PCL_Storage);
                // Rebuild has been done. Updates the blocked operations into the new tree
                pthread_mutex_lock(&working_flag_mutex);
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                int tmp_counter = 0;
                while (!Rebuild_Logger.empty())
                {
                    Operation = Rebuild_Logger.front();
                    max_queue_size = max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    pthread_mutex_unlock(&working_flag_mutex);
                    run_operation(&new_root_node, Operation);
                    tmp_counter++;
                    if (tmp_counter % 10 == 0)
                        usleep(1);
                    pthread_mutex_lock(&working_flag_mutex);
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            /* Replace to original tree*/
            // pthread_mutex_lock(&working_flag_mutex);
            pthread_mutex_lock(&search_flag_mutex);
            while (search_mutex_counter != 0)
            {
                pthread_mutex_unlock(&search_flag_mutex);
                usleep(1);
                pthread_mutex_lock(&search_flag_mutex);
            }
            search_mutex_counter = -1;
            pthread_mutex_unlock(&search_flag_mutex);
            if (father_ptr->left_son_ptr == *Rebuild_Ptr)
            {
                father_ptr->left_son_ptr = new_root_node;
            }
            else if (father_ptr->right_son_ptr == *Rebuild_Ptr)
            {
                father_ptr->right_son_ptr = new_root_node;
            }
            else
            {
                throw "Error: Father ptr incompatible with current node\n";
            }
            if (new_root_node != nullptr)
                new_root_node->father_ptr = father_ptr;
            (*Rebuild_Ptr) = new_root_node;
            int valid_old = old_root_node->TreeSize - old_root_node->invalid_point_num;
            int valid_new = new_root_node->TreeSize - new_root_node->invalid_point_num;
            if (father_ptr == STATIC_ROOT_NODE)
                Root_Node = STATIC_ROOT_NODE->left_son_ptr;
            KD_TREE_NODE *update_root = *Rebuild_Ptr;
            while (update_root != nullptr && update_root != Root_Node)
            {
                update_root = update_root->father_ptr;
                if (update_root->working_flag)
                    break;
                if (update_root == update_root->father_ptr->left_son_ptr && update_root->father_ptr->need_push_down_to_left)
                    break;
                if (update_root == update_root->father_ptr->right_son_ptr && update_root->father_ptr->need_push_down_to_right)
                    break;
                Update(update_root);
            }
            pthread_mutex_lock(&search_flag_mutex);
            search_mutex_counter = 0;
            pthread_mutex_unlock(&search_flag_mutex);
            Rebuild_Ptr = nullptr;
            pthread_mutex_unlock(&working_flag_mutex);
            rebuild_flag = false;
            /* Delete discarded tree nodes */
            delete_tree_nodes(&old_root_node);
        }
        else
        {
            pthread_mutex_unlock(&working_flag_mutex);
        }
        pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        pthread_mutex_lock(&termination_flag_mutex_lock);
        terminated = termination_flag;
        pthread_mutex_unlock(&termination_flag_mutex_lock);
        usleep(100);
    }
    printf("Rebuild thread terminated normally\n");
}

template <typename PointType>
void KD_TREE<PointType>::run_operation(KD_TREE_NODE **root, Operation_Logger_Type operation)
{
    switch (operation.op)
    {
    case ADD_POINT:
        Add_by_point(root, operation.point, false, (*root)->division_axis);
        break;
    case ADD_BOX:
        Add_by_range(root, operation.boxpoint, false);
        break;
    case DELETE_POINT:
        Delete_by_point(root, operation.point, false);
        break;
    case DELETE_BOX:
        Delete_by_range(root, operation.boxpoint, false, false);
        break;
    case DOWNSAMPLE_DELETE:
        Delete_by_range(root, operation.boxpoint, false, true);
        break;
    case PUSH_DOWN:
        (*root)->tree_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->point_downsample_deleted |= operation.tree_downsample_deleted;
        (*root)->tree_deleted = operation.tree_deleted || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = (*root)->tree_deleted || (*root)->point_downsample_deleted;
        if (operation.tree_downsample_deleted)
            (*root)->down_del_num = (*root)->TreeSize;
        if (operation.tree_deleted)
            (*root)->invalid_point_num = (*root)->TreeSize;
        else
            (*root)->invalid_point_num = (*root)->down_del_num;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        break;
    default:
        break;
    }
}

/**
 * @brief 从点云构建KD树（公共接口）
 * @param point_cloud 输入点云
 *
 * 该函数会清除旧树（如果存在），然后从头构建新的平衡KD树。
 * 使用STATIC_ROOT_NODE作为虚拟根节点，实际的树根是其左子节点。
 */
template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud)
{
    // 如果已有树，先删除
    if (Root_Node != nullptr)
    {
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0)
        return;

    // 创建虚拟根节点
    STATIC_ROOT_NODE = new KD_TREE_NODE;
    InitTreeNode(STATIC_ROOT_NODE);

    // 递归构建树（实际的树根是虚拟根节点的左子节点）
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
}

/**
 * @brief K近邻搜索（公共接口）
 * @param point 查询点
 * @param k_nearest K值（需要找到的最近邻点数）
 * @param Nearest_Points 输出：找到的最近邻点集合
 * @param Point_Distance 输出：对应的距离集合
 * @param max_dist 最大搜索距离（超过此距离的点不考虑）
 *
 * 该函数使用最大堆来维护K个最近邻候选点，并处理多线程重建时的并发控制。
 */
template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, double max_dist)
{
    MANUAL_HEAP q(2 * k_nearest);  // 创建容量为2K的最大堆
    q.clear();
    vector<float>().swap(Point_Distance);

    // 检查是否正在重建根节点
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        Search(Root_Node, k_nearest, point, q, max_dist);
    }
    else  // 正在重建根节点，需要获取搜索锁
    {
        pthread_mutex_lock(&search_flag_mutex);
        // 等待重建线程释放搜索锁
        while (search_mutex_counter == -1)
        {
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);  // 短暂休眠
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;  // 增加搜索计数
        pthread_mutex_unlock(&search_flag_mutex);

        Search(Root_Node, k_nearest, point, q, max_dist);

        // 搜索完成，减少搜索计数
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);
    }

    // 从堆中提取结果（距离从小到大排序）
    int k_found = min(k_nearest, int(q.size()));
    PointVector().swap(Nearest_Points);
    vector<float>().swap(Point_Distance);
    for (int i = 0; i < k_found; i++)
    {
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector &PointToAdd, bool downsample_on)
{
    int NewPointSize = PointToAdd.size();
    int tree_size = size();
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    int tmp_counter = 0;
    for (int i = 0; i < PointToAdd.size(); i++)
    {
        if (downsample_switch)
        {
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0] + downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1] + downsample_size;
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2] + downsample_size;
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0] - Box_of_Point.vertex_min[0]) / 2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1] - Box_of_Point.vertex_min[1]) / 2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2] - Box_of_Point.vertex_min[2]) / 2.0;
            PointVector().swap(Downsample_Storage);
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i], mid_point);
            downsample_result = PointToAdd[i];
            for (int index = 0; index < Downsample_Storage.size(); index++)
            {
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist)
                {
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
            {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
                {
                    if (Downsample_Storage.size() > 0)
                        Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    Add_by_point(&Root_Node, downsample_result, true, Root_Node->division_axis);
                    tmp_counter++;
                }
            }
            else
            {
                if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
                {
                    Operation_Logger_Type operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    pthread_mutex_lock(&working_flag_mutex);
                    if (Downsample_Storage.size() > 0)
                        Delete_by_range(&Root_Node, Box_of_Point, false, true);
                    Add_by_point(&Root_Node, downsample_result, false, Root_Node->division_axis);
                    tmp_counter++;
                    if (rebuild_flag)
                    {
                        pthread_mutex_lock(&rebuild_logger_mutex_lock);
                        if (Downsample_Storage.size() > 0)
                            Rebuild_Logger.push(operation_delete);
                        Rebuild_Logger.push(operation);
                        pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                    }
                    pthread_mutex_unlock(&working_flag_mutex);
                };
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
            {
                Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node->division_axis);
            }
            else
            {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;
                pthread_mutex_lock(&working_flag_mutex);
                Add_by_point(&Root_Node, PointToAdd[i], false, Root_Node->division_axis);
                if (rebuild_flag)
                {
                    pthread_mutex_lock(&rebuild_logger_mutex_lock);
                    Rebuild_Logger.push(operation);
                    pthread_mutex_unlock(&rebuild_logger_mutex_lock);
                }
                pthread_mutex_unlock(&working_flag_mutex);
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            Add_by_range(&Root_Node, BoxPoints[i], true);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_range(&Root_Node, BoxPoints[i], false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector &PointToDel)
{
    for (int i = 0; i < PointToDel.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            Delete_by_point(&Root_Node, PointToDel[i], true);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&Root_Node, PointToDel[i], false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    int tmp_counter = 0;
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
        {
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], true, false);
        }
        else
        {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;
            pthread_mutex_lock(&working_flag_mutex);
            tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], false, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector &removed_points)
{
    pthread_mutex_lock(&points_deleted_rebuild_mutex_lock);
    for (int i = 0; i < Points_deleted.size(); i++)
    {
        removed_points.push_back(Points_deleted[i]);
    }
    for (int i = 0; i < Multithread_Points_deleted.size(); i++)
    {
        removed_points.push_back(Multithread_Points_deleted[i]);
    }
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
    pthread_mutex_unlock(&points_deleted_rebuild_mutex_lock);
    return;
}

/**
 * @brief 递归构建KD树
 * @param root 指向根节点指针的指针
 * @param l 点云数组的左边界索引
 * @param r 点云数组的右边界索引
 * @param Storage 存储点云的数组
 *
 * 算法流程：
 * 1. 找到最优分割轴（方差最大的维度）
 * 2. 按分割轴对点进行划分
 * 3. 递归构建左右子树
 */
template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage)
{
    if (l > r)
        return;
    // 创建新节点
    *root = new KD_TREE_NODE;
    InitTreeNode(*root);
    int mid = (l + r) >> 1;  // 中间位置（位运算除以2）
    int div_axis = 0;         // 分割轴
    int i;

    // 第1步：找到最优分割轴（选择范围最大的维度）
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};

    // 计算每个维度的最小值和最大值
    for (i = l; i <= r; i++)
    {
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }

    // 计算每个维度的范围
    for (i = 0; i < 3; i++)
        dim_range[i] = max_value[i] - min_value[i];

    // 选择范围最大的维度作为分割轴
    for (i = 1; i < 3; i++)
        if (dim_range[i] > dim_range[div_axis])
            div_axis = i;

    // 第2步：按分割轴划分点，并选择中位数作为根节点
    (*root)->division_axis = div_axis;
    switch (div_axis)
    {
    case 0:  // x轴
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    case 1:  // y轴
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_y);
        break;
    case 2:  // z轴
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    }
    (*root)->point = Storage[mid];  // 中位数作为根节点

    // 第3步：递归构建左右子树
    KD_TREE_NODE *left_son = nullptr, *right_son = nullptr;
    BuildTree(&left_son, l, mid - 1, Storage);      // 构建左子树
    BuildTree(&right_son, mid + 1, r, Storage);     // 构建右子树
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update((*root));  // 更新节点信息（包围盒、树大小等）
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE **root)
{
    KD_TREE_NODE *father_ptr;
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num)
    {
        if (!pthread_mutex_trylock(&rebuild_ptr_mutex_lock))
        {
            if (Rebuild_Ptr == nullptr || ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize))
            {
                Rebuild_Ptr = root;
            }
            pthread_mutex_unlock(&rebuild_ptr_mutex_lock);
        }
    }
    else
    {
        father_ptr = (*root)->father_ptr;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        delete_tree_nodes(root);
        BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage);
        if (*root != nullptr)
            (*root)->father_ptr = father_ptr;
        if (*root == Root_Node)
            STATIC_ROOT_NODE->left_son_ptr = *root;
    }
    return;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return 0;
    (*root)->working_flag = true;
    Push_Down(*root);
    int tmp_counter = 0;
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return 0;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample)
        {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }
        return tmp_counter;
    }
    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample)
            (*root)->point_downsample_deleted = true;
    }
    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;
    if (is_downsample)
        delete_box_log.op = DOWNSAMPLE_DELETE;
    else
        delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
    {
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
    {
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, false, is_downsample);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(delete_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted)
    {
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize)
            (*root)->tree_deleted = true;
        return;
    }
    Operation_Logger_Type delete_log;
    struct timespec Timeout;
    delete_log.op = DELETE_POINT;
    delete_log.point = point;
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
        {
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->left_son_ptr, point, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    else
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
        {
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(delete_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild)
{
    if ((*root) == nullptr)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] && boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] && boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x && boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y && boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Operation_Logger_Type add_box_log;
    struct timespec Timeout;
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
    {
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
    {
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    }
    else
    {
        pthread_mutex_lock(&working_flag_mutex);
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag)
        {
            pthread_mutex_lock(&rebuild_logger_mutex_lock);
            Rebuild_Logger.push(add_box_log);
            pthread_mutex_unlock(&rebuild_logger_mutex_lock);
        }
        pthread_mutex_unlock(&working_flag_mutex);
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis)
{
    if (*root == nullptr)
    {
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }
    (*root)->working_flag = true;
    Operation_Logger_Type add_log;
    struct timespec Timeout;
    add_log.op = ADD_POINT;
    add_log.point = point;
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) || ((*root)->division_axis == 1 && point.y < (*root)->point.y) || ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->left_son_ptr != *Rebuild_Ptr)
        {
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->left_son_ptr, point, false, (*root)->division_axis);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    else
    {
        if ((Rebuild_Ptr == nullptr) || (*root)->right_son_ptr != *Rebuild_Ptr)
        {
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild, (*root)->division_axis);
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            Add_by_point(&(*root)->right_son_ptr, point, false, (*root)->division_axis);
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(add_log);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    Update(*root);
    if (Rebuild_Ptr != nullptr && *Rebuild_Ptr == *root && (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num)
        Rebuild_Ptr = nullptr;
    bool need_rebuild = allow_rebuild & Criterion_Check((*root));
    if (need_rebuild)
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return;
}

/**
 * @brief K近邻搜索的核心递归函数
 * @param root 当前搜索的子树根节点
 * @param k_nearest K值（需要搜索的最近邻点数）
 * @param point 查询点
 * @param q 最大堆，存储候选的K个最近邻点
 * @param max_dist 最大搜索距离
 *
 * 算法策略：
 * 1. 剪枝：如果节点包围盒到查询点的距离大于max_dist或当前第K近邻点的距离，跳过该子树
 * 2. 检查当前节点的点是否应该加入候选集
 * 3. 优先搜索距离更近的子树（启发式搜索）
 */
template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist)
{
    if (root == nullptr || root->tree_deleted)
        return;

    // 剪枝1：计算查询点到节点包围盒的最小距离
    double cur_dist = calc_box_dist(root, point);
    if (cur_dist > max_dist * max_dist)
        return;

    int retval;
    // 如果需要下推删除标记，先处理
    if (root->need_push_down_to_left || root->need_push_down_to_right)
    {
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0)
        {
            Push_Down(root);  // 下推删除标记
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
        else
        {
            // 等待其他线程完成下推操作
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }

    // 检查当前节点的点是否应该加入候选集
    if (!root->point_deleted)
    {
        float dist = calc_dist(point, root->point);
        // 如果距离满足条件，更新堆
        if (dist <= max_dist && (q.size() < k_nearest || dist < q.top().dist))
        {
            if (q.size() >= k_nearest)
                q.pop();  // 移除当前最远的点
            PointType_CMP current_point{root->point, dist};
            q.push(current_point);  // 添加当前点
        }
    }
    int cur_search_counter;
    // 计算查询点到左右子树包围盒的距离
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);

    // 如果需要搜索两个子树（启发式搜索：优先搜索距离更近的子树）
    if (q.size() < k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist)
    {
        if (dist_left_node <= dist_right_node)  // 左子树更近
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
                {
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
                {
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    }
    else
    {
        if (dist_left_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }
    return;
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] && boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] && boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1])
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x && boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y && boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z)
    {
        if (!root->point_deleted)
            Storage.push_back(root->point);
    }
    if ((Rebuild_Ptr == nullptr) || root->left_son_ptr != *Rebuild_Ptr)
    {
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    if ((Rebuild_Ptr == nullptr) || root->right_son_ptr != *Rebuild_Ptr)
    {
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    }
    else
    {
        pthread_mutex_lock(&search_flag_mutex);
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
        pthread_mutex_unlock(&search_flag_mutex);
    }
    return;
}

/**
 * @brief 检查节点是否需要重建
 * @param root 待检查的节点
 * @return true表示需要重建，false表示不需要
 *
 * 重建判据：
 * 1. 删除率过高：invalid_point_num / TreeSize > delete_criterion_param
 * 2. 树严重不平衡：左右子树大小比例超出balance_criterion_param
 *
 * 这是ikd-Tree的核心机制之一：通过延迟重建（懒惰删除）来平衡动态性和效率。
 */
template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE *root)
{
    // 树太小不需要重建
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size)
    {
        return false;
    }

    float balance_evaluation = 0.0f;  // 平衡度评估
    float delete_evaluation = 0.0f;   // 删除率评估

    // 获取子节点（用于计算平衡度）
    KD_TREE_NODE *son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr)
        son_ptr = root->right_son_ptr;

    // 计算删除率：无效点数 / 总节点数
    delete_evaluation = float(root->invalid_point_num) / root->TreeSize;
    // 计算平衡度：较大子树大小 / (总节点数-1)
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);

    // 判据1：删除率过高
    if (delete_evaluation > delete_criterion_param)
    {
        return true;
    }

    // 判据2：树严重不平衡（一边过重或过轻）
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1 - balance_criterion_param)
    {
        return true;
    }

    return false;
}

/**
 * @brief 向下推送删除标记（懒惰删除机制）
 * @param root 当前节点
 *
 * 懒惰删除：不立即物理删除节点，而是标记为已删除，并将删除标记向下传播到子节点。
 * 这样可以避免频繁的树重建，提高动态操作的效率。
 * 只有在树严重不平衡或删除率过高时才触发重建。
 */
template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root)
{
    if (root == nullptr)
        return;
    // 创建下推操作日志
    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;

    // 向左子节点推送删除标记
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
        {
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
            else
                root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_left = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr)
    {
        if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
        {
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        }
        else
        {
            pthread_mutex_lock(&working_flag_mutex);
            root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag)
            {
                pthread_mutex_lock(&rebuild_logger_mutex_lock);
                Rebuild_Logger.push(operation);
                pthread_mutex_unlock(&rebuild_logger_mutex_lock);
            }
            root->need_push_down_to_right = false;
            pthread_mutex_unlock(&working_flag_mutex);
        }
    }
    return;
}

/**
 * @brief 更新节点信息：包括树大小、无效点数、包围盒范围等
 * @param root 待更新的节点
 *
 * 该函数在节点的子树发生变化后调用，用于维护树的增强信息：
 * 1. TreeSize：子树的总节点数
 * 2. invalid_point_num：子树中被删除的点数
 * 3. node_range_x/y/z：子树的包围盒范围
 * 4. tree_deleted：整个子树是否全部被删除
 * 5. alpha_bal和alpha_del：用于判断是否需要重建
 */
template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE *root)
{
    KD_TREE_NODE *left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE *right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};

    // 情况1：左右子树都存在
    if (left_son_ptr != nullptr && right_son_ptr != nullptr)
    {
        // 更新树大小和无效点数
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        // 更新删除标志
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;

        // 更新包围盒范围
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]), root->point.x);
            tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1], right_son_ptr->node_range_x[1]), root->point.x);
            tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0], right_son_ptr->node_range_y[0]), root->point.y);
            tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1], right_son_ptr->node_range_y[1]), root->point.y);
            tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0], right_son_ptr->node_range_z[0]), root->point.z);
            tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1], right_son_ptr->node_range_z[1]), root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (left_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(left_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(left_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(left_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(left_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(left_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(left_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (right_son_ptr != nullptr)
    {
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(right_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(right_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(right_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(right_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(right_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(right_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else
    {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted ? 1 : 0);
        root->down_del_num = (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y;
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;
    }
    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    memcpy(root->node_range_z, tmp_range_z, sizeof(tmp_range_z));
    if (left_son_ptr != nullptr)
        left_son_ptr->father_ptr = root;
    if (right_son_ptr != nullptr)
        right_son_ptr->father_ptr = root;
    if (root == Root_Node && root->TreeSize > 3)
    {
        KD_TREE_NODE *son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr)
            son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        root->alpha_del = float(root->invalid_point_num) / root->TreeSize;
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1 - tmp_bal;
    }
    return;
}

/**
 * @brief 将树展平为点云数组（中序遍历）
 * @param root 子树根节点
 * @param Storage 输出：存储有效点的数组
 * @param storage_type 删除点的存储方式
 *
 * 该函数递归遍历树，将所有未删除的点添加到Storage中。
 * 根据storage_type参数，可选择是否记录已删除的点。
 */
template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type)
{
    if (root == nullptr)
        return;

    Push_Down(root);  // 先下推删除标记

    // 如果点未被删除，添加到存储中
    if (!root->point_deleted)
    {
        Storage.push_back(root->point);
    }

    // 递归处理左右子树
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);

    // 根据storage_type记录已删除的点
    switch (storage_type)
    {
    case NOT_RECORD:  // 不记录删除点
        break;
    case DELETE_POINTS_REC:  // 记录到主删除列表
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Points_deleted.push_back(root->point);
        }
        break;
    case MULTI_THREAD_REC:  // 记录到多线程删除列表
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Multithread_Points_deleted.push_back(root->point);
        }
        break;
    default:
        break;
    }
    return;
}

/**
 * @brief 递归删除所有树节点，释放内存
 * @param root 指向根节点指针的指针
 *
 * 采用后序遍历（先删除子节点，再删除当前节点）释放所有节点的内存。
 */
template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE **root)
{
    if (*root == nullptr)
        return;

    Push_Down(*root);  // 先下推删除标记

    // 递归删除左右子树
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);

    // 销毁互斥锁并删除节点
    pthread_mutex_destroy(&(*root)->push_down_mutex_lock);
    delete *root;
    *root = nullptr;

    return;
}

/**
 * @brief 判断两个点是否相同
 * @param a 点a
 * @param b 点b
 * @return 如果两点在三个维度上的差值都小于EPSS，则认为相同
 */
template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b)
{
    return (fabs(a.x - b.x) < EPSS && fabs(a.y - b.y) < EPSS && fabs(a.z - b.z) < EPSS);
}

/**
 * @brief 计算两点之间的欧氏距离的平方
 * @param a 点a
 * @param b 点b
 * @return 距离的平方（避免开方运算，提高效率）
 */
template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b)
{
    float dist = 0.0f;
    dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    return dist;
}

/**
 * @brief 计算点到节点包围盒的最小距离平方
 * @param node 节点
 * @param point 查询点
 * @return 点到包围盒的最小距离的平方
 *
 * 如果点在包围盒内部，返回0；否则返回到包围盒表面的最小距离的平方。
 * 这是KD树搜索中的重要剪枝函数。
 */
template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE *node, PointType point)
{
    if (node == nullptr)
        return INFINITY;
    float min_dist = 0.0;
    // x维度：如果点在包围盒外，累加到包围盒边界的距离平方
    if (point.x < node->node_range_x[0])
        min_dist += (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1])
        min_dist += (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
    // y维度
    if (point.y < node->node_range_y[0])
        min_dist += (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1])
        min_dist += (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
    // z维度
    if (point.z < node->node_range_z[0])
        min_dist += (point.z - node->node_range_z[0]) * (point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1])
        min_dist += (point.z - node->node_range_z[1]) * (point.z - node->node_range_z[1]);
    return min_dist;
}

// 点比较函数：用于构建KD树时按不同轴排序
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x; }  // 按x坐标比较
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y; }  // 按y坐标比较
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z; }  // 按z坐标比较

// ========================================
// 手动实现的数据结构（已在头文件中实现）
// ========================================
// Manual heap（手动实现的最大堆）
// Manual queue（手动实现的循环队列）

// ========================================
// 模板类显式实例化
// ========================================
// 为常用的PCL点类型显式实例化模板类，避免链接错误
template class KD_TREE<pcl::PointXYZ>;         // 基本XYZ点
template class KD_TREE<pcl::PointXYZI>;        // 带强度的XYZ点
template class KD_TREE<pcl::PointXYZINormal>;  // 带强度和法向量的XYZ点

