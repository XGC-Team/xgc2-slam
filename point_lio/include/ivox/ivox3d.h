/**
 * @file ivox3d.h
 * @brief iVox增量体素地图实现 - 用于高效点云存储和最近邻搜索
 * @author xiang
 * @date 2021/9/16
 *
 * iVox (incremental Voxel) 是一种增量式体素地图数据结构，主要用于激光SLAM中的点云管理。
 * 主要特性：
 * - 基于哈希表的快速体素索引
 * - 支持增量式点云添加
 * - 高效的K近邻搜索
 * - LRU缓存机制管理体素容量
 * - 支持不同的近邻搜索范围（6邻域、18邻域、26邻域）
 */

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

#include <glog/logging.h>
// #include <execution>
#include <list>
#include <thread>
#include <unordered_map>

#include "eigen_types.h"
#include "ivox3d_node.hpp"

namespace faster_lio {

/**
 * @brief iVox节点类型枚举
 */
enum class IVoxNodeType {
    DEFAULT,  // 线性iVox节点（默认类型，使用线性存储）
    PHC,      // PHC（Perfect Hash Coding）iVox节点（使用完美哈希编码）
};

/**
 * @brief iVox节点类型特征（type traits）
 * @tparam node_type 节点类型（DEFAULT或PHC）
 * @tparam PointT 点云数据类型
 * @tparam dim 维度（默认3维）
 */
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

/**
 * @brief DEFAULT类型节点的特化版本，使用线性存储节点
 */
template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

/**
 * @brief PHC类型节点的特化版本，使用完美哈希编码节点
 */
template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
};

/**
 * @brief iVox增量体素地图类
 * @tparam dim 空间维度（默认3维）
 * @tparam node_type 节点类型（默认为DEFAULT线性节点）
 * @tparam PointType 点云类型（默认为pcl::PointXYZ）
 *
 * 该类实现了基于哈希表的增量体素地图，用于高效的点云存储和检索。
 * 主要功能包括：
 * - 点云的增量添加
 * - 最近邻点查询
 * - K近邻查询
 * - LRU缓存管理
 */
template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;              // 体素网格索引类型（整数坐标）
    using PtType = Eigen::Matrix<float, dim, 1>;             // 点坐标类型（浮点坐标）
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;  // 节点类型
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;    // 点云向量类型（使用Eigen内存对齐）
    using DistPoint = typename NodeType::DistPoint;          // 带距离信息的点类型

    /**
     * @brief 近邻搜索类型枚举
     */
    enum class NearbyType {
        CENTER,   // 仅搜索中心体素
        NEARBY6,  // 搜索6邻域（上下左右前后，共7个体素）
        NEARBY18, // 搜索18邻域（6邻域+12条边邻域，共19个体素）
        NEARBY26, // 搜索26邻域（18邻域+8个顶点邻域，共27个体素）
    };

    /**
     * @brief iVox配置参数结构体
     */
    struct Options {
        float resolution_ = 0.2;                        // 体素分辨率（米），即每个体素的边长
        float inv_resolution_ = 10.0;                   // 分辨率的倒数（用于加速坐标转换）
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // 近邻搜索范围类型
        std::size_t capacity_ = 1000000;                // 体素地图容量上限（超过后使用LRU淘汰）
    };

    /**
     * @brief 构造函数
     * @param options iVox配置参数
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    /**
     * @brief 向体素地图中添加点云
     * @param points_to_add 待添加的点云向量
     */
    void AddPoints(const PointVector& points_to_add);

    /**
     * @brief 查找单个点的最近邻点
     * @param pt 查询点
     * @param closest_pt 输出最近邻点
     * @return 是否找到最近邻点
     */
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /**
     * @brief 查找单个点的K近邻点（带条件约束）
     * @param pt 查询点
     * @param closest_pt 输出K近邻点集合
     * @param max_num 最大近邻点数量（默认5个）
     * @param max_range 最大搜索距离（默认5.0米）
     * @return 是否找到近邻点
     */
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /**
     * @brief 批量查找点云的最近邻点
     * @param cloud 查询点云
     * @param closest_cloud 输出最近邻点云
     * @return 是否成功
     */
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /**
     * @brief 获取地图中的总点数
     * @return 点云总数
     */
    size_t NumPoints() const;

    /**
     * @brief 获取有效体素网格数量
     * @return 有效网格数
     */
    size_t NumValidGrids() const;

    /**
     * @brief 获取点云分布统计信息
     * @return 统计数据向量[有效体素数, 平均点数, 最大点数, 最小点数, 标准差]
     */
    std::vector<float> StatGridPoints() const;

    /**
     * @brief 体素网格哈希表
     * 键：体素网格索引
     * 值：指向缓存链表中对应节点的迭代器
     */
    std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, hash_vec<dim>>
        grids_map_;

    /**
     * @brief 将空间坐标转换为体素网格索引
     * @param pt 空间坐标点
     * @return 体素网格索引
     */
    KeyType Pos2Grid(const PtType& pt) const;

    /**
     * @brief 将空间坐标转换为体素网格索引（使用自定义分辨率）
     * @param pt 空间坐标点
     * @param defined_res 自定义分辨率
     * @return 体素网格索引
     */
    KeyType Pos2Grid_(const PtType& pt, const double &defined_res) const;

   private:
    /**
     * @brief 根据配置选项生成近邻体素网格偏移量
     * 根据nearby_type_生成对应的近邻搜索模板
     */
    void GenerateNearbyGrids();

    Options options_;                                       // iVox配置参数
    std::list<std::pair<KeyType, NodeType>> grids_cache_;   // 体素缓存链表（用于LRU管理，最新访问的在前）
    std::vector<KeyType> nearby_grids_;                     // 近邻体素网格偏移量模板（预计算好的相对坐标）
};

/**
 * @brief 查找单个点的最近邻点（实现）
 * @param pt 查询点
 * @param closest_pt 输出最近邻点
 * @return 是否找到最近邻点
 *
 * 算法流程：
 * 1. 计算查询点所在的体素网格索引
 * 2. 遍历所有近邻体素网格
 * 3. 在每个体素中查找最近邻点
 * 4. 从所有候选点中选择距离最小的点
 */
template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;  // 候选近邻点列表
    auto key = Pos2Grid(ToEigen<float, dim>(pt));  // 将查询点坐标转换为体素网格索引
    // 遍历所有近邻体素网格
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;  // 计算近邻体素的实际索引
        auto iter = grids_map_.find(dkey);  // 在哈希表中查找该体素
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);  // 在体素节点中查找最近邻点
            if (found) {
                candidates.emplace_back(dist_point);  // 添加到候选列表
            }
        }
    });

    if (candidates.empty()) {
        return false;  // 未找到任何候选点
    }

    // 从候选点中选择距离最小的点
    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

/**
 * @brief 查找单个点的K近邻点（带条件约束，实现）
 * @param pt 查询点
 * @param closest_pt 输出K近邻点集合
 * @param max_num 最大近邻点数量
 * @param max_range 最大搜索距离
 * @return 是否找到近邻点
 *
 * 算法流程：
 * 1. 计算查询点所在的体素网格索引
 * 2. 遍历所有近邻体素网格，收集满足距离条件的候选点
 * 3. 使用nth_element进行部分排序，选出最近的max_num个点
 * 4. 返回满足条件的近邻点集合
 */
template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    std::vector<DistPoint> candidates;  // 候选近邻点列表
    candidates.reserve(max_num * nearby_grids_.size());  // 预分配内存以提高效率

    auto key = Pos2Grid(ToEigen<float, dim>(pt));  // 将查询点坐标转换为体素网格索引

// #define INNER_TIMER  // 定义此宏可启用内部性能计时统计
#ifdef INNER_TIMER
    static std::unordered_map<std::string, std::vector<int64_t>> stats;  // 性能统计数据
    if (stats.empty()) {
        stats["knn"] = std::vector<int64_t>();  // KNN搜索耗时
        stats["nth"] = std::vector<int64_t>();  // 排序耗时
    }
#endif

    // 遍历所有近邻体素网格，收集候选点
    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;  // 计算近邻体素的实际索引
        auto iter = grids_map_.find(dkey);  // 在哈希表中查找该体素
        if (iter != grids_map_.end()) {
#ifdef INNER_TIMER
            auto t1 = std::chrono::high_resolution_clock::now();
#endif
            // 在体素节点中查找满足条件的K近邻点
            auto tmp = iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
#ifdef INNER_TIMER
            auto t2 = std::chrono::high_resolution_clock::now();
            auto knn = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
            stats["knn"].emplace_back(knn);
#endif
        }
    }

    if (candidates.empty()) {
        return false;  // 未找到任何候选点
    }

#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif

    // 使用nth_element进行部分排序，选出最近的max_num个点
    if (candidates.size() <= max_num) {
        // 候选点数量不超过max_num，无需裁剪
    } else {
        // 使用nth_element将第max_num小的元素放到正确位置，并保证前max_num个元素都不大于它
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);  // 裁剪到max_num个点
    }
    // 确保第一个元素是最小的（最近的点）
    std::nth_element(candidates.begin(), candidates.begin(), candidates.end());

#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();
    auto nth = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["nth"].emplace_back(nth);

    constexpr int STAT_PERIOD = 100000;  // 统计周期
    if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
        // 定期输出性能统计信息
        for (auto& it : stats) {
            const std::string& key = it.first;
            std::vector<int64_t>& stat = it.second;
            int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
            int64_t num_ = stat.size();
            stat.clear();
            std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_ << " ave=" << 1.0 * sum_ / num_
                      << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << std::endl;
        }
    }
#endif

    // 将候选点转换为输出格式
    closest_pt.clear();
    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

/**
 * @brief 获取有效体素网格数量（实现）
 * @return 有效网格数
 */
template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_map_.size();
}

/**
 * @brief 根据配置选项生成近邻体素网格偏移量（实现）
 *
 * 根据nearby_type_的设置，预计算近邻体素的相对坐标偏移量：
 * - CENTER: 只搜索中心体素（1个）
 * - NEARBY6: 6邻域搜索（7个体素：中心+上下左右前后）
 * - NEARBY18: 18邻域搜索（19个体素：6邻域+12条边）
 * - NEARBY26: 26邻域搜索（27个体素：18邻域+8个顶点）
 */
template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        // 仅中心体素
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        // 6邻域：中心 + 6个面邻居
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        // 18邻域：6邻域 + 12条边邻居
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        // 26邻域：18邻域 + 8个顶点邻居（完整的3x3x3立方体）
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        // LOG(ERROR) << "Unknown nearby_type!";
    }
}

/**
 * @brief 批量查找点云的最近邻点（实现）
 * @param cloud 查询点云
 * @param closest_cloud 输出最近邻点云
 * @return 是否成功
 *
 * 遍历输入点云中的每个点，为每个点查找其最近邻点。
 * 如果某个点未找到最近邻，则在对应位置填充空点。
 */
template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
    std::vector<size_t> index(cloud.size());  // 索引向量（未使用）

    closest_cloud.resize(cloud.size());  // 预分配输出点云大小

    // 遍历输入点云，为每个点查找最近邻
    for (int i = 0; i < cloud.size(); ++i) {
        PointType pt;
        if (GetClosestPoint(cloud[i], pt)) {
            closest_cloud[i] = pt;  // 找到最近邻点
        } else {
            closest_cloud[i] = PointType();  // 未找到，填充空点
        }
    };
    return true;
}

/**
 * @brief 向体素地图中添加点云（实现）
 * @param points_to_add 待添加的点云向量
 *
 * 算法流程（使用LRU缓存策略）：
 * 1. 遍历每个待添加的点
 * 2. 计算点所属的体素网格索引
 * 3. 如果体素不存在：
 *    - 创建新体素节点并插入缓存链表头部
 *    - 将体素索引加入哈希表
 *    - 将点插入新体素
 *    - 如果超过容量限制，删除LRU链表尾部的最久未使用体素
 * 4. 如果体素已存在：
 *    - 将点插入现有体素
 *    - 将该体素移到LRU链表头部（标记为最近使用）
 */
template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    for(size_t i = 0; i<points_to_add.size(); i++) {
        // 计算点所属的体素网格索引
        auto key = Pos2Grid(Eigen::Matrix<float, dim, 1>(points_to_add[i].x, points_to_add[i].y, points_to_add[i].z));
        auto iter = grids_map_.find(key);

        if (iter == grids_map_.end()) {
            // 体素不存在，创建新体素
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;  // 计算体素中心坐标

            grids_cache_.push_front({key, NodeType(center, options_.resolution_)});  // 插入到缓存链表头部
            grids_map_.insert({key, grids_cache_.begin()});  // 在哈希表中记录体素位置

            grids_cache_.front().second.InsertPoint(points_to_add[i]);  // 将点插入新体素

            // 检查容量限制，如果超过则删除最久未使用的体素（LRU淘汰）
            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);  // 从哈希表中删除
                grids_cache_.pop_back();  // 从缓存链表中删除
            }
        } else {
            // 体素已存在，直接插入点
            iter->second->second.InsertPoint(points_to_add[i]);
            // 将该体素移到链表头部，标记为最近使用（LRU更新）
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();  // 更新哈希表中的迭代器
        }
    }
}

/**
 * @brief 将空间坐标转换为体素网格索引（实现）
 * @param pt 空间坐标点（浮点数）
 * @return 体素网格索引（整数坐标）
 *
 * 转换公式：grid_idx = floor(pt / resolution) = floor(pt * inv_resolution)
 * 使用预计算的inv_resolution_加速除法运算
 */
template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().floor().template cast<int>();
}

/**
 * @brief 将空间坐标转换为体素网格索引（使用自定义分辨率，实现）
 * @param pt 空间坐标点（浮点数）
 * @param defined_res 自定义分辨率
 * @return 体素网格索引（整数坐标）
 *
 * 与Pos2Grid功能相同，但使用外部指定的分辨率而非配置中的分辨率
 */
template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid_(const IVox::PtType& pt, const double &defined_res) const {
    return (pt / defined_res).array().floor().template cast<int>();
}

/**
 * @brief 获取点云分布统计信息（实现）
 * @return 统计数据向量[有效体素数, 平均点数, 最大点数, 最小点数, 标准差]
 *
 * 统计所有体素中点的分布情况：
 * - valid_num: 包含点的体素数量
 * - ave: 每个体素的平均点数
 * - max: 单个体素中的最大点数
 * - min: 单个体素中的最小点数
 * - stddev: 点数分布的标准差
 */
template <int dim, IVoxNodeType node_type, typename PointType>
std::vector<float> IVox<dim, node_type, PointType>::StatGridPoints() const {
    int num = grids_cache_.size(), valid_num = 0, max = 0, min = 100000000;
    int sum = 0, sum_square = 0;

    // 遍历所有体素，统计点数信息
    for (auto& it : grids_cache_) {
        int s = it.second.Size();  // 获取该体素中的点数
        valid_num += s > 0;        // 统计非空体素数量
        max = s > max ? s : max;   // 更新最大点数
        min = s < min ? s : min;   // 更新最小点数
        sum += s;                  // 累加总点数
        sum_square += s * s;       // 累加平方和（用于计算标准差）
    }

    // 计算平均值和标准差
    float ave = float(sum) / num;
    float stddev = num > 1 ? sqrt((float(sum_square) - num * ave * ave) / (num - 1)) : 0;

    return std::vector<float>{float(valid_num), ave, float(max), float(min), stddev};
}

}  // namespace faster_lio

#endif
