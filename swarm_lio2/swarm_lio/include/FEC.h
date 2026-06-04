#pragma once
#pragma warning(disable:4996)
//#ifndef PCL_SEGEMENT_FEC_H
//#define PCL_SEGEMENT_FEC_H
//#endif

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <pcl/io/ply_io.h>
#include <ctime>
#include <omp.h>


/**
 * @file FEC.h
 * @brief 快速欧几里得聚类（Fast Euclidean Clustering）实现
 * @details 基于PCL库的点云聚类算法，用于将点云分割成多个簇
 *          相比PCL自带的欧几里得聚类，该实现使用了优化的标签传播策略
 */

/**
 * @namespace FEC
 * @brief 快速欧几里得聚类命名空间
 */
namespace FEC{
    using namespace std;

    /**
     * @struct PointIndex_NumberTag
     * @brief 点索引和标签信息存储结构
     * @details 用于在聚类过程中存储每个点的索引及其所属簇的标签
     */
    struct PointIndex_NumberTag
    {
        float nPointIndex;  // 点在原始点云中的索引
        float nNumberTag;   // 点所属的簇标签
    };

    /**
     * @brief 用于排序的比较函数
     * @param p0 第一个点索引标签对
     * @param p1 第二个点索引标签对
     * @return 如果p0的标签小于p1的标签则返回true
     * @details 按照标签值升序排列，使相同簇的点聚集在一起
     */
    static bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1)
    {
        return p0.nNumberTag < p1.nNumberTag;
    }

    /**
     * @brief 快速欧几里得聚类算法
     * @param cloud 输入点云
     * @param min_component_size 最小簇大小，小于此值的簇将被丢弃
     * @param tolorance 聚类容差（欧几里得距离阈值），单位：米
     * @param max_n 每次半径搜索返回的最大邻居数
     * @return 聚类结果，每个元素是一个簇的点索引集合
     * @details 算法流程：
     *          1. 使用KD树进行半径搜索
     *          2. 为每个点分配簇标签，使用标签传播和合并策略
     *          3. 对标签进行排序和整理
     *          4. 过滤掉小于最小簇大小的簇
     */
    static std::vector<pcl::PointIndices> FastEuclideanClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_component_size, double tolorance, int max_n) {

        unsigned long i, j;
        // 检查点云大小是否满足最小要求
        if (cloud->size() < min_component_size)
        {
            PCL_ERROR("Could not find any cluster");
        }

        // 构建KD树用于快速邻域搜索
        pcl::KdTreeFLANN<pcl::PointXYZ>cloud_kdtreeflann;
        cloud_kdtreeflann.setInputCloud(cloud);

        int cloud_size = cloud->size();
        std::vector<int> marked_indices;  // 存储每个点的簇标签
        marked_indices.resize(cloud_size);

        // 初始化所有点的标签为0（未处理）
        memset(marked_indices.data(), 0, sizeof(int) * cloud_size);
        std::vector<int> pointIdx;  // 存储邻居点索引
        std::vector<float> pointquaredDistance;  // 存储到邻居点的平方距离

        int tag_num = 1, temp_tag_num = -1;  // tag_num: 下一个可用的簇标签

        // 遍历所有点进行聚类
        for (i = 0; i < cloud_size; i++)
        {
            // Clustering process（聚类处理）
            if (marked_indices[i] == 0) // reset to initial value if this point has not been manipulated（如果该点未被处理）
            {
                pointIdx.clear();
                pointquaredDistance.clear();
                // 在tolorance半径内搜索邻居点
                cloud_kdtreeflann.radiusSearch(cloud->points[i], tolorance, pointIdx, pointquaredDistance, max_n);
                /**
                * All neighbors closest to a specified point with a query within a given radius
                * para.tolorance is the radius of the sphere that surrounds all neighbors
                * pointIdx is the resulting index of neighboring points
                * pointquaredDistance is the final square distance to adjacent points
                * pointIdx.size() is the maximum number of neighbors returned by limit
                *
                * 在给定半径内查询指定点的所有最近邻
                * tolorance是包围所有邻居的球体半径
                * pointIdx是邻居点的索引结果
                * pointquaredDistance是到邻居点的平方距离
                * max_n限制返回的最大邻居数
                */
                int min_tag_num = tag_num;
                // 第一次遍历：找到邻域内最小的簇标签
                for (j = 0; j < pointIdx.size(); j++)
                {
                    /**
                     * find the minimum label value contained in the field points, and tag it to this cluster label.
                     * 找到邻域点中包含的最小标签值，用于合并簇
                     */
                    if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num))
                    {
                        min_tag_num = marked_indices[pointIdx[j]];
                    }
                }
                // 第二次遍历：合并标签并统一分配
                for (j = 0; j < pointIdx.size(); j++)
                {
                    temp_tag_num = marked_indices[pointIdx[j]];

                    /*
                     * Each domain point, as well as all points in the same cluster, is uniformly assigned this label
                     * 邻域中的每个点，以及与其属于同一簇的所有点，统一分配此标签
                     */
                    if (temp_tag_num > min_tag_num)
                    {
                        // 将所有具有较大标签的点重新标记为最小标签（簇合并）
                        for (int k = 0; k < cloud_size; k++)
                        {
                            if (marked_indices[k] == temp_tag_num)
                            {
                                marked_indices[k] = min_tag_num;
                            }
                        }
                    }
                    // 为当前邻居点分配最小标签
                    marked_indices[pointIdx[j]] = min_tag_num;
                }
                tag_num++;  // 准备下一个新簇标签
            }
        }

        // 整理聚类结果
        std::vector<PointIndex_NumberTag> indices_tags;  // 点索引-标签对
        std::vector<pcl::PointIndices> cluster_indices;  // 最终的聚类结果
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        indices_tags.resize(cloud_size);

        PointIndex_NumberTag temp_index_tag;

        // 将每个点的索引和标签组合成结构体
        for (i = 0; i < cloud_size; i++)
        {
            /**
            * Put each point index and the corresponding tag value into the indices_tags
            * 将每个点的索引和对应的标签值放入indices_tags
            */
            temp_index_tag.nPointIndex = i;
            temp_index_tag.nNumberTag = marked_indices[i];

            indices_tags[i] = temp_index_tag;
        }

        // 按照标签值排序，使相同簇的点聚集在一起
        sort(indices_tags.begin(), indices_tags.end(), NumberTag);

        // 提取每个簇的点索引
        unsigned long begin_index = 0;
        for (i = 0; i < indices_tags.size(); i++)
        {
            // Relabel each cluster（为每个簇重新标记）
            // 当遇到不同的标签时，说明一个簇结束
            if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag)
            {
                // 只保留大于等于最小簇大小的簇
                if ((i - begin_index) >= min_component_size)
                {
                    unsigned long m = 0;
                    inliers->indices.resize(i - begin_index);
                    // 收集当前簇的所有点索引
                    for (j = begin_index; j < i; j++)
                        inliers->indices[m++] = indices_tags[j].nPointIndex;
                    cluster_indices.push_back(*inliers);
                }
                begin_index = i;  // 更新下一个簇的起始索引
            }
        }

        // 处理最后一个簇
        if ((i - begin_index) >= min_component_size)
        {
            for (j = begin_index; j < i; j++)
            {
                unsigned long m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; j++)
                {
                    inliers->indices[m++] = indices_tags[j].nPointIndex;
                }
                cluster_indices.push_back(*inliers);
            }
        }
        return cluster_indices;

    }
}
