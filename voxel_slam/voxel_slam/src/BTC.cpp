#include "BTC.h"

/**
 * @brief 读取系统配置参数
 * @param nh ROS节点句柄
 * @param config_setting 配置参数结构体，用于存储各种算法参数
 * @param isHighFly 飞行高度标志，0表示低空飞行，1表示高空飞行
 *
 * 该函数根据飞行高度设置不同的参数配置：
 * - 低空飞行：更精细的参数，适用于室内或近距离场景
 * - 高空飞行：更宽松的参数，适用于室外大尺度场景
 */
void read_parameters(ros::NodeHandle &nh, ConfigSetting &config_setting, int isHighFly)
{
  if(!isHighFly)
  {
    // 低空飞行参数配置
    config_setting.useful_corner_num_ = 100;  // 有效角点数量
    config_setting.plane_merge_normal_thre_ = 0.1;  // 平面合并法向量阈值
    config_setting.plane_merge_dis_thre_ = 0.3;  // 平面合并距离阈值
    config_setting.plane_detection_thre_ = 0.01;  // 平面检测阈值（最小特征值）
    config_setting.voxel_size_ = 1;  // 体素大小（米）
    config_setting.voxel_init_num_ = 10;  // 体素初始化所需最小点数
    config_setting.proj_plane_num_ = 2;  // 投影平面数量
    config_setting.proj_image_resolution_ = 0.5;  // 投影图像分辨率
    config_setting.proj_image_high_inc_ = 0.1;  // 投影图像高度增量
    config_setting.proj_dis_min_ = 0;  // 投影最小距离
    config_setting.proj_dis_max_ = 5;  // 投影最大距离
    config_setting.summary_min_thre_ = 10;  // 二进制描述符最小汇总阈值
    config_setting.line_filter_enable_ = 1;  // 是否启用直线滤波
    config_setting.touch_filter_enable_ = 0;  // 是否启用接触滤波

    // STD描述符相关参数
    config_setting.descriptor_near_num_ = 15;  // 描述符近邻点数量
    config_setting.descriptor_min_len_ = 2;  // 三角形边最小长度
    config_setting.descriptor_max_len_ = 50;  // 三角形边最大长度
    config_setting.non_max_suppression_radius_ = 2;  // 非极大值抑制半径
    config_setting.std_side_resolution_ = 0.2;  // STD边长分辨率

    // 回环检测参数
    config_setting.skip_near_num_ = 30;  // 跳过的近邻帧数量
    config_setting.candidate_num_ = 20;  // 候选回环帧数量
    config_setting.rough_dis_threshold_ = 0.01;  // 粗略距离阈值
    config_setting.similarity_threshold_ = 0.7;  // 二进制描述符相似度阈值
    config_setting.icp_threshold_ = 0.15;  // ICP匹配阈值
    config_setting.normal_threshold_ = 0.2;  // 法向量阈值
    config_setting.dis_threshold_ = 0.5;  // 距离阈值
  }
  else
  {
    // 高空飞行参数配置（AVIA飞行模式）
    config_setting.useful_corner_num_ = 200;  // 有效角点数量（更多）
    config_setting.plane_merge_normal_thre_ = 0.3;  // 平面合并法向量阈值（更宽松）
    config_setting.plane_merge_dis_thre_ = 0.6;  // 平面合并距离阈值（更宽松）
    config_setting.plane_detection_thre_ = 0.05;  // 平面检测阈值（更宽松）
    config_setting.voxel_size_ = 2;  // 体素大小（更大）
    config_setting.voxel_init_num_ = 10;  // 体素初始化所需最小点数
    config_setting.proj_plane_num_ = 1;  // 投影平面数量（更少）
    config_setting.proj_image_resolution_ = 0.5;  // 投影图像分辨率
    config_setting.proj_image_high_inc_ = 0.2;  // 投影图像高度增量（更大）
    config_setting.proj_dis_min_ = 0;  // 投影最小距离
    config_setting.proj_dis_max_ = 10;  // 投影最大距离（更远）
    config_setting.summary_min_thre_ = 6;  // 二进制描述符最小汇总阈值（更低）
    config_setting.line_filter_enable_ = 0;  // 禁用直线滤波
    config_setting.touch_filter_enable_ = 0;  // 禁用接触滤波

    // STD描述符相关参数
    config_setting.descriptor_near_num_ = 15;  // 描述符近邻点数量
    config_setting.descriptor_min_len_ = 3;  // 三角形边最小长度（更大）
    config_setting.descriptor_max_len_ = 50;  // 三角形边最大长度
    config_setting.non_max_suppression_radius_ = 3;  // 非极大值抑制半径（更大）
    config_setting.std_side_resolution_ = 0.2;  // STD边长分辨率

    // 回环检测参数
    config_setting.skip_near_num_ = 30;  // 跳过的近邻帧数量
    config_setting.candidate_num_ = 100;  // 候选回环帧数量（更多）
    config_setting.rough_dis_threshold_ = 0.01;  // 粗略距离阈值
    config_setting.similarity_threshold_ = 0.5;  // 二进制描述符相似度阈值（更宽松）
    config_setting.icp_threshold_ = 0.15;  // ICP匹配阈值
    config_setting.normal_threshold_ = 0.2;  // 法向量阈值
    config_setting.dis_threshold_ = 0.5;  // 距离阈值
  }
}

/**
 * @brief 计算两个二进制描述符之间的相似度
 * @param b1 第一个二进制描述符
 * @param b2 第二个二进制描述符
 * @return 返回相似度分数，范围[0,1]，1表示完全相同
 *
 * 使用改进的汉明距离（Hamming Distance）计算相似度：
 * 统计两个描述符占用数组中同时为true的位数，并归一化
 */
double binary_similarity(const BinaryDescriptor &b1,
                         const BinaryDescriptor &b2) {
  double dis = 0;
  for (size_t i = 0; i < b1.occupy_array_.size(); i++) {
    // 统计同时占用的位数
    if (b1.occupy_array_[i] == true && b2.occupy_array_[i] == true) {
      dis += 1;
    }
  }
  // 归一化：2 * 公共占用数 / (总占用数1 + 总占用数2)
  return 2 * dis / (b1.summary_ + b2.summary_);
}

/**
 * @brief 二进制描述符排序比较函数（降序）
 * @param a 第一个二进制描述符
 * @param b 第二个二进制描述符
 * @return 如果a的汇总值大于b返回true
 */
bool binary_greater_sort(BinaryDescriptor a, BinaryDescriptor b) {
  return (a.summary_ > b.summary_);
}

/**
 * @brief 平面排序比较函数（降序）
 * @param plane1 第一个平面指针
 * @param plane2 第二个平面指针
 * @return 如果plane1的点数大于plane2返回true
 */
bool plane_greater_sort(BTCPlane *plane1, BTCPlane *plane2) {
  return plane1->points_size_ > plane2->points_size_;
}

/**
 * @brief 初始化八叉树节点
 *
 * 如果体素内的点数超过阈值，则初始化平面检测
 */
void BTCOctoTree::init_octo_tree() {
  if (voxel_points_.size() > config_setting_.voxel_init_num_) {
    init_plane();
  }
}

/**
 * @brief 初始化平面检测
 *
 * 通过主成分分析（PCA）检测体素内的点是否构成平面
 * 算法步骤：
 * 1. 计算点云的协方差矩阵
 * 2. 特征值分解获取主方向
 * 3. 最小特征值小于阈值则判定为平面
 * 4. 计算平面参数（法向量、中心点、半径）
 */
void BTCOctoTree::init_plane() {
  // 初始化平面参数
  plane_ptr_->covariance_ = Eigen::Matrix3d::Zero();
  plane_ptr_->center_ = Eigen::Vector3d::Zero();
  plane_ptr_->normal_ = Eigen::Vector3d::Zero();
  plane_ptr_->points_size_ = voxel_points_.size();
  plane_ptr_->radius_ = 0;

  // 计算协方差矩阵和中心点
  for (auto &pi : voxel_points_) {
    plane_ptr_->covariance_ += pi * pi.transpose();
    plane_ptr_->center_ += pi;
  }
  plane_ptr_->center_ = plane_ptr_->center_ / plane_ptr_->points_size_;

  // 归一化协方差矩阵
  plane_ptr_->covariance_ =
      plane_ptr_->covariance_ / plane_ptr_->points_size_ -
      plane_ptr_->center_ * plane_ptr_->center_.transpose();

  // 特征值分解
  Eigen::EigenSolver<Eigen::Matrix3d> es(plane_ptr_->covariance_);
  Eigen::Matrix3cd evecs = es.eigenvectors();
  Eigen::Vector3cd evals = es.eigenvalues();
  Eigen::Vector3d evalsReal;
  evalsReal = evals.real();

  // 找到最小和最大特征值的索引
  Eigen::Matrix3d::Index evalsMin, evalsMax;
  evalsReal.rowwise().sum().minCoeff(&evalsMin);
  evalsReal.rowwise().sum().maxCoeff(&evalsMax);
  int evalsMid = 3 - evalsMin - evalsMax;

  // 判断是否为平面：最小特征值小于阈值
  if (evalsReal(evalsMin) < config_setting_.plane_detection_thre_)
  {
    // 最小特征值对应的特征向量即为平面法向量
    plane_ptr_->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
        evecs.real()(2, evalsMin);
    plane_ptr_->min_eigen_value_ = evalsReal(evalsMin);
    plane_ptr_->radius_ = sqrt(evalsReal(evalsMax));  // 平面半径
    plane_ptr_->is_plane_ = true;

    // 计算平面方程 ax + by + cz + d = 0 中的d
    plane_ptr_->d_ = -(plane_ptr_->normal_(0) * plane_ptr_->center_(0) +
                       plane_ptr_->normal_(1) * plane_ptr_->center_(1) +
                       plane_ptr_->normal_(2) * plane_ptr_->center_(2));

    // 保存平面中心点和法向量到PCL点结构
    plane_ptr_->p_center_.x = plane_ptr_->center_(0);
    plane_ptr_->p_center_.y = plane_ptr_->center_(1);
    plane_ptr_->p_center_.z = plane_ptr_->center_(2);
    plane_ptr_->p_center_.normal_x = plane_ptr_->normal_(0);
    plane_ptr_->p_center_.normal_y = plane_ptr_->normal_(1);
    plane_ptr_->p_center_.normal_z = plane_ptr_->normal_(2);
  } else {
    plane_ptr_->is_plane_ = false;
  }
}

// double
// calc_triangle_dis(const std::vector<std::pair<STD, STD>> &match_std_list) {
//   double mean_triangle_dis = 0;
//   for (auto var : match_std_list) {
//     mean_triangle_dis += (var.first.triangle_ - var.second.triangle_).norm() /
//                          var.first.triangle_.norm();
//   }
//   if (match_std_list.size() > 0) {
//     mean_triangle_dis = mean_triangle_dis / match_std_list.size();
//   } else {
//     mean_triangle_dis = -1;
//   }
//   return mean_triangle_dis;
// }

/**
 * @brief 生成稳定三角形描述符（Stable Triangle Descriptors, STD）
 * @param input_cloud 输入点云
 * @param stds_vec 输出STD描述符向量
 * @param id 帧ID
 *
 * 完整的描述符生成流程：
 * 步骤1：体素化和平面检测
 * 步骤2：获取投影平面并合并
 * 步骤3：提取二进制描述符
 * 步骤4：生成稳定三角形描述符
 * 步骤5：清理内存
 */
void STDescManager::GenerateSTDescs(
    pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<STD> &stds_vec, int id)
{
  // 步骤1：体素化和平面检测
  std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> voxel_map;
  init_voxel_map(input_cloud, voxel_map);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud(
      new pcl::PointCloud<pcl::PointXYZINormal>);
  get_plane(voxel_map, plane_cloud);
  plane_cloud->header.seq = id;
  plane_cloud_vec_.push_back(plane_cloud);

  // 步骤2：获取投影平面
  std::vector<BTCPlane *> proj_plane_list;
  std::vector<BTCPlane *> merge_plane_list;
  get_project_plane(voxel_map, proj_plane_list);

  // 如果没有检测到平面，创建默认平面
  if (proj_plane_list.size() == 0)
  {
    BTCPlane *single_plane = new BTCPlane;
    single_plane->normal_ << 0, 0, 1;  // 默认Z轴向上
    single_plane->center_ << input_cloud->points[0].x, input_cloud->points[0].y,
        input_cloud->points[0].z;
    merge_plane_list.push_back(single_plane);
  } else {
    // 按点数降序排序并合并相似平面
    sort(proj_plane_list.begin(), proj_plane_list.end(), plane_greater_sort);
    merge_plane(proj_plane_list, merge_plane_list);
    sort(merge_plane_list.begin(), merge_plane_list.end(), plane_greater_sort);
  }

  // 步骤3：提取二进制描述符
  std::vector<BinaryDescriptor> binary_list;
  binary_extractor(merge_plane_list, input_cloud, binary_list);

  // 步骤4：生成稳定三角形描述符
  stds_vec.clear();
  generate_std(binary_list, current_frame_id_, stds_vec);

  // 步骤5：清理内存
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    delete (iter->second);
  }
  return;
}

/**
 * @brief 搜索回环闭合
 * @param stds_vec 当前帧的STD描述符向量
 * @param loop_result 输出回环结果（帧ID，匹配分数），-1表示未找到回环
 * @param loop_transform 输出回环变换（平移向量，旋转矩阵）
 * @param loop_std_pair 输出成功匹配的STD对
 * @param pl_cur 当前帧的平面点云
 *
 * 回环检测流程：
 * 步骤1：候选帧选择（粗匹配）
 * 步骤2：候选帧验证（精匹配）
 * 步骤3：几何一致性检查
 */
void STDescManager::SearchLoop(
    std::vector<STD> &stds_vec, std::pair<int, double> &loop_result,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    std::vector<std::pair<STD, STD>> &loop_std_pair, pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_cur)
{
  if (stds_vec.size() == 0) {
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
  // 步骤1：选择候选帧（粗匹配）
  std::vector<STDMatchList> candidate_matcher_vec;
  candidate_selector(stds_vec, candidate_matcher_vec);

  // 步骤2：从粗匹配候选中选择最佳候选（精匹配）
  double best_score = 0;
  int best_candidate_id = -1;
  int triggle_candidate = -1;
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> best_transform;
  std::vector<std::pair<STD, STD>> best_sucess_match_vec;

  // 遍历所有候选帧进行验证
  for (size_t i = 0; i < candidate_matcher_vec.size(); i++)
  {
    double verify_score = -1;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> relative_pose;
    std::vector<std::pair<STD, STD>> sucess_match_vec;

    // 候选帧验证：计算相对位姿和匹配分数
    candidate_verify(candidate_matcher_vec[i], verify_score, relative_pose,
                     sucess_match_vec, pl_cur);

    // 更新最佳匹配
    if (verify_score > best_score) {
      best_score = verify_score;
      best_candidate_id = candidate_matcher_vec[i].match_id_.second;
      best_transform = relative_pose;
      best_sucess_match_vec = sucess_match_vec;
      triggle_candidate = i;
    }
  }

  if (best_score > config_setting_.icp_threshold_) {
    loop_result = std::pair<int, double>(best_candidate_id, best_score);
    loop_transform = best_transform;
    loop_std_pair = best_sucess_match_vec;
    return;
  } else {
    loop_result = std::pair<int, double>(-1, 0);
    return;
  }
}

/**
 * @brief 将STD描述符添加到数据库
 * @param stds_vec STD描述符向量
 *
 * 将当前帧的STD描述符添加到哈希表数据库中，用于后续回环检测
 * 使用三角形边长（量化后）作为哈希键，支持快速检索相似描述符
 */
void STDescManager::AddSTDescs(const std::vector<STD> &stds_vec) {
  // 更新帧ID
  current_frame_id_++;

  for (auto &single_std : stds_vec) {
    // 计算STD的哈希位置（将三角形边长量化到整数网格）
    STD_LOC position;
    position.x = (int)(single_std.triangle_[0] + 0.5);
    position.y = (int)(single_std.triangle_[1] + 0.5);
    position.z = (int)(single_std.triangle_[2] + 0.5);

    // 添加到数据库
    auto iter = data_base_.find(position);
    if (iter != data_base_.end()) {
      // 位置已存在，添加到现有列表
      iter->second.push_back(single_std);
    } else {
      // 新位置，创建新列表
      std::vector<STD> descriptor_vec;
      descriptor_vec.push_back(single_std);
      data_base_[position] = descriptor_vec;
    }
  }
  return;
}

/**
 * @brief 初始化体素地图
 * @param input_cloud 输入点云
 * @param voxel_map 输出体素地图（体素位置 -> 八叉树节点）
 *
 * 将输入点云体素化，每个体素包含一个八叉树节点
 * 算法步骤：
 * 1. 遍历所有点，根据体素大小计算体素坐标
 * 2. 将点添加到对应的体素中
 * 3. 初始化每个体素的八叉树（平面检测）
 */
void STDescManager::init_voxel_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> &voxel_map)
{
  uint plsize = input_cloud->size();

  // 遍历所有点，分配到对应体素
  for (uint i = 0; i < plsize; i++) {
    Eigen::Vector3d p_c(input_cloud->points[i].x, input_cloud->points[i].y,
                        input_cloud->points[i].z);

    // 计算体素坐标
    double loc_xyz[3];
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c[j] / config_setting_.voxel_size_;
      // 负数向下取整
      if (loc_xyz[j] < 0) {
        loc_xyz[j] -= 1.0;
      }
    }

    // 体素位置
    BTCVOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                       (int64_t)loc_xyz[2]);

    // 添加点到体素
    auto iter = voxel_map.find(position);
    if (iter != voxel_map.end()) {
      voxel_map[position]->voxel_points_.push_back(p_c);
    } else {
      BTCOctoTree *octo_tree = new BTCOctoTree(config_setting_);
      voxel_map[position] = octo_tree;
      voxel_map[position]->voxel_points_.push_back(p_c);
    }
  }

  // 初始化所有体素的八叉树（平面检测）
  std::vector<std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *>::iterator> iter_list;
  std::vector<size_t> index;
  size_t i = 0;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); ++iter) {
    index.push_back(i);
    i++;
    iter_list.push_back(iter);
  }

  // 顺序初始化每个体素（可改为并行处理）
  for(const size_t &i: index)
    iter_list[i]->second->init_octo_tree();

}

/**
 * @brief 从体素地图中提取平面点云
 * @param voxel_map 体素地图
 * @param plane_cloud 输出平面点云（中心点+法向量）
 *
 * 遍历所有体素，提取被检测为平面的体素中心点和法向量
 */
void STDescManager::get_plane(
    const std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> &voxel_map,
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr &plane_cloud) {
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      pcl::PointXYZINormal pi;
      pi.x = iter->second->plane_ptr_->center_[0];
      pi.y = iter->second->plane_ptr_->center_[1];
      pi.z = iter->second->plane_ptr_->center_[2];
      pi.normal_x = iter->second->plane_ptr_->normal_[0];
      pi.normal_y = iter->second->plane_ptr_->normal_[1];
      pi.normal_z = iter->second->plane_ptr_->normal_[2];
      plane_cloud->push_back(pi);
    }
  }
}

/**
 * @brief 获取投影平面列表（合并相似平面）
 * @param voxel_map 体素地图
 * @param project_plane_list 输出投影平面列表
 *
 * 算法步骤：
 * 1. 提取所有平面
 * 2. 根据法向量和距离合并相似平面
 * 3. 更新合并后平面的参数（协方差、法向量等）
 */
void STDescManager::get_project_plane(
    std::unordered_map<BTCVOXEL_LOC, BTCOctoTree *> &voxel_map,
    std::vector<BTCPlane *> &project_plane_list)
{
  // 提取所有平面
  std::vector<BTCPlane *> origin_list;
  for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    if (iter->second->plane_ptr_->is_plane_) {
      origin_list.push_back(iter->second->plane_ptr_);
    }
  }

  // 初始化平面ID
  for (size_t i = 0; i < origin_list.size(); i++)
    origin_list[i]->id_ = 0;
  // 平面聚类：根据法向量和距离合并相似平面
  int current_id = 1;
  for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--)
  {
    for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++)
    {
      // 计算法向量差异和和
      Eigen::Vector3d normal_diff = (*iter)->normal_ - (*iter2)->normal_;
      Eigen::Vector3d normal_add = (*iter)->normal_ + (*iter2)->normal_;

      // 计算点到平面距离
      double dis1 =
          fabs((*iter)->normal_(0) * (*iter2)->center_(0) +
               (*iter)->normal_(1) * (*iter2)->center_(1) +
               (*iter)->normal_(2) * (*iter2)->center_(2) + (*iter)->d_);
      double dis2 =
          fabs((*iter2)->normal_(0) * (*iter)->center_(0) +
               (*iter2)->normal_(1) * (*iter)->center_(1) +
               (*iter2)->normal_(2) * (*iter)->center_(2) + (*iter2)->d_);

      // 判断两平面是否相似（法向量接近且距离接近）
      if (normal_diff.norm() < config_setting_.plane_merge_normal_thre_ ||
          normal_add.norm() < config_setting_.plane_merge_normal_thre_)
        if (dis1 < config_setting_.plane_merge_dis_thre_ &&
            dis2 < config_setting_.plane_merge_dis_thre_) {
          // 分配或合并ID
          if ((*iter)->id_ == 0 && (*iter2)->id_ == 0)
          {
            (*iter)->id_ = current_id;
            (*iter2)->id_ = current_id;
            current_id++;
          } else if ((*iter)->id_ == 0 && (*iter2)->id_ != 0)
            (*iter)->id_ = (*iter2)->id_;
          else if ((*iter)->id_ != 0 && (*iter2)->id_ == 0)
            (*iter2)->id_ = (*iter)->id_;
        }
    }
  }
  // 合并具有相同ID的平面
  std::vector<BTCPlane *> merge_list;
  std::vector<int> merge_flag;

  for (size_t i = 0; i < origin_list.size(); i++)
  {
    // 检查该ID是否已处理
    auto it =
        std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id_);
    if (it != merge_flag.end())
      continue;
    if (origin_list[i]->id_ == 0) {
      continue;
    }

    // 创建合并后的平面
    BTCPlane *merge_plane = new BTCPlane;
    (*merge_plane) = (*origin_list[i]);
    bool is_merge = false;

    // 查找具有相同ID的所有平面并合并
    for (size_t j = 0; j < origin_list.size(); j++)
    {
      if (i == j)
        continue;
      if (origin_list[j]->id_ == origin_list[i]->id_)
      {
        is_merge = true;
        // 计算合并后的协方差矩阵：先恢复原始P*P^T矩阵
        Eigen::Matrix3d P_PT1 =
            (merge_plane->covariance_ +
             merge_plane->center_ * merge_plane->center_.transpose()) *
            merge_plane->points_size_;
        Eigen::Matrix3d P_PT2 =
            (origin_list[j]->covariance_ +
             origin_list[j]->center_ * origin_list[j]->center_.transpose()) *
            origin_list[j]->points_size_;

        // 计算合并后的中心点
        Eigen::Vector3d merge_center =
            (merge_plane->center_ * merge_plane->points_size_ +
             origin_list[j]->center_ * origin_list[j]->points_size_) /
            (merge_plane->points_size_ + origin_list[j]->points_size_);

        // 计算合并后的协方差矩阵
        Eigen::Matrix3d merge_covariance =
            (P_PT1 + P_PT2) /
                (merge_plane->points_size_ + origin_list[j]->points_size_) -
            merge_center * merge_center.transpose();

        merge_plane->covariance_ = merge_covariance;
        merge_plane->center_ = merge_center;
        merge_plane->points_size_ =
            merge_plane->points_size_ + origin_list[j]->points_size_;
        merge_plane->sub_plane_num_++;

        // 重新计算合并后平面的法向量
        Eigen::EigenSolver<Eigen::Matrix3d> es(merge_plane->covariance_);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);

        // 更新平面参数
        merge_plane->normal_ << evecs.real()(0, evalsMin),
            evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        merge_plane->radius_ = sqrt(evalsReal(evalsMax));
        merge_plane->d_ = -(merge_plane->normal_(0) * merge_plane->center_(0) +
                            merge_plane->normal_(1) * merge_plane->center_(1) +
                            merge_plane->normal_(2) * merge_plane->center_(2));
        merge_plane->p_center_.x = merge_plane->center_(0);
        merge_plane->p_center_.y = merge_plane->center_(1);
        merge_plane->p_center_.z = merge_plane->center_(2);
        merge_plane->p_center_.normal_x = merge_plane->normal_(0);
        merge_plane->p_center_.normal_y = merge_plane->normal_(1);
        merge_plane->p_center_.normal_z = merge_plane->normal_(2);
      }
    }
    if (is_merge) {
      merge_flag.push_back(merge_plane->id_);
      merge_list.push_back(merge_plane);
    }
  }
  project_plane_list = merge_list;
}

/**
 * @brief 合并相似平面
 * @param origin_list 原始平面列表
 * @param merge_plane_list 输出合并后的平面列表
 *
 * 根据法向量和距离阈值合并相似平面，减少平面数量
 */
void STDescManager::merge_plane(std::vector<BTCPlane *> &origin_list,
                                std::vector<BTCPlane *> &merge_plane_list) {
  if (origin_list.size() == 1) {
    merge_plane_list = origin_list;
    return;
  }
  for (size_t i = 0; i < origin_list.size(); i++)
    origin_list[i]->id_ = 0;
  int current_id = 1;
  for (auto iter = origin_list.end() - 1; iter != origin_list.begin(); iter--) 
  {
    for (auto iter2 = origin_list.begin(); iter2 != iter; iter2++) 
    {
      Eigen::Vector3d normal_diff = (*iter)->normal_ - (*iter2)->normal_;
      Eigen::Vector3d normal_add = (*iter)->normal_ + (*iter2)->normal_;
      double dis1 =
          fabs((*iter)->normal_(0) * (*iter2)->center_(0) +
               (*iter)->normal_(1) * (*iter2)->center_(1) +
               (*iter)->normal_(2) * (*iter2)->center_(2) + (*iter)->d_);
      double dis2 =
          fabs((*iter2)->normal_(0) * (*iter)->center_(0) +
               (*iter2)->normal_(1) * (*iter)->center_(1) +
               (*iter2)->normal_(2) * (*iter)->center_(2) + (*iter2)->d_);
      if (normal_diff.norm() < config_setting_.plane_merge_normal_thre_ ||
          normal_add.norm() < config_setting_.plane_merge_normal_thre_)
        if (dis1 < config_setting_.plane_merge_dis_thre_ &&
            dis2 < config_setting_.plane_merge_dis_thre_) {
          if ((*iter)->id_ == 0 && (*iter2)->id_ == 0) {
            (*iter)->id_ = current_id;
            (*iter2)->id_ = current_id;
            current_id++;
          } else if ((*iter)->id_ == 0 && (*iter2)->id_ != 0)
            (*iter)->id_ = (*iter2)->id_;
          else if ((*iter)->id_ != 0 && (*iter2)->id_ == 0)
            (*iter2)->id_ = (*iter)->id_;
        }
    }
  }
  std::vector<int> merge_flag;

  for (size_t i = 0; i < origin_list.size(); i++) {
    auto it =
        std::find(merge_flag.begin(), merge_flag.end(), origin_list[i]->id_);
    if (it != merge_flag.end())
      continue;
    if (origin_list[i]->id_ == 0) {
      merge_plane_list.push_back(origin_list[i]);
      continue;
    }
    BTCPlane *merge_plane = new BTCPlane;
    (*merge_plane) = (*origin_list[i]);
    bool is_merge = false;
    for (size_t j = 0; j < origin_list.size(); j++) {
      if (i == j)
        continue;
      if (origin_list[j]->id_ == origin_list[i]->id_) {
        is_merge = true;
        Eigen::Matrix3d P_PT1 =
            (merge_plane->covariance_ +
             merge_plane->center_ * merge_plane->center_.transpose()) *
            merge_plane->points_size_;
        Eigen::Matrix3d P_PT2 =
            (origin_list[j]->covariance_ +
             origin_list[j]->center_ * origin_list[j]->center_.transpose()) *
            origin_list[j]->points_size_;
        Eigen::Vector3d merge_center =
            (merge_plane->center_ * merge_plane->points_size_ +
             origin_list[j]->center_ * origin_list[j]->points_size_) /
            (merge_plane->points_size_ + origin_list[j]->points_size_);
        Eigen::Matrix3d merge_covariance =
            (P_PT1 + P_PT2) /
                (merge_plane->points_size_ + origin_list[j]->points_size_) -
            merge_center * merge_center.transpose();
        merge_plane->covariance_ = merge_covariance;
        merge_plane->center_ = merge_center;
        merge_plane->points_size_ =
            merge_plane->points_size_ + origin_list[j]->points_size_;
        merge_plane->sub_plane_num_ += origin_list[j]->sub_plane_num_;
        // for (size_t k = 0; k < origin_list[j]->cloud.size(); k++) {
        //   merge_plane->cloud.points.push_back(origin_list[j]->cloud.points[k]);
        // }
        Eigen::EigenSolver<Eigen::Matrix3d> es(merge_plane->covariance_);
        Eigen::Matrix3cd evecs = es.eigenvectors();
        Eigen::Vector3cd evals = es.eigenvalues();
        Eigen::Vector3d evalsReal;
        evalsReal = evals.real();
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff(&evalsMin);
        evalsReal.rowwise().sum().maxCoeff(&evalsMax);
        Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
        merge_plane->normal_ << evecs.real()(0, evalsMin),
            evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        merge_plane->radius_ = sqrt(evalsReal(evalsMax));
        merge_plane->d_ = -(merge_plane->normal_(0) * merge_plane->center_(0) +
                            merge_plane->normal_(1) * merge_plane->center_(1) +
                            merge_plane->normal_(2) * merge_plane->center_(2));
        merge_plane->p_center_.x = merge_plane->center_(0);
        merge_plane->p_center_.y = merge_plane->center_(1);
        merge_plane->p_center_.z = merge_plane->center_(2);
        merge_plane->p_center_.normal_x = merge_plane->normal_(0);
        merge_plane->p_center_.normal_y = merge_plane->normal_(1);
        merge_plane->p_center_.normal_z = merge_plane->normal_(2);
      }
    }
    if (is_merge) {
      merge_flag.push_back(merge_plane->id_);
      merge_plane_list.push_back(merge_plane);
    }
  }
}

/**
 * @brief 提取二进制描述符
 * @param proj_plane_list 投影平面列表
 * @param input_cloud 输入点云
 * @param binary_descriptor_list 输出二进制描述符列表
 *
 * 算法步骤：
 * 1. 选择若干个投影平面（法向量不同的平面）
 * 2. 对每个平面提取二进制描述符
 * 3. 非极大值抑制去除冗余描述符
 * 4. 按汇总值排序，选择前N个最显著的描述符
 */
void STDescManager::binary_extractor(
    const std::vector<BTCPlane *> proj_plane_list,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_descriptor_list)
{
  binary_descriptor_list.clear();
  std::vector<BinaryDescriptor> temp_binary_list;
  Eigen::Vector3d last_normal(0, 0, 0);
  int useful_proj_num = 0;

  // 遍历投影平面，提取二进制描述符
  for (int i = 0; i < proj_plane_list.size(); i++)
  {
    std::vector<BinaryDescriptor> prepare_binary_list;
    Eigen::Vector3d proj_center = proj_plane_list[i]->center_;
    Eigen::Vector3d proj_normal = proj_plane_list[i]->normal_;

    // 选择法向量差异较大的平面
    if ((proj_normal - last_normal).norm() < 0.3 ||
        (proj_normal + last_normal).norm() > 0.3) {
      last_normal = proj_normal;
      useful_proj_num++;

      // 提取该平面的二进制描述符
      extract_binary(proj_center, proj_normal, input_cloud,
                     prepare_binary_list);

      for (auto &bi : prepare_binary_list) {
        temp_binary_list.push_back(bi);
      }

      // 达到所需投影平面数量则停止
      if (useful_proj_num == config_setting_.proj_plane_num_) {
        break;
      }
    }
  }

  // 非极大值抑制去除冗余描述符
  non_maxi_suppression(temp_binary_list);

  // 选择前N个最显著的描述符
  if (config_setting_.useful_corner_num_ > temp_binary_list.size()) {
    binary_descriptor_list = temp_binary_list;
  } else {
    std::sort(temp_binary_list.begin(), temp_binary_list.end(),
              binary_greater_sort);
    for (size_t i = 0; i < config_setting_.useful_corner_num_; i++) {
      binary_descriptor_list.push_back(temp_binary_list[i]);
    }
  }
  return;
}

/**
 * @brief 从单个投影平面提取二进制描述符
 * @param project_center 投影平面中心
 * @param project_normal 投影平面法向量
 * @param input_cloud 输入点云
 * @param binary_list 输出二进制描述符列表
 *
 * 算法核心思想：
 * 1. 将3D点云投影到2D平面
 * 2. 在2D平面上构建网格图像
 * 3. 统计每个网格在不同高度层的点分布
 * 4. 生成二进制占用数组作为描述符
 * 5. 通过非极大值抑制选择关键点
 */
void STDescManager::extract_binary(
    const Eigen::Vector3d &project_center,
    const Eigen::Vector3d &project_normal,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud,
    std::vector<BinaryDescriptor> &binary_list)
{
  binary_list.clear();
  double binary_min_dis = config_setting_.summary_min_thre_;
  double resolution = config_setting_.proj_image_resolution_;
  double dis_threshold_min = config_setting_.proj_dis_min_;
  double dis_threshold_max = config_setting_.proj_dis_max_;
  double high_inc = config_setting_.proj_image_high_inc_;
  bool line_filter_enable = config_setting_.line_filter_enable_;

  // 平面方程: Ax + By + Cz + D = 0
  double A = project_normal[0];
  double B = project_normal[1];
  double C = project_normal[2];
  double D =
      -(A * project_center[0] + B * project_center[1] + C * project_center[2]);
  // 构建投影平面的2D坐标系
  std::vector<Eigen::Vector3d> projection_points;

  // 计算平面的X轴（平面内的一个方向）
  Eigen::Vector3d x_axis(1, 1, 0);
  if (C != 0) {
    x_axis[2] = -(A + B) / C;
  } else if (B != 0) {
    x_axis[1] = -A / B;
  } else {
    x_axis[0] = 0;
    x_axis[1] = 1;
  }
  x_axis.normalize();

  // Y轴 = 法向量 × X轴
  Eigen::Vector3d y_axis = project_normal.cross(x_axis);
  y_axis.normalize();
  // X轴和Y轴的平面方程参数
  double ax = x_axis[0];
  double bx = x_axis[1];
  double cx = x_axis[2];
  double dx = -(ax * project_center[0] + bx * project_center[1] +
                cx * project_center[2]);
  double ay = y_axis[0];
  double by = y_axis[1];
  double cy = y_axis[2];
  double dy = -(ay * project_center[0] + by * project_center[1] +
                cy * project_center[2]);

  // 将3D点投影到2D平面
  std::vector<Eigen::Vector2d> point_list_2d;
  pcl::PointCloud<pcl::PointXYZ> point_list_3d;
  std::vector<double> dis_list_2d;  // 点到投影平面的距离

  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    double x = input_cloud->points[i].x;
    double y = input_cloud->points[i].y;
    double z = input_cloud->points[i].z;

    // 计算点到投影平面的距离
    double dis = fabs(x * A + y * B + z * C + D);
    pcl::PointXYZ pi;

    // 距离滤波：只处理在阈值范围内的点
    if (dis < dis_threshold_min || dis > dis_threshold_max) {
      continue;
    } else {
      if (dis > dis_threshold_min && dis <= dis_threshold_max) {
        pi.x = x;
        pi.y = y;
        pi.z = z;
      }
    }

    // 计算点在投影平面上的投影点
    Eigen::Vector3d cur_project;
    cur_project[0] = (-A * (B * y + C * z + D) + x * (B * B + C * C)) /
                     (A * A + B * B + C * C);
    cur_project[1] = (-B * (A * x + C * z + D) + y * (A * A + C * C)) /
                     (A * A + B * B + C * C);
    cur_project[2] = (-C * (A * x + B * y + D) + z * (A * A + B * B)) /
                     (A * A + B * B + C * C);

    pcl::PointXYZ p;
    p.x = cur_project[0];
    p.y = cur_project[1];
    p.z = cur_project[2];

    // 将投影点转换到2D平面坐标系
    double project_x =
        cur_project[0] * ay + cur_project[1] * by + cur_project[2] * cy + dy;
    double project_y =
        cur_project[0] * ax + cur_project[1] * bx + cur_project[2] * cx + dx;

    Eigen::Vector2d p_2d(project_x, project_y);
    point_list_2d.push_back(p_2d);
    dis_list_2d.push_back(dis);
    point_list_3d.points.push_back(pi);
  }
  // 计算2D投影点的边界框
  double min_x = 10;
  double max_x = -10;
  double min_y = 10;
  double max_y = -10;

  if (point_list_2d.size() <= 5) {
    return;  // 点数太少，无法提取描述符
  }

  for (auto &pi : point_list_2d) {
    if (pi[0] < min_x) {
      min_x = pi[0];
    }
    if (pi[0] > max_x) {
      max_x = pi[0];
    }
    if (pi[1] < min_y) {
      min_y = pi[1];
    }
    if (pi[1] > max_y) {
      max_y = pi[1];
    }
  }

  // 分割投影点云为网格
  int segmen_base_num = 5;  // 每个段的基础网格数
  double segmen_len = segmen_base_num * resolution;
  int x_segment_num = (max_x - min_x) / segmen_len + 1;
  int y_segment_num = (max_y - min_y) / segmen_len + 1;
  int x_axis_len = (int)((max_x - min_x) / resolution + segmen_base_num);
  int y_axis_len = (int)((max_y - min_y) / resolution + segmen_base_num);

  // 创建2D网格数据结构
  std::vector<double> **dis_container = new std::vector<double> *[x_axis_len];  // 距离容器
  BinaryDescriptor **binary_container = new BinaryDescriptor *[x_axis_len];  // 二进制描述符容器
  for (int i = 0; i < x_axis_len; i++) {
    dis_container[i] = new std::vector<double>[y_axis_len];
    binary_container[i] = new BinaryDescriptor[y_axis_len];
  }
  double **img_count = new double *[x_axis_len];  // 每个网格的点数统计
  for (int i = 0; i < x_axis_len; i++) {
    img_count[i] = new double[y_axis_len];
  }
  double **dis_array = new double *[x_axis_len];  // 每个网格的距离数组
  for (int i = 0; i < x_axis_len; i++) {
    dis_array[i] = new double[y_axis_len];
  }
  double **mean_x_list = new double *[x_axis_len];  // 每个网格的X坐标均值
  for (int i = 0; i < x_axis_len; i++) {
    mean_x_list[i] = new double[y_axis_len];
  }
  double **mean_y_list = new double *[x_axis_len];  // 每个网格的Y坐标均值
  for (int i = 0; i < x_axis_len; i++) {
    mean_y_list[i] = new double[y_axis_len];
  }
  // 初始化网格数据
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      img_count[x][y] = 0;
      mean_x_list[x][y] = 0;
      mean_y_list[x][y] = 0;
      dis_array[x][y] = 0;
      std::vector<double> single_dis_container;
      dis_container[x][y] = single_dis_container;
    }
  }

  // 将点分配到对应的网格中
  for (size_t i = 0; i < point_list_2d.size(); i++) {
    int x_index = (int)((point_list_2d[i][0] - min_x) / resolution);
    int y_index = (int)((point_list_2d[i][1] - min_y) / resolution);
    mean_x_list[x_index][y_index] += point_list_2d[i][0];
    mean_y_list[x_index][y_index] += point_list_2d[i][1];
    img_count[x_index][y_index]++;
    dis_container[x_index][y_index].push_back(dis_list_2d[i]);
  }

  // 计算每个网格的二进制描述符
  for (int x = 0; x < x_axis_len; x++) {
    for (int y = 0; y < y_axis_len; y++) {
      // 只处理有点的网格
      if (img_count[x][y] > 0) {
        // 将距离范围分为多个高度层
        int cut_num = (dis_threshold_max - dis_threshold_min) / high_inc;
        std::vector<bool> occup_list;  // 二进制占用数组
        std::vector<double> cnt_list;  // 每层的点数统计
        BinaryDescriptor single_binary;

        for (size_t i = 0; i < cut_num; i++) {
          cnt_list.push_back(0);
          occup_list.push_back(false);
        }

        // 统计每个高度层的点数
        for (size_t j = 0; j < dis_container[x][y].size(); j++) {
          int cnt_index =
              (dis_container[x][y][j] - dis_threshold_min) / high_inc;
          cnt_list[cnt_index]++;
        }

        // 生成二进制占用数组：有点的层标记为true
        double segmnt_dis = 0;
        for (size_t i = 0; i < cut_num; i++) {
          if (cnt_list[i] >= 1) {
            segmnt_dis++;
            occup_list[i] = true;
          }
        }

        dis_array[x][y] = segmnt_dis;
        single_binary.occupy_array_ = occup_list;
        single_binary.summary_ = segmnt_dis;
        binary_container[x][y] = single_binary;
      }
    }
  }

  // 按距离滤波：在每个段中选择最大距离的网格
  std::vector<double> max_dis_list;
  std::vector<int> max_dis_x_index_list;
  std::vector<int> max_dis_y_index_list;

  // 遍历每个段
  for (int x_segment_index = 0; x_segment_index < x_segment_num;
       x_segment_index++) {
    for (int y_segment_index = 0; y_segment_index < y_segment_num;
         y_segment_index++) {
      double max_dis = 0;
      int max_dis_x_index = -10;
      int max_dis_y_index = -10;

      // 在段内查找最大距离的网格
      for (int x_index = x_segment_index * segmen_base_num;
           x_index < (x_segment_index + 1) * segmen_base_num; x_index++) {
        for (int y_index = y_segment_index * segmen_base_num;
             y_index < (y_segment_index + 1) * segmen_base_num; y_index++) {
          if (dis_array[x_index][y_index] > max_dis) {
            max_dis = dis_array[x_index][y_index];
            max_dis_x_index = x_index;
            max_dis_y_index = y_index;
          }
        }
      }

      // 如果距离超过阈值，则保留该网格
      if (max_dis >= binary_min_dis) {
        bool is_touch = true;
        if (config_setting_.touch_filter_enable_) {
          is_touch = binary_container[max_dis_x_index][max_dis_y_index]
                         .occupy_array_[0] ||
                     binary_container[max_dis_x_index][max_dis_y_index]
                         .occupy_array_[1] ||
                     binary_container[max_dis_x_index][max_dis_y_index]
                         .occupy_array_[2] ||
                     binary_container[max_dis_x_index][max_dis_y_index]
                         .occupy_array_[3];
        }

        if (is_touch) {
          max_dis_list.push_back(max_dis);
          max_dis_x_index_list.push_back(max_dis_x_index);
          max_dis_y_index_list.push_back(max_dis_y_index);
        }
      }
    }
  }
  // calc line or not
  std::vector<Eigen::Vector2i> direction_list;
  Eigen::Vector2i d(0, 1);
  direction_list.push_back(d);
  d << 1, 0;
  direction_list.push_back(d);
  d << 1, 1;
  direction_list.push_back(d);
  d << 1, -1;
  direction_list.push_back(d);
  for (size_t i = 0; i < max_dis_list.size(); i++) {
    Eigen::Vector2i p(max_dis_x_index_list[i], max_dis_y_index_list[i]);
    if (p[0] <= 0 || p[0] >= x_axis_len - 1 || p[1] <= 0 ||
        p[1] >= y_axis_len - 1) {
      continue;
    }
    bool is_add = true;

    if (line_filter_enable) {
      for (int j = 0; j < 4; j++) {
        Eigen::Vector2i p(max_dis_x_index_list[i], max_dis_y_index_list[i]);
        if (p[0] <= 0 || p[0] >= x_axis_len - 1 || p[1] <= 0 ||
            p[1] >= y_axis_len - 1) {
          continue;
        }
        Eigen::Vector2i p1 = p + direction_list[j];
        Eigen::Vector2i p2 = p - direction_list[j];
        double threshold = dis_array[p[0]][p[1]] - 3;
        if (dis_array[p1[0]][p1[1]] >= threshold) {
          if (dis_array[p2[0]][p2[1]] >= 0.5 * dis_array[p[0]][p[1]]) {
            is_add = false;
          }
        }
        if (dis_array[p2[0]][p2[1]] >= threshold) {
          if (dis_array[p1[0]][p1[1]] >= 0.5 * dis_array[p[0]][p[1]]) {
            is_add = false;
          }
        }
        if (dis_array[p1[0]][p1[1]] >= threshold) {
          if (dis_array[p2[0]][p2[1]] >= threshold) {
            is_add = false;
          }
        }
        if (dis_array[p2[0]][p2[1]] >= threshold) {
          if (dis_array[p1[0]][p1[1]] >= threshold) {
            is_add = false;
          }
        }
      }
    }
    if (is_add) {
      double px =
          mean_x_list[max_dis_x_index_list[i]][max_dis_y_index_list[i]] /
          img_count[max_dis_x_index_list[i]][max_dis_y_index_list[i]];
      double py =
          mean_y_list[max_dis_x_index_list[i]][max_dis_y_index_list[i]] /
          img_count[max_dis_x_index_list[i]][max_dis_y_index_list[i]];
      Eigen::Vector3d coord = py * x_axis + px * y_axis + project_center;
      pcl::PointXYZ pi;
      pi.x = coord[0];
      pi.y = coord[1];
      pi.z = coord[2];
      BinaryDescriptor single_binary =
          binary_container[max_dis_x_index_list[i]][max_dis_y_index_list[i]];
      single_binary.location_ = coord;
      binary_list.push_back(single_binary);
    }
  }
  for (int i = 0; i < x_axis_len; i++) {
    delete[] binary_container[i];
    delete[] dis_container[i];
    delete[] img_count[i];
    delete[] dis_array[i];
    delete[] mean_x_list[i];
    delete[] mean_y_list[i];
  }
  delete[] binary_container;
  delete[] dis_container;
  delete[] img_count;
  delete[] dis_array;
  delete[] mean_x_list;
  delete[] mean_y_list;
}

/**
 * @brief 非极大值抑制（Non-Maximum Suppression）
 * @param binary_list 二进制描述符列表（输入输出）
 *
 * 在空间邻域内抑制较弱的描述符，只保留局部最显著的特征
 * 算法步骤：
 * 1. 构建KD树用于快速邻域搜索
 * 2. 对每个描述符，搜索半径内的邻居
 * 3. 如果存在更强的邻居，则抑制当前描述符
 * 4. 只保留未被抑制的描述符
 */
void STDescManager::non_maxi_suppression(
    std::vector<BinaryDescriptor> &binary_list) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr prepare_key_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
  std::vector<int> pre_count_list;
  std::vector<bool> is_add_list;

  // 构建点云和统计数据
  for (auto &var : binary_list) {
    pcl::PointXYZ pi;
    pi.x = var.location_[0];
    pi.y = var.location_[1];
    pi.z = var.location_[2];
    prepare_key_cloud->push_back(pi);
    pre_count_list.push_back(var.summary_);
    is_add_list.push_back(true);
  }

  if(prepare_key_cloud->size() == 0) return;

  // 构建KD树
  kd_tree.setInputCloud(prepare_key_cloud);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  double radius = config_setting_.non_max_suppression_radius_;

  // 对每个点进行非极大值抑制
  for (size_t i = 0; i < prepare_key_cloud->size(); i++) {
    pcl::PointXYZ searchPoint = prepare_key_cloud->points[i];
    if (kd_tree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch,
                             pointRadiusSquaredDistance) > 0) {
      Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);

      // 检查邻域内是否有更强的特征
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j) {
        if (pointIdxRadiusSearch[j] == i) {
          continue;
        }
        // 如果当前描述符弱于或等于邻居，则抑制
        if (pre_count_list[i] <= pre_count_list[pointIdxRadiusSearch[j]]) {
          is_add_list[i] = false;
        }
      }
    }
  }

  // 只保留未被抑制的描述符
  std::vector<BinaryDescriptor> pass_binary_list;
  for (size_t i = 0; i < is_add_list.size(); i++) {
    if (is_add_list[i]) {
      pass_binary_list.push_back(binary_list[i]);
    }
  }
  binary_list.clear();
  for (auto &var : pass_binary_list) {
    binary_list.push_back(var);
  }
  return;
}

/**
 * @brief 生成稳定三角形描述符（STD）
 * @param binary_list 二进制描述符列表
 * @param frame_id 帧ID
 * @param std_list 输出STD描述符列表
 *
 * 算法核心思想：
 * 1. 从二进制描述符中选择K个最近邻
 * 2. 在近邻中组合三角形（C(K,3)种组合）
 * 3. 根据三角形边长排序并筛选有效三角形
 * 4. 使用边长作为哈希键去重
 * 5. 生成STD描述符（包含三个顶点的二进制描述符）
 */
void STDescManager::generate_std(
    const std::vector<BinaryDescriptor> &binary_list, const int &frame_id,
    std::vector<STD> &std_list) {
  double scale = 1.0 / config_setting_.std_side_resolution_;
  std::unordered_map<BTCVOXEL_LOC, bool> feat_map;  // 用于去重

  // 构建关键点云
  pcl::PointCloud<pcl::PointXYZ> key_cloud;
  for (auto &var : binary_list) {
    pcl::PointXYZ pi;
    pi.x = var.location_[0];
    pi.y = var.location_[1];
    pi.z = var.location_[2];
    key_cloud.push_back(pi);
  }

  // 构建KD树用于快速近邻搜索
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  kd_tree->setInputCloud(key_cloud.makeShared());
  int K = config_setting_.descriptor_near_num_;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // 对每个关键点，搜索K个最近邻并生成三角形
  for (size_t i = 0; i < key_cloud.size(); i++)
  {
    pcl::PointXYZ searchPoint = key_cloud.points[i];
    if (kd_tree->nearestKSearch(searchPoint, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0)
    {
      // 从K个近邻中选择3个点组成三角形
      for (int m = 1; m < K - 1; m++)
      {
        for (int n = m + 1; n < K; n++)
        {
          // 计算三角形的三条边长
          pcl::PointXYZ p1 = searchPoint;
          pcl::PointXYZ p2 = key_cloud.points[pointIdxNKNSearch[m]];
          pcl::PointXYZ p3 = key_cloud.points[pointIdxNKNSearch[n]];
          double a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                          pow(p1.z - p2.z, 2));
          double b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) +
                          pow(p1.z - p3.z, 2));
          double c = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) +
                          pow(p3.z - p2.z, 2));

          // 边长筛选：必须在允许的范围内
          if (a > config_setting_.descriptor_max_len_ ||
              b > config_setting_.descriptor_max_len_ ||
              c > config_setting_.descriptor_max_len_ ||
              a < config_setting_.descriptor_min_len_ ||
              b < config_setting_.descriptor_min_len_ ||
              c < config_setting_.descriptor_min_len_) {
            continue;
          }
          // 对三角形边长排序：a <= b <= c
          double temp;
          Eigen::Vector3d A, B, C;
          Eigen::Vector3i l1, l2, l3;  // 标记边对应的顶点
          Eigen::Vector3i l_temp;
          l1 << 1, 2, 0;  // 边a连接p1和p2
          l2 << 1, 0, 3;  // 边b连接p1和p3
          l3 << 0, 2, 3;  // 边c连接p2和p3

          // 冒泡排序边长
          if (a > b) {
            temp = a;
            a = b;
            b = temp;
            l_temp = l1;
            l1 = l2;
            l2 = l_temp;
          }
          if (b > c) {
            temp = b;
            b = c;
            c = temp;
            l_temp = l2;
            l2 = l3;
            l3 = l_temp;
          }
          if (a > b) {
            temp = a;
            a = b;
            b = temp;
            l_temp = l1;
            l1 = l2;
            l2 = l_temp;
          }

          // 退化三角形检测：接近直线的三角形不予考虑
          if (fabs(c - (a + b)) < 0.2) {
            continue;
          }

          pcl::PointXYZ d_p;
          d_p.x = a * 1000;
          d_p.y = b * 1000;
          d_p.z = c * 1000;
          BTCVOXEL_LOC position((int64_t)d_p.x, (int64_t)d_p.y, (int64_t)d_p.z);
          auto iter = feat_map.find(position);
          Eigen::Vector3d normal_1, normal_2, normal_3;
          BinaryDescriptor binary_A;
          BinaryDescriptor binary_B;
          BinaryDescriptor binary_C;
          if (iter == feat_map.end()) {
            if (l1[0] == l2[0]) {
              A << p1.x, p1.y, p1.z;
              binary_A = binary_list[i];
            } else if (l1[1] == l2[1]) {
              A << p2.x, p2.y, p2.z;
              binary_A = binary_list[pointIdxNKNSearch[m]];
            } else {
              A << p3.x, p3.y, p3.z;
              binary_A = binary_list[pointIdxNKNSearch[n]];
            }
            if (l1[0] == l3[0]) {
              B << p1.x, p1.y, p1.z;
              binary_B = binary_list[i];
            } else if (l1[1] == l3[1]) {
              B << p2.x, p2.y, p2.z;
              binary_B = binary_list[pointIdxNKNSearch[m]];
            } else {
              B << p3.x, p3.y, p3.z;
              binary_B = binary_list[pointIdxNKNSearch[n]];
            }
            if (l2[0] == l3[0]) {
              C << p1.x, p1.y, p1.z;
              binary_C = binary_list[i];
            } else if (l2[1] == l3[1]) {
              C << p2.x, p2.y, p2.z;
              binary_C = binary_list[pointIdxNKNSearch[m]];
            } else {
              C << p3.x, p3.y, p3.z;
              binary_C = binary_list[pointIdxNKNSearch[n]];
            }
            STD single_descriptor;
            single_descriptor.binary_A_ = binary_A;
            single_descriptor.binary_B_ = binary_B;
            single_descriptor.binary_C_ = binary_C;
            single_descriptor.center_ = (A + B + C) / 3;
            single_descriptor.triangle_ << scale * a, scale * b, scale * c;
            single_descriptor.angle_[0] = fabs(5 * normal_1.dot(normal_2));
            single_descriptor.angle_[1] = fabs(5 * normal_1.dot(normal_3));
            single_descriptor.angle_[2] = fabs(5 * normal_3.dot(normal_2));
            // single_descriptor.angle << 0, 0, 0;
            single_descriptor.frame_number_ = frame_id;
            // single_descriptor.score_frame_.push_back(frame_number);
            // Eigen::Matrix3d triangle_positon;
            // triangle_positon.block<3, 1>(0, 0) = A;
            // triangle_positon.block<3, 1>(0, 1) = B;
            // triangle_positon.block<3, 1>(0, 2) = C;
            // single_descriptor.position_list_.push_back(triangle_positon);
            // single_descriptor.triangle_scale_ = scale;
            feat_map[position] = true;
            std_list.push_back(single_descriptor);
          }
        }
      }
    }
  }
}

/**
 * @brief 候选帧选择（粗匹配）
 * @param current_STD_list 当前帧的STD描述符列表
 * @param candidate_matcher_vec 输出候选匹配列表
 *
 * 算法步骤：
 * 1. 在数据库中搜索与当前STD相似的历史STD
 * 2. 统计每个历史帧的匹配数量（投票）
 * 3. 选择投票数最高的前N个帧作为候选
 */
void STDescManager::candidate_selector(
    std::vector<STD> &current_STD_list,
    std::vector<STDMatchList> &candidate_matcher_vec)
{
  std::vector<double> match_array(plane_cloud_vec_.size(), 0);  // 每个历史帧的投票数
  std::vector<int> match_list_index;
  std::vector<Eigen::Vector3i> voxel_round;

  // 构建3x3x3的体素邻域（用于容错搜索）
  for (int x = -1; x <= 1; x++) {
    for (int y = -1; y <= 1; y++) {
      for (int z = -1; z <= 1; z++) {
        Eigen::Vector3i voxel_inc(x, y, z);
        voxel_round.push_back(voxel_inc);
      }
    }
  }
  std::vector<bool> useful_match(current_STD_list.size(), false);
  std::vector<std::vector<size_t>> useful_match_index(current_STD_list.size());
  std::vector<std::vector<STD_LOC>> useful_match_position(
      current_STD_list.size());
  std::vector<size_t> index(current_STD_list.size());
  // for (size_t i = 0; i < index.size(); ++i) {
  //   index[i] = i;
  //   useful_match[i] = false;
  // }
  // std::mutex mylock;
  // auto t0 = std::chrono::high_resolution_clock::now();

  // std::for_each(
  //     std::execution::par_unseq, index.begin(), index.end(),
  //     [&](const size_t &i) 
  // for(const size_t &i: index)
  // 遍历当前帧的所有STD，在数据库中搜索匹配
  for (size_t i = 0; i < current_STD_list.size(); ++i)
  {
    STD &descriptor = current_STD_list[i];
    STD_LOC position;

    // 计算距离阈值（自适应）
    double dis_threshold =
        descriptor.triangle_.norm() *
        config_setting_.rough_dis_threshold_;

    // 在体素邻域内搜索
    for (auto &voxel_inc : voxel_round)
    {
      position.x = (int)(descriptor.triangle_[0] + voxel_inc[0]);
      position.y = (int)(descriptor.triangle_[1] + voxel_inc[1]);
      position.z = (int)(descriptor.triangle_[2] + voxel_inc[2]);
      Eigen::Vector3d voxel_center((double)position.x + 0.5,
                                    (double)position.y + 0.5,
                                    (double)position.z + 0.5);

      // 只搜索距离较近的体素
      if ((descriptor.triangle_ - voxel_center).norm() < 1.5)
      {
        auto iter = data_base_.find(position);
        if (iter != data_base_.end())
        {
          bool is_push_position = false;
          // 遍历该体素内的所有历史STD
          for (size_t j = 0; j < iter->second.size(); j++)
          {
            // 跳过时间上接近的帧（避免局部误匹配）
            if ((descriptor.frame_number_ -
                  iter->second[j].frame_number_) >
                config_setting_.skip_near_num_)
            {
              // 检查三角形边长是否相似
              double dis =
                  (descriptor.triangle_ - iter->second[j].triangle_)
                      .norm();
              if (dis < dis_threshold) {
                // 计算二进制描述符相似度
                double similarity =
                    (binary_similarity(descriptor.binary_A_,
                                        iter->second[j].binary_A_) +
                      binary_similarity(descriptor.binary_B_,
                                        iter->second[j].binary_B_) +
                      binary_similarity(descriptor.binary_C_,
                                        iter->second[j].binary_C_)) /
                    3;

                // 相似度超过阈值，记录为有效匹配
                if (similarity > config_setting_.similarity_threshold_) {
                  useful_match[i] = true;
                  useful_match_position[i].push_back(position);
                  useful_match_index[i].push_back(j);
                }
              }
            }
          }
        }
      }
    }
  }
  // );
  std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>>
      index_recorder;
  
  for (size_t i = 0; i < useful_match.size(); i++) 
  {
    if (useful_match[i]) 
    {
      for (size_t j = 0; j < useful_match_index[i].size(); j++) 
      {
        match_array[data_base_[useful_match_position[i][j]]
                              [useful_match_index[i][j]]
                                  .frame_number_] += 1;
        Eigen::Vector2i match_index(i, j);
        index_recorder.push_back(match_index);
        // match_list.push_back(single_match_pair);
        match_list_index.push_back(
            data_base_[useful_match_position[i][j]][useful_match_index[i][j]]
                .frame_number_);
      }
    }
  }

  // 选择投票数最高的前N个候选帧
  for (int cnt = 0; cnt < config_setting_.candidate_num_; cnt++)
  {
    // 找到投票数最高的帧
    double max_vote_index = std::max_element(match_array.begin(), match_array.end()) - match_array.begin();
    int max_vote = match_array[max_vote_index];

    STDMatchList match_triangle_list;

    // 投票数必须超过阈值（至少5个匹配）
    if (max_vote_index >= 0 && max_vote >= 5)
    {
      // 清零该帧的投票，以便选择下一个候选
      match_array[max_vote_index] = 0;
      match_triangle_list.match_frame_ = max_vote_index;
      match_triangle_list.match_id_.first = current_frame_id_;
      match_triangle_list.match_id_.second = max_vote_index;
      double mean_dis = 0;

      // 收集该候选帧的所有匹配对
      for (size_t i = 0; i < index_recorder.size(); i++)
      {
        if (match_list_index[i] == max_vote_index)
        {
          std::pair<STD, STD> single_match_pair;
          single_match_pair.first = current_STD_list[index_recorder[i][0]];
          single_match_pair.second =
              data_base_[useful_match_position[index_recorder[i][0]]
                                              [index_recorder[i][1]]]
                        [useful_match_index[index_recorder[i][0]]
                                           [index_recorder[i][1]]];
          match_triangle_list.match_list_.push_back(single_match_pair);
        }
      }
      candidate_matcher_vec.push_back(match_triangle_list);
    }
    else
      break;  // 没有更多有效候选，退出

  }
}

/**
 * @brief 候选帧验证（精匹配）
 * @param candidate_matcher 候选匹配列表
 * @param verify_score 输出验证分数
 * @param relative_pose 输出相对位姿（平移，旋转）
 * @param sucess_match_list 输出成功匹配的STD对
 * @param pl_cur 当前帧的平面点云
 *
 * 算法步骤：
 * 1. RANSAC框架：从候选匹配中采样
 * 2. 对每个样本，计算相对位姿
 * 3. 验证位姿的内点数量（投票）
 * 4. 选择内点最多的位姿
 * 5. 使用平面点云进行几何一致性检查
 */
void STDescManager::candidate_verify(
    STDMatchList &candidate_matcher, double &verify_score,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &relative_pose,
    std::vector<std::pair<STD, STD>> &sucess_match_list, pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_cur)
{
  sucess_match_list.clear();
  double dis_threshold = 3;  // 内点距离阈值

  // RANSAC采样：跳跃式选择样本
  int skip_len = (int)(candidate_matcher.match_list_.size() / 50) + 1;
  int use_size = candidate_matcher.match_list_.size() / skip_len;

  int max_vote_index = 0;
  int max_vote = 0;
  Eigen::Matrix3d best_rot;
  Eigen::Vector3d best_t;
  // for (size_t i = 0; i < index.size(); i++) {
  //   index[i] = i;
  // }
  // std::mutex mylock;
  // auto t0 = std::chrono::high_resolution_clock::now();
  // std::for_each(
  //     std::execution::par_unseq, index.begin(), index.end(),
  //     [&](const size_t &i) 
  // for(const size_t &i: index)
  // RANSAC循环：测试每个采样的匹配对
  for (size_t i = 0; i < use_size; i++)
  {
    auto &single_pair = candidate_matcher.match_list_[i * skip_len];
    int vote = 0;
    Eigen::Matrix3d test_rot;
    Eigen::Vector3d test_t;

    // 根据单个STD匹配对计算相对位姿
    triangle_solver(single_pair, test_t, test_rot);

    // 用该位姿验证所有匹配对，统计内点数
    for (size_t j = 0; j < candidate_matcher.match_list_.size(); j++)
    {
      auto &verify_pair = candidate_matcher.match_list_[j];

      // 变换当前帧的三个顶点
      Eigen::Vector3d A_transform = test_rot * verify_pair.first.binary_A_.location_ + test_t;
      if((A_transform - verify_pair.second.binary_A_.location_).norm() >= dis_threshold)
        continue;

      Eigen::Vector3d B_transform = test_rot * verify_pair.first.binary_B_.location_ + test_t;
      if((B_transform - verify_pair.second.binary_B_.location_).norm() >= dis_threshold)
        continue;

      Eigen::Vector3d C_transform = test_rot * verify_pair.first.binary_C_.location_ + test_t;
      if((C_transform - verify_pair.second.binary_C_.location_).norm() < dis_threshold)
        vote++;  // 三个顶点都接近，则计为内点
    }

    // 更新最佳位姿
    if (max_vote < vote) {
      max_vote_index = i;
      max_vote = vote;
      best_rot = test_rot;
      best_t = test_t;
    }
  }
  // );

  // for (size_t i = 0; i < vote_list.size(); i++) {
  //   if (max_vote < vote_list[i]) {
  //     max_vote_index = i;
  //     max_vote = vote_list[i];
  //   }
  // }
  // printf("maxvote: %d %d\n", max_vote_index, max_vote);

  // 内点数量必须超过阈值
  if (max_vote >= 4)
  {
    relative_pose.first = best_t;
    relative_pose.second = best_rot;

    // 使用平面点云进行几何一致性检查
    verify_score = plane_geometric_verify(
        pl_cur,
        plane_cloud_vec_[candidate_matcher.match_id_.second], relative_pose);
  } else {
    verify_score = -1;  // 内点太少，验证失败
  }
  return;
}

/**
 * @brief 三角形求解器：根据匹配的三角形对计算相对位姿
 * @param std_pair STD匹配对（源帧和目标帧）
 * @param t 输出平移向量
 * @param rot 输出旋转矩阵
 *
 * 使用SVD分解求解刚体变换（类似ICP的闭式解）
 * 算法：Arun's method / Kabsch algorithm
 */
void STDescManager::triangle_solver(std::pair<STD, STD> &std_pair,
                                    Eigen::Vector3d &t, Eigen::Matrix3d &rot) {
  // 构建源点集和目标点集（相对于各自中心）
  Eigen::Matrix3d src = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d ref = Eigen::Matrix3d::Zero();
  src.col(0) = std_pair.first.binary_A_.location_ - std_pair.first.center_;
  src.col(1) = std_pair.first.binary_B_.location_ - std_pair.first.center_;
  src.col(2) = std_pair.first.binary_C_.location_ - std_pair.first.center_;
  ref.col(0) = std_pair.second.binary_A_.location_ - std_pair.second.center_;
  ref.col(1) = std_pair.second.binary_B_.location_ - std_pair.second.center_;
  ref.col(2) = std_pair.second.binary_C_.location_ - std_pair.second.center_;

  // 计算协方差矩阵
  Eigen::Matrix3d covariance = src * ref.transpose();

  // SVD分解
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeThinU |
                                                        Eigen::ComputeThinV);
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d U = svd.matrixU();

  // 计算旋转矩阵
  rot = V * U.transpose();

  // 处理镜像情况（行列式为负）
  if (rot.determinant() < 0) {
    Eigen::Matrix3d K;
    K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
    rot = V * K * U.transpose();
  }

  // 计算平移向量
  t = -rot * std_pair.first.center_ + std_pair.second.center_;
}

/**
 * @brief 平面几何一致性验证
 * @param source_cloud 源帧平面点云
 * @param target_cloud 目标帧平面点云
 * @param transform 相对位姿变换
 * @return 匹配分数（有效匹配比例）
 *
 * 使用平面点云验证位姿的几何一致性
 * 算法步骤：
 * 1. 用位姿变换源点云
 * 2. 在目标点云中搜索最近邻
 * 3. 检查法向量一致性和点到平面距离
 * 4. 统计有效匹配比例
 */
double STDescManager::plane_geometric_verify(
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &source_cloud,
    const pcl::PointCloud<pcl::PointXYZINormal>::Ptr &target_cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform) {
  Eigen::Vector3d t = transform.first;
  Eigen::Matrix3d rot = transform.second;

  // 构建目标点云的KD树
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kd_tree(
      new pcl::KdTreeFLANN<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < target_cloud->size(); i++) {
    pcl::PointXYZ pi;
    pi.x = target_cloud->points[i].x;
    pi.y = target_cloud->points[i].y;
    pi.z = target_cloud->points[i].z;
    input_cloud->push_back(pi);
  }

  kd_tree->setInputCloud(input_cloud);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  double useful_match = 0;
  double normal_threshold = config_setting_.normal_threshold_;
  double dis_threshold = config_setting_.dis_threshold_;

  // 遍历源点云的每个平面
  for (size_t i = 0; i < source_cloud->size(); i++) {
    pcl::PointXYZINormal searchPoint = source_cloud->points[i];
    pcl::PointXYZ use_search_point;
    use_search_point.x = searchPoint.x;
    use_search_point.y = searchPoint.y;
    use_search_point.z = searchPoint.z;

    // 变换点位置
    Eigen::Vector3d pi(searchPoint.x, searchPoint.y, searchPoint.z);
    pi = rot * pi + t;
    use_search_point.x = pi[0];
    use_search_point.y = pi[1];
    use_search_point.z = pi[2];

    // 变换法向量
    Eigen::Vector3d ni(searchPoint.normal_x, searchPoint.normal_y,
                       searchPoint.normal_z);
    ni = rot * ni;

    // 在目标点云中搜索最近邻
    if (kd_tree->nearestKSearch(use_search_point, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0) {
      pcl::PointXYZINormal nearstPoint =
          target_cloud->points[pointIdxNKNSearch[0]];
      Eigen::Vector3d tpi(nearstPoint.x, nearstPoint.y, nearstPoint.z);
      Eigen::Vector3d tni(nearstPoint.normal_x, nearstPoint.normal_y,
                          nearstPoint.normal_z);

      // 检查法向量一致性
      Eigen::Vector3d normal_inc = ni - tni;
      Eigen::Vector3d normal_add = ni + tni;

      // 计算点到平面距离
      double point_to_plane = fabs(tni.transpose() * (pi - tpi));

      // 法向量接近且点到平面距离小，则计为有效匹配
      if ((normal_inc.norm() < normal_threshold ||
           normal_add.norm() < normal_threshold) &&
          point_to_plane < dis_threshold) {
        useful_match++;
      }
    }
  }

  // 返回有效匹配比例
  return useful_match / source_cloud->size();
}
