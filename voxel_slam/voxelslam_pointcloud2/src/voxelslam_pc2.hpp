/*
 * 版权所有 (c) 2008, Willow Garage, Inc.
 * 保留所有权利
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file voxelslam_pc2.hpp
 * @brief Voxel-SLAM点云显示插件头文件
 * @details 该文件定义了用于RViz的PointCloud2消息可视化显示类，
 *          专门用于Voxel-SLAM系统中的点云数据可视化
 */

// 头文件保护宏，防止重复包含
#ifndef VOXELSLAM_PC2_H
#define VOXELSLAM_PC2_H

// ROS传感器消息类型 - PointCloud2点云消息
#include <sensor_msgs/PointCloud2.h>

// RViz消息过滤显示基类
#include <rviz/message_filter_display.h>

// RViz命名空间前向声明
namespace rviz
{
class IntProperty;        // 整数属性类，用于配置参数
class PointCloudCommon;   // 点云通用处理类，封装点云渲染的公共功能
}

/**
 * @class PointCloud2Display
 * @brief 显示sensor_msgs::PointCloud2类型的点云数据
 *
 * 功能说明：
 * - 默认情况下，将点云的通道0视为强度值，并根据强度值进行着色
 * - 如果将通道名称设置为"rgb"，则将该通道解释为整数RGB值，
 *   其中r、g、b各占8位
 * - 支持多种点云渲染风格和颜色映射方式
 *
 * \class PointCloud2Display
 * \brief Displays a point cloud of type sensor_msgs::PointCloud2
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by
 * intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r,
 * g and b
 * all being 8 bits.
 */
namespace voxelslam_pointcloud2
{

/**
 * @class PointCloud2Display
 * @brief PointCloud2点云显示类
 * @details 继承自RViz的MessageFilterDisplay模板类，专门用于处理和显示
 *          sensor_msgs::PointCloud2类型的点云消息
 */
class PointCloud2Display : public rviz::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
  Q_OBJECT  // Qt元对象宏，用于支持信号槽机制
public:
  /**
   * @brief 构造函数
   * @details 初始化PointCloud2Display对象，设置默认参数
   */
  PointCloud2Display();

  /**
   * @brief 析构函数
   * @details 清理资源，释放点云数据和渲染对象
   */
  ~PointCloud2Display() override;

  /**
   * @brief 重置显示状态
   * @details 清除当前显示的所有点云数据，重置到初始状态
   *          重写自父类MessageFilterDisplay
   */
  void reset() override;

  /**
   * @brief 更新显示
   * @param wall_dt 墙上时钟时间增量（实际流逝的时间，单位：秒）
   * @param ros_dt ROS时钟时间增量（ROS系统时间，单位：秒）
   * @details 每帧调用一次，用于更新点云的渲染和动画效果
   *          重写自父类MessageFilterDisplay
   */
  void update(float wall_dt, float ros_dt) override;

protected:
  /**
   * @brief 初始化函数
   * @details 在Display被添加到RViz时调用，用于初始化属性和资源
   *          重写自MessageFilterDisplay基类
   */
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  void onInitialize() override;

  /**
   * @brief 处理单个点云消息
   * @param cloud 指向PointCloud2消息的常量智能指针
   * @details 当接收到新的点云消息时调用，负责解析和渲染点云数据
   *          重写自MessageFilterDisplay基类
   */
  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  void processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud) override;

  /**
   * @brief 点云通用处理对象指针
   * @details 封装了点云的渲染、颜色映射、样式设置等公共功能
   *          该对象负责实际的点云可视化工作
   */
  rviz::PointCloudCommon* point_cloud_common_;
};

} // namespace voxelslam_pointcloud2

#endif // VOXELSLAM_PC2_H
