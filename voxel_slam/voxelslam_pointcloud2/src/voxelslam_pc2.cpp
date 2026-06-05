/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>

#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/point_cloud.h>
#include <rviz/properties/int_property.h>
#include <rviz/validate_floats.h>

#include "voxelslam_pc2.hpp"

namespace voxelslam_pointcloud2
{
/**
 * @brief PointCloud2Display构造函数
 *
 * 初始化点云显示器，创建PointCloudCommon对象用于处理点云数据的通用操作
 * PointCloudCommon负责点云的渲染、变换和属性管理
 */
PointCloud2Display::PointCloud2Display() : point_cloud_common_(new rviz::PointCloudCommon(this))
{
}

/**
 * @brief PointCloud2Display析构函数
 *
 * 清理资源：
 * 1. 取消订阅点云话题
 * 2. 释放PointCloudCommon对象的内存
 */
PointCloud2Display::~PointCloud2Display()
{
  PointCloud2Display::unsubscribe();
  delete point_cloud_common_;
}

/**
 * @brief 初始化点云显示器
 *
 * 该函数在显示器被添加到RViz时调用，执行以下初始化步骤：
 * 1. 设置消息回调队列为线程队列，确保消息处理不阻塞主线程
 * 2. 调用父类的初始化函数
 * 3. 初始化点云通用处理对象，传入显示上下文和场景节点
 */
void PointCloud2Display::onInitialize()
{
  // 使用线程队列处理接收到的消息，避免阻塞主线程
  update_nh_.setCallbackQueue(context_->getThreadedQueue());

  MFDClass::onInitialize();
  point_cloud_common_->initialize(context_, scene_node_);
}

/**
 * @brief 处理接收到的PointCloud2消息
 *
 * @param cloud 输入的点云消息（常量智能指针）
 *
 * 主要功能：
 * 1. 过滤点云中的NaN值（非数值），防止渲染错误和性能下降
 * 2. 验证点云数据的完整性和一致性
 * 3. 将有效的点云数据传递给PointCloudCommon进行渲染
 *
 * 注意：NaN值如果不过滤会导致点被放置在错误位置，影响渲染性能
 */
void PointCloud2Display::processMessage(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  // 过滤点云中的NaN值。如果NaN值传递到PointCloudBase，
  // 这些点会被放置在错误的位置，但仍会被处理和渲染，严重影响性能
  sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);

  // 查找x, y, z坐标字段在点云数据结构中的索引
  int32_t xi = rviz::findChannelIndex(cloud, "x");
  int32_t yi = rviz::findChannelIndex(cloud, "y");
  int32_t zi = rviz::findChannelIndex(cloud, "z");

  // 如果找不到x, y, z任何一个坐标字段，则无法处理该点云，直接返回
  if (xi == -1 || yi == -1 || zi == -1)
  {
    return;
  }

  // 获取x, y, z字段在每个点数据中的字节偏移量
  const uint32_t xoff = cloud->fields[xi].offset;
  const uint32_t yoff = cloud->fields[yi].offset;
  const uint32_t zoff = cloud->fields[zi].offset;
  // 每个点的数据大小（字节数）
  const uint32_t point_step = cloud->point_step;
  // 点云中的总点数（宽度 × 高度）
  const size_t point_count = cloud->width * cloud->height;

  // 验证数据完整性：检查数据大小是否与点数和点步长匹配
  if (point_count * point_step != cloud->data.size())
  {
    std::stringstream ss;
    ss << "Data size (" << cloud->data.size() << " bytes) does not match width (" << cloud->width
       << ") times height (" << cloud->height << ") times point_step (" << point_step
       << ").  Dropping message.";
    setStatusStd(rviz::StatusProperty::Error, "Message", ss.str());
    return;
  }

  // 为过滤后的点云分配与原始点云相同大小的内存
  filtered->data.resize(cloud->data.size());
  uint32_t output_count;

  if (point_count == 0)
  {
    output_count = 0;
  }
  else
  {
    // 输出缓冲区指针，指向过滤后点云数据的起始位置
    uint8_t* output_ptr = &filtered->data.front();
    // ptr: 当前处理的点的指针; ptr_end: 数据末尾; ptr_init: 待复制连续点的起始位置
    const uint8_t *ptr = &cloud->data.front(), *ptr_end = &cloud->data.back(), *ptr_init;
    // 记录连续有效点的数量，用于批量复制优化性能
    size_t points_to_copy = 0;

    // 遍历所有点，每次移动一个点的步长
    for (; ptr < ptr_end; ptr += point_step)
    {
      // 从原始数据中提取x, y, z坐标值
      float x = *reinterpret_cast<const float*>(ptr + xoff);
      float y = *reinterpret_cast<const float*>(ptr + yoff);
      float z = *reinterpret_cast<const float*>(ptr + zoff);

      // 验证x, y, z是否都是有效的浮点数（非NaN、非Inf）
      if (rviz::validateFloats(x) && rviz::validateFloats(y) && rviz::validateFloats(z))
      {
        if (points_to_copy == 0)
        {
          // 如果这是新的有效点序列的开始，记录起始位置
          ptr_init = ptr;
          points_to_copy = 1;
        }
        else
        {
          // 如果是连续的有效点，增加计数器
          ++points_to_copy;
        }
      }
      else
      {
        // 遇到无效点，需要将之前累积的有效点批量复制到输出
        if (points_to_copy)
        {
          // 批量复制所有累积的有效点数据
          memcpy(output_ptr, ptr_init, point_step * points_to_copy);
          output_ptr += point_step * points_to_copy;
          points_to_copy = 0;
        }
      }
    }

    // 处理最后剩余的有效点（如果存在）
    if (points_to_copy)
    {
      memcpy(output_ptr, ptr_init, point_step * points_to_copy);
      output_ptr += point_step * points_to_copy;
    }

    // 计算过滤后的点数
    output_count = (output_ptr - &filtered->data.front()) / point_step;
  }

  // 填充过滤后点云的元数据
  filtered->header = cloud->header;           // 复制时间戳和坐标系信息
  filtered->fields = cloud->fields;           // 复制字段定义（包含x,y,z等字段描述）
  filtered->data.resize(output_count * point_step);  // 调整数据大小为实际有效点的大小
  filtered->height = 1;                       // 设置为无序点云（高度为1）
  filtered->width = output_count;             // 宽度等于有效点的数量
  filtered->is_bigendian = cloud->is_bigendian;  // 保持字节序
  filtered->point_step = point_step;          // 每个点的字节步长
  filtered->row_step = output_count;          // 每行的点数（无序点云row_step等于width）

  // 如果有有效点，将过滤后的点云添加到渲染队列；否则清空显示
  if(output_count > 0)
    point_cloud_common_->addMessage(filtered);
  else
    point_cloud_common_->reset();

}


/**
 * @brief 更新点云显示
 *
 * @param wall_dt 自上次更新以来的实际时间间隔（秒）
 * @param ros_dt  自上次更新以来的ROS时间间隔（秒）
 *
 * 该函数在RViz的每个渲染帧中被调用，用于更新点云的显示状态
 */
void PointCloud2Display::update(float wall_dt, float ros_dt)
{
  point_cloud_common_->update(wall_dt, ros_dt);
}

/**
 * @brief 重置点云显示器
 *
 * 清除所有已显示的点云数据，将显示器恢复到初始状态
 * 该函数在用户点击"Reset"按钮或需要清空显示时被调用
 */
void PointCloud2Display::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

}  // namespace voxelslam_pointcloud2

// 使用pluginlib宏将该类导出为RViz插件
// 使得RViz能够动态加载此点云显示器
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(voxelslam_pointcloud2::PointCloud2Display, rviz::Display)
