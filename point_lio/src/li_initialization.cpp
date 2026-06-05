/**
 * @file li_initialization.cpp
 * @brief Point-LIO初始化模块实现文件
 *
 * 本文件实现了Point-LIO系统的初始化功能，主要包括：
 * 1. 激光雷达数据回调处理（支持标准PCL格式和Livox自定义格式）
 * 2. IMU数据回调处理
 * 3. 传感器数据同步与配准
 * 4. 数据缓冲区管理
 * 5. 时间戳同步处理
 *
 * 该模块是Point-LIO系统的数据输入前端，负责接收和预处理传感器数据，
 * 为后续的状态估计和建图提供同步的传感器测量数据。
 */

#include "li_initialization.h"

// 初始化状态标志位
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;                                                                     // 初始化帧数计数器
double time_lag_IMU_wtr_lidar = 0.0, move_start_time = 0.0, online_calib_starts_time = 0.0; // 时间偏移相关变量
double imu_first_time = 0.0;                                                                // IMU首帧时间戳
bool lose_lid = false;                                                                      // 激光雷达数据丢失标志
double timediff_imu_wrt_lidar = 0.0;                                                        // IMU相对于激光雷达的时间差
bool timediff_set_flg = false;                                                              // 时间差设置标志
V3D gravity_lio = V3D::Zero();                                                              // LIO系统重力向量
mutex mtx_buffer;                                                                           // 缓冲区互斥锁
sensor_msgs::Imu imu_last, imu_next;                                                        // 前后相邻的IMU数据
// sensor_msgs::Imu::ConstPtr imu_last_ptr;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());                                         // 连续帧点云指针
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];               // 性能统计数组

condition_variable sig_buffer;                                                              // 缓冲区条件变量
int scan_count = 0;                                                                         // 扫描计数器
int frame_ct = 0, wait_num = 0;                                                             // 帧计数器和等待数
std::mutex m_time;                                                                          // 时间互斥锁
bool lidar_pushed = false, imu_pushed = false;                                              // 数据推送标志
std::deque<PointCloudXYZI::Ptr>  lidar_buffer;                                              // 激光雷达数据缓冲队列
std::deque<double>               time_buffer;                                               // 时间戳缓冲队列
std::deque<sensor_msgs::Imu::Ptr> imu_deque;                                                // IMU数据缓冲队列

/**
 * @brief 标准点云消息回调函数
 *
 * 处理标准ROS PointCloud2格式的激光雷达数据，支持多种激光雷达类型
 * （VELO16, OUST64, HESAIxt32等）
 *
 * @param msg 输入的点云消息（ROS PointCloud2格式）
 *
 * 功能说明：
 * 1. 检查时间戳有效性，防止时间回退
 * 2. 根据配置选择切帧或连续帧模式处理点云
 * 3. 将处理后的点云数据存入缓冲区
 * 4. 统计预处理耗时
 */
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();  // 记录预处理开始时间

    // 检查时间戳是否回退，防止时间异常
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = msg->header.stamp.toSec();  // 更新最新激光雷达时间戳
    // printf("check lidar time %f\n", last_timestamp_lidar);
    // 自动同步IMU和激光雷达的时间差（可选功能，已注释）
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    // 根据雷达类型和配置选择处理方式
    // 切帧模式：将一帧数据切分成多个子帧（适用于VELO16, OUST64, HESAIxt32）
    if ((lidar_type == VELO16 || lidar_type == OUST64 || lidar_type == HESAIxt32) && cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        // 调用预处理器的切帧函数
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        // 将切分后的子帧依次存入缓冲区
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));  // 时间单位转换为秒
            timestamp_lidar.pop_front();
        }
    }
    else  // 非切帧模式
    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
    p_pre->process(msg, ptr);  // 标准点云预处理

    // 连续帧模式：将多帧点云合并为一个大帧
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar;  // 记录连续帧的起始时间
        }
        if (frame_ct < 10)  // 累积10帧数据
        {
            // 调整点的时间戳，将相对时间转换为相对于起始帧的时间
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else  // 累积满10帧后，将合并的点云存入缓冲区
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();  // 清空连续帧缓存
            frame_ct = 0;      // 重置帧计数器
        }
    }
    else  // 单帧模式：每帧独立处理
    {
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;  // 统计预处理耗时
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

/**
 * @brief Livox点云消息回调函数
 *
 * 处理Livox自定义格式的激光雷达数据（livox_ros_driver::CustomMsg）
 * Livox雷达采用非重复扫描模式，具有独特的数据格式
 *
 * @param msg 输入的Livox自定义点云消息
 *
 * 功能说明：
 * 1. 检查时间戳有效性，防止时间回退
 * 2. 根据配置选择切帧或连续帧模式处理点云
 * 3. 将处理后的点云数据存入缓冲区
 * 4. 统计预处理耗时
 */
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    // mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();  // 记录预处理开始时间
    scan_count ++;

    // 检查时间戳是否回退，防止时间异常
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
        // lidar_buffer.shrink_to_fit();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();  // 更新最新激光雷达时间戳
    // 自动同步IMU和激光雷达的时间差（可选功能，已注释）
    // if (abs(last_timestamp_imu - last_timestamp_lidar) > 1.0 && !timediff_set_flg && !imu_deque.empty()) {
    //     timediff_set_flg = true;
    //     timediff_imu_wrt_lidar = last_timestamp_imu - last_timestamp_lidar;
    //     printf("Self sync IMU and LiDAR, HARD time lag is %.10lf \n \n", timediff_imu_wrt_lidar);
    // }

    // 切帧模式：将一帧Livox数据切分成多个子帧
    if (cut_frame_init) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        // 调用预处理器的Livox切帧函数
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);

        // 将切分后的子帧依次存入缓冲区
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));  // 时间单位转换为秒
            timestamp_lidar.pop_front();
        }
    }
    else  // 非切帧模式
    {
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
    p_pre->process(msg, ptr);  // Livox点云预处理

    // 连续帧模式：将多帧点云合并为一个大帧
    if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar;  // 记录连续帧的起始时间
        }
        if (frame_ct < 10)  // 累积10帧数据
        {
            // 调整点的时间戳，将相对时间转换为相对于起始帧的时间
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else  // 累积满10帧后，将合并的点云存入缓冲区
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI(10000,1));
            // cout << "ptr div num:" << ptr_div->size() << endl;
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();  // 清空连续帧缓存
            frame_ct = 0;      // 重置帧计数器
        }
    }
    else  // 单帧模式：每帧独立处理
    {
        if (ptr->points.size() > 0)
        {
            lidar_buffer.emplace_back(ptr);
            time_buffer.emplace_back(msg->header.stamp.toSec());
        }
    }
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;  // 统计预处理耗时
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

/**
 * @brief IMU数据回调函数
 *
 * 接收并处理IMU传感器数据，进行时间戳校正后存入缓冲队列
 *
 * @param msg_in 输入的IMU消息
 *
 * 功能说明：
 * 1. 复制IMU消息以避免修改原始数据
 * 2. 校正IMU时间戳（补偿IMU与激光雷达的时间差）
 * 3. 检查时间戳有效性，防止时间回退
 * 4. 将有效的IMU数据存入缓冲队列
 */
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    // mtx_buffer.lock();

    // publish_count ++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));  // 复制IMU消息

    // 时间戳校正：减去IMU相对于激光雷达的时间差和额外的时间延迟
    msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);

    double timestamp = msg->header.stamp.toSec();
    // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

    // 检查时间戳是否回退，防止时间异常
    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        // cout << "check time:" << timestamp << ";" << last_timestamp_imu << endl;
        // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);

        // mtx_buffer.unlock();
        // sig_buffer.notify_all();
        return;
    }
    imu_deque.emplace_back(msg);      // 将IMU数据加入缓冲队列
    last_timestamp_imu = timestamp;   // 更新最新IMU时间戳
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
}

/**
 * @brief 传感器数据同步函数
 *
 * 将激光雷达和IMU数据进行时间同步，组成一个测量组（MeasureGroup）
 * 该函数是传感器融合的关键，确保激光雷达帧与对应时间段的IMU数据正确配对
 *
 * @param meas 输出的测量组，包含同步后的激光雷达和IMU数据
 * @return true 数据同步成功，可以进行后续处理
 * @return false 数据同步失败，需要等待更多数据
 *
 * 功能说明：
 * 1. 支持仅激光雷达模式（不使用IMU）
 * 2. 检查激光雷达点云有效性
 * 3. 计算激光雷达扫描的结束时间
 * 4. 收集激光雷达扫描期间的所有IMU数据
 * 5. 处理数据丢失的情况
 */
bool sync_packages(MeasureGroup &meas)
{
    {
    // 模式1：不使用IMU，仅激光雷达模式
    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {
            if (!lidar_pushed)
            {
                meas.lidar = lidar_buffer.front();
                meas.lidar_beg_time = time_buffer.front();
                lose_lid = false;

                // 检查点云是否有效（点数大于0）
                if(meas.lidar->points.size() < 1)
                {
                    cout << "lose lidar" << std::endl;
                    // return false;
                    lose_lid = true;
                }
                else
                {
                    // 计算点云扫描的结束时间
                    // 遍历所有点，找到最大的曲率值（存储的是相对扫描起始的时间戳）
                    double end_time = meas.lidar->points.back().curvature;
                    for (auto pt: meas.lidar->points)
                    {
                        if (pt.curvature > end_time)
                        {
                            end_time = pt.curvature;
                        }
                    }
                    // 计算绝对结束时间（起始时间 + 相对时间）
                    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
                    meas.lidar_last_time = lidar_end_time;
                }
                lidar_pushed = true;
            }

            // 从缓冲区移除已处理的数据
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            lidar_pushed = false;

            // 根据激光雷达数据有效性返回结果
            if (!lose_lid)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        return false;
    }

    // 模式2：使用IMU的激光雷达-惯性融合模式
    // 检查缓冲区是否有数据
    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }

    /*** 步骤1：推送一帧激光雷达扫描数据 ***/
    if(!lidar_pushed)
    {
        lose_lid = false;
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

        // 检查点云是否有效
        if(meas.lidar->points.size() < 1)
        {
            cout << "lose lidar" << endl;
            lose_lid = true;
            // lidar_buffer.pop_front();
            // time_buffer.pop_front();
            // return false;
        }
        else
        {
            // 计算点云扫描的结束时间
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            // cout << "check time lidar:" << end_time << endl;
            meas.lidar_last_time = lidar_end_time;
        }
        lidar_pushed = true;
    }

    // 检查是否有足够的IMU数据覆盖激光雷达扫描时间段
    // 情况1：正常扫描，需要IMU数据覆盖到扫描结束时间
    if (!lose_lid && (last_timestamp_imu < lidar_end_time))
    {
        return false;  // IMU数据不足，等待更多IMU数据
    }
    // 情况2：激光雷达数据丢失，至少需要一定时间间隔的IMU数据
    if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte)
    {
        return false;  // IMU数据不足，等待更多IMU数据
    }

    /*** 步骤2：正常扫描情况下，收集对应时间段的IMU数据 ***/
    if (!lose_lid && !imu_pushed)
    {
        /*** 从IMU缓冲区推送数据并弹出 ***/
        if (p_imu->imu_need_init_)  // 如果IMU需要初始化
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            imu_next = *(imu_deque.front());
            meas.imu.shrink_to_fit();

            // 收集所有时间戳小于激光雷达扫描结束时间的IMU数据
            while (imu_time < lidar_end_time)
            {
                meas.imu.emplace_back(imu_deque.front());  // 将IMU数据加入测量组
                imu_last = imu_next;
                imu_deque.pop_front();  // 从缓冲区移除已使用的IMU数据
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec();
                imu_next = *(imu_deque.front());
            }
        }
        imu_pushed = true;
    }

    /*** 步骤3：激光雷达数据丢失情况下，收集一定时间间隔的IMU数据 ***/
    if (lose_lid && !imu_pushed)
    {
        /*** 从IMU缓冲区推送数据并弹出 ***/
        if (p_imu->imu_need_init_)  // 如果IMU需要初始化
        {
            double imu_time = imu_deque.front()->header.stamp.toSec();
            meas.imu.shrink_to_fit();

            imu_next = *(imu_deque.front());
            // 收集激光雷达起始时间加上一定时间间隔内的IMU数据
            // 这样即使激光雷达数据丢失，系统仍可通过IMU进行状态预测
            while (imu_time < meas.lidar_beg_time + lidar_time_inte)
            {
                meas.imu.emplace_back(imu_deque.front());  // 将IMU数据加入测量组
                imu_last = imu_next;
                imu_deque.pop_front();  // 从缓冲区移除已使用的IMU数据
                if(imu_deque.empty()) break;
                imu_time = imu_deque.front()->header.stamp.toSec();
                imu_next = *(imu_deque.front());
            }
        }
        imu_pushed = true;
    }

    /*** 步骤4：清理缓冲区，重置标志位 ***/
    lidar_buffer.pop_front();  // 移除已处理的激光雷达数据
    time_buffer.pop_front();   // 移除对应的时间戳
    lidar_pushed = false;      // 重置激光雷达推送标志
    imu_pushed = false;        // 重置IMU推送标志
    return true;               // 数据同步成功
    }
}
