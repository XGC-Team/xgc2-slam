/**
 * @file Teammate.hpp
 * @brief 队友类定义 - 用于管理集群中其他无人机的信息和通信状态
 */

#ifndef TEAMMATE_HPP
#define TEAMMATE_HPP
#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include "fmt/color.h"
#include "udp_bridge/protocol.h"
#include "algorithm"
#include <string>
#include <fstream>

using namespace std;
using namespace udp_bridge;

/**
 * @class Teammate
 * @brief 队友类 - 存储和管理队友无人机的网络信息、时间同步状态和连接状态
 */
class Teammate{
public:
    /**
     * @brief 构造函数 - 初始化队友对象
     * @param ip 队友的IP地址
     * @param id 队友的无人机ID
     * @param rcv_time 首次接收消息的时间
     * @param start_time 队友UDP模块启动时间
     */
    Teammate(const string &ip, const int &id, const double &rcv_time, const double &start_time){
        ip_ = ip;
        id_ = id;
        last_rcv_time_ = rcv_time;
        udp_send_fd_ptr_ = -1;
        offset_time_ = 0.0;
        offset_ts_.clear();
        delay_ts_.clear();
        sync_done_ = false;
        write_done_ = false;
        udp_start_time_ = start_time;
    }

    /**
     * @brief 检查队友连接状态
     * @param cur_time 当前时间
     * @return 如果2秒内收到过消息返回true，否则返回false
     */
    bool is_connect(double &cur_time){
        if(cur_time - last_rcv_time_ > 2.0)
            return false;  // 超过2秒未收到消息，认为连接断开
        else
            return true;
    }

    /**
     * @brief 析构函数
     */
    ~Teammate() = default;

    // 公共成员变量
    string ip_;                      // 队友的IP地址
    int id_;                         // 队友的无人机ID
    int udp_send_fd_ptr_;            // UDP发送套接字文件描述符
    double offset_time_;             // 时间偏移量(秒) - 相对于本地时钟的偏移
    vector<double> offset_ts_;       // 时间偏移采样序列 - 用于计算平均偏移
    vector<double> delay_ts_;        // 网络延迟采样序列 - 用于计算平均延迟
    bool sync_done_;                 // 时间同步是否完成标志
    bool write_done_;                // 偏移数据是否已写入文件标志
    double last_rcv_time_;           // 最后一次接收消息的时间
    sockaddr_in addr_udp_send_;      // UDP发送地址结构
    double udp_start_time_;          // 队友UDP模块启动时间

private:

};
#endif