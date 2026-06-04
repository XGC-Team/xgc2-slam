//
// Created by yunfan on 2021/3/19.
// Version: 1.0.0
//


#ifndef SCROPE_TIMER_HPP//SRC_POLY_VISUAL_UTILS_HPP
#define SCROPE_TIMER_HPP

#include <chrono>
#include "fmt/color.h"
#include "cstring"

using namespace std;

/**
 * @brief 时间消耗统计类
 * @details 用于测量代码块执行时间的工具类，支持以下功能：
 *          - 自动计时：构造时开始计时，析构时输出结果
 *          - 重复测量：可以设置重复次数以计算平均时间
 *          - 多单位输出：自动选择合适的时间单位（ns, us, ms, s）
 *          - 手动控制：支持手动开始/停止计时
 * @note 典型用法：在作用域开始处创建对象，离开作用域时自动输出耗时
 */
class TimeConsuming {
public:
    TimeConsuming();

    /**
     * @brief 构造函数（重复测量模式）
     * @param msg 计时器描述信息，用于标识当前测量的代码块
     * @param repeat_time 重复次数，用于计算平均时间
     * @details 创建计时器并立即开始计时，结果将除以重复次数得到平均值
     */
    TimeConsuming(string msg, int repeat_time) {
        repeat_time_ = repeat_time;
        msg_ = msg;
        tc_start = std::chrono::high_resolution_clock::now();
        has_shown = false;
    }

    /**
     * @brief 构造函数（标准模式）
     * @param msg 计时器描述信息
     * @param enable 是否启用计时器输出，默认为false（不输出）
     * @details 如果enable为true，析构时会自动输出耗时；否则需要手动调用stop()
     */
    TimeConsuming(string msg, bool enable = false) {
        msg_ = msg;
        repeat_time_ = 1;
        tc_start = std::chrono::high_resolution_clock::now();
        has_shown = !enable;
    }

    /**
     * @brief 析构函数
     * @details 如果计时结果尚未显示且计时器已启用，则自动计算并输出耗时
     *          根据时间长度自动选择合适的单位（ns, us, ms, s）
     *          输出格式：[TIMER] <msg> time consuming <time> <unit>（绿色高亮显示）
     */
    ~TimeConsuming() {
        if (!has_shown && enable_) {
            tc_end = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
            double t_us = (double) dt * 1e6 / repeat_time_;
            // 小于1微秒，使用纳秒显示
            if (t_us < 1) {
                t_us *= 1000;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e6) {  // 大于1秒，使用秒显示
                t_us /= 1e6;
                printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e3) {  // 大于1毫秒，使用毫秒显示
                t_us /= 1e3;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
            }else  // 默认使用微秒显示
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
        }
    }

    /**
     * @brief 设置计时器是否启用
     * @param enable true表示启用输出，false表示禁用输出
     * @details 可以在运行时动态控制是否输出计时结果
     */
    void set_enbale(bool enable){
        enable_ = enable;
    }

    /**
     * @brief 开始计时（或重新开始）
     * @details 记录当前高精度时间戳作为起始时间点
     */
    void start() {
        tc_start = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief 停止计时并输出结果
     * @return 返回耗时（单位：秒），如果计时器未启用则返回-1
     * @details 计算从start()到stop()之间的时间差，并根据时间长度自动选择合适的单位输出
     *          设置has_shown标志，防止析构函数重复输出
     */
    double stop() {
        if(!enable_){return -1;}
        tc_end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
        double t_us = (double) dt * 1e6 / repeat_time_;
        if(!has_shown){
            // 小于1微秒，使用纳秒显示
            if (t_us < 1) {
                t_us *= 1000;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e6) {  // 大于1秒，使用秒显示
                t_us /= 1e6;
                printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e3) {  // 大于1毫秒，使用毫秒显示
                t_us /= 1e3;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
            }else  // 默认使用微秒显示
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
        }
        has_shown = true;

        return dt;
    }

private:
    std::chrono::high_resolution_clock::time_point tc_start, tc_end;  // 开始和结束时间点（高精度）
    string msg_;           // 计时器描述信息
    int repeat_time_;      // 重复次数，用于计算平均时间
    bool has_shown = false;  // 标记是否已经显示过结果，防止重复输出
    bool enable_{true};    // 计时器是否启用
};

#endif //SRC_POLY_VISUAL_UTILS_HPP
