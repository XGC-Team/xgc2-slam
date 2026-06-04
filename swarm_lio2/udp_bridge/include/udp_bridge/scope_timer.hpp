/**
 * @file scope_timer.hpp
 * @brief 性能计时工具 - 用于测量代码块执行时间
 * @author yunfan
 * @date 2021/3/19
 * @version 1.0.0
 */

#ifndef SCROPE_TIMER_HPP//SRC_POLY_VISUAL_UTILS_HPP
#define SCROPE_TIMER_HPP

#include <chrono>
#include "fmt/color.h"
#include "cstring"
#include "fstream"
#include "iostream"

using namespace std;

/**
 * @class TimeConsuming
 * @brief 时间消耗计时类 - RAII风格的性能计时器，支持自动或手动计时
 *
 * 使用方法:
 * 1. 自动计时: TimeConsuming timer("operation name"); (析构时自动打印)
 * 2. 手动计时: TimeConsuming timer("operation name"); ... timer.stop();
 * 3. 多次测量平均: TimeConsuming timer("operation name", 100); (除以repeat_time)
 */
class TimeConsuming {
public:
    /**
     * @brief 默认构造函数
     */
    TimeConsuming();

    /**
     * @brief 构造函数 - 用于重复测量的场景
     * @param msg 计时描述信息
     * @param repeat_time 重复次数，用于计算平均时间
     */
    TimeConsuming(string msg, int repeat_time) {
        repeat_time_ = repeat_time;
        msg_ = msg;
        tc_start = std::chrono::high_resolution_clock::now();
        has_shown = false;
		print_ = true;
    }

    /**
     * @brief 构造函数 - 标准计时场景
     * @param msg 计时描述信息
     * @param print_log 是否打印日志，默认为true
     */
    TimeConsuming(string msg, bool print_log = true) {
        msg_ = msg;
        repeat_time_ = 1;
        print_ = print_log;
        tc_start = std::chrono::high_resolution_clock::now();
        has_shown = false;
    }

    /**
     * @brief 析构函数 - 如果未手动调用stop()，则自动打印时间消耗
     * 根据时间长度自动选择合适的单位(ns/us/ms/s)
     */
    ~TimeConsuming() {
        if (!has_shown && enable_ && print_) {
            tc_end = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
            double t_us = (double) dt * 1e6 / repeat_time_;
            if (t_us < 1) {
                t_us *= 1000;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e6) {
                t_us /= 1e6;
                printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
            } else if (t_us > 1e3) {
                t_us /= 1e3;
                printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
            } else
                printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
        }
    }

    /**
     * @brief 设置计时器是否启用
     * @param enable 启用标志
     */
    void set_enbale(bool enable) {
        enable_ = enable;
    }

    /**
     * @brief 重新开始计时
     */
    void start() {
        tc_start = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief 停止计时并打印/返回结果
     * @return 经过的时间(秒)
     */
    double stop() {
        if (!enable_) { return 0; }
        has_shown = true;
        tc_end = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(tc_end - tc_start).count();
        if (!print_) {
            return dt;
        }
        double t_us = (double) dt * 1e6 / repeat_time_;
        if (t_us < 1) {
            t_us *= 1000;
            printf(" -- [TIMER] %s time consuming \033[32m %lf ns\033[0m\n", msg_.c_str(), t_us);
        } else if (t_us > 1e6) {
            t_us /= 1e6;
            printf(" -- [TIMER] %s time consuming \033[32m %lf s\033[0m\n", msg_.c_str(), t_us);
        } else if (t_us > 1e3) {
            t_us /= 1e3;
            printf(" -- [TIMER] %s time consuming \033[32m %lf ms\033[0m\n", msg_.c_str(), t_us);
        } else
            printf(" -- [TIMER] %s time consuming \033[32m %lf us\033[0m\n", msg_.c_str(), t_us);
        return dt;
    }

private:
    std::chrono::high_resolution_clock::time_point tc_start, tc_end;  // 开始和结束时间点
    string msg_;                 // 计时描述信息
    int repeat_time_{1};         // 重复次数，用于计算平均时间
    bool has_shown = false;      // 是否已显示结果标志
    bool enable_{true};          // 计时器是否启用
    bool print_{true};           // 是否打印输出

};

#endif //SRC_POLY_VISUAL_UTILS_HPP
