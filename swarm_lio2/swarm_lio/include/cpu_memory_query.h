//
// Created by fangcheng on 2023/1/5.
//

#ifndef SRC_CPU_MEMORY_QUERY_H
#define SRC_CPU_MEMORY_QUERY_H

#include <iostream>
#include <thread>
#include <chrono>
#include <string.h>

#ifdef WIN32
#include <windows.h>
#include <psapi.h>
//#include <tlhelp32.h>
#include <direct.h>
#include <process.h>
#else
#include <sys/stat.h>
#include <sys/sysinfo.h>
#include <sys/time.h>
#include <unistd.h>
#endif

/**
 * @namespace CpuMemoryQuery
 * @brief CPU和内存查询工具命名空间
 * @details 提供跨平台的系统资源监控功能，包括：
 *          - 获取进程ID
 *          - 查询进程CPU占用率
 *          - 查询进程内存使用量
 *          支持Windows和Linux两个平台
 */
namespace CpuMemoryQuery {
    /**
     * @brief 获取当前进程的PID
     * @return 当前进程的进程ID
     */
    inline int GetCurrentPid()
    {
        return getpid();
    }

// ============== Windows平台实现 ==============
#ifdef WIN32
    /**
     * @brief 将Windows FILETIME格式转换为64位整数
     * @param ftime 指向FILETIME结构的指针
     * @return 转换后的64位时间值（100纳秒为单位）
     * @details Windows系统时间格式转换，用于计算CPU使用时间
     */
static uint64_t convert_time_format(const FILETIME* ftime)
{
    LARGE_INTEGER li;

    li.LowPart = ftime->dwLowDateTime;
    li.HighPart = ftime->dwHighDateTime;
    return li.QuadPart;
}
#else
// ============== Linux平台实现 ==============
// FIXME: can also get cpu and mem status from popen cmd
// the info line num in /proc/{pid}/status file
#define VMRSS_LINE 22      // VmRSS（常驻内存）信息在status文件中的行号
#define PROCESS_ITEM 14    // /proc/{pid}/stat文件中的关键字段位置

    /**
     * @brief 从缓冲区中提取指定位置的项
     * @param buffer 输入缓冲区（通常是/proc文件的一行）
     * @param item 要提取的项的索引（从1开始）
     * @return 指向提取项起始位置的指针
     * @details 用于解析/proc/{pid}/stat文件，通过空格分隔符定位到指定字段
     */
    static const char* get_items(const char* buffer, unsigned int item)
    {
        // read from buffer by offset
        const char* p = buffer;

        int len = strlen(buffer);
        int count = 0;

        // 遍历缓冲区，通过空格计数定位到目标项
        for (int i = 0; i < len; i++)
        {
            if (' ' == *p)
            {
                count++;
                if (count == item - 1)
                {
                    p++;
                    break;
                }
            }
            p++;
        }

        return p;
    }

    /**
     * @brief 获取系统总CPU占用时间
     * @return CPU总占用时间（用户态+低优先级+系统态+空闲）
     * @details 读取/proc/stat文件获取CPU时间统计，用于计算CPU使用率的分母
     *          返回值单位为jiffies（通常1 jiffy = 10ms）
     */
    static inline unsigned long get_cpu_total_occupy()
    {
        // get total cpu use time

        // different mode cpu occupy time
        unsigned long user_time;     // 用户态时间
        unsigned long nice_time;     // 低优先级用户态时间
        unsigned long system_time;   // 系统态（内核）时间
        unsigned long idle_time;     // 空闲时间

        FILE* fd;
        char buff[1024] = { 0 };

        // 打开/proc/stat文件读取CPU统计信息
        fd = fopen("/proc/stat", "r");
        if (nullptr == fd)
            return 0;

        // 读取第一行（总CPU时间）
        char* temp = fgets(buff, sizeof(buff), fd);
        char name[64] = { 0 };
        sscanf(buff, "%s %ld %ld %ld %ld", name, &user_time, &nice_time, &system_time, &idle_time);
        fclose(fd);

        // 返回所有时间的总和
        return (user_time + nice_time + system_time + idle_time);
    }

    /**
     * @brief 获取指定进程的CPU占用时间
     * @param pid 进程ID
     * @return 该进程的CPU总占用时间（用户态+内核态+子进程时间）
     * @details 读取/proc/{pid}/stat文件获取进程CPU时间统计
     *          返回值单位为jiffies（通常1 jiffy = 10ms）
     */
    static inline unsigned long get_cpu_proc_occupy(int pid)
    {
        // get specific pid cpu use time
        unsigned int tmp_pid;
        unsigned long utime;  // user time（用户态时间）
        unsigned long stime;  // kernel time（内核态时间）
        unsigned long cutime; // all user time（子进程用户态时间）
        unsigned long cstime; // all dead time（子进程内核态时间）

        char file_name[64] = { 0 };
        FILE* fd;
        char line_buff[1024] = { 0 };
        sprintf(file_name, "/proc/%d/stat", pid);

        // 打开/proc/{pid}/stat文件
        fd = fopen(file_name, "r");
        if (nullptr == fd)
            return 0;

        char* temp = fgets(line_buff, sizeof(line_buff), fd);

        // 解析进程ID和CPU时间字段
        sscanf(line_buff, "%u", &tmp_pid);
        const char* q = get_items(line_buff, PROCESS_ITEM);
        sscanf(q, "%ld %ld %ld %ld", &utime, &stime, &cutime, &cstime);
        fclose(fd);

        // 返回进程及其子进程的总CPU时间
        return (utime + stime + cutime + cstime);
    }
#endif

    /**
     * @brief 获取指定进程的CPU使用率
     * @param pid 进程ID
     * @return CPU使用率，范围[0.0, cpu_num]，例如在4核CPU上最大为4.0
     * @details 通过计算两次采样之间的CPU时间差来得到使用率
     *          Windows: 使用GetProcessTimes获取进程时间
     *          Linux: 读取/proc/stat和/proc/{pid}/stat计算
     *          注意：首次调用返回0.0，需要至少两次调用才能得到准确结果
     */
    inline float GetCpuUsageRatio(int pid)
    {
#ifdef WIN32
        // Windows平台实现
        static int64_t last_time = 0;
    static int64_t last_system_time = 0;

    FILETIME now;
    FILETIME creation_time;
    FILETIME exit_time;
    FILETIME kernel_time;
    FILETIME user_time;
    int64_t system_time;
    int64_t time;
    int64_t system_time_delta;
    int64_t time_delta;

    // get cpu num（获取CPU核心数）
    SYSTEM_INFO info;
    GetSystemInfo(&info);
    int cpu_num = info.dwNumberOfProcessors;

    float cpu_ratio = 0.0;

    // get process hanlde by pid（通过PID获取进程句柄）
    HANDLE process = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    // use GetCurrentProcess() can get current process and no need to close handle

    // get now time（获取当前系统时间）
    GetSystemTimeAsFileTime(&now);

    if (!GetProcessTimes(process, &creation_time, &exit_time, &kernel_time, &user_time))
    {
        // We don't assert here because in some cases (such as in the Task Manager)
        // we may call this function on a process that has just exited but we have
        // not yet received the notification.
        printf("GetCpuUsageRatio GetProcessTimes failed\n");
        return 0.0;
    }

    // should handle the multiple cpu num（处理多核CPU情况）
    system_time = (convert_time_format(&kernel_time) + convert_time_format(&user_time)) / cpu_num;
    time = convert_time_format(&now);

    // 首次调用，初始化基准值
    if ((last_system_time == 0) || (last_time == 0))
    {
        // First call, just set the last values.
        last_system_time = system_time;
        last_time = time;
        return 0.0;
    }

    // 计算时间增量
    system_time_delta = system_time - last_system_time;
    time_delta = time - last_time;

    CloseHandle(process);

    if (time_delta == 0)
    {
        printf("GetCpuUsageRatio time_delta is 0, error\n");
        return 0.0;
    }

    // We add time_delta / 2 so the result is rounded.（添加time_delta/2进行四舍五入）
    cpu_ratio = (int)((system_time_delta * 100 + time_delta / 2) / time_delta); // the % unit
    last_system_time = system_time;
    last_time = time;

    cpu_ratio /= 100.0; // convert to float number（转换为浮点数）

    return cpu_ratio;
#else
        // Linux平台实现
        // 保存上次的CPU时间（静态变量在首次调用时初始化）
        static unsigned long totalcputime1 = get_cpu_total_occupy();
        static unsigned long procputime1 = get_cpu_proc_occupy(pid);

        // 获取当前的CPU时间
        unsigned long totalcputime2 = get_cpu_total_occupy();
        unsigned long procputime2 = get_cpu_proc_occupy(pid);

        float pcpu = 0.0;
        // 计算CPU使用率 = 进程CPU时间增量 / 系统总CPU时间增量
        if (0 != totalcputime2 - totalcputime1)
            pcpu = (procputime2 - procputime1) / float(totalcputime2 - totalcputime1); // float number

        // 获取CPU核心数
        int cpu_num = get_nprocs();
        pcpu *= cpu_num; // should multiply cpu num in multiple cpu machine（多核需要乘以核心数）

        // 更新基准值供下次调用使用
        totalcputime1 = totalcputime2;
        procputime1 = procputime2;

        return pcpu;
#endif
    }

    /**
     * @brief 获取指定进程的物理内存使用量
     * @param pid 进程ID
     * @return 内存使用量（单位：MB）
     * @details Windows: 使用GetProcessMemoryInfo获取WorkingSetSize（工作集大小）
     *          Linux: 读取/proc/{pid}/status文件中的VmRSS（常驻内存集）
     */
    inline float GetMemoryUsage(int pid)
    {
#ifdef WIN32
        // Windows平台实现
        uint64_t mem = 0, vmem = 0;
    PROCESS_MEMORY_COUNTERS pmc;

    // get process hanlde by pid（通过PID获取进程句柄）
    HANDLE process = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    if (GetProcessMemoryInfo(process, &pmc, sizeof(pmc)))
    {
        mem = pmc.WorkingSetSize;   // 物理内存使用量（工作集大小）
        vmem = pmc.PagefileUsage;   // 虚拟内存使用量（页面文件使用量）
    }
    CloseHandle(process);

    // use GetCurrentProcess() can get current process and no need to close handle

    // convert mem from B to MB（从字节转换为MB）
    return mem / 1024.0 / 1024.0;

#else
        // Linux平台实现
        char file_name[64] = { 0 };
        FILE* fd;
        char line_buff[512] = { 0 };
        sprintf(file_name, "/proc/%d/status", pid);

        // 打开/proc/{pid}/status文件
        fd = fopen(file_name, "r");
        if (nullptr == fd)
            return 0;

        char name[64];
        int vmrss = 0;
        // 跳过前21行，定位到VmRSS所在行（第22行）
        for (int i = 0; i < VMRSS_LINE - 1; i++)
            char* temp = fgets(line_buff, sizeof(line_buff), fd);

        // 读取VmRSS（常驻内存集大小）
        char* temp = fgets(line_buff, sizeof(line_buff), fd);
        sscanf(line_buff, "%s %d", name, &vmrss);
        fclose(fd);

        // cnvert VmRSS from KB to MB（从KB转换为MB）
        return vmrss / 1024.0;
#endif
    }
}


#endif //SRC_CPU_MEMORY_QUERY_H
