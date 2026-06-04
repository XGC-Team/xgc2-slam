//
// Created by usl on 11/7/20.
//

#ifndef LINCALIB_COLOR_H
#define LINCALIB_COLOR_H

/**
 * @file color.h
 * @brief 终端文本颜色控制宏定义
 * @details 定义了一组ANSI转义序列宏，用于在终端输出彩色文本
 *          使用方法：cout << RED << "红色文本" << RESET << endl;
 *          注意：需要终端支持ANSI颜色代码（大多数Linux/Unix终端都支持）
 */

#define RESET       "\033[0m"              /* 重置所有文本属性到默认状态 */
#define BLACK       "\033[30m"             /* 黑色 */
#define RED         "\033[31m"             /* 红色 */
#define GREEN       "\033[32m"             /* 绿色 */
#define YELLOW      "\033[33m"             /* 黄色 */
#define BLUE        "\033[34m"             /* 蓝色 */
#define MAGENTA     "\033[35m"             /* 品红色/洋红色 */
#define CYAN        "\033[36m"             /* 青色 */
#define WHITE       "\033[37m"             /* 白色 */
#define REDPURPLE   "\033[95m"             /* 红紫色 */
#define BOLDBLACK   "\033[1m\033[30m"      /* 粗体黑色 */
#define BOLDRED     "\033[1m\033[31m"      /* 粗体红色 */
#define BOLDGREEN   "\033[1m\033[32m"      /* 粗体绿色 */
#define BOLDYELLOW  "\033[1m\033[33m"      /* 粗体黄色 */
#define BOLDBLUE    "\033[1m\033[34m"      /* 粗体蓝色 */
#define BOLDMAGENTA "\033[1m\033[35m"      /* 粗体品红色 */
#define BOLDCYAN    "\033[1m\033[36m"      /* 粗体青色 */
#define BOLDWHITE   "\033[1m\033[37m"      /* 粗体白色 */
#define BOLDREDPURPLE   "\033[1m\033[95m"  /* 粗体红紫色 */

#endif //LINCALIB_COLOR_H
