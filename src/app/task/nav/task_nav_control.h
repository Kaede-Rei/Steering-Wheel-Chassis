#ifndef _APP_TASK_NAV_CONTROL_H_
#define _APP_TASK_NAV_CONTROL_H_

/**
 * @file task_nav_control.h
 * @brief 导航底盘跟踪控制接口
 */

#include "odom.h"
#include "../task_context.h"

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 按 S 曲线轨迹跟踪当前导航点
 * @param nav 导航上下文，提供起点、终点和当前导航段时间
 * @param od 当前里程计位置
 */
void task_nav_control_follow_s_curve(const TaskNavigationContext* nav, const Vector3* od);

#endif
