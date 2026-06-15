#ifndef _task_nav_control_h_
#define _task_nav_control_h_

#include "task.h"

#include "odom.h"

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void task_nav_control_follow_s_curve(const TaskNavigationContext* nav, const Vector3* od);

#endif
