#ifndef _task_nav_h_
#define _task_nav_h_

#include "task.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    TASK_NAV_RESULT_RUNNING = 0,
    TASK_NAV_RESULT_REACHED,
    TASK_NAV_RESULT_FINISHED,
    TASK_NAV_RESULT_ERROR
} TaskNavResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void task_nav_reset(TaskNavigationContext* nav);
bool task_nav_load_target(TaskNavigationContext* nav, uint8_t index);
TaskNavResult task_nav_process(TaskNavigationContext* nav);
bool task_nav_target_requires_pollen(const TaskNavigationContext* nav);
bool task_nav_is_last_target(const TaskNavigationContext* nav);
bool task_nav_advance(TaskNavigationContext* nav);
bool task_nav_return_home_start(TaskNavigationContext* nav, TaskReturnHomeContext* ret);
TaskNavResult task_nav_return_home_process(TaskNavigationContext* nav, TaskReturnHomeContext* ret);

#endif
