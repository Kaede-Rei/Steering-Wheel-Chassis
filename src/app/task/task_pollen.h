#ifndef _task_pollen_h_
#define _task_pollen_h_

#include "task.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    TASK_POLLEN_RESULT_RUNNING = 0,
    TASK_POLLEN_RESULT_FINISHED,
    TASK_POLLEN_RESULT_ERROR
} TaskPollenResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void task_pollen_reset(TaskPollenContext* pollen);
void task_pollen_start(TaskPollenContext* pollen, uint8_t nav_index, AreaType area, const NavPoint* point);
TaskPollenResult task_pollen_process(TaskPollenContext* pollen);

#endif
