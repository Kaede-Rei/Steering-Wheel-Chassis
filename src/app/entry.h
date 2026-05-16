#ifndef _entry_h_
#define _entry_h_

// ! system ! //



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"

// ! device ! //



// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t chassis_task = 0;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 程序初始化入口函数
 */
static inline void entry_init(void) {
    assemble_init();
    chassis.set_velocity(0.0f, 0.0f, 1.0f);
}

/**
 * @brief 程序主循环入口函数
 */
static inline void entry_loop(void) {
    if(delay_nb_ms(&chassis_task, 10)) {
        chassis.task_500hz();
    }
}

#endif
