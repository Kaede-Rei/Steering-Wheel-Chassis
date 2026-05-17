#ifndef _entry_h_
#define _entry_h_

// ! system ! //



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"

// ! device ! //
#include "imu/BMI088/inc/BMI088driver.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t log_task = 0;
// static ms_t imu_task = 0;

static float accel[3] = { 0.0f };
static float gyro[3] = { 0.0f };
static float temp = 0.0f;

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
    // ! 事 件 驱 动 任 务 ! //
    if(chassis_control_flag) {
        chassis_control_flag = false;
        chassis.process();
    }

    BMI088_async_poll();
    BMI088_async_get_accel(accel);
    BMI088_async_get_gyro(gyro);

    // ! 周 期 性 任 务 ! //
    if(delay_nb_ms(&log_task, 1000)) {
        SteerWheelState state = *chassis.get_state();

        BMI088_read_temp(&temp);
        log_vofa(state.cur_vx, state.cur_vy, state.cur_wz, gyro[0], gyro[1], gyro[2], temp);
    }
}

#endif
