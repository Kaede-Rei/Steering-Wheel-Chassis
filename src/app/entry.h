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

static ms_t chassis_task = 0;
static ms_t log_task = 0;
static ms_t imu_task = 0;

static float accel[3] = { 0.0f };
static float gyro[3] = { 0.0f };
static float temp = 0.0f;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 程序初始化入口函数
 */
static inline void entry_init(void) {
    assemble_init();

    uint8_t res = BMI088_init();
    if(res != BMI088_NO_ERROR) {
        log_error("BMI088 initialization failed: %d", res);
    }

    chassis.set_velocity(0.0f, 0.0f, 1.0f);
}

/**
 * @brief 程序主循环入口函数
 */
static inline void entry_loop(void) {
    if(delay_nb_ms(&chassis_task, 2)) {
        chassis.process();
    }

    if(delay_nb_ms(&imu_task, 10)) {
        BMI088_read(gyro, accel, &temp);
    }

    if(delay_nb_ms(&log_task, 100)) {
        SteerWheelState state = *chassis.get_state();
        log_vofa(state.cur_vx, state.cur_vy, state.cur_wz, gyro[0], gyro[1], gyro[2]);
    }
}

#endif
