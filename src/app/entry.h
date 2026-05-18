#ifndef _entry_h_
#define _entry_h_

// ! system ! //



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"
#include "remote.h"

// ! device ! //
#include "imu/imu.h"


// ! domain ! //



// ! infra ! //
// #include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接口变量 / Typedef 声明 ========================= ! //

static ms_t log_task = 0;
// static ms_t imu_task = 0;

static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };

static uint8_t remote = 0;

// ! ========================= 接口函数声明 ========================= ! //

/**
 * @brief 程序初始化入口函数
 */
static inline void entry_init(void) {
    assemble_init();
    chassis.set_velocity(0.0f, 0.0f, 0.0f);
}

/**
 * @brief 程序主循环入口函数
 */
static inline void entry_loop(void) {
    // ! 事件驱动任务 ! //
    if(chassis_control_flag) {
        chassis_control_flag = false;
        chassis.process();

        if(remote++ % 5 == 0) {
            remote_process();
            remote = 0;
        }
    }

    if(imu.update() == IMU_STATUS_OK) {
        accel = imu.get_acc();
        gyro = imu.get_gyro();
        angle = imu.get_angle();
    }

    // ! 周期性任务 ! //
    if(delay_nb_ms(&log_task, 1000)) {

    }
}

#endif
