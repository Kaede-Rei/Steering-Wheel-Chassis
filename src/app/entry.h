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
#include "imu/bmi088.h"
#include "fs_ia10b.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接口变量 / Typedef 声明 ========================= ! //

static ms_t log_task = 0;
// static ms_t imu_task = 0;

static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };
static float temp = 0.0f;
static FsIa10bData rc_data = { 0 };

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
        if(ibus_get_data(&rc_data)) {
            log_info(
                "RC: CH1=%u CH2=%u CH3=%u CH4=%u CH5=%u CH6=%u CH7=%u CH8=%u CH9=%u CH10=%u CH11=%u CH12=%u CH13=%u CH14=%u FrameCount=%lu ErrorCount=%lu",
                rc_data.channel[0], rc_data.channel[1], rc_data.channel[2],
                rc_data.channel[3], rc_data.channel[4], rc_data.channel[5],
                rc_data.channel[6], rc_data.channel[7], rc_data.channel[8],
                rc_data.channel[9], rc_data.channel[10], rc_data.channel[11],
                rc_data.channel[12], rc_data.channel[13],
                (unsigned long)rc_data.frame_count,
                (unsigned long)rc_data.error_count);
        }
        else {
            log_info("RC: No signal");
        }

        temp = bmi088_get_temp();
    }
}

#endif
