#ifndef _entry_h_
#define _entry_h_

// ! system ! //
#include <stdbool.h>



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"
#include "remote.h"

// ! device ! //
#include "imu/imu.h"
#include "fs_ia10b.h"
#include "rgb_led/rgb_led.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t log_task = 0;

static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };

static uint8_t remote = 0;
static uint8_t chassis_ready_led_state = 0u;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 更新底盘就绪状态指示灯
 */
static inline void entry_update_chassis_ready_led(void) {
    RemoteCommand command;
    const bool ready = chassis.is_ready() && remote_get_command(&command);
    const uint8_t target_state = ready ? 2u : 1u;

    if(chassis_ready_led_state == target_state) {
        return;
    }

    if(ready) {
        rgb_led.fill(0U, 255U, 0U);
    }
    else {
        rgb_led.fill(255U, 0U, 0U);
    }

    if(rgb_led.show() == RGB_LED_STATUS_OK) {
        chassis_ready_led_state = target_state;
    }
}

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

        entry_update_chassis_ready_led();
    }

    if(imu.update() == IMU_STATUS_OK) {
        accel = imu.get_acc();
        gyro = imu.get_gyro();
        angle = imu.get_angle();
    }

    // ! 周期性任务 ! //
    if(delay_nb_ms(&log_task, 1000)) {
        log_info("CH5=%d, CH6=%d, CH7=%d, CH8=%d",
            ibus_get_channel(4),
            ibus_get_channel(5),
            ibus_get_channel(6),
            ibus_get_channel(7));
    }
}

#endif
