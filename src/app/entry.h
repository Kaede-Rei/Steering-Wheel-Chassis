#ifndef _entry_h_
#define _entry_h_

// ! system ! //



// ! app ! //



// ! service ! //
#include "assemble.h"
#include "chassis.h"

// ! device ! //
#include "bus_motor/agv_motor.h"


// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t chassis_task = 0;
static ms_t motor_log_task = 0;

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
    if(delay_nb_ms(&chassis_task, 2)) {
        chassis.process();
    }

    if(delay_nb_ms(&motor_log_task, 1000)) {
        log_info("steer[1] cur=%.3f tgt=%.3f spd=%.3f | drive[1] cur=%.3f tgt=%.3f tor=%.3f",
            steer_motor.get_pos(1u), chassis.control()->wheels[0].steer_angle, steer_motor.get_spd(1u),
            drive_motor.get_spd(1u), chassis.control()->wheels[0].wheel_omega, drive_motor.get_tor(1u));
        log_info("steer[2] cur=%.3f tgt=%.3f spd=%.3f | drive[2] cur=%.3f tgt=%.3f tor=%.3f",
            steer_motor.get_pos(2u), chassis.control()->wheels[1].steer_angle, steer_motor.get_spd(2u),
            drive_motor.get_spd(2u), chassis.control()->wheels[1].wheel_omega, drive_motor.get_tor(2u));
        log_info("steer[3] cur=%.3f tgt=%.3f spd=%.3f | drive[3] cur=%.3f tgt=%.3f tor=%.3f",
            steer_motor.get_pos(3u), chassis.control()->wheels[2].steer_angle, steer_motor.get_spd(3u),
            drive_motor.get_spd(3u), chassis.control()->wheels[2].wheel_omega, drive_motor.get_tor(3u));
        log_info("steer[4] cur=%.3f tgt=%.3f spd=%.3f | drive[4] cur=%.3f tgt=%.3f tor=%.3f",
            steer_motor.get_pos(4u), chassis.control()->wheels[3].steer_angle, steer_motor.get_spd(4u),
            drive_motor.get_spd(4u), chassis.control()->wheels[3].wheel_omega, drive_motor.get_tor(4u));
    }
}

#endif
