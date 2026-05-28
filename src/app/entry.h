#ifndef _entry_h_
#define _entry_h_

// ! system ! //
#include <stdbool.h>



// ! app ! //



// ! service ! //
#include "assemble/assemble.h"
#include "chassis.h"
#include "remote.h"
#include "attitude.h"

// ! device ! //
#include "imu/imu.h"
#include "rgb_led/rgb_led.h"
#include "gw_gray.h"

// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! platform ! //



// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

static ms_t log_task = 0;
static ms_t heartbeat_task = 0;
static ImuAcc accel = { 0.0f, 0.0f, 0.0f };
static ImuGyro gyro = { 0.0f, 0.0f, 0.0f };
static ImuAngle angle = { 0.0f, 0.0f, 0.0f };
static uint8_t remote = 0;
static uint8_t led_state = 0u;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 程序初始化入口函数
 *
 * 该函数由 main 初始化完成后调用；
 * 负责装配各服务并清零底盘速度命令
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK) return;
    if(assemble_log() != SYSTEM_STATUS_OK) return;
    log_info("BOOT log ready");
    delay_ms(100);

    if(assemble_rgb() != SYSTEM_STATUS_OK) return;
    log_info("BOOT rgb init step done");
    delay_ms(100);

    if(assemble_imu() != SYSTEM_STATUS_OK) return;
    log_info("BOOT imu init step done");
    delay_ms(100);

    if(assemble_chassis() != SYSTEM_STATUS_OK) return;
    log_info("BOOT chassis init step done");
    delay_ms(100);

    if(assemble_sensor() != SYSTEM_STATUS_OK) return;
    log_info("BOOT sensor init step done");
    delay_ms(100);

    if(assemble_attitude() != SYSTEM_STATUS_OK) return;
    log_info("BOOT attitude init step done");
    delay_ms(100);

    if(assemble_remote() != SYSTEM_STATUS_OK) return;
    log_info("BOOT remote init step done");
    delay_ms(100);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK) return;
    log_info("BOOT tim6 500hz init step done");
    delay_ms(100);

    log_info("System initialized successfully");
    delay_ms(500);
}

/**
 * @brief 程序主循环入口函数
 *
 * 该函数在 while(1) 中持续调用；
 * 根据定时器事件执行底盘、遥控、IMU 和日志任务
 */
static inline void entry_loop(void) {
    // ! 事件驱动任务 ! //
    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        if(imu.update() == IMU_STATUS_OK) {
            accel = imu.get_acc();
            gyro = imu.get_gyro();
            angle = imu.get_angle();
            attitude_update();
        }

        chassis.process();

        if(remote++ % 5 == 0) {
            remote_process();
            remote = 0;

            gw_gray_update();
        }
    }

    // ! 周期性任务 ! //
    if(delay_nb_ms(&heartbeat_task, 1000)) {
        RemoteCommand remote_command;
        const bool chassis_ready = chassis.is_ready();
        const bool remote_online = remote_get_command(&remote_command);

        uint8_t target_state = chassis_ready ? 1u : 0u;
        if(target_state == 1u && remote_online) target_state = 2u;

        if(led_state != target_state) {
            if(target_state == 2u) rgb_led.fill(0U, 0U, 255U);
            else if(target_state == 1u) rgb_led.fill(0U, 255U, 0U);
            else rgb_led.fill(255U, 0U, 0U);
            if(rgb_led.show() == RGB_LED_STATUS_OK) led_state = target_state;
            log_info("Chassis %s, Remote %s", chassis_ready ? "Ready" : "Not Ready", remote_online ? "Online" : "Offline");
        }
    }

    if(delay_nb_ms(&log_task, 1000)) {
        // log_info("Heartbeat");

        const AttitudeState* att = attitude_get_state();

        log_info(
            "att in=(%.2f,%.2f,%.2f) yaw=%.4f ref=%.4f err=%.4f ctrl_err=%.4f gz=%.4f hold=%u manual=%u static=%u latched=%u trim=%.4f corr=%.4f out_wz=%.4f mc=%u sc=%u",
            att->input_vx,
            att->input_vy,
            att->input_wz,
            att->yaw_total,
            att->yaw_ref,
            att->yaw_error,
            att->yaw_error_for_control,
            att->gyro_z,
            att->yaw_hold_active,
            att->manual_rotate_active,
            att->static_active,
            att->hold_latched,
            att->empirical_trim,
            att->final_wz_correction,
            att->output_wz,
            att->manual_count,
            att->static_count
        );
    }
}

#endif
