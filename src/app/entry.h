#ifndef _entry_h_
#define _entry_h_

// ! system ! //
#include <stdbool.h>


// ! app ! //
#include "remote.h"
#include "task.h"

// ! device ! //
#include "fs_ia10b.h"


// ! service ! //
#include "assemble/assemble.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "odom.h"
#include "arm.h"

// ! device ! //
#include "rgb_led/rgb_led.h"


// ! domain ! //



// ! infra ! //
#include "log.h"
#include "delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

static ms_t log_task = 0;
static ms_t heartbeat_task = 0;
static uint8_t remote_tick = 0;
static uint8_t arm_tick = 0;
static uint8_t odom_tick = 0;
static uint8_t led_state = 0u;
static bool remote_takeover_latched = false;

/**
 * @brief SWD 三挡开关通道索引
 */
#define ENTRY_REMOTE_CH_SWD 7u

/**
 * @brief VRA 旋钮通道索引
 */
#define REMOTE_CH_VRA 8u

/**
 * @brief VRB 旋钮通道索引
 */
#define REMOTE_CH_VRB 9u

/**
 * @brief SWD 低位原始值
 */
#define ENTRY_REMOTE_SW_LOW 2000u
#define ENTRY_REMOTE_SW_HIGH 1000u

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 程序初始化入口函数
 */
static inline void entry_init(void) {
    if(assemble_delay() != SYSTEM_STATUS_OK)
        return;

    if(assemble_log() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT log ready");
    delay_ms(100);

    if(assemble_rgb() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT rgb init step done");
    delay_ms(100);

    if(assemble_imu() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT imu init step done");
    delay_ms(100);

    if(assemble_odom() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT odom init step done");
    delay_ms(100);

    if(assemble_chassis() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT chassis init step done");
    delay_ms(100);

    if(assemble_arm() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT arm init step done");
    delay_ms(100);

    if(assemble_remote() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT remote init step done");
    delay_ms(100);

    remote_init();
    delay_ms(100);

    task_init(&g_app_task);
    log_info("BOOT task init step done");
    delay_ms(100);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6 500hz init step done");
    delay_ms(100);

    log_info("System initialized successfully");
    delay_ms(500);
}

/**
 * @brief 程序主循环入口函数
 */
static inline void entry_loop(void) {
    // ! 事件驱动任务 ! //
    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        chassis.process();

        if(++odom_tick % 2 == 0) {
            odom.process();
            odom_tick = 0;
        }

        if(++remote_tick % 5 == 0) {
            remote_process();
            remote_tick = 0;
        }

        static bool start = false;

        if(ibus_is_online(100u) && ibus_get_channel(ENTRY_REMOTE_CH_SWD) == ENTRY_REMOTE_SW_LOW) {
            if(remote_takeover_latched == false) {
                start = false;
                remote_takeover_latched = true;
                log_info("Remote takeover by SWD low");
                (void)chassis.brake();
                (void)chassis_yaw_hold_disable();
                (void)task_post(&g_app_task, TASK_EVENT_SWITCH_TO_REMOTE);
            }
        }
        else if(remote_takeover_latched == true) {
            start = false;
            remote_takeover_latched = false;
            log_info("Remote release, switch to auto");
            (void)task_post(&g_app_task, TASK_EVENT_SWITCH_TO_AUTO);
        }

        if(start == false && ibus_get_channel(ENTRY_REMOTE_CH_SWD) == ENTRY_REMOTE_SW_HIGH && ibus_get_channel(REMOTE_CH_VRA) == ENTRY_REMOTE_SW_LOW && ibus_get_channel(REMOTE_CH_VRB) == ENTRY_REMOTE_SW_LOW) {
            start = true;
            log_info("Auto start triggered by remote");
            task_post(&g_app_task, TASK_EVENT_START);
        }

        task_process(&g_app_task);

        if(++arm_tick % 10 == 0) {
            arm.refresh_current_state();
            arm_tick = 0;
        }
    }

    // ! 周期性任务 ! //
    if(delay_nb_ms(&heartbeat_task, 1000)) {
        RemoteCommand remote_command;
        const bool chassis_ready = chassis.is_ready();
        const bool remote_online = remote_get_command(&remote_command);

        uint8_t target_state = chassis_ready ? 1u : 0u;
        if(target_state == 1u && remote_online)
            target_state = 2u;

        if(led_state != target_state) {
            if(target_state == 2u)
                rgb_led.fill(0U, 0U, 255U);
            else if(target_state == 1u)
                rgb_led.fill(0U, 255U, 0U);
            else
                rgb_led.fill(255U, 0U, 0U);
            if(rgb_led.show() == RGB_LED_STATUS_OK)
                led_state = target_state;
            log_info("Chassis %s, Remote %s", chassis_ready ? "Ready" : "Not Ready", remote_online ? "Online" : "Offline");
        }
    }

    if(delay_nb_ms(&log_task, 100)) {
        Vector3 od = { 0 };
        Vector3 ag = { 0 };

        odom.get_odom(&od);
        odom.get_angle(&ag);
        log_vofa(od.x, od.y, od.z, ag.x, ag.y, ag.z);
    }
}

#endif
