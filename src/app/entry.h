#ifndef _app_entry_h_
#define _app_entry_h_

/**
 * @file entry.h
 * @brief CubeMX `main.c` 的应用入口替代层
 */

#include "app_runtime.h"
#include "app_status.h"
#include "arm.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "remote.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 系统初始化状态标志
 */
static bool init_ok = false;

/**
 * @brief 250Hz 节拍计数器
 * @details 以 500Hz 主循环为基准，每 2 个周期触发一次
 */
static uint8_t s_entry_250hz_tick = 0u;

/**
 * @brief 100Hz 节拍计数器
 * @details 以 500Hz 主循环为基准，每 5 个周期触发一次
 */
static uint8_t s_entry_100hz_tick = 0u;

/**
 * @brief 50Hz 节拍计数器
 * @details 以 500Hz 主循环为基准，每 10 个周期触发一次
 */
static uint8_t s_entry_50hz_tick = 0u;

/**
 * @brief 机械臂刷新异常日志节流计时器
 */
static ms_t s_entry_arm_refresh_log_timer = 0;

/**
 * @brief 机械臂状态刷新连续失败计数
 */
static uint16_t s_entry_arm_refresh_fail_count = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 判断当前 500Hz 周期是否命中 250Hz 节拍
 */
static inline bool entry_tick_250hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 100Hz 节拍
 */
static inline bool entry_tick_100hz(void);

/**
 * @brief 判断当前 500Hz 周期是否命中 50Hz 节拍
 */
static inline bool entry_tick_50hz(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 系统初始化入口
 * @details 按照实际启动依赖顺序完成底层装配、应用层初始化与定时调度启动
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
    app_runtime_init();
    app_status_init();
    log_info("BOOT app runtime/status init step done");
    delay_ms(100);

    if(assemble_tim6_500hz() != SYSTEM_STATUS_OK)
        return;
    log_info("BOOT tim6 500hz init step done");
    delay_ms(100);

    log_info("System initialized successfully");
    delay_ms(500);

    init_ok = true;
}

/**
 * @brief 主循环调度入口
 * @details
 *
 * 500Hz:
 * - chassis.process()
 * - app_runtime_process()
 *
 * 250Hz:
 * - odom.process()
 *
 * 100Hz:
 * - remote_process()
 *
 * 50Hz:
 * - arm.refresh_current_state()
 *
 * background:
 * - app_status_process()
 *
 */
static inline void entry_loop(void) {
    if(!init_ok) {
        log_error("System not initialized, skipping entry loop");
        return;
    }

    assemble_log_process();

    if(tim6_500hz_flag) {
        tim6_500hz_flag = false;

        /** 500Hz 底盘基础控制主循环 */
        chassis.process();

        /** 250Hz 里程计与姿态融合更新 */
        if(entry_tick_250hz())
            odom.process();

        /** 100Hz 遥控输入解析与底盘手动控制 */
        if(entry_tick_100hz())
            remote_process();

        /** 50Hz 机械臂反馈刷新与稳定性监测 */
        if(entry_tick_50hz()) {
            ArmStatus arm_status = arm.refresh_current_state();

            if(arm_status == ARM_OK) {
                if(s_entry_arm_refresh_fail_count >= 10u)
                    log_info("ARM refresh recovered after %u failures", s_entry_arm_refresh_fail_count);
                s_entry_arm_refresh_fail_count = 0u;
            }
            else {
                s_entry_arm_refresh_fail_count++;
                if(s_entry_arm_refresh_fail_count >= 10u && delay_nb_ms(&s_entry_arm_refresh_log_timer, 1000))
                    log_warn("ARM refresh unstable: %s, consecutive=%u", arm.status_str(arm_status), s_entry_arm_refresh_fail_count);
            }
        }

        /** 500Hz 应用层运行入口，内部统一处理 supervisor + task 职责 */
        app_runtime_process();
    }

    /** 后台状态显示与调试输出 */
    app_status_process();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 250Hz 节拍 helper
 */
static inline bool entry_tick_250hz(void) {
    s_entry_250hz_tick = (uint8_t)((s_entry_250hz_tick + 1u) % 2u);
    return s_entry_250hz_tick == 0u;
}

/**
 * @brief 100Hz 节拍 helper
 */
static inline bool entry_tick_100hz(void) {
    s_entry_100hz_tick = (uint8_t)((s_entry_100hz_tick + 1u) % 5u);
    return s_entry_100hz_tick == 0u;
}

/**
 * @brief 50Hz 节拍 helper
 */
static inline bool entry_tick_50hz(void) {
    s_entry_50hz_tick = (uint8_t)((s_entry_50hz_tick + 1u) % 10u);
    return s_entry_50hz_tick == 0u;
}

#endif
