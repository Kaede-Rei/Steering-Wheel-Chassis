#include "app_supervisor.h"
#include "arm.h"
#include "assemble/assemble.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "remote.h"
#include "rgb_led/rgb_led.h"
#include "task/task.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

static ms_t log_task = 0;
static ms_t heartbeat_task = 0;
static uint8_t remote_tick = 0;
static uint8_t arm_tick = 0;
static uint8_t odom_tick = 0;
static uint8_t led_state = 0u;
static ms_t arm_refresh_error_task = 0;
static uint16_t arm_refresh_fail_count = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void entry_fast_loop_500hz(void);
static void entry_chassis_process_500hz(void);
static void entry_odom_process_250hz(void);
static void entry_remote_process_100hz(void);
static void entry_arm_refresh_50hz(void);
static void entry_app_process_500hz(void);
static void entry_background_loop(void);
static void entry_led_status_process(void);
static void entry_debug_log_process(void);

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
    app_supervisor_init();
    delay_ms(100);

    task_init();
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
    entry_fast_loop_500hz();
    entry_background_loop();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 500Hz 快速调度循环
 */
static void entry_fast_loop_500hz(void) {
    if(tim6_500hz_flag == false)
        return;

    tim6_500hz_flag = false;

    entry_chassis_process_500hz();
    entry_odom_process_250hz();
    entry_remote_process_100hz();
    entry_arm_refresh_50hz();
    entry_app_process_500hz();
}

/**
 * @brief 执行后台低频任务
 */
static void entry_background_loop(void) {
    entry_led_status_process();
    entry_debug_log_process();
}

/**
 * @brief 执行底盘基础反馈刷新
 */
static void entry_chassis_process_500hz(void) {
    chassis.process();
}

/**
 * @brief 执行 250Hz 里程计基础反馈刷新
 */
static void entry_odom_process_250hz(void) {
    if(++odom_tick % 2 != 0)
        return;

    odom.process();
    odom_tick = 0;
}

/**
 * @brief 执行 100Hz 遥控输入和接管控制刷新
 */
static void entry_remote_process_100hz(void) {
    if(++remote_tick % 5 != 0)
        return;

    remote_process();
    remote_tick = 0;
}

/**
 * @brief 执行 50Hz 机械臂反馈刷新并统计连续失败次数
 */
static void entry_arm_refresh_50hz(void) {
    ArmStatus arm_status;

    if(++arm_tick % 10 != 0)
        return;

    arm_status = arm.refresh_current_state();
    if(arm_status == ARM_OK) {
        if(arm_refresh_fail_count >= 10u) {
            log_info("ARM refresh recovered after %u failures", arm_refresh_fail_count);
        }
        arm_refresh_fail_count = 0u;
    }
    else {
        arm_refresh_fail_count++;
        if(arm_refresh_fail_count >= 10u && delay_nb_ms(&arm_refresh_error_task, 1000)) {
            log_warn("ARM refresh unstable: %s, consecutive=%u",
                     arm.status_str(arm_status),
                     arm_refresh_fail_count);
        }
    }

    arm_tick = 0;
}

/**
 * @brief 执行应用层监督器和任务状态机
 */
static void entry_app_process_500hz(void) {
    app_supervisor_process();
    task_process();
}

/**
 * @brief 刷新 LED 状态显示
 */
static void entry_led_status_process(void) {
    if(delay_nb_ms(&heartbeat_task, 1000) == false)
        return;

    RemoteCommand remote_command;
    const bool chassis_ready = chassis.is_ready();
    const bool remote_online = remote_get_command(&remote_command);
    uint8_t target_state = chassis_ready ? 1u : 0u;

    if(task_has_fault())
        target_state = 3u;
    else if(target_state == 1u && remote_online)
        target_state = 2u;

    if(led_state == target_state)
        return;

    if(target_state == 3u)
        rgb_led.fill(255U, 128U, 0U);
    else if(target_state == 2u)
        rgb_led.fill(0U, 0U, 255U);
    else if(target_state == 1u)
        rgb_led.fill(0U, 255U, 0U);
    else
        rgb_led.fill(255U, 0U, 0U);

    if(rgb_led.show() == RGB_LED_STATUS_OK)
        led_state = target_state;

    log_info("Chassis %s, Remote %s, TaskFault %s",
             chassis_ready ? "Ready" : "Not Ready",
             remote_online ? "Online" : "Offline",
             task_has_fault() ? "Yes" : "No");
}

/**
 * @brief 周期性输出调试日志
 */
static void entry_debug_log_process(void) {
    if(delay_nb_ms(&log_task, 1000) == false)
        return;

    const FiveDofArmJointArray* arm_joints;

    arm_joints = arm.get_current_joints();
    if(arm_joints != 0)
        log_info("J0-4: %f, %f, %f, %f, %f", arm_joints->q[0], arm_joints->q[1], arm_joints->q[2], arm_joints->q[3], arm_joints->q[4]);
}
