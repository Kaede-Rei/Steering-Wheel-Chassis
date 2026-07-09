/**
 * @file app_status.c
 * @brief 应用层状态显示与调试输出实现
 */

#include "app_status.h"

#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "remote.h"
#include "rgb_led/rgb_led.h"
#include "task/task.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 调试日志周期定时器
 */
static ms_t s_log_timer = 0;

/**
 * @brief 心跳状态显示周期定时器
 */
static ms_t s_heartbeat_timer = 0;

/**
 * @brief 当前 LED 状态编码
 * @details 0=未就绪, 1=底盘就绪, 2=遥控在线, 3=任务故障
 */
static uint8_t s_led_state = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 刷新 LED 颜色与系统状态心跳输出
 */
static void app_status_update_led(void);

/**
 * @brief 周期性输出日志
 */
static void app_status_log(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化应用层状态输出模块
 */
void app_status_init(void) {
    s_log_timer = 0;
    s_heartbeat_timer = 0;
    s_led_state = 0u;
}

/**
 * @brief 执行一次后台状态输出轮询
 */
void app_status_process(void) {
    app_status_update_led();
    app_status_log();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 刷新 LED 状态显示
 * @details 每秒执行一次，根据底盘就绪、遥控在线、任务故障状态选择颜色，出现切换后输出日志
 */
static void app_status_update_led(void) {
    if(delay_nb_ms(&s_heartbeat_timer, 1000) == false)
        return;

    RemoteCommand remote_command;
    const bool chassis_ready = chassis.is_ready();
    uint8_t target_state = chassis_ready ? 1u : 0u;
    const bool remote_online = remote_get_command(&remote_command);

    if(task_has_fault())
        target_state = 3u;
    else if(target_state == 1u && remote_online)
        target_state = 2u;

    if(s_led_state != target_state) {
        if(target_state == 3u)
            rgb_led.fill(255U, 128U, 0U);
        else if(target_state == 2u)
            rgb_led.fill(0U, 0U, 255U);
        else if(target_state == 1u)
            rgb_led.fill(0U, 255U, 0U);
        else
            rgb_led.fill(255U, 0U, 0U);

        if(rgb_led.show() == RGB_LED_STATUS_OK)
            s_led_state = target_state;

        log_info("Chassis %s, Remote %s, TaskFault %s", chassis_ready ? "Ready" : "Not Ready", remote_online ? "Online" : "Offline", task_has_fault() ? "Yes" : "No");
    }
}

/**
 * @brief 周期性输出日志
 */
static void app_status_log(void) {
    if(delay_nb_ms(&s_log_timer, 1000) == false)
        return;

    log_info("Heartbeat");
}
