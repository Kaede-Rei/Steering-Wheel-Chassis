#include "app_supervisor.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "fs_ia10b.h"
#include "log.h"
#include "task/task.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 变 量 声 明 ========================= ! //

static bool remote_takeover_latched = false;
static bool auto_started = false;

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
#define REMOTE_SW_LOW 2000u

/**
 * @brief SWD 高位原始值
 */
#define REMOTE_SW_HIGH 1000u

/**
 * @brief VR 旋钮高位原始值阈值
 */
#define REMOTE_VR_HIGH_THRESHOLD 1800u

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化应用层监督器
 */
void app_supervisor_init(void) {
    remote_takeover_latched = false;
    auto_started = false;
}

/**
 * @brief 执行一次应用层监督器轮询
 */
void app_supervisor_process(void) {
    if(ibus_is_online(100u) && ibus_get_channel(ENTRY_REMOTE_CH_SWD) == REMOTE_SW_LOW) {
        if(remote_takeover_latched == false) {
            auto_started = false;
            remote_takeover_latched = true;
            log_info("Remote 接管，切换到遥控模式");
            (void)chassis.brake();
            (void)chassis_yaw_hold_disable();
            (void)task_post(&g_app_task, TASK_EVENT_SWITCH_TO_REMOTE);
        }
    }
    else if(remote_takeover_latched == true) {
        auto_started = false;
        remote_takeover_latched = false;
        log_info("Remote 释放，切换到自主任务模式");
        (void)task_post(&g_app_task, TASK_EVENT_SWITCH_TO_AUTO);
    }

    if(auto_started == false &&
       ibus_get_channel(ENTRY_REMOTE_CH_SWD) == REMOTE_SW_HIGH &&
       ibus_get_channel(REMOTE_CH_VRA) >= REMOTE_VR_HIGH_THRESHOLD &&
       ibus_get_channel(REMOTE_CH_VRB) >= REMOTE_VR_HIGH_THRESHOLD) {
        auto_started = true;
        log_info("自主任务模式启动");
        (void)task_post(&g_app_task, TASK_EVENT_START);
    }
}
