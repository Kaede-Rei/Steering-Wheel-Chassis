/**
 * @file app_runtime.c
 * @brief 应用层运行入口实现
 */

#include "app_runtime.h"

#include "arm.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "fs_ia10b.h"
#include "log.h"
#include "task/task.h"

#include <stdbool.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief SWD 三挡开关通道索引
 * @details 用于切换自动/遥控接管模式
 */
#define APP_RUNTIME_REMOTE_CH_SWD 7u

/**
 * @brief SWA 三挡开关通道索引
 * @details 遥控接管时用于机械臂卸力控制
 */
#define APP_RUNTIME_REMOTE_CH_SWA 4u

/**
 * @brief VRA 旋钮通道索引
 * @details 自动任务启动确认条件之一
 */
#define APP_RUNTIME_REMOTE_CH_VRA 8u

/**
 * @brief VRB 旋钮通道索引
 * @details 自动任务启动确认条件之一
 */
#define APP_RUNTIME_REMOTE_CH_VRB 9u

/**
 * @brief SWD 低位原始值
 */
#define APP_RUNTIME_REMOTE_SW_LOW 2000u

/**
 * @brief SWD 高位原始值
 */
#define APP_RUNTIME_REMOTE_SW_HIGH 1000u

/**
 * @brief 自动启动所需的旋钮高位阈值
 */
#define APP_RUNTIME_REMOTE_VR_HIGH_THRESHOLD 1800u

/**
 * @brief 遥控接管锁存标记
 * @details `true` 表示当前由遥控接管，任务状态机应处于手动分支
 */
static bool s_remote_takeover_latched = false;

/**
 * @brief 自动任务已启动锁存标记
 * @details 避免启动条件持续满足时重复投递 `TASK_EVENT_START`
 */
static bool s_auto_started = false;

/**
 * @brief 机械臂已卸力锁存标记
 * @details 避免遥控接管期间重复发送 stop/enable 指令
 */
static bool s_arm_torque_released = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 根据遥控 SWD 状态更新自动/手动模式
 */
static void app_runtime_update_remote_mode(void);

/**
 * @brief 检查自动任务启动条件并投递启动事件
 */
static void app_runtime_update_auto_start(void);

/**
 * @brief 根据遥控接管状态更新机械臂卸力/使能
 */
static void app_runtime_update_arm_torque(void);

/**
 * @brief 应用层统一安全策略
 * @details 预留给应用层运行时的集中安全策略入口
 */
static void app_runtime_apply_safety(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化应用层运行时
 */
void app_runtime_init(void) {
    s_remote_takeover_latched = false;
    s_auto_started = false;
    s_arm_torque_released = false;
    task_init();
}

/**
 * @brief 执行一次应用层运行时轮询
 */
void app_runtime_process(void) {
    app_runtime_update_remote_mode();
    app_runtime_update_auto_start();
    app_runtime_update_arm_torque();
    app_runtime_apply_safety();
    task_process();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 根据遥控 SWD 状态更新任务模式
 */
static void app_runtime_update_remote_mode(void) {
    if(ibus_is_online(100u) &&
       ibus_get_channel(APP_RUNTIME_REMOTE_CH_SWD) == APP_RUNTIME_REMOTE_SW_LOW) {
        if(s_remote_takeover_latched == false) {
            s_auto_started = false;
            s_remote_takeover_latched = true;
            (void)chassis.brake();
            (void)chassis_yaw_hold_disable();
            log_info("Remote takeover, switch to manual mode");
            (void)task_force_post(TASK_EVENT_SWITCH_TO_REMOTE);
        }
    }
    else if(s_remote_takeover_latched == true) {
        s_auto_started = false;
        s_remote_takeover_latched = false;
        log_info("Remote released, switch to auto mode");
        (void)task_force_post(TASK_EVENT_SWITCH_TO_AUTO);
    }
}

/**
 * @brief 判断自动任务启动条件并投递启动事件
 */
static void app_runtime_update_auto_start(void) {
    if(s_auto_started == false &&
       s_remote_takeover_latched == false &&
       ibus_is_online(100u) &&
       ibus_get_channel(APP_RUNTIME_REMOTE_CH_SWD) == APP_RUNTIME_REMOTE_SW_HIGH &&
       ibus_get_channel(APP_RUNTIME_REMOTE_CH_VRA) >= APP_RUNTIME_REMOTE_VR_HIGH_THRESHOLD &&
       ibus_get_channel(APP_RUNTIME_REMOTE_CH_VRB) >= APP_RUNTIME_REMOTE_VR_HIGH_THRESHOLD) {
        s_auto_started = true;
        log_info("Auto task start");
        (void)task_post(TASK_EVENT_START);
    }
}

/**
 * @brief 根据遥控接管状态和 SWA 档位更新机械臂卸力状态
 */
static void app_runtime_update_arm_torque(void) {
    bool should_release = false;
    ArmStatus status;

    if(s_remote_takeover_latched == true &&
       ibus_is_online(100u) &&
       ibus_get_channel(APP_RUNTIME_REMOTE_CH_SWA) == APP_RUNTIME_REMOTE_SW_HIGH) {
        should_release = true;
    }

    if(should_release == true && s_arm_torque_released == false) {
        status = arm.stop();
        if(status == ARM_OK) {
            s_arm_torque_released = true;
            log_info("ARM torque released for chassis remote");
        }
        else {
            log_warn("ARM torque release failed: %s", arm.status_str(status));
        }
    }
    else if(should_release == false && s_arm_torque_released == true) {
        status = arm.enable();
        if(status == ARM_OK) {
            s_arm_torque_released = false;
            log_info("ARM torque enabled");
        }
        else {
            log_warn("ARM torque enable failed: %s", arm.status_str(status));
        }
    }
}

/**
 * @brief 应用层统一安全策略
 * @details 当前安全动作在模式切换瞬间完成，这里保留为集中扩展点
 */
static void app_runtime_apply_safety(void) {
    (void)s_remote_takeover_latched;
}
