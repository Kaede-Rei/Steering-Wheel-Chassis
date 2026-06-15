#ifndef _APP_TASK_H_
#define _APP_TASK_H_

/**
 * @file task.h
 * @brief 比赛任务应用层层级状态机接口
 * @details 状态树
 *
 * ---- 1. 错误状态
 * ---- 2. 正常非遥控状态
 * -------- 2.1. 空闲状态
 * -------- 2.2. 导航状态
 * ------------ 2.2.1. 正常导航状态
 * ------------ 2.2.2. 返回起点状态
 * -------- 2.3. 授粉状态
 * ------------ 2.3.1. A 区授粉状态
 * ------------ 2.3.2. B 区授粉状态
 * ------------ 2.3.3. C 区授粉状态
 * ---- 3. 正常遥控状态
 *
 */

#include "hfsm/hfsm.h"
#include "navigation_map.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 任务状态机状态枚举
 */
typedef enum {
    TASK_STATE_ERROR = 0,              /**< 错误状态 */
    TASK_STATE_NORMAL,                 /**< 正常自动任务父状态 */
    TASK_STATE_IDLE,                   /**< 空闲待命状态 */
    TASK_STATE_NAVIGATION,             /**< 导航父状态 */
    TASK_STATE_NAVIGATION_START,       /**< 起始播报状态 */
    TASK_STATE_NAVIGATION_NORMAL,      /**< 正常导航状态 */
    TASK_STATE_NAVIGATION_RETURN_HOME, /**< 返航状态 */
    TASK_STATE_POLLEN,                 /**< 授粉父状态 */
    TASK_STATE_POLLEN_A,               /**< A 区授粉状态 */
    TASK_STATE_POLLEN_B,               /**< B 区授粉状态 */
    TASK_STATE_POLLEN_C,               /**< C 区授粉状态 */
    TASK_STATE_REMOTE                  /**< 遥控接管状态 */
} TaskStateId;

/**
 * @brief 任务状态机事件枚举
 */
typedef enum {
    TASK_EVENT_START = 1,          /**< 开始自动任务 */
    TASK_EVENT_STOP,               /**< 结束任务并回到空闲 */
    TASK_EVENT_SWITCH_TO_REMOTE,   /**< 切换到遥控模式 */
    TASK_EVENT_SWITCH_TO_AUTO,     /**< 从遥控切回自动待命 */
    TASK_EVENT_NAV_START_FINISHED, /**< 起始播报完成 */
    TASK_EVENT_NAV_REACHED,        /**< 当前导航点到达完成 */
    TASK_EVENT_POLLEN_FINISHED,    /**< 当前授粉动作完成 */
    TASK_EVENT_ERROR,              /**< 进入错误状态 */
    TASK_EVENT_ERROR_CLEAR         /**< 清除错误并回到空闲 */
} TaskEventId;

/**
 * @brief 任务状态机上下文
 * @param current_state_id 当前状态 ID
 * @param current_area 当前区域
 * @param nav_start_point 当前导航段起点
 * @param current_nav_point 当前导航点
 * @param current_nav_point.area_type 当前点所属区域，决定是否切授粉状态
 * @param current_nav_point.pre_detect_joints 当前点预识别机械臂关节角
 * @param back_home_index 当前返航点索引
 * @param nav_start_ms 当前导航段起始时刻
 * @param nav_brake_start_ms 当前导航点刹车保持起始时刻
 * @param nav_braking 是否正在导航点刹车保持
 */
typedef struct {
    TaskStateId current_state_id;
    AreaType current_area;
    NavPoint nav_start_point;
    NavPoint current_nav_point;
    uint8_t back_home_index;
    uint32_t nav_start_ms;
    uint32_t nav_brake_start_ms;
    bool nav_braking;
} TaskContext;

/**
 * @brief 任务状态机实例
 * @param fsm HFSM 状态机实例
 * @param ctx 任务状态机上下文
 */
typedef struct {
    Hfsm fsm;
    TaskContext ctx;
} Task;

extern Task g_app_task;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void task_init(Task* task);
void task_process(Task* task);
bool task_post(Task* task, TaskEventId event_id);

#endif
