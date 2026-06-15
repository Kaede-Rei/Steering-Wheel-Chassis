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
 * ---- 3. 正常遥控状态
 *
 */

#include "hfsm/hfsm.h"
#include "navigation_route.h"

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
    TASK_STATE_NAVIGATION_NORMAL,      /**< 正常导航状态 */
    TASK_STATE_NAVIGATION_RETURN_HOME, /**< 返航状态 */
    TASK_STATE_POLLEN,                 /**< 授粉状态 */
    TASK_STATE_REMOTE                  /**< 遥控接管状态 */
} TaskStateId;

/**
 * @brief 任务状态机事件枚举
 */
typedef enum {
    TASK_EVENT_START = 1,        /**< 开始自动任务 */
    TASK_EVENT_STOP,             /**< 结束任务并回到空闲 */
    TASK_EVENT_SWITCH_TO_REMOTE, /**< 切换到遥控模式 */
    TASK_EVENT_SWITCH_TO_AUTO,   /**< 从遥控切回自动待命 */
    TASK_EVENT_NAV_REACHED,      /**< 当前导航点到达完成 */
    TASK_EVENT_POLLEN_FINISHED,  /**< 当前授粉动作完成 */
    TASK_EVENT_ERROR,            /**< 进入错误状态 */
    TASK_EVENT_ERROR_CLEAR       /**< 清除错误并回到空闲 */
} TaskEventId;

/**
 * @brief 导航状态上下文
 * @param target_index 当前导航点索引
 * @param current_area 当前区域
 * @param start_point 当前导航段起点
 * @param target_point 当前导航点
 * @param segment_start_ms 当前导航段起始时刻
 * @param brake_start_ms 当前导航点刹车保持起始时刻
 * @param braking 是否正在导航点刹车保持
 */
typedef struct {
    uint8_t target_index;
    AreaType current_area;
    NavPoint start_point;
    NavPoint target_point;
    uint32_t segment_start_ms;
    uint32_t brake_start_ms;
    bool braking;
} TaskNavigationContext;

/**
 * @brief 返航状态上下文
 * @param back_home_index 当前返航点索引
 */
typedef struct {
    uint8_t back_home_index;
} TaskReturnHomeContext;

/**
 * @brief 授粉机械臂动作序列上下文
 * @param steps 授粉动作序列
 * @param step_count 授粉动作总数
 * @param current_step 当前授粉步骤索引
 * @param step_started 当前步骤是否已开始
 * @param step_start_ms 当前步骤开始时刻
 */
#define TASK_POLLEN_MAX_STEPS 6u
typedef struct {
    FiveDofArmJointArray steps[TASK_POLLEN_MAX_STEPS];
    uint8_t step_count;
    uint8_t current_step;
    bool step_started;
    uint32_t step_start_ms;

    bool broadcast_waiting;
    uint32_t broadcast_start_ms;

    bool prepose_waiting;
    FiveDofArmJointArray prepose_target;
    uint32_t prepose_start_ms;

    bool broadcast_pending;
    uint8_t broadcast_cmd;

    bool step_interval_waiting;
    uint32_t step_interval_start_ms;

    uint32_t last_feedback_ms;
} TaskPollenSequenceContext;

/**
 * @brief 授粉状态上下文
 */
typedef struct {
    TaskPollenSequenceContext sequence;
} TaskPollenContext;

/**
 * @brief 任务状态机整体上下文
 * @param current_state_id 当前状态 ID
 * @param navigation 导航相关上下文
 * @param return_home 返航相关上下文
 * @param pollen 授粉相关上下文
 */
typedef struct {
    TaskStateId current_state_id;
    TaskStateId state_before_remote;
    bool resume_from_remote;
    TaskNavigationContext navigation;
    TaskReturnHomeContext return_home;
    TaskPollenContext pollen;
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

/**
 * @brief 任务状态机实例
 */
extern Task g_app_task;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void task_init(Task* task);
void task_process(Task* task);
bool task_post(Task* task, TaskEventId event_id);

#endif
