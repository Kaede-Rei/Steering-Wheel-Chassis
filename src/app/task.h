#ifndef _APP_TASK_H_
#define _APP_TASK_H_

/**
 * @file task.h
 * @brief 比赛任务应用层层级状态机接口
 */

#include "hfsm/hfsm.h"
#include "navigation_map.h"

#include <stdbool.h>

// ! ========================= 接口号量 / Typedef 声明 ========================= ! //

/**
 * @brief 任务应用层统一入口别名
 */
#define task task_interface

/**
 * @brief 任务应用层状态码表
 */
#define TASK_STATUS_TABLE                 \
    X(OK, "OK")                           \
    X(INVALID_PARAM, "Invalid Param")     \
    X(NOT_INITIALIZED, "Not Initialized") \
    X(HFSM_ERROR, "HFSM Error")

/**
 * @brief 任务应用层状态码
 */
#define X(name, str) TASK_##name,
typedef enum {
    TASK_STATUS_TABLE
} TaskStatus;
#undef X

/**
 * @brief 任务状态机状态枚举
 * @param TASK_STATE_ERROR 错误状态，禁止自主任务，允许遥控
 * @param TASK_STATE_NORMAL 正常非遥控父状态
 * @param TASK_STATE_IDLE 空闲状态
 * @param TASK_STATE_NAVIGATION 导航父状态
 * @param TASK_STATE_NAVIGATION_START 启动导航状态
 * @param TASK_STATE_NAVIGATION_ABC ABC 区导航状态
 * @param TASK_STATE_NAVIGATION_RETURN_ZERO 返回零点导航状态
 * @param TASK_STATE_POLLEN 授粉父状态
 * @param TASK_STATE_POLLEN_A A 区授粉状态
 * @param TASK_STATE_POLLEN_B B 区授粉状态
 * @param TASK_STATE_POLLEN_C C 区授粉状态
 * @param TASK_STATE_REMOTE 正常遥控状态
 */
typedef enum {
    TASK_STATE_ERROR = 0,
    TASK_STATE_NORMAL,
    TASK_STATE_IDLE,
    TASK_STATE_NAVIGATION,
    TASK_STATE_NAVIGATION_START,
    TASK_STATE_NAVIGATION_ABC,
    TASK_STATE_NAVIGATION_RETURN_ZERO,
    TASK_STATE_POLLEN,
    TASK_STATE_POLLEN_A,
    TASK_STATE_POLLEN_B,
    TASK_STATE_POLLEN_C,
    TASK_STATE_REMOTE
} TaskStateId;

/**
 * @brief 任务状态机事件枚举
 */
typedef enum {
    TASK_EVENT_START_AUTO = 1,
    TASK_EVENT_STOP_AUTO,
    TASK_EVENT_ENABLE_REMOTE,
    TASK_EVENT_DISABLE_REMOTE,
    TASK_EVENT_NAV_REACHED,
    TASK_EVENT_POLLEN_FINISHED,
    TASK_EVENT_FAULT,
    TASK_EVENT_CLEAR_FAULT
} TaskEventId;

/**
 * @brief 任务应用层实例数据
 * @param fsm HFSM 状态机实例
 * @param current_state_id 当前业务状态 ID
 * @param current_area 当前正在处理的区域
 * @param current_nav_point 当前导航目标点
 * @param remote_enabled 是否请求进入遥控状态
 * @param auto_running 是否正在执行自主任务
 * @param error_active 是否处于错误态
 * @param initialized 是否已经完成初始化
 */
typedef struct {
    Hfsm fsm;
    TaskStateId current_state_id;
    AreaType current_area;
    NavPoint current_nav_point;
    bool remote_enabled;
    bool auto_running;
    bool error_active;
    bool initialized;
} Task;

/**
 * @brief 任务应用层统一接口表
 */
#define X(name, str) TaskStatus name;
extern const struct TaskInterface {
    struct {
        TASK_STATUS_TABLE
    };

    /**
     * @brief 初始化任务状态机
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*init)(void);
    /**
     * @brief 执行一次任务状态机轮询
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*process)(void);
    /**
     * @brief 请求启动自主任务
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*start_auto)(void);
    /**
     * @brief 请求停止自主任务并回到空闲状态
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*stop_auto)(void);
    /**
     * @brief 请求进入遥控状态
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*enable_remote)(void);
    /**
     * @brief 请求退出遥控状态
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*disable_remote)(void);
    /**
     * @brief 通知当前导航目标点已经到达
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*notify_nav_reached)(void);
    /**
     * @brief 通知当前授粉流程已经完成
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*notify_pollen_finished)(void);
    /**
     * @brief 请求进入错误状态
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*enter_error)(void);
    /**
     * @brief 请求清除错误状态
     * @return TaskStatus 任务应用层状态码
     */
    TaskStatus (*clear_error)(void);
    /**
     * @brief 获取当前任务状态 ID
     * @return TaskStateId 当前任务状态 ID
     */
    TaskStateId (*state_id)(void);
    /**
     * @brief 获取当前任务状态名称
     * @return const char* 当前任务状态名称
     */
    const char* (*state_name)(void);
    /**
     * @brief 查询当前是否允许自主任务
     * @return bool `true` 表示允许自主任务
     */
    bool (*allow_auto)(void);
    /**
     * @brief 查询当前是否允许遥控
     * @return bool `true` 表示允许遥控
     */
    bool (*allow_remote)(void);
    /**
     * @brief 获取任务应用层只读视图
     * @return const Task* 任务应用层实例指针
     */
    const Task* (*get_task)(void);
} task_interface;
#undef X

// ! ========================= 接口函数声明 ========================= ! //

TaskStatus task_init(void);
TaskStatus task_process(void);
TaskStatus task_start_auto(void);
TaskStatus task_stop_auto(void);
TaskStatus task_enable_remote(void);
TaskStatus task_disable_remote(void);
TaskStatus task_notify_nav_reached(void);
TaskStatus task_notify_pollen_finished(void);
TaskStatus task_enter_error(void);
TaskStatus task_clear_error(void);
TaskStateId task_state_id(void);
const char* task_state_name(void);
bool task_allow_auto(void);
bool task_allow_remote(void);
const Task* task_get_task(void);

#endif
