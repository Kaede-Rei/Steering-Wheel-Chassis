#ifndef _APP_TASK_CONTEXT_H_
#define _APP_TASK_CONTEXT_H_

/**
 * @file task_context.h
 * @brief 任务状态、事件、故障和上下文公共类型
 */

#include "nav/navigation_route.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 任务状态机状态 ID
 */
typedef enum {
    TASK_STATE_FAULT = 0,     /**< 故障收口状态，按故障取消范围执行安全动作 */
    TASK_STATE_AUTO,          /**< 自主任务父状态，不直接执行业务动作 */
    TASK_STATE_IDLE,          /**< 自主任务未启动，等待启动事件 */
    TASK_STATE_NAVIGATE,      /**< 前往当前目标点 */
    TASK_STATE_POLLINATE,     /**< 执行当前目标点授粉动作 */
    TASK_STATE_RETURN_HOME,   /**< 按返航路线回到起点 */
    TASK_STATE_MANUAL         /**< 遥控接管状态 */
} TaskStateId;

/**
 * @brief 任务状态机事件 ID
 */
typedef enum {
    TASK_EVENT_START = 1,        /**< 开始自主任务 */
    TASK_EVENT_STOP,             /**< 结束任务并回到空闲 */
    TASK_EVENT_SWITCH_TO_REMOTE, /**< 切换到遥控接管 */
    TASK_EVENT_SWITCH_TO_AUTO,   /**< 从遥控接管切回自主任务 */
    TASK_EVENT_NAV_REACHED,      /**< 当前导航点到达 */
    TASK_EVENT_POLLEN_FINISHED,  /**< 当前授粉动作完成 */
    TASK_EVENT_FAULT,            /**< 进入故障收口状态 */
    TASK_EVENT_FAULT_CLEAR       /**< 清除故障并回到空闲 */
} TaskEventId;

/**
 * @brief 任务故障来源
 */
typedef enum {
    TASK_FAULT_SOURCE_NONE = 0, /**< 无故障来源 */
    TASK_FAULT_SOURCE_CHASSIS,  /**< 底盘服务或运动控制故障 */
    TASK_FAULT_SOURCE_ODOM,     /**< 里程计或姿态反馈故障 */
    TASK_FAULT_SOURCE_ARM,      /**< 机械臂命令或反馈故障 */
    TASK_FAULT_SOURCE_REMOTE,   /**< 遥控输入或遥控输出故障 */
    TASK_FAULT_SOURCE_NAV,      /**< 导航流程故障 */
    TASK_FAULT_SOURCE_POLLEN,   /**< 授粉流程故障 */
    TASK_FAULT_SOURCE_ROUTE,    /**< 路线或动作表缺失故障 */
    TASK_FAULT_SOURCE_SYSTEM    /**< 系统级或未知故障 */
} TaskFaultSource;

/**
 * @brief 任务故障等级
 */
typedef enum {
    TASK_FAULT_LEVEL_NONE = 0,  /**< 无故障 */
    TASK_FAULT_LEVEL_WARN,      /**< 告警，不一定需要取消动作 */
    TASK_FAULT_LEVEL_DEGRADE,   /**< 降级运行，需要取消局部动作 */
    TASK_FAULT_LEVEL_RECOVERABLE, /**< 可恢复故障，需要进入故障状态等待清除 */
    TASK_FAULT_LEVEL_FATAL      /**< 致命故障，需要取消全部上层动作 */
} TaskFaultLevel;

/**
 * @brief 故障收口时需要取消的上层动作掩码
 */
typedef enum {
    TASK_CANCEL_NONE = 0,          /**< 不取消上层动作 */
    TASK_CANCEL_NAV = 1u << 0,     /**< 取消导航动作 */
    TASK_CANCEL_POLLEN = 1u << 1,  /**< 取消授粉动作 */
    TASK_CANCEL_REMOTE = 1u << 2,  /**< 取消遥控控制输出 */
    TASK_CANCEL_ALL = 0xFFu        /**< 取消全部上层动作 */
} TaskCancelMask;

/**
 * @brief 任务故障描述
 */
typedef struct {
    TaskFaultSource source;     /**< 故障来源，用于定位故障来自哪个子系统 */
    TaskFaultLevel level;       /**< 故障等级，用于决定收口策略 */
    TaskCancelMask cancel_mask; /**< 故障收口时需要取消的动作范围 */
    TaskStateId owner_state;    /**< 故障发生时所属任务状态 */
    int32_t code;               /**< 子模块返回码或本地错误码 */
    uint32_t time_ms;           /**< 故障发生时间，单位 ms */
} TaskFault;

/**
 * @brief 导航状态上下文
 */
typedef struct {
    uint8_t target_index;       /**< 当前导航点索引 */
    AreaType current_area;      /**< 当前导航点区域 */
    NavPoint start_point;       /**< 当前导航段起点 */
    NavPoint target_point;      /**< 当前导航目标点 */
    uint32_t segment_start_ms;  /**< 当前导航段开始时间 */
    uint32_t brake_start_ms;    /**< 到点刹停保持开始时间 */
    bool braking;               /**< 是否处于到点刹停保持阶段 */
} TaskNavigationContext;

/**
 * @brief 返航状态上下文
 */
typedef struct {
    uint8_t back_home_index; /**< 当前返航路线点索引 */
} TaskReturnHomeContext;

#define TASK_POLLEN_MAX_STEPS 6u

/**
 * @brief 授粉机械臂动作序列上下文
 */
typedef struct {
    FiveDofArmJointArray steps[TASK_POLLEN_MAX_STEPS]; /**< 授粉动作序列 */
    uint8_t step_count;                                /**< 授粉动作总数 */
    uint8_t current_step;                              /**< 当前动作索引 */
    bool step_started;                                 /**< 当前动作是否已下发 */
    uint32_t step_start_ms;                            /**< 当前动作下发时间 */
    bool broadcast_waiting;                            /**< 是否正在等待语音播报完成 */
    uint32_t broadcast_start_ms;                       /**< 语音播报等待开始时间 */
    bool prepose_waiting;                              /**< 是否正在等待预识别姿态到位 */
    FiveDofArmJointArray prepose_target;               /**< 预识别姿态目标 */
    uint32_t prepose_start_ms;                         /**< 预识别等待开始时间 */
    bool broadcast_pending;                            /**< 是否有待发送播报命令 */
    uint8_t broadcast_cmd;                             /**< 待发送播报命令 */
    bool step_interval_waiting;                        /**< 是否正在等待动作间隔 */
    uint32_t step_interval_start_ms;                   /**< 动作间隔开始时间 */
    uint32_t last_feedback_ms;                         /**< 最近一次有效反馈时间 */
} TaskPollenSequenceContext;

/**
 * @brief 授粉状态上下文
 */
typedef struct {
    TaskPollenSequenceContext sequence; /**< 授粉动作序列上下文 */
} TaskPollenContext;

/**
 * @brief 任务状态机整体上下文
 */
typedef struct {
    TaskStateId current_state_id;       /**< 当前任务状态 */
    TaskStateId state_before_remote;    /**< 遥控接管前的任务状态 */
    bool resume_from_remote;            /**< 从遥控接管返回时是否恢复原状态 */
    TaskNavigationContext navigation;   /**< 导航上下文 */
    TaskReturnHomeContext return_home;  /**< 返航上下文 */
    TaskPollenContext pollen;           /**< 授粉上下文 */
    TaskFault fault;                    /**< 当前锁存故障 */
    bool fault_latched;                 /**< 是否存在未清除故障 */
} TaskContext;

#endif
