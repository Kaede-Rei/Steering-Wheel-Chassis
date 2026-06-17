#ifndef _APP_TASK_H_
#define _APP_TASK_H_

/**
 * @file task.h
 * @brief 比赛任务状态机对外接口
 * @details 本头文件只暴露应用层调度、事件投递和故障查询接口，外部模块不直接访问 HFSM 实例和任务上下文
 */

#include "task_context.h"

#include <stdbool.h>

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化全局任务状态机
 * @details 在底盘、里程计、机械臂、遥控等基础服务完成 assemble 后调用一次
 */
void task_init(void);

/**
 * @brief 执行一次任务状态机轮询
 * @details 建议在 500Hz fast loop 中调用，内部会处理待处理事件并执行当前状态动作
 */
void task_process(void);

/**
 * @brief 投递普通任务事件
 * @param event_id 任务事件 ID，通常用于启动、停止、模式切换和子流程完成通知
 * @return bool `true` 表示事件进入队列成功
 */
bool task_post(TaskEventId event_id);

/**
 * @brief 清空旧事件并投递一个高优先级任务事件
 * @param event_id 任务事件 ID，用于遥控接管、急停、故障等需要覆盖旧队列的场景
 * @return bool `true` 表示事件进入队列成功
 */
bool task_force_post(TaskEventId event_id);

/**
 * @brief 上报任务故障并切换到故障收口状态
 * @param fault 故障描述，包含来源、等级、取消范围、归属状态、错误码和发生时间
 * @return bool `true` 表示故障已锁存并成功投递故障事件
 */
bool task_raise_fault(const TaskFault* fault);

/**
 * @brief 清除当前锁存故障并回到空闲状态
 * @return bool `true` 表示清除事件投递成功
 */
bool task_clear_fault(void);

/**
 * @brief 获取当前任务状态 ID
 * @return TaskStateId 当前任务状态，状态机未初始化前返回 `TASK_STATE_IDLE`
 */
TaskStateId task_get_state(void);

/**
 * @brief 获取当前锁存故障
 * @return const TaskFault* 指向内部故障信息的只读指针；没有故障时仍返回内部空故障结构
 */
const TaskFault* task_get_fault(void);

/**
 * @brief 判断当前是否存在锁存故障
 * @return bool `true` 表示任务状态机已有未清除故障
 */
bool task_has_fault(void);

#endif
