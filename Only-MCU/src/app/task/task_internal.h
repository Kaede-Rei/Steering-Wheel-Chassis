#ifndef _app_task_internal_h_
#define _app_task_internal_h_

/**
 * @file task_internal.h
 * @brief 任务模块内部共享定义
 */

#include "hfsm/hfsm.h"
#include "task_context.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 任务状态机内部实例
 */
typedef struct {
    Hfsm fsm;        /**< HFSM 状态机实例 */
    TaskContext ctx; /**< 任务业务上下文 */
} Task;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取任务模块内部实例
 * @return Task* 全局任务实例指针，仅内部模块使用
 */
Task* task_internal_instance(void);

#endif
