#ifndef _app_runtime_h_
#define _app_runtime_h_

/**
 * @file app_runtime.h
 * @brief 应用层运行入口
 */

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化应用层运行时
 * @details 复位运行时锁存状态，并初始化任务流程状态机
 */
void app_runtime_init(void);

/**
 * @brief 执行一次应用层运行时轮询
 * @details 内部统一处理遥控接管、自动启动、安全策略，再驱动 `task_process()`
 */
void app_runtime_process(void);

#endif
