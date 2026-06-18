#ifndef _app_task_nav_h_
#define _app_task_nav_h_

/**
 * @file task_nav.h
 * @brief 自主导航子任务接口
 */

#include "../task_context.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 导航流程处理结果
 */
typedef enum {
    TASK_NAV_RESULT_RUNNING = 0,   /**< 导航仍在执行 */
    TASK_NAV_RESULT_REACHED,       /**< 当前目标点已经到达 */
    TASK_NAV_RESULT_FINISHED,      /**< 当前导航流程全部完成 */
    TASK_NAV_RESULT_ROUTE_ERROR,   /**< 路线点缺失或索引非法 */
    TASK_NAV_RESULT_ODOM_ERROR,    /**< 里程计反馈不可用 */
    TASK_NAV_RESULT_CHASSIS_ERROR, /**< 底盘命令执行失败 */
    TASK_NAV_RESULT_TIMEOUT        /**< 导航或刹停过程超时 */
} TaskNavResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 重置导航上下文
 * @param nav 导航上下文，传入 `NULL` 时不执行任何操作
 */
void task_nav_reset(TaskNavigationContext* nav);

/**
 * @brief 装载指定导航点
 * @param nav 导航上下文
 * @param index 导航点索引
 * @return bool `true` 表示装载成功
 */
bool task_nav_load_target(TaskNavigationContext* nav, uint8_t index);

/**
 * @brief 执行当前导航点控制流程
 * @param nav 导航上下文
 * @return TaskNavResult 当前导航处理结果
 */
TaskNavResult task_nav_process(TaskNavigationContext* nav);

/**
 * @brief 判断当前导航点是否需要进入授粉流程
 * @param nav 导航上下文
 * @return bool `true` 表示该点需要授粉
 */
bool task_nav_target_requires_pollen(const TaskNavigationContext* nav);

/**
 * @brief 判断当前导航点是否为最后一个自主导航点
 * @param nav 导航上下文
 * @return bool `true` 表示当前点已是最后一个导航点
 */
bool task_nav_is_last_target(const TaskNavigationContext* nav);

/**
 * @brief 推进到下一个导航点
 * @param nav 导航上下文
 * @return bool `true` 表示下一个点装载成功
 */
bool task_nav_advance(TaskNavigationContext* nav);

/**
 * @brief 开始返航导航
 * @param nav 导航上下文
 * @param ret 返航上下文
 * @return bool `true` 表示返航首点装载成功
 */
bool task_nav_return_home_start(TaskNavigationContext* nav, TaskReturnHomeContext* ret);

/**
 * @brief 执行返航导航流程
 * @param nav 导航上下文
 * @param ret 返航上下文
 * @return TaskNavResult 当前返航处理结果
 */
TaskNavResult task_nav_return_home_process(TaskNavigationContext* nav, TaskReturnHomeContext* ret);

/**
 * @brief 取消导航上层动作
 * @details 故障收口或遥控接管时调用，重置导航上下文、关闭 yaw hold 并请求底盘刹车
 * @param nav 导航上下文
 */
void task_nav_cancel(TaskNavigationContext* nav);

/**
 * @brief 获取导航处理结果字符串
 * @param result 导航处理结果
 * @return const char* 固定字符串，用于日志输出
 */
const char* task_nav_result_str(TaskNavResult result);

#endif
