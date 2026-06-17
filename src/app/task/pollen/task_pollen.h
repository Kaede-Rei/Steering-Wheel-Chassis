#ifndef _APP_TASK_POLLEN_H_
#define _APP_TASK_POLLEN_H_

/**
 * @file task_pollen.h
 * @brief 授粉子任务接口
 */

#include "../task_context.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 授粉流程处理结果
 */
typedef enum {
    TASK_POLLEN_RESULT_RUNNING = 0,          /**< 授粉流程仍在执行 */
    TASK_POLLEN_RESULT_FINISHED,             /**< 授粉流程完成 */
    TASK_POLLEN_RESULT_ROUTE_MISSING,        /**< 授粉动作表缺失 */
    TASK_POLLEN_RESULT_ARM_COMMAND_FAILED,   /**< 机械臂命令下发失败 */
    TASK_POLLEN_RESULT_ARM_FEEDBACK_TIMEOUT, /**< 机械臂动作反馈超时 */
    TASK_POLLEN_RESULT_PREPOSE_TIMEOUT,      /**< 预识别姿态等待超时 */
    TASK_POLLEN_RESULT_BROADCAST_TIMEOUT     /**< 语音播报等待超时 */
} TaskPollenResult;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 重置授粉上下文
 * @param pollen 授粉上下文，传入 `NULL` 时不执行任何操作
 */
void task_pollen_reset(TaskPollenContext* pollen);

/**
 * @brief 开始当前导航点的授粉动作序列
 * @param pollen 授粉上下文
 * @param nav_index 当前导航点索引
 * @param area 当前导航点所在区域
 * @param point 当前导航点完整信息
 * @return TaskPollenResult `TASK_POLLEN_RESULT_RUNNING` 表示启动成功，否则表示启动阶段故障
 */
TaskPollenResult task_pollen_start(TaskPollenContext* pollen, uint8_t nav_index, AreaType area, const NavPoint* point);

/**
 * @brief 执行一次授粉动作序列轮询
 * @param pollen 授粉上下文
 * @return TaskPollenResult 当前授粉处理结果
 */
TaskPollenResult task_pollen_process(TaskPollenContext* pollen);

/**
 * @brief 取消授粉动作序列
 * @details 故障收口时调用，清空授粉上下文并请求机械臂进入安全停止状态
 * @param pollen 授粉上下文
 */
void task_pollen_cancel(TaskPollenContext* pollen);

/**
 * @brief 获取授粉处理结果字符串
 * @param result 授粉处理结果
 * @return const char* 固定字符串，用于日志输出
 */
const char* task_pollen_result_str(TaskPollenResult result);

#endif
