#ifndef _app_status_h_
#define _app_status_h_

/**
 * @file app_status.h
 * @brief 应用层状态显示与调试输出入口
 */

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化应用层状态输出模块
 */
void app_status_init(void);

/**
 * @brief 执行一次后台状态输出轮询
 */
void app_status_process(void);

#endif
