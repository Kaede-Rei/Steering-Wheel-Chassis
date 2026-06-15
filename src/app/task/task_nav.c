#include "task_nav.h"

#include "arm.h"
#include "chassis.h"
#include "delay.h"
#include "log.h"
#include "odom.h"
#include "task_nav_control.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define IS_REACH_THRESHOLD 0.001f
#define TASK_ARM_SPEED_RAD_S 12.56f
#define TASK_NAV_BRAKE_HOLD_MS 300u
#define TASK_NAV_BRAKE_TIMEOUT_MS 1500u

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool reach_at_nav_point(const NavPoint* nav_point, const Vector3* odom);
static void navigation_start_brake_hold(TaskNavigationContext* nav);
static bool navigation_brake_hold_done(const TaskNavigationContext* nav);
static bool navigation_service_brake_hold(TaskNavigationContext* nav);
static bool navigation_load_point(TaskNavigationContext* nav, uint8_t index, bool return_home);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 重置导航上下文
 * @param nav 导航上下文
 */
void task_nav_reset(TaskNavigationContext* nav) {
    if(nav == NULL)
        return;

    memset(nav, 0, sizeof(*nav));
    nav->target_index = 1u;
    nav->current_area = START_END;
}

/**
 * @brief 装载指定导航点
 * @param nav 导航上下文
 * @param index 导航点索引
 * @return bool `true` 表示装载成功
 */
bool task_nav_load_target(TaskNavigationContext* nav, uint8_t index) {
    return navigation_load_point(nav, index, false);
}

/**
 * @brief 执行当前导航点的到点控制流程
 * @param nav 导航上下文
 * @return TaskNavResult 导航处理结果
 */
TaskNavResult task_nav_process(TaskNavigationContext* nav) {
    Vector3 od = { 0 };
    bool was_braking;

    if(nav == NULL)
        return TASK_NAV_RESULT_ERROR;

    was_braking = nav->braking;
    if(navigation_service_brake_hold(nav))
        return TASK_NAV_RESULT_RUNNING;
    if(was_braking)
        return TASK_NAV_RESULT_REACHED;

    odom.get_odom(&od);
    if(reach_at_nav_point(&nav->target_point, &od)) {
        navigation_start_brake_hold(nav);
        return TASK_NAV_RESULT_RUNNING;
    }

    task_nav_control_follow_s_curve(nav, &od);
    return TASK_NAV_RESULT_RUNNING;
}

/**
 * @brief 判断当前导航点是否需要进入授粉状态
 * @param nav 导航上下文
 * @return bool `true` 表示需要授粉
 */
bool task_nav_target_requires_pollen(const TaskNavigationContext* nav) {
    if(nav == NULL)
        return false;

    return nav->target_point.area_type == AREA_A ||
           nav->target_point.area_type == AREA_B ||
           nav->target_point.area_type == AREA_C;
}

/**
 * @brief 判断当前导航点是否为最后一个点
 * @param nav 导航上下文
 * @return bool `true` 表示当前为最后一个点
 */
bool task_nav_is_last_target(const TaskNavigationContext* nav) {
    if(nav == NULL || nav_route_count() == 0u)
        return true;

    return nav->target_index >= (nav_route_count() - 1u);
}

/**
 * @brief 推进到下一个导航点
 * @param nav 导航上下文
 * @return bool `true` 表示推进成功
 */
bool task_nav_advance(TaskNavigationContext* nav) {
    if(nav == NULL || task_nav_is_last_target(nav))
        return false;

    return task_nav_load_target(nav, (uint8_t)(nav->target_index + 1u));
}

/**
 * @brief 开始返航导航
 * @param nav 导航上下文
 * @param ret 返航上下文
 * @return bool `true` 表示开始成功
 */
bool task_nav_return_home_start(TaskNavigationContext* nav, TaskReturnHomeContext* ret) {
    if(nav == NULL || ret == NULL)
        return false;

    ret->back_home_index = 0u;
    return navigation_load_point(nav, ret->back_home_index, true);
}

/**
 * @brief 执行返航导航
 * @param nav 导航上下文
 * @param ret 返航上下文
 * @return TaskNavResult 导航处理结果
 */
TaskNavResult task_nav_return_home_process(TaskNavigationContext* nav, TaskReturnHomeContext* ret) {
    TaskNavResult result;

    if(nav == NULL || ret == NULL)
        return TASK_NAV_RESULT_ERROR;

    result = task_nav_process(nav);
    if(result != TASK_NAV_RESULT_REACHED)
        return result;

    log_info("Reach return home point %u -> (%.2f, %.2f)",
             ret->back_home_index,
             nav->target_point.x,
             nav->target_point.y);

    if(ret->back_home_index + 1u >= nav_return_route_count())
        return TASK_NAV_RESULT_FINISHED;

    ret->back_home_index++;
    if(navigation_load_point(nav, ret->back_home_index, true) == false)
        return TASK_NAV_RESULT_ERROR;

    return TASK_NAV_RESULT_RUNNING;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 判断当前位置是否到达目标导航点
 * @param nav_point 目标导航点
 * @param odom 当前里程计位置
 * @return bool `true` 表示已到达
 */
static bool reach_at_nav_point(const NavPoint* nav_point, const Vector3* odom) {
    float err_x;
    float err_y;

    if(nav_point == NULL || odom == NULL)
        return false;

    err_x = nav_point->x - odom->x;
    err_y = nav_point->y - odom->y;

    return fabsf(err_x) < IS_REACH_THRESHOLD && fabsf(err_y) < IS_REACH_THRESHOLD;
}

/**
 * @brief 启动导航点到点后的刹停保持
 * @param nav 导航上下文
 */
static void navigation_start_brake_hold(TaskNavigationContext* nav) {
    if(nav == NULL)
        return;

    nav->braking = true;
    nav->brake_start_ms = delay_now_ms();
    (void)chassis.brake();
}

/**
 * @brief 判断导航点刹停保持是否结束
 * @param nav 导航上下文
 * @return bool `true` 表示刹停保持已结束
 */
static bool navigation_brake_hold_done(const TaskNavigationContext* nav) {
    const Chassis* ch = chassis.get_chassis();
    uint32_t elapsed_ms;

    if(nav == NULL || !nav->braking)
        return false;

    elapsed_ms = delay_now_ms() - nav->brake_start_ms;
    if(elapsed_ms < TASK_NAV_BRAKE_HOLD_MS)
        return false;

    return (ch != NULL && ch->brake_latched) || elapsed_ms >= TASK_NAV_BRAKE_TIMEOUT_MS;
}

/**
 * @brief 执行导航点刹停保持服务
 * @param nav 导航上下文
 * @return bool `true` 表示当前仍处于刹停保持阶段
 */
static bool navigation_service_brake_hold(TaskNavigationContext* nav) {
    if(nav == NULL || !nav->braking)
        return false;

    (void)chassis.brake();
    if(navigation_brake_hold_done(nav))
        nav->braking = false;

    return nav->braking;
}

/**
 * @brief 装载导航点并刷新导航段上下文
 * @param nav 导航上下文
 * @param index 导航点索引
 * @param return_home 是否为返航路径
 * @return bool `true` 表示装载成功
 */
static bool navigation_load_point(TaskNavigationContext* nav, uint8_t index, bool return_home) {
    Vector3 od = { 0 };
    bool ok;

    if(nav == NULL)
        return false;

    ok = return_home ? nav_return_route_get(index, &nav->target_point) : nav_route_get(index, &nav->target_point);
    if(ok == false)
        return false;

    odom.get_odom(&od);
    nav->target_index = index;
    nav->start_point.x = od.x;
    nav->start_point.y = od.y;
    nav->current_area = nav->target_point.area_type;
    nav->segment_start_ms = delay_now_ms();
    nav->braking = false;

    if(return_home == false && nav->target_point.pre_detect_joints.exist) {
        (void)arm.move_joints(&nav->target_point.pre_detect_joints.joints, TASK_ARM_SPEED_RAD_S);
    }
    else
        (void)arm.move_servo_zero(TASK_ARM_SPEED_RAD_S);

    log_info("%s point %u -> (%.2f, %.2f)",
             return_home ? "Return home" : "Navigate",
             index,
             nav->target_point.x,
             nav->target_point.y);
    return true;
}
