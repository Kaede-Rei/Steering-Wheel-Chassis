/**
 * @file task.c
 * @brief 比赛任务应用层层级状态机实现
 */

#include "task.h"

#include "asrpro.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "log.h"
#include "delay.h"
#include "odom.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define IS_REACH_THRESHOLD 0.001f
#define REACH_TIME_S 1.0f
#define TASK_CONTROL_PERIOD_S 0.002f
#define TASK_NAV_ACCEL_RATIO 0.05f
#define TASK_NAV_TRACK_KP 4.0f
#define TASK_NAV_TRACK_SPEED_MARGIN_M_S 0.04f
#define TASK_NAV_BRAKE_HOLD_MS 300u
#define TASK_NAV_BRAKE_TIMEOUT_MS 1500u
#define TASK_ARM_SPEED_RAD_S 12.56f

Task g_app_task = { 0 };

static HfsmState* s_error = NULL;
static HfsmState* s_normal = NULL;
static HfsmState* s_idle = NULL;
static HfsmState* s_navigation = NULL;
static HfsmState* s_navigation_start = NULL;
static HfsmState* s_navigation_normal = NULL;
static HfsmState* s_navigation_return_home = NULL;
static HfsmState* s_pollen = NULL;
static HfsmState* s_pollen_a = NULL;
static HfsmState* s_pollen_b = NULL;
static HfsmState* s_pollen_c = NULL;
static HfsmState* s_remote = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static HfsmResult error_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult normal_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult navigation_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult pollen_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult remote_handle(HfsmMachine* m, const HfsmEvent* e);

static void error_entry(HfsmMachine* m);
static void idle_entry(HfsmMachine* m);
static void navigation_entry(HfsmMachine* m);
static void navigation_start_entry(HfsmMachine* m);
static void navigation_normal_entry(HfsmMachine* m);
static void navigation_normal_action(HfsmMachine* m);
static void navigation_return_home_entry(HfsmMachine* m);
static void navigation_return_home_action(HfsmMachine* m);
static void pollen_entry(HfsmMachine* m);
static void pollen_a_entry(HfsmMachine* m);
static void pollen_a_action(HfsmMachine* m);
static void pollen_b_entry(HfsmMachine* m);
static void pollen_b_action(HfsmMachine* m);
static void pollen_c_entry(HfsmMachine* m);
static void pollen_c_action(HfsmMachine* m);
static void remote_entry(HfsmMachine* m);

static bool reach_at_nav_point(const NavPoint* nav_point, const Vector3* odom);
static void follow_s_curve_nav(TaskContext* ctx, const Vector3* od);
static void go_to_nav_point(TaskContext* ctx);
static void navigation_load_next_point(TaskContext* ctx);
static void navigation_start_brake_hold(TaskContext* ctx);
static bool navigation_brake_hold_done(const TaskContext* ctx);
static bool navigation_service_brake_hold(TaskContext* ctx);
static void navigation_finish_return_home_brake(TaskContext* ctx);
static void navigation_s_curve_profile(float progress, float* position_ratio, float* speed_ratio);
static void navigation_clamp_velocity(float* vx, float* vy, float max_speed);
static HfsmState* navigation_get_pollen_state(AreaType area);
static bool navigation_point_requires_pollen(const NavPoint* nav_point);
static AsrProCmd navigation_make_x_broadcast_cmd(XFlowerType flowers);
static AsrProCmd navigation_make_y_broadcast_cmd(YFlowerType flowers);
static void navigation_broadcast_current_point(const TaskContext* ctx);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化任务状态机
 * @param task 任务状态机实例
 */
void task_init(Task* task) {
    if(task == NULL)
        return;

    memset(task, 0, sizeof(*task));
    nav_map_init();
    chassis_yaw_hold_set_target(0.0f);

    hfsm.init(&task->fsm, &task->ctx);

    s_error = hfsm.add_state(&task->fsm, "Error");
    s_normal = hfsm.add_state(&task->fsm, "Normal");
    s_remote = hfsm.add_state(&task->fsm, "Remote");
    s_idle = hfsm.add_substate(&task->fsm, s_normal, "Idle");
    s_navigation = hfsm.add_substate(&task->fsm, s_normal, "Navigation");
    s_navigation_start = hfsm.add_substate(&task->fsm, s_navigation, "NavigationStart");
    s_navigation_normal = hfsm.add_substate(&task->fsm, s_navigation, "NavigationNormal");
    s_navigation_return_home = hfsm.add_substate(&task->fsm, s_navigation, "NavigationReturnHome");
    s_pollen = hfsm.add_substate(&task->fsm, s_normal, "Pollen");
    s_pollen_a = hfsm.add_substate(&task->fsm, s_pollen, "PollenA");
    s_pollen_b = hfsm.add_substate(&task->fsm, s_pollen, "PollenB");
    s_pollen_c = hfsm.add_substate(&task->fsm, s_pollen, "PollenC");

    hfsm.set_handle(s_error, error_handle);
    hfsm.set_entry(s_error, error_entry);

    hfsm.set_handle(s_normal, normal_handle);
    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_entry(s_idle, idle_entry);

    hfsm.set_handle(s_navigation, navigation_handle);
    hfsm.set_entry(s_navigation, navigation_entry);
    hfsm.set_entry(s_navigation_start, navigation_start_entry);
    hfsm.set_entry(s_navigation_normal, navigation_normal_entry);
    hfsm.set_action(s_navigation_normal, navigation_normal_action);
    hfsm.set_entry(s_navigation_return_home, navigation_return_home_entry);
    hfsm.set_action(s_navigation_return_home, navigation_return_home_action);

    hfsm.set_handle(s_pollen, pollen_handle);
    hfsm.set_entry(s_pollen, pollen_entry);
    hfsm.set_entry(s_pollen_a, pollen_a_entry);
    hfsm.set_action(s_pollen_a, pollen_a_action);
    hfsm.set_entry(s_pollen_b, pollen_b_entry);
    hfsm.set_action(s_pollen_b, pollen_b_action);
    hfsm.set_entry(s_pollen_c, pollen_c_entry);
    hfsm.set_action(s_pollen_c, pollen_c_action);

    hfsm.set_handle(s_remote, remote_handle);
    hfsm.set_entry(s_remote, remote_entry);

    hfsm.set_initial(&task->fsm, s_idle);
    hfsm.start(&task->fsm);
}

/**
 * @brief 执行一次任务状态机轮询
 * @param task 任务状态机实例
 */
void task_process(Task* task) {
    if(task == NULL)
        return;

    hfsm.process(&task->fsm);
}

/**
 * @brief 投递一个任务事件
 * @param task 任务状态机实例
 * @param event_id 任务事件 ID
 * @return bool `true` 表示投递成功
 */
bool task_post(Task* task, TaskEventId event_id) {
    if(task == NULL)
        return false;

    return hfsm.post(&task->fsm, (HfsmEventId)event_id, NULL) == hfsm.OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 错误状态事件处理函数
 * @param m 状态机实例
 * @param e 当前事件
 * @return HfsmResult 事件处理结果
 */
static HfsmResult error_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_ERROR_CLEAR)
        return hfsm.res.transition(s_idle);

    return hfsm.res.ignore();
}

/**
 * @brief 正常工作父状态事件处理函数
 * @param m 状态机实例
 * @param e 当前事件
 * @return HfsmResult 事件处理结果
 */
static HfsmResult normal_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_SWITCH_TO_REMOTE)
        return hfsm.res.transition(s_remote);

    if(e->id == TASK_EVENT_STOP)
        return hfsm.res.transition(s_idle);

    if(e->id == TASK_EVENT_ERROR)
        return hfsm.res.transition(s_error);

    return hfsm.res.ignore();
}

/**
 * @brief 空闲状态事件处理函数
 * @param m 状态机实例
 * @param e 当前事件
 * @return HfsmResult 事件处理结果
 */
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_START) {
        nav_map_init();
        ctx->back_home_index = 0u;
        return hfsm.res.transition(s_navigation_start);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 导航父状态事件处理函数
 * @param m 状态机实例
 * @param e 当前事件
 * @return HfsmResult 事件处理结果
 */
static HfsmResult navigation_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_NAV_START_FINISHED)
        return hfsm.res.transition(s_navigation_normal);

    if(e->id == TASK_EVENT_NAV_REACHED) {
        log_info("Reach point %u -> (%.2f, %.2f)",
                 get_current_nav_point_index(),
                 ctx->current_nav_point.x,
                 ctx->current_nav_point.y);
        finish_current_nav_point();

        if(navigation_point_requires_pollen(&ctx->current_nav_point)) {
            HfsmState* pollen_state = navigation_get_pollen_state(ctx->current_nav_point.area_type);

            if(pollen_state != NULL)
                return hfsm.res.transition(pollen_state);
        }

        if(get_current_nav_point_index() >= (get_nav_point_max() - 1u)) {
            log_info("Navigation finished, switch to return home");
            return hfsm.res.transition(s_navigation_return_home);
        }

        navigation_load_next_point(ctx);
        return hfsm.res.handled();
    }

    return hfsm.res.ignore();
}

/**
 * @brief 授粉父状态事件处理函数
 * @param m 状态机实例
 * @param e 当前事件
 * @return HfsmResult 事件处理结果
 */
static HfsmResult pollen_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_POLLEN_FINISHED) {
        finish_current_nav_point();

        if(get_current_nav_point_index() >= (get_nav_point_max() - 1u)) {
            ctx->back_home_index = 0u;
            ctx->current_area = START_END;
            return hfsm.res.transition(s_navigation_return_home);
        }

        return hfsm.res.transition(s_navigation_normal);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 遥控状态事件处理函数
 * @param m 状态机实例
 * @param e 当前事件
 * @return HfsmResult 事件处理结果
 */
static HfsmResult remote_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_SWITCH_TO_AUTO)
        return hfsm.res.transition(s_idle);

    if(e->id == TASK_EVENT_ERROR)
        return hfsm.res.transition(s_error);

    return hfsm.res.ignore();
}

/**
 * @brief 错误状态入口函数
 * @param m 状态机实例
 */
static void error_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_ERROR;
}

/**
 * @brief 空闲状态入口函数
 * @param m 状态机实例
 */
static void idle_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_IDLE;
    ctx->current_area = START_END;
    ctx->back_home_index = 0u;

    (void)chassis.set_steer_then_drive_enabled(false);
}

/**
 * @brief 导航父状态入口函数
 * @param m 状态机实例
 */
static void navigation_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_NAVIGATION;

    (void)chassis.set_steer_then_drive_enabled(true);
}

/**
 * @brief 导航起始播报状态入口函数
 * @param m 状态机实例
 */
static void navigation_start_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    ctx->current_state_id = TASK_STATE_NAVIGATION_START;
    (void)asrpro_broadcast(TEAM_INTRO);
    (void)task_post(&g_app_task, TASK_EVENT_NAV_START_FINISHED);
}

/**
 * @brief 正常导航状态入口函数
 * @param m 状态机实例
 */
static void navigation_normal_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    ctx->current_state_id = TASK_STATE_NAVIGATION_NORMAL;
    navigation_load_next_point(ctx);

    if(ctx->current_nav_point.pre_detect_joints.exist)
        arm.move_joints(&ctx->current_nav_point.pre_detect_joints.joints, TASK_ARM_SPEED_RAD_S);
    else
        arm.move_servo_zero(TASK_ARM_SPEED_RAD_S);
}

/**
 * @brief 正常导航状态动作函数
 * @param m 状态机实例
 */
static void navigation_normal_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    go_to_nav_point(ctx);
}

/**
 * @brief 返航导航状态入口函数
 * @param m 状态机实例
 */
static void navigation_return_home_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    NavPoint* back_home_points = get_back_home_points();
    Vector3 od = { 0 };

    ctx->current_state_id = TASK_STATE_NAVIGATION_RETURN_HOME;
    odom.get_odom(&od);
    ctx->nav_start_point.x = od.x;
    ctx->nav_start_point.y = od.y;
    ctx->current_nav_point = back_home_points[ctx->back_home_index];
    ctx->current_area = START_END;
    ctx->nav_start_ms = delay_now_ms();

    log_info("Return home point %u -> (%.2f, %.2f)",
             ctx->back_home_index,
             ctx->current_nav_point.x,
             ctx->current_nav_point.y);
}

/**
 * @brief 返航导航状态动作函数
 * @param m 状态机实例
 */
static void navigation_return_home_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    Vector3 od = { 0 };
    bool was_braking = ctx->nav_braking;

    if(navigation_service_brake_hold(ctx)) {
        return;
    }
    if(was_braking) {
        navigation_finish_return_home_brake(ctx);
        return;
    }

    odom.get_odom(&od);
    if(reach_at_nav_point(&ctx->current_nav_point, &od)) {
        navigation_start_brake_hold(ctx);
        log_info("Reach return home point %u -> (%.2f, %.2f)",
                 ctx->back_home_index,
                 ctx->current_nav_point.x,
                 ctx->current_nav_point.y);
        return;
    }

    follow_s_curve_nav(ctx, &od);
}

/**
 * @brief 返航点刹停结束后的续航处理函数
 * @param ctx 任务上下文
 */
static void navigation_finish_return_home_brake(TaskContext* ctx) {
    NavPoint* back_home_points = get_back_home_points();
    Vector3 od = { 0 };

    if(ctx->back_home_index + 1u >= get_back_home_point_count()) {
        log_info("Return home finished");
        (void)task_post(&g_app_task, TASK_EVENT_STOP);
        return;
    }

    odom.get_odom(&od);
    ctx->back_home_index++;
    ctx->nav_start_point.x = od.x;
    ctx->nav_start_point.y = od.y;
    ctx->current_nav_point = back_home_points[ctx->back_home_index];
    ctx->nav_start_ms = delay_now_ms();
    ctx->nav_braking = false;

    log_info("Return home point %u -> (%.2f, %.2f)",
             ctx->back_home_index,
             ctx->current_nav_point.x,
             ctx->current_nav_point.y);
}

/**
 * @brief 授粉父状态入口函数
 * @param m 状态机实例
 */
static void pollen_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN;

    (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
    navigation_broadcast_current_point(ctx);
}

/**
 * @brief A 区授粉状态入口函数
 * @param m 状态机实例
 */
static void pollen_a_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN_A;
}

/**
 * @brief A 区授粉状态动作函数
 * @param m 状态机实例
 */
static void pollen_a_action(HfsmMachine* m) {
    (void)m;

    /* A 区授粉状态:
     * 在这里填写 A 区的授粉动作
     * 动作完成后，请调用:
     * (void)task_post(&g_app_task, TASK_EVENT_POLLEN_FINISHED);
     */
}

/**
 * @brief B 区授粉状态入口函数
 * @param m 状态机实例
 */
static void pollen_b_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN_B;
}

/**
 * @brief B 区授粉状态动作函数
 * @param m 状态机实例
 */
static void pollen_b_action(HfsmMachine* m) {
    (void)m;

    /* B 区授粉状态:
     * 在这里填写 B 区的授粉动作
     * 动作完成后，请调用:
     * (void)task_post(&g_app_task, TASK_EVENT_POLLEN_FINISHED);
     */
}

/**
 * @brief C 区授粉状态入口函数
 * @param m 状态机实例
 */
static void pollen_c_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN_C;
}

/**
 * @brief C 区授粉状态动作函数
 * @param m 状态机实例
 */
static void pollen_c_action(HfsmMachine* m) {
    (void)m;

    /* C 区授粉状态:
     * 在这里填写 C 区的授粉动作
     * 动作完成后，请调用:
     * (void)task_post(&g_app_task, TASK_EVENT_POLLEN_FINISHED);
     */
}

/**
 * @brief 遥控状态入口函数
 * @param m 状态机实例
 */
static void remote_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_REMOTE;

    /* 遥控状态关闭“先转向再驱动”门控：
     * 保证遥控接管后速度指令直接生效
     */
    (void)chassis.set_steer_then_drive_enabled(false);
}

/**
 * @brief 判断当前位置是否到达目标导航点
 * @param nav_point 目标导航点
 * @param odom 当前里程计位置
 * @return bool `true` 表示已到达
 */
static bool reach_at_nav_point(const NavPoint* nav_point, const Vector3* odom) {
    float err_x = nav_point->x - odom->x;
    float err_y = nav_point->y - odom->y;

    return fabsf(err_x) < IS_REACH_THRESHOLD && fabsf(err_y) < IS_REACH_THRESHOLD;
}

/**
 * @brief 按 S 曲线轨迹跟踪当前导航点
 * @param ctx 任务上下文
 * @param od 当前里程计位置
 */
static void follow_s_curve_nav(TaskContext* ctx, const Vector3* od) {
    Vector3 angle = { 0 };
    Vector3 gyro_corrected = { 0 };
    float elapsed_s = (float)(delay_now_ms() - ctx->nav_start_ms) / 1000.0f;
    float progress = elapsed_s / REACH_TIME_S;
    float profile_pos;
    float profile_speed;
    float delta_x = ctx->current_nav_point.x - ctx->nav_start_point.x;
    float delta_y = ctx->current_nav_point.y - ctx->nav_start_point.y;
    float expect_x;
    float expect_y;
    float vx;
    float vy;
    float segment_len;
    float cruise_speed;
    float speed_limit;

    if(progress > 1.0f)
        progress = 1.0f;
    else if(progress < 0.0f)
        progress = 0.0f;

    navigation_s_curve_profile(progress, &profile_pos, &profile_speed);
    expect_x = ctx->nav_start_point.x + delta_x * profile_pos;
    expect_y = ctx->nav_start_point.y + delta_y * profile_pos;
    vx = delta_x * profile_speed / REACH_TIME_S + (expect_x - od->x) * TASK_NAV_TRACK_KP;
    vy = delta_y * profile_speed / REACH_TIME_S + (expect_y - od->y) * TASK_NAV_TRACK_KP;
    segment_len = sqrtf(delta_x * delta_x + delta_y * delta_y);
    cruise_speed = segment_len / (REACH_TIME_S * (1.0f - TASK_NAV_ACCEL_RATIO));
    speed_limit = cruise_speed + TASK_NAV_TRACK_SPEED_MARGIN_M_S;
    navigation_clamp_velocity(&vx, &vy, speed_limit);

    (void)odom.get_angle(&angle);
    (void)odom.get_gyro_corrected(&gyro_corrected);
    (void)chassis.set_velocity(
        vx,
        vy,
        chassis_yaw_hold_apply(vx, vy, 0.0f, angle.z, gyro_corrected.z, TASK_CONTROL_PERIOD_S));
}

/**
 * @brief 执行当前导航点的到点控制流程
 * @param ctx 任务上下文
 */
static void go_to_nav_point(TaskContext* ctx) {
    Vector3 od = { 0 };
    bool was_braking = ctx->nav_braking;

    if(navigation_service_brake_hold(ctx)) {
        return;
    }
    if(was_braking) {
        (void)task_post(&g_app_task, TASK_EVENT_NAV_REACHED);
        return;
    }

    odom.get_odom(&od);
    if(reach_at_nav_point(&ctx->current_nav_point, &od)) {
        navigation_start_brake_hold(ctx);
        return;
    }

    follow_s_curve_nav(ctx, &od);
}

/**
 * @brief 装载下一个导航点并刷新导航段上下文
 * @param ctx 任务上下文
 */
static void navigation_load_next_point(TaskContext* ctx) {
    Vector3 od = { 0 };

    odom.get_odom(&od);
    ctx->nav_start_point.x = od.x;
    ctx->nav_start_point.y = od.y;
    ctx->current_nav_point = get_next_nav_point();
    ctx->current_area = ctx->current_nav_point.area_type;
    ctx->nav_start_ms = delay_now_ms();
    ctx->nav_braking = false;
    log_info("Navigate point %u -> (%.2f, %.2f)",
             get_current_nav_point_index(),
             ctx->current_nav_point.x,
             ctx->current_nav_point.y);
}

/**
 * @brief 根据区域类型获取对应授粉子状态
 * @param area 区域类型
 * @return HfsmState* 对应的授粉子状态；无授粉状态时返回 `NULL`
 */
static HfsmState* navigation_get_pollen_state(AreaType area) {
    switch(area) {
        case AREA_A:
            return s_pollen_a;
        case AREA_B:
            return s_pollen_b;
        case AREA_C:
            return s_pollen_c;
        default:
            return NULL;
    }
}

/**
 * @brief 判断当前导航点是否需要进入授粉状态
 * @param nav_point 当前导航点
 * @return bool `true` 表示需要授粉
 */
static bool navigation_point_requires_pollen(const NavPoint* nav_point) {
    if(nav_point == NULL)
        return false;

    return navigation_get_pollen_state(nav_point->area_type) != NULL;
}

/**
 * @brief 将横向花型信息编码为语音播报指令
 * @param flowers 横向花型信息
 * @return AsrProCmd 语音播报指令
 */
static AsrProCmd navigation_make_x_broadcast_cmd(XFlowerType flowers) {
    return (AsrProCmd)(X_000 + ((flowers.left ? 1 : 0) << 2) + ((flowers.mid ? 1 : 0) << 1) + (flowers.right ? 1 : 0));
}

/**
 * @brief 将纵向花型信息编码为语音播报指令
 * @param flowers 纵向花型信息
 * @return AsrProCmd 语音播报指令
 */
static AsrProCmd navigation_make_y_broadcast_cmd(YFlowerType flowers) {
    return (AsrProCmd)(Y_000 + ((flowers.up ? 1 : 0) << 2) + ((flowers.mid ? 1 : 0) << 1) + (flowers.down ? 1 : 0));
}

/**
 * @brief 按当前点位信息发送对应语音播报
 * @param ctx 任务上下文
 */
static void navigation_broadcast_current_point(const TaskContext* ctx) {
    AsrProCmd cmd;

    if(ctx == NULL)
        return;

    switch(ctx->current_nav_point.area_type) {
        case AREA_A:
            cmd = navigation_make_y_broadcast_cmd(ctx->current_nav_point.y_flowers);
            break;
        case AREA_B:
        case AREA_C:
            cmd = navigation_make_x_broadcast_cmd(ctx->current_nav_point.x_flowers);
            break;
        default:
            return;
    }

    (void)asrpro_broadcast(cmd);
}

/**
 * @brief 启动导航点到点后的刹停保持
 * @param ctx 任务上下文
 */
static void navigation_start_brake_hold(TaskContext* ctx) {
    if(ctx == NULL)
        return;

    ctx->nav_braking = true;
    ctx->nav_brake_start_ms = delay_now_ms();
    (void)chassis.brake();
}

/**
 * @brief 判断导航点刹停保持是否结束
 * @param ctx 任务上下文
 * @return bool `true` 表示刹停保持已结束
 */
static bool navigation_brake_hold_done(const TaskContext* ctx) {
    const Chassis* ch = chassis.get_chassis();
    uint32_t elapsed_ms;

    if(ctx == NULL || !ctx->nav_braking)
        return false;

    elapsed_ms = delay_now_ms() - ctx->nav_brake_start_ms;
    if(elapsed_ms < TASK_NAV_BRAKE_HOLD_MS)
        return false;

    return (ch != NULL && ch->brake_latched) || elapsed_ms >= TASK_NAV_BRAKE_TIMEOUT_MS;
}

/**
 * @brief 执行导航点刹停保持服务
 * @param ctx 任务上下文
 * @return bool `true` 表示当前仍处于刹停保持阶段
 */
static bool navigation_service_brake_hold(TaskContext* ctx) {
    if(ctx == NULL || !ctx->nav_braking)
        return false;

    (void)chassis.brake();
    if(navigation_brake_hold_done(ctx)) {
        ctx->nav_braking = false;
    }

    return ctx->nav_braking;
}

/**
 * @brief 根据进度生成 S 曲线位置与速度比例
 * @param progress 当前归一化进度
 * @param position_ratio 输出位置比例
 * @param speed_ratio 输出速度比例
 */
static void navigation_s_curve_profile(float progress, float* position_ratio, float* speed_ratio) {
    const float accel_ratio = TASK_NAV_ACCEL_RATIO;
    const float cruise_speed_ratio = 1.0f / (1.0f - accel_ratio);
    float p = progress;

    if(position_ratio == NULL || speed_ratio == NULL)
        return;

    if(p <= 0.0f) {
        *position_ratio = 0.0f;
        *speed_ratio = 0.0f;
        return;
    }
    if(p >= 1.0f) {
        *position_ratio = 1.0f;
        *speed_ratio = 0.0f;
        return;
    }

    if(p < accel_ratio) {
        float u = p / accel_ratio;
        *position_ratio = cruise_speed_ratio * accel_ratio * (u * u * u - 0.5f * u * u * u * u);
        *speed_ratio = cruise_speed_ratio * u * u * (3.0f - 2.0f * u);
        return;
    }

    if(p <= 1.0f - accel_ratio) {
        *position_ratio = cruise_speed_ratio * (p - accel_ratio * 0.5f);
        *speed_ratio = cruise_speed_ratio;
        return;
    }

    p = (1.0f - p) / accel_ratio;
    *position_ratio = 1.0f - cruise_speed_ratio * accel_ratio * (p * p * p - 0.5f * p * p * p * p);
    *speed_ratio = cruise_speed_ratio * p * p * (3.0f - 2.0f * p);
}

/**
 * @brief 对平面速度进行幅值限幅
 * @param vx x 方向速度指针
 * @param vy y 方向速度指针
 * @param max_speed 最大允许速度
 */
static void navigation_clamp_velocity(float* vx, float* vy, float max_speed) {
    float speed;
    float scale;

    if(vx == NULL || vy == NULL || max_speed <= 0.0f)
        return;

    speed = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if(speed <= max_speed || speed <= 0.0f)
        return;

    scale = max_speed / speed;
    *vx *= scale;
    *vy *= scale;
}
