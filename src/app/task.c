/**
 * @file task.c
 * @brief 比赛任务应用层层级状态机实现
 */

#include "task.h"

#include "asrpro.h"
#include "chassis.h"
#include "log.h"
#include "delay.h"
#include "odom.h"
#include "arm.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define IS_REACH_THRESHOLD 0.001f
#define REACH_TIME_S 1.0f
#define TASK_NAV_ACCEL_RATIO 0.05f
#define TASK_NAV_TRACK_KP 4.0f
#define TASK_NAV_TRACK_SPEED_MARGIN_M_S 0.04f
#define TASK_NAV_BRAKE_HOLD_MS 300u
#define TASK_NAV_BRAKE_TIMEOUT_MS 1500u
#define TASK_ARM_SPEED_RAD_S 12.56f

static const FiveDofArmJointArray arm_a_joints = {
    .q = { 1.54f, 2.36f, 5.66f, 2.33f, 3.14f }
};

Task g_app_task = { 0 };

static HfsmState* s_error = NULL;
static HfsmState* s_normal = NULL;
static HfsmState* s_idle = NULL;
static HfsmState* s_navigation = NULL;
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

    hfsm.init(&task->fsm, &task->ctx);

    s_error = hfsm.add_state(&task->fsm, "Error");
    s_normal = hfsm.add_state(&task->fsm, "Normal");
    s_remote = hfsm.add_state(&task->fsm, "Remote");
    s_idle = hfsm.add_substate(&task->fsm, s_normal, "Idle");
    s_navigation = hfsm.add_substate(&task->fsm, s_normal, "Navigation");
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

static HfsmResult error_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_ERROR_CLEAR)
        return hfsm.res.transition(s_idle);

    return hfsm.res.ignore();
}

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

static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_START) {
        ctx->back_home_index = 0u;
        return hfsm.res.transition(s_navigation_normal);
    }

    return hfsm.res.ignore();
}

static HfsmResult navigation_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_NAV_REACHED) {
        log_info("Reach point %u -> (%.2f, %.2f)",
                 get_current_nav_point_index(),
                 ctx->current_nav_point.x,
                 ctx->current_nav_point.y);
        finish_current_nav_point();

        /* 临时跑图模式：
         * 当前先不进入授粉状态；
         * 到任何导航点后都直接继续下一个导航点，直到最后切返航
         */
        if(get_current_nav_point_index() >= (get_nav_point_max() - 1u)) {
            log_info("Navigation finished, switch to return home");
            return hfsm.res.transition(s_navigation_return_home);
        }

        navigation_load_next_point(ctx);
        return hfsm.res.handled();
    }

    return hfsm.res.ignore();
}

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

static HfsmResult remote_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_SWITCH_TO_AUTO)
        return hfsm.res.transition(s_idle);

    if(e->id == TASK_EVENT_ERROR)
        return hfsm.res.transition(s_error);

    return hfsm.res.ignore();
}

static void error_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_ERROR;
}

static void idle_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_IDLE;
    ctx->current_area = START_END;
    ctx->back_home_index = 0u;

    /* 非导航状态先关闭“先转向再驱动”门控：
     * 避免空闲/切态期间保留导航期的底盘约束
     */
    (void)chassis.set_steer_then_drive_enabled(false);
}

static void navigation_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_NAVIGATION;

    /* 导航状态要求底盘先完成转向，再开始驱动 */
    (void)chassis.set_steer_then_drive_enabled(true);
}

static void navigation_normal_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    ctx->current_state_id = TASK_STATE_NAVIGATION_NORMAL;
    navigation_load_next_point(ctx);
}

static void navigation_normal_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    go_to_nav_point(ctx);
}

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

static void pollen_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN;

    /* 授粉状态:
     * 这里预留“语音播报 + 机械臂工作”的公共位置
     */
}

static void pollen_a_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN_A;
}

static void pollen_a_action(HfsmMachine* m) {
    (void)m;

    /* A 区授粉状态:
     * 这里预留 A 区机械臂工作流程
     */
}

static void pollen_b_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN_B;
}

static void pollen_b_action(HfsmMachine* m) {
    (void)m;

    /* B 区授粉状态:
     * 这里预留 B 区机械臂工作流程
     */
}

static void pollen_c_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN_C;
}

static void pollen_c_action(HfsmMachine* m) {
    (void)m;

    /* C 区授粉状态:
     * 这里预留 C 区机械臂工作流程
     */
}

static void remote_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_REMOTE;

    /* 遥控状态关闭“先转向再驱动”门控：
     * 保证遥控接管后速度指令直接生效
     */
    (void)chassis.set_steer_then_drive_enabled(false);
}

static bool reach_at_nav_point(const NavPoint* nav_point, const Vector3* odom) {
    float err_x = nav_point->x - odom->x;
    float err_y = nav_point->y - odom->y;

    return fabsf(err_x) < IS_REACH_THRESHOLD && fabsf(err_y) < IS_REACH_THRESHOLD;
}

static void follow_s_curve_nav(TaskContext* ctx, const Vector3* od) {
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

    /* 5% 加速 + 90% 匀速 + 5% 减速：
     * 巡航速度按曲线面积反推，确保理论段时间等于 REACH_TIME_S。
     */
    navigation_s_curve_profile(progress, &profile_pos, &profile_speed);
    expect_x = ctx->nav_start_point.x + delta_x * profile_pos;
    expect_y = ctx->nav_start_point.y + delta_y * profile_pos;
    vx = delta_x * profile_speed / REACH_TIME_S + (expect_x - od->x) * TASK_NAV_TRACK_KP;
    vy = delta_y * profile_speed / REACH_TIME_S + (expect_y - od->y) * TASK_NAV_TRACK_KP;
    segment_len = sqrtf(delta_x * delta_x + delta_y * delta_y);
    cruise_speed = segment_len / (REACH_TIME_S * (1.0f - TASK_NAV_ACCEL_RATIO));
    speed_limit = cruise_speed + TASK_NAV_TRACK_SPEED_MARGIN_M_S;
    navigation_clamp_velocity(&vx, &vy, speed_limit);

    (void)chassis.set_velocity(vx, vy, 0.0f);
}

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

static void navigation_load_next_point(TaskContext* ctx) {
    Vector3 od = { 0 };

    odom.get_odom(&od);
    ctx->nav_start_point.x = od.x;
    ctx->nav_start_point.y = od.y;
    ctx->current_nav_point = get_next_nav_point();
    ctx->current_area = ctx->current_nav_point.area_type;
    ctx->nav_start_ms = delay_now_ms();
    ctx->nav_braking = false;

    if(ctx->current_area == AREA_A) {
        /* 临时排查导航停在第一点的问题：
         * 先屏蔽 A 区导航入口的机械臂同步动作；
         * 确认是否是机械臂动作阻塞了后续导航
         */
    }

    // (void)asrpro_broadcast(TEAM_INTRO);
    log_info("Navigate point %u -> (%.2f, %.2f)",
             get_current_nav_point_index(),
             ctx->current_nav_point.x,
             ctx->current_nav_point.y);
}

static void navigation_start_brake_hold(TaskContext* ctx) {
    if(ctx == NULL)
        return;

    ctx->nav_braking = true;
    ctx->nav_brake_start_ms = delay_now_ms();
    (void)chassis.brake();
}

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

static bool navigation_service_brake_hold(TaskContext* ctx) {
    if(ctx == NULL || !ctx->nav_braking)
        return false;

    (void)chassis.brake();
    if(navigation_brake_hold_done(ctx)) {
        ctx->nav_braking = false;
    }

    return ctx->nav_braking;
}

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
