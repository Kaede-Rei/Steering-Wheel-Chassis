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

#define IS_REACH_THRESHOLD 0.01f
#define REACH_TIME_S 1.0f
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
static void go_to_nav_point(TaskContext* ctx);

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
        (void)chassis.brake();

        if(ctx->current_area == PASS_BY) {
            finish_current_nav_point();
            if(get_current_nav_point_index() >= (get_nav_point_max() - 1u))
                return hfsm.res.transition(s_navigation_return_home);
            return hfsm.res.transition(s_navigation_normal);
        }

        if(ctx->current_area == AREA_A)
            return hfsm.res.transition(s_pollen_a);
        if(ctx->current_area == AREA_B)
            return hfsm.res.transition(s_pollen_b);
        if(ctx->current_area == AREA_C)
            return hfsm.res.transition(s_pollen_c);

        return hfsm.res.transition(s_idle);
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
}

static void navigation_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_NAVIGATION;
}

static void navigation_normal_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    ctx->current_state_id = TASK_STATE_NAVIGATION_NORMAL;
    ctx->current_nav_point = get_next_nav_point();
    ctx->current_area = ctx->current_nav_point.area_type;
    ctx->nav_start_ms = delay_now_ms();

    if(ctx->current_area == AREA_A) {
        if(arm.is_ready())
            (void)arm.move_joints(&arm_a_joints, TASK_ARM_SPEED_RAD_S);
    }

    (void)asrpro_broadcast(TEAM_INTRO);
    log_info("Navigate point %u -> (%.2f, %.2f)", get_current_nav_point_index(), ctx->current_nav_point.x, ctx->current_nav_point.y);
}

static void navigation_normal_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    go_to_nav_point(ctx);
}

static void navigation_return_home_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    NavPoint* back_home_points = get_back_home_points();

    ctx->current_state_id = TASK_STATE_NAVIGATION_RETURN_HOME;
    ctx->current_nav_point = back_home_points[ctx->back_home_index];
    ctx->current_area = START_END;
    ctx->nav_start_ms = delay_now_ms();
}

static void navigation_return_home_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    Vector3 od = { 0 };
    NavPoint* back_home_points = get_back_home_points();
    float remain_s;
    uint32_t elapsed_ms = delay_now_ms() - ctx->nav_start_ms;

    odom.get_odom(&od);
    if(reach_at_nav_point(&ctx->current_nav_point, &od) || elapsed_ms >= (uint32_t)(REACH_TIME_S * 1000.0f)) {
        (void)chassis.brake();

        if(ctx->back_home_index + 1u >= get_back_home_point_count()) {
            (void)task_post(&g_app_task, TASK_EVENT_STOP);
            return;
        }

        ctx->back_home_index++;
        ctx->current_nav_point = back_home_points[ctx->back_home_index];
        ctx->nav_start_ms = delay_now_ms();
        return;
    }

    remain_s = REACH_TIME_S - (float)elapsed_ms / 1000.0f;
    if(remain_s < 0.05f)
        remain_s = 0.05f;

    (void)chassis.set_velocity(
        (ctx->current_nav_point.x - od.x) / remain_s,
        (ctx->current_nav_point.y - od.y) / remain_s,
        0.0f);
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
}

static bool reach_at_nav_point(const NavPoint* nav_point, const Vector3* odom) {
    float err_x = nav_point->x - odom->x;
    float err_y = nav_point->y - odom->y;

    return fabsf(err_x) < IS_REACH_THRESHOLD && fabsf(err_y) < IS_REACH_THRESHOLD;
}

static void go_to_nav_point(TaskContext* ctx) {
    Vector3 od = { 0 };
    float remain_s;
    uint32_t elapsed_ms = delay_now_ms() - ctx->nav_start_ms;

    odom.get_odom(&od);
    if(reach_at_nav_point(&ctx->current_nav_point, &od) || elapsed_ms >= (uint32_t)(REACH_TIME_S * 1000.0f)) {
        (void)task_post(&g_app_task, TASK_EVENT_NAV_REACHED);
        return;
    }

    remain_s = REACH_TIME_S - (float)elapsed_ms / 1000.0f;
    if(remain_s < 0.05f)
        remain_s = 0.05f;

    (void)chassis.set_velocity(
        (ctx->current_nav_point.x - od.x) / remain_s,
        (ctx->current_nav_point.y - od.y) / remain_s,
        0.0f);
}
