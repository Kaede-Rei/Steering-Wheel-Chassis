/**
 * @file task.c
 * @brief 比赛任务应用层层级状态机实现
 */

#include "task.h"

#include "asrpro.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "log.h"
#include "task_nav.h"
#include "task_pollen.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

Task g_app_task = { 0 };

static HfsmState* s_error = NULL;
static HfsmState* s_normal = NULL;
static HfsmState* s_idle = NULL;
static HfsmState* s_navigation = NULL;
static HfsmState* s_navigation_normal = NULL;
static HfsmState* s_navigation_return_home = NULL;
static HfsmState* s_pollen = NULL;
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
static void pollen_action(HfsmMachine* m);
static void remote_entry(HfsmMachine* m);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化任务状态机
 * @param task 任务状态机实例
 */
void task_init(Task* task) {
    if(task == NULL)
        return;

    memset(task, 0, sizeof(*task));
    task_nav_reset(&task->ctx.navigation);
    task_pollen_reset(&task->ctx.pollen);
    chassis_yaw_hold_set_target(0.0f);

    hfsm.init(&task->fsm, &task->ctx);

    s_error = hfsm.add_state(&task->fsm, "Error");
    s_normal = hfsm.add_state(&task->fsm, "Normal");
    s_remote = hfsm.add_state(&task->fsm, "Remote");
    s_idle = hfsm.add_substate(&task->fsm, s_normal, "Idle");
    s_navigation = hfsm.add_substate(&task->fsm, s_normal, "Navigation");
    s_navigation_normal = hfsm.add_substate(&task->fsm, s_navigation, "NavigationNormal");
    s_navigation_return_home = hfsm.add_substate(&task->fsm, s_navigation, "NavigationReturnHome");
    s_pollen = hfsm.add_substate(&task->fsm, s_normal, "Pollen");

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
    hfsm.set_action(s_pollen, pollen_action);

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
        task_nav_reset(&ctx->navigation);
        ctx->return_home.back_home_index = 0u;
        task_pollen_reset(&ctx->pollen);
        (void)asrpro_broadcast(TEAM_INTRO);

        if(task_nav_load_target(&ctx->navigation, ctx->navigation.target_index) == false)
            return hfsm.res.transition(s_error);

        return hfsm.res.transition(s_navigation_normal);
    }

    return hfsm.res.ignore();
}

static HfsmResult navigation_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_NAV_REACHED) {
        log_info("Reach point %u -> (%.2f, %.2f)",
                 ctx->navigation.target_index,
                 ctx->navigation.target_point.x,
                 ctx->navigation.target_point.y);

        if(task_nav_target_requires_pollen(&ctx->navigation))
            return hfsm.res.transition(s_pollen);

        if(task_nav_is_last_target(&ctx->navigation)) {
            log_info("Navigation finished, switch to return home");
            return hfsm.res.transition(s_navigation_return_home);
        }

        if(task_nav_advance(&ctx->navigation) == false)
            return hfsm.res.transition(s_error);

        return hfsm.res.handled();
    }

    return hfsm.res.ignore();
}

static HfsmResult pollen_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_POLLEN_FINISHED) {
        if(task_nav_is_last_target(&ctx->navigation))
            return hfsm.res.transition(s_navigation_return_home);

        if(task_nav_advance(&ctx->navigation) == false)
            return hfsm.res.transition(s_error);

        return hfsm.res.transition(s_navigation_normal);
    }

    return hfsm.res.ignore();
}

static HfsmResult remote_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_SWITCH_TO_AUTO) {
        ctx->resume_from_remote = true;

        switch(ctx->state_before_remote) {
            case TASK_STATE_NAVIGATION:
            case TASK_STATE_NAVIGATION_NORMAL:
                return hfsm.res.transition(s_navigation_normal);

            case TASK_STATE_NAVIGATION_RETURN_HOME:
                return hfsm.res.transition(s_navigation_return_home);

            case TASK_STATE_POLLEN:
                return hfsm.res.transition(s_pollen);

            case TASK_STATE_IDLE:
            default:
                return hfsm.res.transition(s_idle);
        }
    }

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
    ctx->resume_from_remote = false;
    task_nav_reset(&ctx->navigation);
    ctx->return_home.back_home_index = 0u;
    task_pollen_reset(&ctx->pollen);

    (void)chassis.set_steer_then_drive_enabled(false);
}

static void navigation_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_NAVIGATION;

    (void)chassis.set_steer_then_drive_enabled(true);
}

static void navigation_normal_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);

    ctx->current_state_id = TASK_STATE_NAVIGATION_NORMAL;
    ctx->resume_from_remote = false;
}

static void navigation_normal_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    TaskNavResult result = task_nav_process(&ctx->navigation);

    if(result == TASK_NAV_RESULT_REACHED)
        (void)task_post(&g_app_task, TASK_EVENT_NAV_REACHED);
    else if(result == TASK_NAV_RESULT_ERROR)
        (void)task_post(&g_app_task, TASK_EVENT_ERROR);
}

static void navigation_return_home_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_NAVIGATION_RETURN_HOME;
    ctx->navigation.current_area = START_END;

    if(ctx->resume_from_remote) {
        ctx->resume_from_remote = false;
        return;
    }

    if(task_nav_return_home_start(&ctx->navigation, &ctx->return_home) == false)
        (void)task_post(&g_app_task, TASK_EVENT_ERROR);
}

static void navigation_return_home_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    TaskNavResult result = task_nav_return_home_process(&ctx->navigation, &ctx->return_home);

    if(result == TASK_NAV_RESULT_FINISHED) {
        log_info("Return home finished");
        (void)task_post(&g_app_task, TASK_EVENT_STOP);
    }
    else if(result == TASK_NAV_RESULT_ERROR) {
        (void)task_post(&g_app_task, TASK_EVENT_ERROR);
    }
}

static void pollen_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    ctx->current_state_id = TASK_STATE_POLLEN;

    (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
    if(ctx->resume_from_remote) {
        ctx->resume_from_remote = false;
        return;
    }

    task_pollen_start(&ctx->pollen, ctx->navigation.target_index, ctx->navigation.current_area, &ctx->navigation.target_point);
}

static void pollen_action(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    TaskPollenResult result = task_pollen_process(&ctx->pollen);

    if(result == TASK_POLLEN_RESULT_FINISHED)
        (void)task_post(&g_app_task, TASK_EVENT_POLLEN_FINISHED);
    else if(result == TASK_POLLEN_RESULT_ERROR)
        (void)task_post(&g_app_task, TASK_EVENT_ERROR);
}

static void remote_entry(HfsmMachine* m) {
    TaskContext* ctx = (TaskContext*)hfsm_core.context(m);
    if(ctx->current_state_id != TASK_STATE_REMOTE)
        ctx->state_before_remote = ctx->current_state_id;
    ctx->current_state_id = TASK_STATE_REMOTE;

    (void)chassis.set_steer_then_drive_enabled(false);
}
