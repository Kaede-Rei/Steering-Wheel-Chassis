/**
 * @file task.c
 * @brief 比赛任务应用层层级状态机实现
 */

#include "task.h"

#include "asrpro.h"
#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"
#include "log.h"
#include "remote.h"
#include "nav/task_nav.h"
#include "pollen/task_pollen.h"
#include "task_internal.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

static Task s_app_task = { 0 };

static HfsmState* s_fault = NULL;
static HfsmState* s_auto = NULL;
static HfsmState* s_idle = NULL;
static HfsmState* s_navigate = NULL;
static HfsmState* s_pollinate = NULL;
static HfsmState* s_return_home = NULL;
static HfsmState* s_manual = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static HfsmResult fault_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult auto_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult navigate_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult pollinate_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult manual_handle(HfsmMachine* m, const HfsmEvent* e);

static void fault_entry(HfsmMachine* m);
static void idle_entry(HfsmMachine* m);
static void auto_entry(HfsmMachine* m);
static void navigate_entry(HfsmMachine* m);
static void navigate_action(HfsmMachine* m);
static void pollinate_entry(HfsmMachine* m);
static void pollinate_action(HfsmMachine* m);
static void return_home_entry(HfsmMachine* m);
static void return_home_action(HfsmMachine* m);
static void manual_entry(HfsmMachine* m);

static TaskContext* task_ctx_from_machine(HfsmMachine* m);
static void task_latch_fault(TaskContext* ctx, const TaskFault* fault);
static void task_make_fault(TaskFault* fault,
                            TaskFaultSource source,
                            TaskFaultLevel level,
                            TaskCancelMask cancel_mask,
                            TaskStateId owner_state,
                            int32_t code);
static void task_raise_nav_fault(TaskContext* ctx, TaskNavResult result, TaskStateId owner_state);
static void task_raise_pollen_fault(TaskContext* ctx, TaskPollenResult result);
static void task_apply_fault_cancel(TaskContext* ctx);
static const char* task_fault_source_str(TaskFaultSource source);
static const char* task_fault_level_str(TaskFaultLevel level);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化全局任务状态机
 */
void task_init(void) {
    Task* task = &s_app_task;

    memset(task, 0, sizeof(*task));
    task_nav_reset(&task->ctx.navigation);
    task_pollen_reset(&task->ctx.pollen);
    task->ctx.current_state_id = TASK_STATE_IDLE;
    task->ctx.state_before_remote = TASK_STATE_IDLE;
    chassis_yaw_hold_set_target(0.0f);

    hfsm.init(&task->fsm, &task->ctx);

    s_fault = hfsm.add_state(&task->fsm, "Fault");
    s_auto = hfsm.add_state(&task->fsm, "Auto");
    s_manual = hfsm.add_state(&task->fsm, "Manual");
    s_idle = hfsm.add_substate(&task->fsm, s_auto, "Idle");
    s_navigate = hfsm.add_substate(&task->fsm, s_auto, "Navigate");
    s_pollinate = hfsm.add_substate(&task->fsm, s_auto, "Pollinate");
    s_return_home = hfsm.add_substate(&task->fsm, s_auto, "ReturnHome");

    hfsm.set_handle(s_fault, fault_handle);
    hfsm.set_entry(s_fault, fault_entry);

    hfsm.set_handle(s_auto, auto_handle);
    hfsm.set_entry(s_auto, auto_entry);
    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_entry(s_idle, idle_entry);

    hfsm.set_handle(s_navigate, navigate_handle);
    hfsm.set_entry(s_navigate, navigate_entry);
    hfsm.set_action(s_navigate, navigate_action);

    hfsm.set_handle(s_pollinate, pollinate_handle);
    hfsm.set_entry(s_pollinate, pollinate_entry);
    hfsm.set_action(s_pollinate, pollinate_action);

    hfsm.set_entry(s_return_home, return_home_entry);
    hfsm.set_action(s_return_home, return_home_action);

    hfsm.set_handle(s_manual, manual_handle);
    hfsm.set_entry(s_manual, manual_entry);

    hfsm.set_initial(&task->fsm, s_idle);
    hfsm.start(&task->fsm);
}

/**
 * @brief 执行一次任务状态机轮询
 */
void task_process(void) {
    (void)hfsm.process(&s_app_task.fsm);
}

/**
 * @brief 投递普通任务事件
 * @param event_id 任务事件 ID
 * @return bool `true` 表示事件进入队列成功
 */
bool task_post(TaskEventId event_id) {
    return hfsm.post(&s_app_task.fsm, (HfsmEventId)event_id, NULL) == hfsm.OK;
}

/**
 * @brief 清空旧事件并投递一个高优先级任务事件
 * @param event_id 任务事件 ID
 * @return bool `true` 表示事件进入队列成功
 */
bool task_force_post(TaskEventId event_id) {
    (void)hfsm.clear(&s_app_task.fsm);
    return task_post(event_id);
}

/**
 * @brief 上报任务故障并切换到故障收口状态
 * @param fault 故障描述
 * @return bool `true` 表示故障已锁存并成功投递故障事件
 */
bool task_raise_fault(const TaskFault* fault) {
    TaskFault fallback;

    if(fault == NULL) {
        task_make_fault(&fallback,
                        TASK_FAULT_SOURCE_SYSTEM,
                        TASK_FAULT_LEVEL_FATAL,
                        TASK_CANCEL_ALL,
                        task_get_state(),
                        -1);
        fault = &fallback;
    }

    task_latch_fault(&s_app_task.ctx, fault);
    return task_force_post(TASK_EVENT_FAULT);
}

/**
 * @brief 清除当前锁存故障并回到空闲状态
 * @return bool `true` 表示清除事件投递成功
 */
bool task_clear_fault(void) {
    memset(&s_app_task.ctx.fault, 0, sizeof(s_app_task.ctx.fault));
    s_app_task.ctx.fault_latched = false;
    return task_force_post(TASK_EVENT_FAULT_CLEAR);
}

/**
 * @brief 获取当前任务状态 ID
 * @return TaskStateId 当前任务状态
 */
TaskStateId task_get_state(void) {
    return s_app_task.ctx.current_state_id;
}

/**
 * @brief 获取当前锁存故障
 * @return const TaskFault* 指向内部故障信息的只读指针
 */
const TaskFault* task_get_fault(void) {
    return &s_app_task.ctx.fault;
}

/**
 * @brief 判断当前是否存在锁存故障
 * @return bool `true` 表示存在未清除故障
 */
bool task_has_fault(void) {
    return s_app_task.ctx.fault_latched;
}

/**
 * @brief 获取任务模块内部实例
 * @return Task* 全局任务实例指针
 */
Task* task_internal_instance(void) {
    return &s_app_task;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static HfsmResult fault_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_FAULT_CLEAR)
        return hfsm.res.transition(s_idle);

    return hfsm.res.ignore();
}

static HfsmResult auto_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e->id == TASK_EVENT_SWITCH_TO_REMOTE)
        return hfsm.res.transition(s_manual);

    if(e->id == TASK_EVENT_STOP)
        return hfsm.res.transition(s_idle);

    if(e->id == TASK_EVENT_FAULT)
        return hfsm.res.transition(s_fault);

    return hfsm.res.ignore();
}

static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskFault fault;

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == TASK_EVENT_START) {
        task_nav_reset(&ctx->navigation);
        ctx->return_home.back_home_index = 0u;
        task_pollen_reset(&ctx->pollen);
        (void)asrpro_broadcast(TEAM_INTRO);

        if(task_nav_load_target(&ctx->navigation, ctx->navigation.target_index) == false) {
            task_make_fault(&fault,
                            TASK_FAULT_SOURCE_ROUTE,
                            TASK_FAULT_LEVEL_RECOVERABLE,
                            TASK_CANCEL_NAV,
                            TASK_STATE_IDLE,
                            TASK_NAV_RESULT_ROUTE_ERROR);
            task_latch_fault(ctx, &fault);
            return hfsm.res.transition(s_fault);
        }

        return hfsm.res.transition(s_navigate);
    }

    return hfsm.res.ignore();
}

static HfsmResult navigate_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskFault fault;

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == TASK_EVENT_NAV_REACHED) {
        log_info("Reach point %u -> (%.2f, %.2f)",
                 ctx->navigation.target_index,
                 ctx->navigation.target_point.x,
                 ctx->navigation.target_point.y);

        if(task_nav_target_requires_pollen(&ctx->navigation))
            return hfsm.res.transition(s_pollinate);

        if(task_nav_is_last_target(&ctx->navigation)) {
            log_info("Navigation finished, switch to return home");
            return hfsm.res.transition(s_return_home);
        }

        if(task_nav_advance(&ctx->navigation) == false) {
            task_make_fault(&fault,
                            TASK_FAULT_SOURCE_ROUTE,
                            TASK_FAULT_LEVEL_RECOVERABLE,
                            TASK_CANCEL_NAV,
                            TASK_STATE_NAVIGATE,
                            TASK_NAV_RESULT_ROUTE_ERROR);
            task_latch_fault(ctx, &fault);
            return hfsm.res.transition(s_fault);
        }

        return hfsm.res.handled();
    }

    if(e->id == TASK_EVENT_FAULT)
        return hfsm.res.transition(s_fault);

    return hfsm.res.ignore();
}

static HfsmResult pollinate_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskFault fault;

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == TASK_EVENT_POLLEN_FINISHED) {
        if(task_nav_is_last_target(&ctx->navigation))
            return hfsm.res.transition(s_return_home);

        if(task_nav_advance(&ctx->navigation) == false) {
            task_make_fault(&fault,
                            TASK_FAULT_SOURCE_ROUTE,
                            TASK_FAULT_LEVEL_RECOVERABLE,
                            TASK_CANCEL_NAV,
                            TASK_STATE_POLLINATE,
                            TASK_NAV_RESULT_ROUTE_ERROR);
            task_latch_fault(ctx, &fault);
            return hfsm.res.transition(s_fault);
        }

        return hfsm.res.transition(s_navigate);
    }

    if(e->id == TASK_EVENT_FAULT)
        return hfsm.res.transition(s_fault);

    return hfsm.res.ignore();
}

static HfsmResult manual_handle(HfsmMachine* m, const HfsmEvent* e) {
    TaskContext* ctx = task_ctx_from_machine(m);

    if(ctx == NULL)
        return hfsm.res.ignore();

    if(e->id == TASK_EVENT_SWITCH_TO_AUTO) {
        ctx->resume_from_remote = true;

        switch(ctx->state_before_remote) {
            case TASK_STATE_NAVIGATE:
                return hfsm.res.transition(s_navigate);

            case TASK_STATE_RETURN_HOME:
                return hfsm.res.transition(s_return_home);

            case TASK_STATE_POLLINATE:
                return hfsm.res.transition(s_pollinate);

            case TASK_STATE_IDLE:
            default:
                return hfsm.res.transition(s_idle);
        }
    }

    if(e->id == TASK_EVENT_FAULT)
        return hfsm.res.transition(s_fault);

    return hfsm.res.ignore();
}

static void fault_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);

    if(ctx == NULL)
        return;

    ctx->current_state_id = TASK_STATE_FAULT;
    task_apply_fault_cancel(ctx);
    log_error("Task fault: source=%s level=%s owner=%d code=%ld time=%lu",
              task_fault_source_str(ctx->fault.source),
              task_fault_level_str(ctx->fault.level),
              (int)ctx->fault.owner_state,
              (long)ctx->fault.code,
              (unsigned long)ctx->fault.time_ms);
}

static void auto_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);

    if(ctx != NULL)
        ctx->current_state_id = TASK_STATE_AUTO;
}

static void idle_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);

    if(ctx == NULL)
        return;

    ctx->current_state_id = TASK_STATE_IDLE;
    ctx->resume_from_remote = false;
    task_nav_reset(&ctx->navigation);
    ctx->return_home.back_home_index = 0u;
    task_pollen_reset(&ctx->pollen);

    (void)chassis.set_steer_then_drive_enabled(false);
}

static void navigate_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);

    if(ctx == NULL)
        return;

    ctx->current_state_id = TASK_STATE_NAVIGATE;
    ctx->resume_from_remote = false;
    (void)chassis.set_steer_then_drive_enabled(true);
}

static void navigate_action(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskNavResult result;

    if(ctx == NULL || ctx->fault_latched)
        return;

    result = task_nav_process(&ctx->navigation);

    if(result == TASK_NAV_RESULT_REACHED)
        (void)hfsm.machine.post(m, TASK_EVENT_NAV_REACHED, NULL);
    else if(result != TASK_NAV_RESULT_RUNNING) {
        log_error("Navigate failed: %s", task_nav_result_str(result));
        task_raise_nav_fault(ctx, result, TASK_STATE_NAVIGATE);
        (void)hfsm.machine.clear(m);
        (void)hfsm.machine.post(m, TASK_EVENT_FAULT, NULL);
    }
}

static void pollinate_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskPollenResult result;

    if(ctx == NULL || ctx->fault_latched)
        return;

    ctx->current_state_id = TASK_STATE_POLLINATE;
    (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);

    if(ctx->resume_from_remote) {
        ctx->resume_from_remote = false;
        return;
    }

    result = task_pollen_start(&ctx->pollen,
                               ctx->navigation.target_index,
                               ctx->navigation.current_area,
                               &ctx->navigation.target_point);
    if(result != TASK_POLLEN_RESULT_RUNNING) {
        log_error("Pollinate start failed: %s", task_pollen_result_str(result));
        task_raise_pollen_fault(ctx, result);
        (void)hfsm.machine.clear(m);
        (void)hfsm.machine.post(m, TASK_EVENT_FAULT, NULL);
    }
}

static void pollinate_action(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskPollenResult result;

    if(ctx == NULL || ctx->fault_latched)
        return;

    result = task_pollen_process(&ctx->pollen);

    if(result == TASK_POLLEN_RESULT_FINISHED)
        (void)hfsm.machine.post(m, TASK_EVENT_POLLEN_FINISHED, NULL);
    else if(result != TASK_POLLEN_RESULT_RUNNING) {
        log_error("Pollinate failed: %s", task_pollen_result_str(result));
        task_raise_pollen_fault(ctx, result);
        (void)hfsm.machine.clear(m);
        (void)hfsm.machine.post(m, TASK_EVENT_FAULT, NULL);
    }
}

static void return_home_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskFault fault;

    if(ctx == NULL)
        return;

    ctx->current_state_id = TASK_STATE_RETURN_HOME;
    ctx->navigation.current_area = START_END;

    if(ctx->resume_from_remote) {
        ctx->resume_from_remote = false;
        return;
    }

    if(task_nav_return_home_start(&ctx->navigation, &ctx->return_home) == false) {
        task_make_fault(&fault,
                        TASK_FAULT_SOURCE_ROUTE,
                        TASK_FAULT_LEVEL_RECOVERABLE,
                        TASK_CANCEL_NAV,
                        TASK_STATE_RETURN_HOME,
                        TASK_NAV_RESULT_ROUTE_ERROR);
        task_latch_fault(ctx, &fault);
        (void)hfsm.machine.clear(m);
        (void)hfsm.machine.post(m, TASK_EVENT_FAULT, NULL);
    }
}

static void return_home_action(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);
    TaskNavResult result;

    if(ctx == NULL || ctx->fault_latched)
        return;

    result = task_nav_return_home_process(&ctx->navigation, &ctx->return_home);

    if(result == TASK_NAV_RESULT_FINISHED) {
        log_info("Return home finished");
        (void)hfsm.machine.post(m, TASK_EVENT_STOP, NULL);
    }
    else if(result != TASK_NAV_RESULT_RUNNING) {
        log_error("Return home failed: %s", task_nav_result_str(result));
        task_raise_nav_fault(ctx, result, TASK_STATE_RETURN_HOME);
        (void)hfsm.machine.clear(m);
        (void)hfsm.machine.post(m, TASK_EVENT_FAULT, NULL);
    }
}

static void manual_entry(HfsmMachine* m) {
    TaskContext* ctx = task_ctx_from_machine(m);

    if(ctx == NULL)
        return;

    if(ctx->current_state_id != TASK_STATE_MANUAL)
        ctx->state_before_remote = ctx->current_state_id;
    ctx->current_state_id = TASK_STATE_MANUAL;

    (void)chassis.set_steer_then_drive_enabled(false);
    (void)chassis.brake();
    (void)chassis_yaw_hold_disable();
}

static TaskContext* task_ctx_from_machine(HfsmMachine* m) {
    return (TaskContext*)hfsm.machine.context(m);
}

static void task_latch_fault(TaskContext* ctx, const TaskFault* fault) {
    if(ctx == NULL || fault == NULL)
        return;

    ctx->fault = *fault;
    if(ctx->fault.time_ms == 0u)
        ctx->fault.time_ms = delay_now_ms();
    if(ctx->fault.owner_state == TASK_STATE_AUTO)
        ctx->fault.owner_state = ctx->current_state_id;
    ctx->fault_latched = true;
}

static void task_make_fault(TaskFault* fault,
                            TaskFaultSource source,
                            TaskFaultLevel level,
                            TaskCancelMask cancel_mask,
                            TaskStateId owner_state,
                            int32_t code) {
    if(fault == NULL)
        return;

    fault->source = source;
    fault->level = level;
    fault->cancel_mask = cancel_mask;
    fault->owner_state = owner_state;
    fault->code = code;
    fault->time_ms = delay_now_ms();
}

static void task_raise_nav_fault(TaskContext* ctx, TaskNavResult result, TaskStateId owner_state) {
    TaskFault fault;
    TaskFaultSource source = TASK_FAULT_SOURCE_NAV;

    if(result == TASK_NAV_RESULT_ROUTE_ERROR)
        source = TASK_FAULT_SOURCE_ROUTE;
    else if(result == TASK_NAV_RESULT_ODOM_ERROR)
        source = TASK_FAULT_SOURCE_ODOM;
    else if(result == TASK_NAV_RESULT_CHASSIS_ERROR)
        source = TASK_FAULT_SOURCE_CHASSIS;

    task_make_fault(&fault,
                    source,
                    TASK_FAULT_LEVEL_RECOVERABLE,
                    TASK_CANCEL_NAV,
                    owner_state,
                    (int32_t)result);
    task_latch_fault(ctx, &fault);
}

static void task_raise_pollen_fault(TaskContext* ctx, TaskPollenResult result) {
    TaskFault fault;
    TaskFaultSource source = TASK_FAULT_SOURCE_POLLEN;

    if(result == TASK_POLLEN_RESULT_ROUTE_MISSING)
        source = TASK_FAULT_SOURCE_ROUTE;
    else if(result == TASK_POLLEN_RESULT_ARM_COMMAND_FAILED ||
            result == TASK_POLLEN_RESULT_ARM_FEEDBACK_TIMEOUT ||
            result == TASK_POLLEN_RESULT_PREPOSE_TIMEOUT)
        source = TASK_FAULT_SOURCE_ARM;

    task_make_fault(&fault,
                    source,
                    TASK_FAULT_LEVEL_RECOVERABLE,
                    TASK_CANCEL_POLLEN,
                    TASK_STATE_POLLINATE,
                    (int32_t)result);
    task_latch_fault(ctx, &fault);
}

static void task_apply_fault_cancel(TaskContext* ctx) {
    TaskCancelMask mask;

    if(ctx == NULL)
        return;

    mask = ctx->fault.cancel_mask;
    if(ctx->fault.level == TASK_FAULT_LEVEL_FATAL)
        mask = TASK_CANCEL_ALL;

    if((mask & TASK_CANCEL_NAV) != 0u)
        task_nav_cancel(&ctx->navigation);

    if((mask & TASK_CANCEL_POLLEN) != 0u) {
        task_pollen_cancel(&ctx->pollen);
        (void)chassis.brake();
    }

    if((mask & TASK_CANCEL_REMOTE) != 0u) {
        remote_control_cancel();
        (void)chassis.brake();
    }

    if(mask == TASK_CANCEL_ALL) {
        task_nav_cancel(&ctx->navigation);
        task_pollen_cancel(&ctx->pollen);
        remote_control_cancel();
        (void)chassis.brake();
    }
}

static const char* task_fault_source_str(TaskFaultSource source) {
    switch(source) {
        case TASK_FAULT_SOURCE_NONE:
            return "NONE";
        case TASK_FAULT_SOURCE_CHASSIS:
            return "CHASSIS";
        case TASK_FAULT_SOURCE_ODOM:
            return "ODOM";
        case TASK_FAULT_SOURCE_ARM:
            return "ARM";
        case TASK_FAULT_SOURCE_REMOTE:
            return "REMOTE";
        case TASK_FAULT_SOURCE_NAV:
            return "NAV";
        case TASK_FAULT_SOURCE_POLLEN:
            return "POLLEN";
        case TASK_FAULT_SOURCE_ROUTE:
            return "ROUTE";
        case TASK_FAULT_SOURCE_SYSTEM:
            return "SYSTEM";
        default:
            return "UNKNOWN";
    }
}

static const char* task_fault_level_str(TaskFaultLevel level) {
    switch(level) {
        case TASK_FAULT_LEVEL_NONE:
            return "NONE";
        case TASK_FAULT_LEVEL_WARN:
            return "WARN";
        case TASK_FAULT_LEVEL_DEGRADE:
            return "DEGRADE";
        case TASK_FAULT_LEVEL_RECOVERABLE:
            return "RECOVERABLE";
        case TASK_FAULT_LEVEL_FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}
