/**
 * @file task.c
 * @brief 比赛任务应用层层级状态机实现
 */
#include "task.h"

#include "arm.h"
#include "asrpro.h"
#include "chassis.h"
#include "log.h"

#include <string.h>

// ! ========================= 常量声明 ========================= ! //

/**
 * @brief 任务状态名称表
 */
static const char* const s_task_state_name_table[] = {
    "Error",
    "Normal",
    "Idle",
    "Navigation",
    "NavigationStart",
    "NavigationABC",
    "NavigationReturnZero",
    "Pollen",
    "PollenA",
    "PollenB",
    "PollenC",
    "Remote"
};

// ! ========================= 变量声明 ========================= ! //

/**
 * @brief 任务应用层单例实例
 */
static Task s_task = { 0 };

/**
 * @brief 顶层错误状态
 */
static HfsmState* s_error = NULL;

/**
 * @brief 正常非遥控父状态
 */
static HfsmState* s_normal = NULL;

/**
 * @brief 空闲状态
 */
static HfsmState* s_idle = NULL;

/**
 * @brief 导航父状态
 */
static HfsmState* s_navigation = NULL;

/**
 * @brief 启动导航状态
 */
static HfsmState* s_navigation_start = NULL;

/**
 * @brief ABC 区导航状态
 */
static HfsmState* s_navigation_abc = NULL;

/**
 * @brief 返回零点导航状态
 */
static HfsmState* s_navigation_return_zero = NULL;

/**
 * @brief 授粉父状态
 */
static HfsmState* s_pollen = NULL;

/**
 * @brief A 区授粉状态
 */
static HfsmState* s_pollen_a = NULL;

/**
 * @brief B 区授粉状态
 */
static HfsmState* s_pollen_b = NULL;

/**
 * @brief C 区授粉状态
 */
static HfsmState* s_pollen_c = NULL;

/**
 * @brief 正常遥控状态
 */
static HfsmState* s_remote = NULL;

// ! ========================= 私有函数声明 ========================= ! //

static TaskStatus task_post_event(TaskEventId event_id);
static TaskStatus task_build_state_tree(void);
static void task_set_state(TaskStateId state_id);
static void task_prepare_area_nav(AreaType area);
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
static void navigation_start_action(HfsmMachine* m);
static void navigation_abc_entry(HfsmMachine* m);
static void navigation_abc_action(HfsmMachine* m);
static void navigation_return_zero_entry(HfsmMachine* m);
static void navigation_return_zero_action(HfsmMachine* m);
static void pollen_entry(HfsmMachine* m);
static void pollen_a_entry(HfsmMachine* m);
static void pollen_a_action(HfsmMachine* m);
static void pollen_b_entry(HfsmMachine* m);
static void pollen_b_action(HfsmMachine* m);
static void pollen_c_entry(HfsmMachine* m);
static void pollen_c_action(HfsmMachine* m);
static void remote_entry(HfsmMachine* m);

// ! ========================= 接口函数实现 ========================= ! //

/**
 * @brief 任务应用层接口表
 */
#define X(name, str) .name = TASK_##name,
const struct TaskInterface task_interface = {
    { TASK_STATUS_TABLE },
    .init = task_init,
    .process = task_process,
    .start_auto = task_start_auto,
    .stop_auto = task_stop_auto,
    .enable_remote = task_enable_remote,
    .disable_remote = task_disable_remote,
    .notify_nav_reached = task_notify_nav_reached,
    .notify_pollen_finished = task_notify_pollen_finished,
    .enter_error = task_enter_error,
    .clear_error = task_clear_error,
    .state_id = task_state_id,
    .state_name = task_state_name,
    .allow_auto = task_allow_auto,
    .allow_remote = task_allow_remote,
    .get_task = task_get_task
};
#undef X

/**
 * @brief 初始化任务状态机
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_init(void) {
    memset(&s_task, 0, sizeof(s_task));
    nav_map_init();

    if(task_build_state_tree() != TASK_OK)
        return TASK_HFSM_ERROR;

    s_task.current_state_id = TASK_STATE_IDLE;
    s_task.current_area = START_END;
    s_task.current_nav_point.x = 0.0f;
    s_task.current_nav_point.y = 0.0f;
    s_task.current_nav_point.area_type = START_END;

    if(hfsm.set_initial(&s_task.fsm, s_idle) != hfsm.OK)
        return TASK_HFSM_ERROR;
    if(hfsm.start(&s_task.fsm) != hfsm.OK)
        return TASK_HFSM_ERROR;

    s_task.initialized = true;
    return TASK_OK;
}

/**
 * @brief 执行一次任务状态机轮询
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_process(void) {
    if(!s_task.initialized)
        return TASK_NOT_INITIALIZED;

    return hfsm.process(&s_task.fsm) == hfsm.OK ? TASK_OK : TASK_HFSM_ERROR;
}

/**
 * @brief 请求启动自主任务
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_start_auto(void) {
    return task_post_event(TASK_EVENT_START_AUTO);
}

/**
 * @brief 请求停止自主任务并回到空闲状态
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_stop_auto(void) {
    return task_post_event(TASK_EVENT_STOP_AUTO);
}

/**
 * @brief 请求进入遥控状态
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_enable_remote(void) {
    return task_post_event(TASK_EVENT_ENABLE_REMOTE);
}

/**
 * @brief 请求退出遥控状态
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_disable_remote(void) {
    return task_post_event(TASK_EVENT_DISABLE_REMOTE);
}

/**
 * @brief 通知当前导航目标点已经到达
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_notify_nav_reached(void) {
    return task_post_event(TASK_EVENT_NAV_REACHED);
}

/**
 * @brief 通知当前授粉流程已经完成
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_notify_pollen_finished(void) {
    return task_post_event(TASK_EVENT_POLLEN_FINISHED);
}

/**
 * @brief 请求进入错误状态
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_enter_error(void) {
    return task_post_event(TASK_EVENT_FAULT);
}

/**
 * @brief 请求清除错误状态
 * @return TaskStatus 任务应用层状态码
 */
TaskStatus task_clear_error(void) {
    return task_post_event(TASK_EVENT_CLEAR_FAULT);
}

/**
 * @brief 获取当前任务状态 ID
 * @return TaskStateId 当前任务状态 ID
 */
TaskStateId task_state_id(void) {
    return s_task.current_state_id;
}

/**
 * @brief 获取当前任务状态名称
 * @return const char* 当前任务状态名称
 */
const char* task_state_name(void) {
    if((uint32_t)s_task.current_state_id >= (sizeof(s_task_state_name_table) / sizeof(s_task_state_name_table[0])))
        return "Unknown";

    return s_task_state_name_table[s_task.current_state_id];
}

/**
 * @brief 查询当前是否允许自主任务
 * @return bool `true` 表示允许自主任务
 */
bool task_allow_auto(void) {
    if(!s_task.initialized)
        return false;

    return s_task.current_state_id != TASK_STATE_ERROR && s_task.current_state_id != TASK_STATE_REMOTE;
}

/**
 * @brief 查询当前是否允许遥控
 * @return bool `true` 表示允许遥控
 */
bool task_allow_remote(void) {
    if(!s_task.initialized)
        return false;

    return s_task.current_state_id == TASK_STATE_ERROR || s_task.current_state_id == TASK_STATE_REMOTE;
}

/**
 * @brief 获取任务应用层只读视图
 * @return const Task* 任务应用层实例指针
 */
const Task* task_get_task(void) {
    return &s_task;
}

// ! ========================= 私有函数实现 ========================= ! //

/**
 * @brief 发送任务状态机事件
 * @param event_id 任务事件 ID
 * @return TaskStatus 任务应用层状态码
 */
static TaskStatus task_post_event(TaskEventId event_id) {
    if(!s_task.initialized)
        return TASK_NOT_INITIALIZED;

    return hfsm.post(&s_task.fsm, (HfsmEventId)event_id, NULL) == hfsm.OK ? TASK_OK : TASK_HFSM_ERROR;
}

/**
 * @brief 构建任务状态树
 * @return TaskStatus 任务应用层状态码
 */
static TaskStatus task_build_state_tree(void) {
    if(hfsm.init(&s_task.fsm, &s_task) != hfsm.OK)
        return TASK_HFSM_ERROR;

    s_error = hfsm.add_state(&s_task.fsm, "Error");
    s_normal = hfsm.add_state(&s_task.fsm, "Normal");
    s_remote = hfsm.add_state(&s_task.fsm, "Remote");
    s_idle = hfsm.add_substate(&s_task.fsm, s_normal, "Idle");
    s_navigation = hfsm.add_substate(&s_task.fsm, s_normal, "Navigation");
    s_navigation_start = hfsm.add_substate(&s_task.fsm, s_navigation, "NavigationStart");
    s_navigation_abc = hfsm.add_substate(&s_task.fsm, s_navigation, "NavigationABC");
    s_navigation_return_zero = hfsm.add_substate(&s_task.fsm, s_navigation, "NavigationReturnZero");
    s_pollen = hfsm.add_substate(&s_task.fsm, s_normal, "Pollen");
    s_pollen_a = hfsm.add_substate(&s_task.fsm, s_pollen, "PollenA");
    s_pollen_b = hfsm.add_substate(&s_task.fsm, s_pollen, "PollenB");
    s_pollen_c = hfsm.add_substate(&s_task.fsm, s_pollen, "PollenC");

    if(s_error == NULL || s_normal == NULL || s_remote == NULL || s_idle == NULL || s_navigation == NULL || s_navigation_start == NULL ||
       s_navigation_abc == NULL || s_navigation_return_zero == NULL || s_pollen == NULL || s_pollen_a == NULL || s_pollen_b == NULL ||
       s_pollen_c == NULL)
        return TASK_HFSM_ERROR;

    hfsm.set_handle(s_error, error_handle);
    hfsm.set_entry(s_error, error_entry);

    hfsm.set_handle(s_normal, normal_handle);
    hfsm.set_handle(s_idle, idle_handle);
    hfsm.set_entry(s_idle, idle_entry);

    hfsm.set_handle(s_navigation, navigation_handle);
    hfsm.set_entry(s_navigation, navigation_entry);
    hfsm.set_entry(s_navigation_start, navigation_start_entry);
    hfsm.set_action(s_navigation_start, navigation_start_action);
    hfsm.set_entry(s_navigation_abc, navigation_abc_entry);
    hfsm.set_action(s_navigation_abc, navigation_abc_action);
    hfsm.set_entry(s_navigation_return_zero, navigation_return_zero_entry);
    hfsm.set_action(s_navigation_return_zero, navigation_return_zero_action);

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

    return TASK_OK;
}

/**
 * @brief 更新当前状态 ID
 * @param state_id 当前状态 ID
 */
static void task_set_state(TaskStateId state_id) {
    s_task.current_state_id = state_id;
}

/**
 * @brief 根据区域准备导航目标点
 * @param area 当前准备前往的区域
 */
static void task_prepare_area_nav(AreaType area) {
    s_task.current_area = area;
    s_task.current_nav_point = get_next_nav_point();
    s_task.current_nav_point.area_type = area;
}

/**
 * @brief 错误状态事件处理
 * @param m 状态机实例指针
 * @param e 当前事件
 * @return HfsmResult 状态处理结果
 */
static HfsmResult error_handle(HfsmMachine* m, const HfsmEvent* e) {
    Task* ctx = (Task*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_CLEAR_FAULT) {
        ctx->error_active = false;
        if(ctx->remote_enabled)
            return hfsm.res.transition(s_remote);
        return hfsm.res.transition(s_idle);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 正常非遥控父状态事件处理
 * @param m 状态机实例指针
 * @param e 当前事件
 * @return HfsmResult 状态处理结果
 */
static HfsmResult normal_handle(HfsmMachine* m, const HfsmEvent* e) {
    Task* ctx = (Task*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_ENABLE_REMOTE) {
        ctx->remote_enabled = true;
        ctx->auto_running = false;
        return hfsm.res.transition(s_remote);
    }

    if(e->id == TASK_EVENT_FAULT) {
        ctx->error_active = true;
        ctx->auto_running = false;
        return hfsm.res.transition(s_error);
    }

    if(e->id == TASK_EVENT_STOP_AUTO) {
        ctx->auto_running = false;
        return hfsm.res.transition(s_idle);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 空闲状态事件处理
 * @param m 状态机实例指针
 * @param e 当前事件
 * @return HfsmResult 状态处理结果
 */
static HfsmResult idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    Task* ctx = (Task*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_START_AUTO) {
        ctx->auto_running = true;
        ctx->current_area = AREA_A;
        return hfsm.res.transition(s_navigation_start);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 导航父状态事件处理
 * @param m 状态机实例指针
 * @param e 当前事件
 * @return HfsmResult 状态处理结果
 */
static HfsmResult navigation_handle(HfsmMachine* m, const HfsmEvent* e) {
    Task* ctx = (Task*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_NAV_REACHED) {
        finish_current_nav_point();

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

/**
 * @brief 授粉父状态事件处理
 * @param m 状态机实例指针
 * @param e 当前事件
 * @return HfsmResult 状态处理结果
 */
static HfsmResult pollen_handle(HfsmMachine* m, const HfsmEvent* e) {
    Task* ctx = (Task*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_POLLEN_FINISHED) {
        if(ctx->current_area == AREA_A) {
            ctx->current_area = AREA_B;
            return hfsm.res.transition(s_navigation_abc);
        }
        if(ctx->current_area == AREA_B) {
            ctx->current_area = AREA_C;
            return hfsm.res.transition(s_navigation_abc);
        }

        ctx->current_area = START_END;
        return hfsm.res.transition(s_navigation_return_zero);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 遥控状态事件处理
 * @param m 状态机实例指针
 * @param e 当前事件
 * @return HfsmResult 状态处理结果
 */
static HfsmResult remote_handle(HfsmMachine* m, const HfsmEvent* e) {
    Task* ctx = (Task*)hfsm_core.context(m);

    if(e->id == TASK_EVENT_DISABLE_REMOTE) {
        ctx->remote_enabled = false;
        return hfsm.res.transition(s_idle);
    }

    if(e->id == TASK_EVENT_FAULT) {
        ctx->error_active = true;
        return hfsm.res.transition(s_error);
    }

    return hfsm.res.ignore();
}

/**
 * @brief 错误状态入口动作
 * @param m 状态机实例指针
 */
static void error_entry(HfsmMachine* m) {
    (void)m;
    (void)chassis.stop();
    (void)arm.stop();
    task_set_state(TASK_STATE_ERROR);
    log_error("TASK -> Error");
}

/**
 * @brief 空闲状态入口动作
 * @param m 状态机实例指针
 */
static void idle_entry(HfsmMachine* m) {
    Task* ctx = (Task*)hfsm_core.context(m);

    ctx->auto_running = false;
    ctx->current_area = START_END;
    (void)chassis.stop();
    (void)arm.stop();
    task_set_state(TASK_STATE_IDLE);
    log_info("TASK -> Idle");
}

/**
 * @brief 导航父状态入口动作
 * @param m 状态机实例指针
 */
static void navigation_entry(HfsmMachine* m) {
    (void)m;
    task_set_state(TASK_STATE_NAVIGATION);
}

/**
 * @brief 启动导航状态入口动作
 * @param m 状态机实例指针
 */
static void navigation_start_entry(HfsmMachine* m) {
    Task* ctx = (Task*)hfsm_core.context(m);

    task_prepare_area_nav(AREA_A);
    task_set_state(TASK_STATE_NAVIGATION_START);

    /* 启动导航状态：此处只保留播报触发位置，具体播报内容由你后续决定 */
    (void)asrpro_broadcast(START);

    log_info("TASK -> NavigationStart");
    log_info("TASK nav target area: A");
    (void)ctx;
}

/**
 * @brief 启动导航状态动作
 * @param m 状态机实例指针
 */
static void navigation_start_action(HfsmMachine* m) {
    (void)m;

    /* 启动导航状态：
     * 这里预留“从启动点前往下一个导航点”的移动控制位置
     * 具体底盘怎么移动、是否调用路径跟踪、是否做角度对齐由你后续补充
     */
}

/**
 * @brief ABC 区导航状态入口动作
 * @param m 状态机实例指针
 */
static void navigation_abc_entry(HfsmMachine* m) {
    Task* ctx = (Task*)hfsm_core.context(m);

    task_prepare_area_nav(ctx->current_area);
    task_set_state(TASK_STATE_NAVIGATION_ABC);

    if(ctx->current_area == AREA_B)
        log_info("TASK -> NavigationABC, target area: B");
    else if(ctx->current_area == AREA_C)
        log_info("TASK -> NavigationABC, target area: C");
    else
        log_info("TASK -> NavigationABC");
}

/**
 * @brief ABC 区导航状态动作
 * @param m 状态机实例指针
 */
static void navigation_abc_action(HfsmMachine* m) {
    (void)m;

    /* ABC 区导航状态：
     * 这里预留“正常导航”控制位置
     * 你后续可以在这里接入导航控制、到点判定、重新规划或姿态修正逻辑
     */
}

/**
 * @brief 返回零点导航状态入口动作
 * @param m 状态机实例指针
 */
static void navigation_return_zero_entry(HfsmMachine* m) {
    (void)m;

    task_prepare_area_nav(START_END);
    task_set_state(TASK_STATE_NAVIGATION_RETURN_ZERO);
    log_info("TASK -> NavigationReturnZero");
}

/**
 * @brief 返回零点导航状态动作
 * @param m 状态机实例指针
 */
static void navigation_return_zero_action(HfsmMachine* m) {
    (void)m;

    /* 返回零点导航状态：
     * 这里预留“从最后一个导航点返回终点/启动点”的移动控制位置
     * 具体返回路径和收尾动作由你后续补充
     */
}

/**
 * @brief 授粉父状态入口动作
 * @param m 状态机实例指针
 */
static void pollen_entry(HfsmMachine* m) {
    (void)m;
    task_set_state(TASK_STATE_POLLEN);

    /* 授粉父状态：
     * 这里预留“语音播报 + 机械臂进入授粉工作流程”的公共入口位置
     * 如果后续想把 A/B/C 共用准备动作放在父状态，这里就是合适的位置
     */
}

/**
 * @brief A 区授粉状态入口动作
 * @param m 状态机实例指针
 */
static void pollen_a_entry(HfsmMachine* m) {
    (void)m;
    task_set_state(TASK_STATE_POLLEN_A);
    log_info("TASK -> PollenA");
}

/**
 * @brief A 区授粉状态动作
 * @param m 状态机实例指针
 */
static void pollen_a_action(HfsmMachine* m) {
    (void)m;

    /* A 区授粉状态：
     * 这里预留 A 区机械臂授粉工作流程
     * 具体识别、对位、伸臂、授粉与结束判定由你后续补充
     */
}

/**
 * @brief B 区授粉状态入口动作
 * @param m 状态机实例指针
 */
static void pollen_b_entry(HfsmMachine* m) {
    (void)m;
    task_set_state(TASK_STATE_POLLEN_B);
    log_info("TASK -> PollenB");
}

/**
 * @brief B 区授粉状态动作
 * @param m 状态机实例指针
 */
static void pollen_b_action(HfsmMachine* m) {
    (void)m;

    /* B 区授粉状态：
     * 这里预留 B 区机械臂授粉工作流程
     * 具体识别、对位、伸臂、授粉与结束判定由你后续补充
     */
}

/**
 * @brief C 区授粉状态入口动作
 * @param m 状态机实例指针
 */
static void pollen_c_entry(HfsmMachine* m) {
    (void)m;
    task_set_state(TASK_STATE_POLLEN_C);
    log_info("TASK -> PollenC");
}

/**
 * @brief C 区授粉状态动作
 * @param m 状态机实例指针
 */
static void pollen_c_action(HfsmMachine* m) {
    (void)m;

    /* C 区授粉状态：
     * 这里预留 C 区机械臂授粉工作流程
     * 具体识别、对位、伸臂、授粉与结束判定由你后续补充
     */
}

/**
 * @brief 遥控状态入口动作
 * @param m 状态机实例指针
 */
static void remote_entry(HfsmMachine* m) {
    Task* ctx = (Task*)hfsm_core.context(m);

    ctx->auto_running = false;
    (void)chassis.stop();
    (void)arm.stop();
    task_set_state(TASK_STATE_REMOTE);
    log_info("TASK -> Remote");
}
