#include "competition.h"

#include "arm.h"
#include "chassis.h"
#include "delay.h"
#include "hfsm/hfsm_core.h"
#include "line_sensor.h"
#include "log.h"
#include "visual_comms.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 私 有 宏 定 义 ========================= ! //

#define COMPETITION_D_HANDOFF_STEP_READY 0u

// ! ========================= 私 有 类 型 声 明 ========================= ! //

typedef struct {
    HfsmMachine machine;
    Competition snapshot;
    uint32_t state_entry_timestamp_ms;
    uint32_t scan_request_timestamp_ms;
    uint32_t uav_handoff_start_timestamp_ms;
    bool remote_link_loss_logged;
} CompetitionContext;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static HfsmResult competition_root_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_active_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_idle_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_start_broadcast_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_go_a_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_a_scan_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_a_pollen_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_go_b_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_b_scan_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_b_pollen_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_go_c_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_c_scan_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_c_pollen_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_go_d_handoff_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_go_home_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_finish_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_stopped_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_fault_handle(HfsmMachine* m, const HfsmEvent* e);
static HfsmResult competition_estop_handle(HfsmMachine* m, const HfsmEvent* e);

static void competition_idle_entry(HfsmMachine* m);
static void competition_start_broadcast_entry(HfsmMachine* m);
static void competition_go_a_entry(HfsmMachine* m);
static void competition_a_scan_entry(HfsmMachine* m);
static void competition_a_pollen_entry(HfsmMachine* m);
static void competition_go_b_entry(HfsmMachine* m);
static void competition_b_scan_entry(HfsmMachine* m);
static void competition_b_pollen_entry(HfsmMachine* m);
static void competition_go_c_entry(HfsmMachine* m);
static void competition_c_scan_entry(HfsmMachine* m);
static void competition_c_pollen_entry(HfsmMachine* m);
static void competition_go_d_handoff_entry(HfsmMachine* m);
static void competition_go_home_entry(HfsmMachine* m);
static void competition_finish_entry(HfsmMachine* m);
static void competition_stopped_entry(HfsmMachine* m);
static void competition_fault_entry(HfsmMachine* m);
static void competition_estop_entry(HfsmMachine* m);

static CompetitionContext* competition_context(HfsmMachine* m);
static CompetitionState competition_state_from_node(const HfsmState* state);
static bool competition_post_raw(CompetitionEvent event_id, uint32_t argument);
static MissionFaultCause competition_event_fault_cause(const HfsmEvent* e, MissionFaultCause fallback);
static void competition_clear_runtime(CompetitionContext* ctx);
static void competition_set_state(HfsmMachine* m, CompetitionState state);
static void competition_set_phase(HfsmMachine* m, MissionPhase phase);
static void competition_sync_attempts(HfsmMachine* m);
static void competition_emit_mission_event(void);
static void competition_send_voice_event(MissionVoiceEventId event_id, MissionZoneId zone, uint8_t sex_or_result);
static void competition_stop_outputs(bool brake_chassis);
static void competition_enter_scan(HfsmMachine* m, CompetitionState state, MissionZoneId zone);
static HfsmResult competition_handle_scan_policy(
    HfsmMachine* m,
    const HfsmEvent* e,
    MissionZoneId zone,
    const HfsmState* female_target,
    const HfsmState* advance_target);
static void competition_finish_zone(HfsmMachine* m);
static void competition_prepare_fault(HfsmMachine* m, MissionFaultCause cause);
static bool competition_reset_allowed(const HfsmState* current);
static bool competition_is_active_state(const HfsmState* state);
static bool competition_is_scan_state(const HfsmState* state);
static bool competition_is_pollen_state(const HfsmState* state);
static bool competition_is_reportable_fault(MissionFaultCause cause);
static void competition_watch_runtime_safety(CompetitionContext* ctx, uint32_t now_ms);
static void competition_handle_scan_retry_request(HfsmMachine* m, MissionZoneId zone, MissionRecognitionAnomaly anomaly);
static void competition_handle_scan_skip(HfsmMachine* m, MissionZoneId zone, const HfsmState* advance_target, MissionRecognitionAnomaly anomaly);
static const char* competition_recognition_anomaly_str(MissionRecognitionAnomaly anomaly);

// ! ========================= 私 有 常 量 声 明 ========================= ! //

static const char* s_competition_status_str[] = {
    "OK",
    "Invalid Parameter",
    "Not Initialized",
    "No Event Space",
};

static const char* s_competition_state_str[] = {
    "IDLE",
    "START_BROADCAST",
    "GO_A",
    "A_SCAN",
    "A_POLLEN",
    "GO_B",
    "B_SCAN",
    "B_POLLEN",
    "GO_C",
    "C_SCAN",
    "C_POLLEN",
    "GO_D_HANDOFF",
    "GO_HOME",
    "FINISH",
    "STOPPED",
    "FAULT",
    "ESTOP",
};

static const char* s_competition_event_str[] = {
    "<invalid>",
    "START",
    "STOP",
    "ESTOP",
    "RESET",
    "ZONE_REACHED",
    "FEMALE_RESULT",
    "MALE_RESULT",
    "RETRY_SCAN",
    "SKIP_TARGET",
    "ACTION_COMPLETE",
    "TIMEOUT",
    "TERMINAL_FAULT",
};

static const HfsmState s_competition_root = {
    .name = "competition.root",
    .parent = NULL,
    .handle = competition_root_handle,
};

static const HfsmState s_competition_active = {
    .name = "competition.active",
    .parent = &s_competition_root,
    .handle = competition_active_handle,
};

static const HfsmState s_competition_idle = {
    .name = "competition.idle",
    .parent = &s_competition_root,
    .handle = competition_idle_handle,
    .entry = competition_idle_entry,
};

static const HfsmState s_competition_start_broadcast = {
    .name = "competition.start_broadcast",
    .parent = &s_competition_active,
    .handle = competition_start_broadcast_handle,
    .entry = competition_start_broadcast_entry,
};

static const HfsmState s_competition_go_a = {
    .name = "competition.go_a",
    .parent = &s_competition_active,
    .handle = competition_go_a_handle,
    .entry = competition_go_a_entry,
};

static const HfsmState s_competition_a_scan = {
    .name = "competition.a_scan",
    .parent = &s_competition_active,
    .handle = competition_a_scan_handle,
    .entry = competition_a_scan_entry,
};

static const HfsmState s_competition_a_pollen = {
    .name = "competition.a_pollen",
    .parent = &s_competition_active,
    .handle = competition_a_pollen_handle,
    .entry = competition_a_pollen_entry,
};

static const HfsmState s_competition_go_b = {
    .name = "competition.go_b",
    .parent = &s_competition_active,
    .handle = competition_go_b_handle,
    .entry = competition_go_b_entry,
};

static const HfsmState s_competition_b_scan = {
    .name = "competition.b_scan",
    .parent = &s_competition_active,
    .handle = competition_b_scan_handle,
    .entry = competition_b_scan_entry,
};

static const HfsmState s_competition_b_pollen = {
    .name = "competition.b_pollen",
    .parent = &s_competition_active,
    .handle = competition_b_pollen_handle,
    .entry = competition_b_pollen_entry,
};

static const HfsmState s_competition_go_c = {
    .name = "competition.go_c",
    .parent = &s_competition_active,
    .handle = competition_go_c_handle,
    .entry = competition_go_c_entry,
};

static const HfsmState s_competition_c_scan = {
    .name = "competition.c_scan",
    .parent = &s_competition_active,
    .handle = competition_c_scan_handle,
    .entry = competition_c_scan_entry,
};

static const HfsmState s_competition_c_pollen = {
    .name = "competition.c_pollen",
    .parent = &s_competition_active,
    .handle = competition_c_pollen_handle,
    .entry = competition_c_pollen_entry,
};

static const HfsmState s_competition_go_d_handoff = {
    .name = "competition.go_d_handoff",
    .parent = &s_competition_active,
    .handle = competition_go_d_handoff_handle,
    .entry = competition_go_d_handoff_entry,
};

static const HfsmState s_competition_go_home = {
    .name = "competition.go_home",
    .parent = &s_competition_active,
    .handle = competition_go_home_handle,
    .entry = competition_go_home_entry,
};

static const HfsmState s_competition_finish = {
    .name = "competition.finish",
    .parent = &s_competition_root,
    .handle = competition_finish_handle,
    .entry = competition_finish_entry,
};

static const HfsmState s_competition_stopped = {
    .name = "competition.stopped",
    .parent = &s_competition_root,
    .handle = competition_stopped_handle,
    .entry = competition_stopped_entry,
};

static const HfsmState s_competition_fault = {
    .name = "competition.fault",
    .parent = &s_competition_root,
    .handle = competition_fault_handle,
    .entry = competition_fault_entry,
};

static const HfsmState s_competition_estop = {
    .name = "competition.estop",
    .parent = &s_competition_root,
    .handle = competition_estop_handle,
    .entry = competition_estop_entry,
};

// ! ========================= 私 有 变 量 声 明 ========================= ! //

static CompetitionContext s_competition = { 0 };

// ! ========================= 接 口 单 例 定 义 ========================= ! //

const struct CompetitionInterface competition_interface = {
#define X(name, str) .name = COMPETITION_##name,
    { COMPETITION_STATUS_TABLE },
#undef X
    .init = competition_init,
    .process = competition_process,
    .process_all = competition_process_all,
    .post_event = competition_post_event,
    .handle_command = competition_handle_command,
    .handle_recognition = competition_handle_recognition,
    .handle_uav_handoff_ack = competition_handle_uav_handoff_ack,
    .report_fault = competition_report_fault,
    .get_state_id = competition_get_state_id,
    .get_state = competition_get_state,
    .status_str = competition_status_str,
    .state_str = competition_state_str,
    .event_str = competition_event_str,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

CompetitionStatus competition_init(void) {
    memset(&s_competition, 0, sizeof(s_competition));
    mission.init();
    hfsm_core.init(&s_competition.machine, &s_competition_idle, &s_competition);
    s_competition.snapshot.initialized = true;
    s_competition.snapshot.current_state = competition_state_from_node(hfsm_core.state(&s_competition.machine));
    competition_sync_attempts(&s_competition.machine);
    return COMPETITION_OK;
}

CompetitionStatus competition_process(void) {
    const ms_t now_ms = delay_now_ms();

    if(!s_competition.snapshot.initialized) {
        return COMPETITION_NOT_INITIALIZED;
    }

    mission.touch_dependency(MISSION_DEPENDENCY_ID_MISSION_HEARTBEAT, now_ms);
    mission.update_freshness(now_ms);
    competition_watch_runtime_safety(&s_competition, now_ms);
    hfsm_core.process(&s_competition.machine);
    s_competition.snapshot.current_state = competition_state_from_node(hfsm_core.state(&s_competition.machine));
    return COMPETITION_OK;
}

CompetitionStatus competition_process_all(void) {
    const ms_t now_ms = delay_now_ms();

    if(!s_competition.snapshot.initialized) {
        return COMPETITION_NOT_INITIALIZED;
    }

    mission.touch_dependency(MISSION_DEPENDENCY_ID_MISSION_HEARTBEAT, now_ms);
    mission.update_freshness(now_ms);
    competition_watch_runtime_safety(&s_competition, now_ms);
    hfsm_core.process_all(&s_competition.machine);
    s_competition.snapshot.current_state = competition_state_from_node(hfsm_core.state(&s_competition.machine));
    return COMPETITION_OK;
}

CompetitionStatus competition_post_event(CompetitionEvent event_id, uint32_t argument) {
    if(!s_competition.snapshot.initialized) {
        return COMPETITION_NOT_INITIALIZED;
    }
    if(event_id < COMPETITION_EVENT_START || event_id > COMPETITION_EVENT_TERMINAL_FAULT) {
        return COMPETITION_INVALID_PARAM;
    }

    return competition_post_raw(event_id, argument) ? COMPETITION_OK : COMPETITION_NO_SPACE;
}

CompetitionStatus competition_handle_command(MissionCommand command) {
    CompetitionEvent event_id;

    switch(command) {
    case MISSION_COMMAND_START:
        event_id = COMPETITION_EVENT_START;
        break;
    case MISSION_COMMAND_STOP:
        event_id = COMPETITION_EVENT_STOP;
        break;
    case MISSION_COMMAND_ESTOP:
        event_id = COMPETITION_EVENT_ESTOP;
        break;
    case MISSION_COMMAND_RESET:
        event_id = COMPETITION_EVENT_RESET;
        break;
    default:
        return COMPETITION_INVALID_PARAM;
    }

    return competition_post_event(event_id, 0u);
}

CompetitionStatus competition_handle_recognition(const MissionRecognitionResult* result, uint32_t now_ms) {
    CompetitionContext* ctx = &s_competition;
    const HfsmState* current = NULL;
    MissionRecognitionPolicy policy = { 0 };
    MissionStatus mission_status = MISSION_OK;
    MissionZoneId zone = MISSION_ZONE_NONE;

    if(!ctx->snapshot.initialized) {
        return COMPETITION_NOT_INITIALIZED;
    }

    current = hfsm_core.state(&ctx->machine);
    if(current == NULL) {
        return COMPETITION_INVALID_PARAM;
    }

    if(competition_is_scan_state(current) == false) {
        mission_status = mission.note_recognition(result, now_ms);
        return (mission_status == MISSION_OK) ? COMPETITION_OK : COMPETITION_INVALID_PARAM;
    }

    mission_status = mission.evaluate_recognition(result, now_ms, &policy);
    if(mission_status != MISSION_OK) {
        return COMPETITION_INVALID_PARAM;
    }

    zone = mission.get_state()->current_zone;
    switch(policy.action) {
    case MISSION_RECOGNITION_ACTION_FEMALE_POLL:
        return competition_post_event(COMPETITION_EVENT_FEMALE_RESULT, 0u);

    case MISSION_RECOGNITION_ACTION_ADVANCE_NO_POLL:
        return competition_post_event(COMPETITION_EVENT_MALE_RESULT, 0u);

    case MISSION_RECOGNITION_ACTION_RETRY_SCAN:
        if(policy.anomaly == MISSION_RECOGNITION_ANOMALY_WRONG_ZONE) {
            ctx->snapshot.attempts.wrong_zone_count++;
        }
        else {
            ctx->snapshot.attempts.scan_retry_count++;
        }
        competition_sync_attempts(&ctx->machine);
        log_warn(
            "competition: %s recognition for zone %u -> retry",
            competition_recognition_anomaly_str(policy.anomaly),
            (unsigned)zone);
        return competition_post_event(COMPETITION_EVENT_RETRY_SCAN, (uint32_t)policy.anomaly);

    case MISSION_RECOGNITION_ACTION_SKIP_TARGET:
        log_warn(
            "competition: %s recognition for zone %u -> skip",
            competition_recognition_anomaly_str(policy.anomaly),
            (unsigned)zone);
        return competition_post_event(COMPETITION_EVENT_SKIP_TARGET, (uint32_t)policy.anomaly);

    case MISSION_RECOGNITION_ACTION_TERMINAL_FAULT:
        return competition_post_event(COMPETITION_EVENT_TERMINAL_FAULT, (uint32_t)policy.fault_cause);

    case MISSION_RECOGNITION_ACTION_IGNORE:
    default:
        return COMPETITION_OK;
    }
}

CompetitionStatus competition_handle_uav_handoff_ack(const MissionUavHandoffAck* ack, uint32_t now_ms) {
    CompetitionContext* ctx = &s_competition;
    const HfsmState* current = NULL;
    MissionStatus mission_status = MISSION_OK;

    if(!ctx->snapshot.initialized) {
        return COMPETITION_NOT_INITIALIZED;
    }
    if(ack == NULL) {
        return COMPETITION_INVALID_PARAM;
    }

    mission_status = mission.note_uav_handoff_ack(ack, now_ms);
    if(mission_status != MISSION_OK) {
        return COMPETITION_INVALID_PARAM;
    }

    current = hfsm_core.state(&ctx->machine);
    if(current != &s_competition_go_d_handoff) {
        return COMPETITION_OK;
    }

    switch(ack->status) {
    case MISSION_UAV_HANDOFF_SUCCESS:
        return competition_post_event(COMPETITION_EVENT_ACTION_COMPLETE, 0u);

    case MISSION_UAV_HANDOFF_FAIL_TERMINAL:
        return competition_post_event(COMPETITION_EVENT_TERMINAL_FAULT, MISSION_FAULT_UAV_HANDOFF_FAIL_TERMINAL);

    case MISSION_UAV_HANDOFF_BUSY_RETRYABLE:
        if((now_ms - ctx->uav_handoff_start_timestamp_ms) >= UAV_HANDOFF_TIMEOUT_MS) {
            return competition_post_event(COMPETITION_EVENT_TIMEOUT, MISSION_FAULT_UAV_HANDOFF_TIMEOUT);
        }
        if(ctx->snapshot.attempts.uav_handoff_retry_count < 1u) {
            ctx->snapshot.attempts.uav_handoff_retry_count++;
            competition_sync_attempts(&ctx->machine);
            (void)mission.start_uav_handoff(COMPETITION_D_HANDOFF_STEP_READY, ctx->snapshot.attempts.uav_handoff_retry_count, now_ms);
            (void)visual_comms.send_uav_handoff_request(COMPETITION_D_HANDOFF_STEP_READY);
            log_warn("competition: UAV handoff busy, retry %u", (unsigned)ctx->snapshot.attempts.uav_handoff_retry_count);
        }
        else {
            log_warn("competition: UAV handoff busy after retry budget exhausted; waiting for timeout");
        }
        return COMPETITION_OK;

    default:
        return COMPETITION_INVALID_PARAM;
    }
}

CompetitionStatus competition_report_fault(MissionFaultCause cause) {
    if(!s_competition.snapshot.initialized) {
        return COMPETITION_NOT_INITIALIZED;
    }
    if(competition_is_reportable_fault(cause) == false) {
        return COMPETITION_INVALID_PARAM;
    }

    return competition_post_event(COMPETITION_EVENT_TERMINAL_FAULT, (uint32_t)cause);
}

CompetitionState competition_get_state_id(void) {
    if(!s_competition.snapshot.initialized) {
        return COMPETITION_STATE_IDLE;
    }

    return competition_state_from_node(hfsm_core.state(&s_competition.machine));
}

const Competition* competition_get_state(void) {
    return &s_competition.snapshot;
}

const char* competition_status_str(CompetitionStatus status) {
    if(status < COMPETITION_OK || status > COMPETITION_NO_SPACE) {
        return "Unknown CompetitionStatus";
    }

    return s_competition_status_str[status];
}

const char* competition_state_str(CompetitionState state) {
    if(state < COMPETITION_STATE_IDLE || state > COMPETITION_STATE_ESTOP) {
        return "UNKNOWN_STATE";
    }

    return s_competition_state_str[state];
}

const char* competition_event_str(CompetitionEvent event_id) {
    if(event_id < COMPETITION_EVENT_START || event_id > COMPETITION_EVENT_TERMINAL_FAULT) {
        return s_competition_event_str[0];
    }

    return s_competition_event_str[event_id];
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static HfsmResult competition_root_handle(HfsmMachine* m, const HfsmEvent* e) {
    CompetitionContext* ctx = competition_context(m);
    const HfsmState* current = hfsm_core.state(m);

    if(e == NULL || ctx == NULL || current == NULL) {
        return hfsm_core.res.ignore();
    }

    switch((CompetitionEvent)e->id) {
    case COMPETITION_EVENT_RESET:
        if(competition_reset_allowed(current)) {
            hfsm_core.clear(m);
            if(mission.reset() == MISSION_OK) {
                competition_clear_runtime(ctx);
                return (current == &s_competition_idle)
                    ? hfsm_core.res.handled()
                    : hfsm_core.res.transition(&s_competition_idle);
            }
        }
        log_warn("competition: RESET ignored while %s", competition_state_str(competition_state_from_node(current)));
        return hfsm_core.res.handled();

    case COMPETITION_EVENT_ESTOP:
        if(current != &s_competition_estop) {
            ctx->snapshot.pending_fault_cause = MISSION_FAULT_ESTOP_REQUESTED;
            return hfsm_core.res.transition(&s_competition_estop);
        }
        return hfsm_core.res.handled();

    case COMPETITION_EVENT_TERMINAL_FAULT:
        if(current != &s_competition_fault && current != &s_competition_estop) {
            competition_prepare_fault(m, competition_event_fault_cause(e, MISSION_FAULT_MISSION_HEARTBEAT_LOSS));
            return hfsm_core.res.transition(&s_competition_fault);
        }
        return hfsm_core.res.handled();

    default:
        return hfsm_core.res.ignore();
    }
}

static HfsmResult competition_active_handle(HfsmMachine* m, const HfsmEvent* e) {
    CompetitionContext* ctx = competition_context(m);

    if(e == NULL || ctx == NULL) {
        return hfsm_core.res.ignore();
    }

    switch((CompetitionEvent)e->id) {
    case COMPETITION_EVENT_START:
        if(!ctx->snapshot.duplicate_start_logged) {
            log_warn("competition: duplicate START ignored while active");
            ctx->snapshot.duplicate_start_logged = true;
        }
        return hfsm_core.res.handled();

    case COMPETITION_EVENT_STOP:
        return hfsm_core.res.transition(&s_competition_stopped);

    default:
        return hfsm_core.res.ignore();
    }
}

static HfsmResult competition_idle_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e != NULL && e->id == COMPETITION_EVENT_START) {
        return hfsm_core.res.transition(&s_competition_start_broadcast);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_start_broadcast_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e != NULL && e->id == COMPETITION_EVENT_ACTION_COMPLETE) {
        return hfsm_core.res.transition(&s_competition_go_a);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_go_a_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e != NULL && e->id == COMPETITION_EVENT_ZONE_REACHED) {
        return hfsm_core.res.transition(&s_competition_a_scan);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_a_scan_handle(HfsmMachine* m, const HfsmEvent* e) {
    return competition_handle_scan_policy(m, e, MISSION_ZONE_A, &s_competition_a_pollen, &s_competition_go_b);
}

static HfsmResult competition_a_pollen_handle(HfsmMachine* m, const HfsmEvent* e) {
    CompetitionContext* ctx = competition_context(m);

    if(e == NULL || ctx == NULL) {
        return hfsm_core.res.ignore();
    }
    if(e->id == COMPETITION_EVENT_ACTION_COMPLETE) {
        competition_send_voice_event(MISSION_VOICE_EVENT_POLLINATION_DONE, MISSION_ZONE_A, FLOWER_SEX_FEMALE);
        competition_finish_zone(m);
        return hfsm_core.res.transition(&s_competition_go_b);
    }
    if(e->id == COMPETITION_EVENT_TIMEOUT) {
        competition_prepare_fault(m, competition_event_fault_cause(e, MISSION_FAULT_ARM_ACTION_TIMEOUT));
        return hfsm_core.res.transition(&s_competition_fault);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_go_b_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e != NULL && e->id == COMPETITION_EVENT_ZONE_REACHED) {
        return hfsm_core.res.transition(&s_competition_b_scan);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_b_scan_handle(HfsmMachine* m, const HfsmEvent* e) {
    return competition_handle_scan_policy(m, e, MISSION_ZONE_B, &s_competition_b_pollen, &s_competition_go_c);
}

static HfsmResult competition_b_pollen_handle(HfsmMachine* m, const HfsmEvent* e) {
    if(e == NULL) {
        return hfsm_core.res.ignore();
    }
    if(e->id == COMPETITION_EVENT_ACTION_COMPLETE) {
        competition_send_voice_event(MISSION_VOICE_EVENT_POLLINATION_DONE, MISSION_ZONE_B, FLOWER_SEX_FEMALE);
        competition_finish_zone(m);
        return hfsm_core.res.transition(&s_competition_go_c);
    }
    if(e->id == COMPETITION_EVENT_TIMEOUT) {
        competition_prepare_fault(m, competition_event_fault_cause(e, MISSION_FAULT_ARM_ACTION_TIMEOUT));
        return hfsm_core.res.transition(&s_competition_fault);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_go_c_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e != NULL && e->id == COMPETITION_EVENT_ZONE_REACHED) {
        return hfsm_core.res.transition(&s_competition_c_scan);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_c_scan_handle(HfsmMachine* m, const HfsmEvent* e) {
    return competition_handle_scan_policy(m, e, MISSION_ZONE_C, &s_competition_c_pollen, &s_competition_go_d_handoff);
}

static HfsmResult competition_c_pollen_handle(HfsmMachine* m, const HfsmEvent* e) {
    if(e == NULL) {
        return hfsm_core.res.ignore();
    }
    if(e->id == COMPETITION_EVENT_ACTION_COMPLETE) {
        competition_send_voice_event(MISSION_VOICE_EVENT_POLLINATION_DONE, MISSION_ZONE_C, FLOWER_SEX_FEMALE);
        competition_finish_zone(m);
        return hfsm_core.res.transition(&s_competition_go_d_handoff);
    }
    if(e->id == COMPETITION_EVENT_TIMEOUT) {
        competition_prepare_fault(m, competition_event_fault_cause(e, MISSION_FAULT_ARM_ACTION_TIMEOUT));
        return hfsm_core.res.transition(&s_competition_fault);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_go_d_handoff_handle(HfsmMachine* m, const HfsmEvent* e) {
    CompetitionContext* ctx = competition_context(m);

    if(e == NULL || ctx == NULL) {
        return hfsm_core.res.ignore();
    }
    if(e->id == COMPETITION_EVENT_ACTION_COMPLETE) {
        mission.clear_uav_handoff();
        competition_send_voice_event(MISSION_VOICE_EVENT_D_HANDOFF_DONE, MISSION_ZONE_D, 0u);
        return hfsm_core.res.transition(&s_competition_go_home);
    }
    if(e->id == COMPETITION_EVENT_TIMEOUT) {
        competition_prepare_fault(m, competition_event_fault_cause(e, MISSION_FAULT_UAV_HANDOFF_TIMEOUT));
        return hfsm_core.res.transition(&s_competition_fault);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_go_home_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;

    if(e != NULL && e->id == COMPETITION_EVENT_ZONE_REACHED) {
        return hfsm_core.res.transition(&s_competition_finish);
    }

    return hfsm_core.res.ignore();
}

static HfsmResult competition_finish_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    (void)e;
    return hfsm_core.res.ignore();
}

static HfsmResult competition_stopped_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    (void)e;
    return hfsm_core.res.ignore();
}

static HfsmResult competition_fault_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    (void)e;
    return hfsm_core.res.ignore();
}

static HfsmResult competition_estop_handle(HfsmMachine* m, const HfsmEvent* e) {
    (void)m;
    (void)e;
    return hfsm_core.res.ignore();
}

static void competition_idle_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    competition_set_state(m, COMPETITION_STATE_IDLE);
    ctx->snapshot.pending_fault_cause = MISSION_FAULT_NONE;
    ctx->snapshot.duplicate_start_logged = false;
    mission.set_phase(MISSION_PHASE_IDLE);
    competition_sync_attempts(m);
    competition_emit_mission_event();
}

static void competition_start_broadcast_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    ctx->snapshot.duplicate_start_logged = false;
    competition_set_state(m, COMPETITION_STATE_START_BROADCAST);
    competition_set_phase(m, MISSION_PHASE_START);
}

static void competition_go_a_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    ctx->snapshot.attempts.scan_retry_count = 0u;
    ctx->snapshot.attempts.wrong_zone_count = 0u;
    competition_set_state(m, COMPETITION_STATE_GO_A);
    competition_set_phase(m, MISSION_PHASE_A);
}

static void competition_a_scan_entry(HfsmMachine* m) {
    competition_enter_scan(m, COMPETITION_STATE_A_SCAN, MISSION_ZONE_A);
}

static void competition_a_pollen_entry(HfsmMachine* m) {
    competition_set_state(m, COMPETITION_STATE_A_POLLEN);
    competition_set_phase(m, MISSION_PHASE_A);
    competition_send_voice_event(MISSION_VOICE_EVENT_POLLINATION_START, MISSION_ZONE_A, FLOWER_SEX_FEMALE);
}

static void competition_go_b_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    ctx->snapshot.attempts.scan_retry_count = 0u;
    ctx->snapshot.attempts.wrong_zone_count = 0u;
    competition_set_state(m, COMPETITION_STATE_GO_B);
    competition_set_phase(m, MISSION_PHASE_B);
}

static void competition_b_scan_entry(HfsmMachine* m) {
    competition_enter_scan(m, COMPETITION_STATE_B_SCAN, MISSION_ZONE_B);
}

static void competition_b_pollen_entry(HfsmMachine* m) {
    competition_set_state(m, COMPETITION_STATE_B_POLLEN);
    competition_set_phase(m, MISSION_PHASE_B);
    competition_send_voice_event(MISSION_VOICE_EVENT_POLLINATION_START, MISSION_ZONE_B, FLOWER_SEX_FEMALE);
}

static void competition_go_c_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    ctx->snapshot.attempts.scan_retry_count = 0u;
    ctx->snapshot.attempts.wrong_zone_count = 0u;
    competition_set_state(m, COMPETITION_STATE_GO_C);
    competition_set_phase(m, MISSION_PHASE_C);
}

static void competition_c_scan_entry(HfsmMachine* m) {
    competition_enter_scan(m, COMPETITION_STATE_C_SCAN, MISSION_ZONE_C);
}

static void competition_c_pollen_entry(HfsmMachine* m) {
    competition_set_state(m, COMPETITION_STATE_C_POLLEN);
    competition_set_phase(m, MISSION_PHASE_C);
    competition_send_voice_event(MISSION_VOICE_EVENT_POLLINATION_START, MISSION_ZONE_C, FLOWER_SEX_FEMALE);
}

static void competition_go_d_handoff_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);
    const ms_t now_ms = delay_now_ms();

    if(ctx == NULL) {
        return;
    }

    competition_set_state(m, COMPETITION_STATE_GO_D_HANDOFF);
    competition_set_phase(m, MISSION_PHASE_D_HANDOFF);
    ctx->uav_handoff_start_timestamp_ms = now_ms;
    mission.start_uav_handoff(
        COMPETITION_D_HANDOFF_STEP_READY,
        ctx->snapshot.attempts.uav_handoff_retry_count,
        now_ms);
    competition_send_voice_event(MISSION_VOICE_EVENT_D_HANDOFF_READY, MISSION_ZONE_D, 0u);
    (void)visual_comms.send_uav_handoff_request(COMPETITION_D_HANDOFF_STEP_READY);
}

static void competition_go_home_entry(HfsmMachine* m) {
    competition_set_state(m, COMPETITION_STATE_GO_HOME);
    competition_set_phase(m, MISSION_PHASE_HOME);
}

static void competition_finish_entry(HfsmMachine* m) {
    competition_set_state(m, COMPETITION_STATE_FINISH);
    mission.finish_success();
    competition_emit_mission_event();
    competition_send_voice_event(MISSION_VOICE_EVENT_RUN_FINISHED, MISSION_ZONE_HOME, MISSION_RUN_RESULT_SUCCESS);
    hfsm_core.clear(m);
    competition_stop_outputs(false);
}

static void competition_stopped_entry(HfsmMachine* m) {
    competition_set_state(m, COMPETITION_STATE_STOPPED);
    mission.stop();
    competition_emit_mission_event();
    competition_send_voice_event(MISSION_VOICE_EVENT_RUN_STOPPED, MISSION_ZONE_NONE, MISSION_RUN_RESULT_STOPPED);
    hfsm_core.clear(m);
    competition_stop_outputs(true);
}

static void competition_fault_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    competition_set_state(m, COMPETITION_STATE_FAULT);
    mission.record_fault(ctx->snapshot.pending_fault_cause);
    competition_emit_mission_event();
    competition_send_voice_event(MISSION_VOICE_EVENT_RUN_FAULT, MISSION_ZONE_NONE, ctx->snapshot.pending_fault_cause);
    hfsm_core.clear(m);
    competition_stop_outputs(true);
}

static void competition_estop_entry(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    competition_set_state(m, COMPETITION_STATE_ESTOP);
    ctx->snapshot.pending_fault_cause = MISSION_FAULT_ESTOP_REQUESTED;
    mission.estop();
    competition_emit_mission_event();
    competition_send_voice_event(MISSION_VOICE_EVENT_RUN_ESTOP, MISSION_ZONE_NONE, MISSION_RUN_RESULT_ESTOP);
    hfsm_core.clear(m);
    competition_stop_outputs(true);
}

static CompetitionContext* competition_context(HfsmMachine* m) {
    return (CompetitionContext*)hfsm_core.context(m);
}

static CompetitionState competition_state_from_node(const HfsmState* state) {
    if(state == &s_competition_start_broadcast) return COMPETITION_STATE_START_BROADCAST;
    if(state == &s_competition_go_a) return COMPETITION_STATE_GO_A;
    if(state == &s_competition_a_scan) return COMPETITION_STATE_A_SCAN;
    if(state == &s_competition_a_pollen) return COMPETITION_STATE_A_POLLEN;
    if(state == &s_competition_go_b) return COMPETITION_STATE_GO_B;
    if(state == &s_competition_b_scan) return COMPETITION_STATE_B_SCAN;
    if(state == &s_competition_b_pollen) return COMPETITION_STATE_B_POLLEN;
    if(state == &s_competition_go_c) return COMPETITION_STATE_GO_C;
    if(state == &s_competition_c_scan) return COMPETITION_STATE_C_SCAN;
    if(state == &s_competition_c_pollen) return COMPETITION_STATE_C_POLLEN;
    if(state == &s_competition_go_d_handoff) return COMPETITION_STATE_GO_D_HANDOFF;
    if(state == &s_competition_go_home) return COMPETITION_STATE_GO_HOME;
    if(state == &s_competition_finish) return COMPETITION_STATE_FINISH;
    if(state == &s_competition_stopped) return COMPETITION_STATE_STOPPED;
    if(state == &s_competition_fault) return COMPETITION_STATE_FAULT;
    if(state == &s_competition_estop) return COMPETITION_STATE_ESTOP;
    return COMPETITION_STATE_IDLE;
}

static bool competition_post_raw(CompetitionEvent event_id, uint32_t argument) {
    HfsmEventData data = { 0 };

    data.u32 = argument;
    log_info("competition: post %s (%lu)", competition_event_str(event_id), (unsigned long)argument);
    return hfsm_core.post(&s_competition.machine, (HfsmEventId)event_id, &data);
}

static MissionFaultCause competition_event_fault_cause(const HfsmEvent* e, MissionFaultCause fallback) {
    const MissionFaultCause cause = (MissionFaultCause)e->data.u32;

    if(cause <= MISSION_FAULT_NONE || cause > MISSION_FAULT_ESTOP_REQUESTED) {
        return fallback;
    }

    return cause;
}

static void competition_clear_runtime(CompetitionContext* ctx) {
    if(ctx == NULL) {
        return;
    }

    memset(&ctx->snapshot.attempts, 0, sizeof(ctx->snapshot.attempts));
    ctx->snapshot.pending_fault_cause = MISSION_FAULT_NONE;
    ctx->snapshot.duplicate_start_logged = false;
    ctx->state_entry_timestamp_ms = 0u;
    ctx->scan_request_timestamp_ms = 0u;
    ctx->uav_handoff_start_timestamp_ms = 0u;
    ctx->remote_link_loss_logged = false;
}

static void competition_set_state(HfsmMachine* m, CompetitionState state) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    ctx->snapshot.current_state = state;
    ctx->state_entry_timestamp_ms = delay_now_ms();
    log_info("competition: enter %s", competition_state_str(state));
}

static void competition_set_phase(HfsmMachine* m, MissionPhase phase) {
    (void)m;
    mission.set_phase(phase);
    competition_sync_attempts(m);
    competition_emit_mission_event();
}

static void competition_sync_attempts(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    mission.set_attempt_counters(&ctx->snapshot.attempts);
}

static void competition_emit_mission_event(void) {
    const MissionRuntime* runtime = mission.get_state();

    if(runtime == NULL) {
        return;
    }

    (void)visual_comms.send_mission_event(runtime->current_phase, runtime->current_zone, runtime->run_result);
}

static void competition_send_voice_event(MissionVoiceEventId event_id, MissionZoneId zone, uint8_t sex_or_result) {
    (void)visual_comms.send_voice_event(event_id, zone, sex_or_result);
}

static void competition_stop_outputs(bool brake_chassis) {
    (void)arm.stop();
    if(brake_chassis) {
        (void)chassis.brake();
    }
    else {
        (void)chassis.stop();
    }
}

static void competition_enter_scan(HfsmMachine* m, CompetitionState state, MissionZoneId zone) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    competition_set_state(m, state);
    switch(zone) {
    case MISSION_ZONE_A:
        competition_set_phase(m, MISSION_PHASE_A);
        break;
    case MISSION_ZONE_B:
        competition_set_phase(m, MISSION_PHASE_B);
        break;
    case MISSION_ZONE_C:
        competition_set_phase(m, MISSION_PHASE_C);
        break;
    default:
        break;
    }

    ctx->scan_request_timestamp_ms = delay_now_ms();
    (void)visual_comms.send_scan_request(zone, ctx->snapshot.attempts.target_index, ctx->snapshot.attempts.scan_retry_count);
}

static HfsmResult competition_handle_scan_policy(
    HfsmMachine* m,
    const HfsmEvent* e,
    MissionZoneId zone,
    const HfsmState* female_target,
    const HfsmState* advance_target) {
    CompetitionContext* ctx = competition_context(m);

    if(e == NULL || ctx == NULL) {
        return hfsm_core.res.ignore();
    }

    switch((CompetitionEvent)e->id) {
    case COMPETITION_EVENT_FEMALE_RESULT:
        competition_send_voice_event(MISSION_VOICE_EVENT_FLOWER_FEMALE, zone, FLOWER_SEX_FEMALE);
        return hfsm_core.res.transition(female_target);

    case COMPETITION_EVENT_MALE_RESULT:
        competition_send_voice_event(MISSION_VOICE_EVENT_FLOWER_MALE, zone, FLOWER_SEX_MALE);
        competition_finish_zone(m);
        return hfsm_core.res.transition(advance_target);

    case COMPETITION_EVENT_RETRY_SCAN:
        competition_handle_scan_retry_request(m, zone, (MissionRecognitionAnomaly)e->data.u32);
        return hfsm_core.res.handled();

    case COMPETITION_EVENT_SKIP_TARGET:
        competition_handle_scan_skip(m, zone, advance_target, (MissionRecognitionAnomaly)e->data.u32);
        return hfsm_core.res.transition(advance_target);

    case COMPETITION_EVENT_TIMEOUT:
        competition_prepare_fault(m, competition_event_fault_cause(e, MISSION_FAULT_VISION_REPLY_TIMEOUT));
        return hfsm_core.res.transition(&s_competition_fault);

    default:
        return hfsm_core.res.ignore();
    }
}

static void competition_finish_zone(HfsmMachine* m) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    competition_send_voice_event(MISSION_VOICE_EVENT_ZONE_COMPLETE, mission.get_state()->current_zone, 0u);
    ctx->snapshot.attempts.scan_retry_count = 0u;
    ctx->snapshot.attempts.wrong_zone_count = 0u;
    ctx->snapshot.attempts.target_index = 0u;
    competition_sync_attempts(m);
}

static void competition_prepare_fault(HfsmMachine* m, MissionFaultCause cause) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    ctx->snapshot.pending_fault_cause = cause;
}

static bool competition_reset_allowed(const HfsmState* current) {
    return current == &s_competition_idle
        || current == &s_competition_stopped
        || current == &s_competition_fault
        || current == &s_competition_estop;
}

static bool competition_is_active_state(const HfsmState* state) {
    return state != NULL && state->parent == &s_competition_active;
}

static bool competition_is_scan_state(const HfsmState* state) {
    return state == &s_competition_a_scan || state == &s_competition_b_scan || state == &s_competition_c_scan;
}

static bool competition_is_pollen_state(const HfsmState* state) {
    return state == &s_competition_a_pollen || state == &s_competition_b_pollen || state == &s_competition_c_pollen;
}

static bool competition_is_reportable_fault(MissionFaultCause cause) {
    switch(cause) {
    case MISSION_FAULT_COLLISION:
    case MISSION_FAULT_OUT_OF_BOUNDS:
    case MISSION_FAULT_LINE_SENSOR_CONTRADICTION:
    case MISSION_FAULT_VISION_REPLY_TIMEOUT:
    case MISSION_FAULT_WRONG_ZONE_REPEATED:
    case MISSION_FAULT_ARM_ACTION_TIMEOUT:
    case MISSION_FAULT_MISSION_HEARTBEAT_LOSS:
    case MISSION_FAULT_UAV_HANDOFF_TIMEOUT:
    case MISSION_FAULT_UAV_HANDOFF_FAIL_TERMINAL:
    case MISSION_FAULT_ESTOP_REQUESTED:
        return true;
    default:
        return false;
    }
}

static void competition_watch_runtime_safety(CompetitionContext* ctx, uint32_t now_ms) {
    const HfsmState* current = NULL;
    const MissionRuntime* mission_state = NULL;
    const LineSensorState* line_state = NULL;

    if(ctx == NULL || ctx->snapshot.initialized == false) {
        return;
    }

    current = hfsm_core.state(&ctx->machine);
    mission_state = mission.get_state();
    line_state = line_sensor_get_state();
    if(current == NULL || mission_state == NULL) {
        return;
    }

    if(competition_is_active_state(current) && line_state != NULL && line_state->invalid_pattern) {
        (void)competition_report_fault(MISSION_FAULT_LINE_SENSOR_CONTRADICTION);
        return;
    }

    if(competition_is_active_state(current) && mission_state->dependency_health.remote_link == MISSION_DEPENDENCY_LOST) {
        if(ctx->remote_link_loss_logged == false) {
            log_warn("competition: remote link lost during active autonomy (log-only)");
            ctx->remote_link_loss_logged = true;
        }
    }
    else {
        ctx->remote_link_loss_logged = false;
    }

    if(competition_is_scan_state(current) && ctx->scan_request_timestamp_ms != 0u
        && (now_ms - ctx->scan_request_timestamp_ms) >= VISION_REPLY_TIMEOUT_MS) {
        ctx->scan_request_timestamp_ms = 0u;
        (void)competition_post_event(COMPETITION_EVENT_TIMEOUT, MISSION_FAULT_VISION_REPLY_TIMEOUT);
        return;
    }

    if(competition_is_pollen_state(current) && ctx->state_entry_timestamp_ms != 0u
        && (now_ms - ctx->state_entry_timestamp_ms) >= ARM_ACTION_TIMEOUT_MS) {
        ctx->state_entry_timestamp_ms = 0u;
        (void)competition_post_event(COMPETITION_EVENT_TIMEOUT, MISSION_FAULT_ARM_ACTION_TIMEOUT);
        return;
    }

    if(current == &s_competition_go_d_handoff && ctx->uav_handoff_start_timestamp_ms != 0u
        && (now_ms - ctx->uav_handoff_start_timestamp_ms) >= UAV_HANDOFF_TIMEOUT_MS) {
        ctx->uav_handoff_start_timestamp_ms = 0u;
        (void)competition_post_event(COMPETITION_EVENT_TIMEOUT, MISSION_FAULT_UAV_HANDOFF_TIMEOUT);
    }
}

static void competition_handle_scan_retry_request(HfsmMachine* m, MissionZoneId zone, MissionRecognitionAnomaly anomaly) {
    CompetitionContext* ctx = competition_context(m);

    if(ctx == NULL) {
        return;
    }

    if(anomaly != MISSION_RECOGNITION_ANOMALY_WRONG_ZONE) {
        competition_send_voice_event(MISSION_VOICE_EVENT_FLOWER_UNKNOWN_OR_STALE, zone, FLOWER_SEX_UNKNOWN);
    }

    ctx->scan_request_timestamp_ms = delay_now_ms();
    (void)visual_comms.send_scan_request(zone, ctx->snapshot.attempts.target_index, ctx->snapshot.attempts.scan_retry_count);
}

static void competition_handle_scan_skip(HfsmMachine* m, MissionZoneId zone, const HfsmState* advance_target, MissionRecognitionAnomaly anomaly) {
    (void)advance_target;

    if(anomaly != MISSION_RECOGNITION_ANOMALY_WRONG_ZONE) {
        competition_send_voice_event(MISSION_VOICE_EVENT_FLOWER_UNKNOWN_OR_STALE, zone, FLOWER_SEX_UNKNOWN);
    }
    competition_finish_zone(m);
}

static const char* competition_recognition_anomaly_str(MissionRecognitionAnomaly anomaly) {
    switch(anomaly) {
    case MISSION_RECOGNITION_ANOMALY_UNKNOWN_OR_STALE:
        return "unknown-or-stale";
    case MISSION_RECOGNITION_ANOMALY_MALFORMED:
        return "malformed";
    case MISSION_RECOGNITION_ANOMALY_WRONG_ZONE:
        return "wrong-zone";
    case MISSION_RECOGNITION_ANOMALY_NONE:
    default:
        return "none";
    }
}
