#include "mission.h"

#include "arm.h"
#include "chassis.h"

#include <string.h>

// ! ========================= 私 有 宏 定 义 ========================= ! //

#define MISSION_GENERIC_STALE_TIMEOUT_MS MISSION_HEARTBEAT_LOSS_MS

// ! ========================= 私 有 变 量 声 明 ========================= ! //

static MissionRuntime s_mission_runtime = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void mission_reset_runtime(MissionRuntime* runtime);
static bool mission_phase_is_valid(MissionPhase phase);
static bool mission_zone_is_valid(MissionZoneId zone);
static bool mission_dependency_id_is_valid(MissionDependencyId dependency);
static bool mission_dependency_freshness_is_valid(MissionDependencyFreshness freshness);
static bool mission_flower_sex_is_valid(FlowerSex sex);
static MissionZoneId mission_zone_from_phase(MissionPhase phase);
static MissionRunResult mission_run_result_from_phase(MissionPhase phase);
static bool mission_poll_sequence_step_is_valid(MissionPollSequenceStep step);
static bool mission_zone_supports_poll_sequence(MissionZoneId zone);
static MissionDependencyFreshness mission_compute_freshness(uint32_t now_ms, uint32_t timestamp_ms, uint32_t stale_after_ms, uint32_t lost_after_ms);
static MissionDependencyFreshness* mission_dependency_slot(MissionDependencyHealth* health, MissionDependencyId dependency);
static uint32_t* mission_dependency_timestamp_slot(MissionFreshnessBookkeeping* freshness, MissionDependencyId dependency);
static bool mission_recognition_is_valid(const MissionRecognitionResult* result);
static bool mission_recognition_is_fresh(const MissionRecognitionResult* result, uint32_t now_ms, uint32_t timestamp_ms);
static void mission_reset_poll_sequence(MissionRuntime* runtime);
static bool mission_poll_sequence_execution_active(const MissionRuntime* runtime);
static void mission_fill_poll_update(MissionPollSequenceUpdate* update, const MissionRuntime* runtime, MissionPollDecision decision, bool should_trigger_female_action, bool navigation_released);
static MissionStatus mission_request_poll_chassis_hold(MissionRuntime* runtime);
static MissionStatus mission_release_poll_navigation(MissionRuntime* runtime);

// ! ========================= 接 口 单 例 定 义 ========================= ! //

const struct MissionInterface mission_interface = {
#define X(name, str) .name = MISSION_##name,
    { MISSION_STATUS_TABLE },
#undef X
    .init = mission_init,
    .reset = mission_reset,
    .set_phase = mission_set_phase,
    .set_zone = mission_set_zone,
    .set_attempt_counters = mission_set_attempt_counters,
    .note_recognition = mission_note_recognition,
    .run_poll_sequence = mission_run_poll_sequence,
    .touch_dependency = mission_touch_dependency,
    .set_dependency_freshness = mission_set_dependency_freshness,
    .update_freshness = mission_update_freshness,
    .record_fault = mission_record_fault,
    .stop = mission_stop,
    .estop = mission_estop,
    .finish_success = mission_finish_success,
    .start_uav_handoff = mission_start_uav_handoff,
    .note_uav_handoff_ack = mission_note_uav_handoff_ack,
    .clear_uav_handoff = mission_clear_uav_handoff,
    .get_state = mission_get_state,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void mission_init(void) {
    mission_reset_runtime(&s_mission_runtime);
    s_mission_runtime.initialized = true;
}

MissionStatus mission_reset(void) {
    if(!s_mission_runtime.initialized) {
        mission_init();
        return MISSION_OK;
    }

    switch(s_mission_runtime.current_phase) {
    case MISSION_PHASE_IDLE:
    case MISSION_PHASE_STOPPED:
    case MISSION_PHASE_FAULT:
    case MISSION_PHASE_ESTOP:
        mission_reset_runtime(&s_mission_runtime);
        s_mission_runtime.initialized = true;
        return MISSION_OK;
    default:
        s_mission_runtime.last_fault_cause = MISSION_FAULT_INVALID_RESET;
        return MISSION_INVALID_STATE;
    }
}

MissionStatus mission_set_phase(MissionPhase phase) {
    const MissionZoneId mapped_zone = mission_zone_from_phase(phase);

    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(!mission_phase_is_valid(phase)) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.current_phase = phase;
    s_mission_runtime.run_result = mission_run_result_from_phase(phase);
    if(mapped_zone != MISSION_ZONE_NONE) {
        s_mission_runtime.current_zone = mapped_zone;
    }

    return MISSION_OK;
}

MissionStatus mission_set_zone(MissionZoneId zone) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(!mission_zone_is_valid(zone)) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.current_zone = zone;
    return MISSION_OK;
}

MissionStatus mission_set_attempt_counters(const MissionAttemptCounters* counters) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(counters == NULL) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.attempts = *counters;
    return MISSION_OK;
}

MissionStatus mission_note_recognition(const MissionRecognitionResult* result, uint32_t now_ms) {
    MissionDependencyFreshness vision_freshness = MISSION_DEPENDENCY_UNKNOWN;

    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(result == NULL || !mission_recognition_is_valid(result)) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.latest_recognition = *result;
    s_mission_runtime.has_latest_recognition = true;
    s_mission_runtime.freshness.latest_recognition_timestamp_ms = now_ms;
    s_mission_runtime.freshness.vision_timestamp_ms = now_ms;

    s_mission_runtime.latest_recognition_fresh = mission_recognition_is_fresh(result, now_ms, now_ms);
    if(s_mission_runtime.latest_recognition_fresh) {
        s_mission_runtime.last_valid_recognition = *result;
        s_mission_runtime.has_last_valid_recognition = true;
        s_mission_runtime.freshness.last_valid_recognition_timestamp_ms = now_ms;
    }

    vision_freshness = mission_compute_freshness(now_ms, now_ms, VISION_STALE_MAX_MS, VISION_REPLY_TIMEOUT_MS);
    s_mission_runtime.dependency_health.vision = vision_freshness;
    s_mission_runtime.freshness.last_runtime_update_ms = now_ms;
    return MISSION_OK;
}

MissionStatus mission_run_poll_sequence(
    MissionPollSequenceStep step,
    const MissionRecognitionResult* recognition,
    uint32_t now_ms,
    MissionPollSequenceUpdate* update) {
    MissionPollDecision decision = MISSION_POLL_DECISION_NONE;
    MissionStatus status = MISSION_OK;
    const bool has_retry_budget = s_mission_runtime.attempts.scan_retry_count < UNKNOWN_OR_STALE_SCAN_MAX_RETRIES;

    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(!mission_poll_sequence_step_is_valid(step)) {
        return MISSION_INVALID_PARAM;
    }

    switch(step) {
    case MISSION_POLL_SEQUENCE_STEP_PREPARE_APPROACH:
        if(mission_zone_supports_poll_sequence(s_mission_runtime.current_zone) == false) {
            return MISSION_INVALID_STATE;
        }
        if(chassis.is_ready() == false || arm.is_ready() == false) {
            return MISSION_NOT_READY;
        }

        mission_reset_poll_sequence(&s_mission_runtime);
        s_mission_runtime.poll_sequence.state = MISSION_POLL_SEQUENCE_WAITING_RECOGNITION;
        s_mission_runtime.poll_sequence.approach_ready = true;
        s_mission_runtime.poll_sequence.sequence_start_timestamp_ms = now_ms;
        s_mission_runtime.poll_sequence.last_decision = MISSION_POLL_DECISION_WAIT_RECOGNITION;
        s_mission_runtime.motion_owner = MISSION_MOTION_OWNER_NAVIGATION;
        s_mission_runtime.chassis_hold_state = MISSION_CHASSIS_HOLD_NONE;
        decision = MISSION_POLL_DECISION_WAIT_RECOGNITION;
        break;

    case MISSION_POLL_SEQUENCE_STEP_CONSUME_RECOGNITION:
        if(s_mission_runtime.poll_sequence.approach_ready == false || recognition == NULL) {
            return MISSION_INVALID_STATE;
        }
        if(!mission_recognition_is_valid(recognition)) {
            return MISSION_INVALID_PARAM;
        }
        if(recognition->zone != s_mission_runtime.current_zone) {
            return MISSION_INVALID_STATE;
        }

        status = mission_note_recognition(recognition, now_ms);
        if(status != MISSION_OK) {
            return status;
        }

        s_mission_runtime.poll_sequence.selected_recognition = *recognition;
        s_mission_runtime.poll_sequence.has_selected_recognition = true;
        s_mission_runtime.poll_sequence.decided_sex = recognition->sex;

        if(mission_recognition_is_fresh(recognition, now_ms, now_ms) == false || recognition->sex == FLOWER_SEX_UNKNOWN) {
            s_mission_runtime.motion_owner = MISSION_MOTION_OWNER_NAVIGATION;
            s_mission_runtime.chassis_hold_state = MISSION_CHASSIS_HOLD_NONE;
            s_mission_runtime.poll_sequence.chassis_hold_active = false;
            s_mission_runtime.poll_sequence.last_decision = has_retry_budget
                ? MISSION_POLL_DECISION_RETRY_SCAN
                : MISSION_POLL_DECISION_SKIP_TARGET;
            if(has_retry_budget == false) {
                s_mission_runtime.poll_sequence.state = MISSION_POLL_SEQUENCE_SKIPPED;
                s_mission_runtime.poll_sequence.navigation_release_pending = true;
            }
            decision = s_mission_runtime.poll_sequence.last_decision;
            break;
        }

        if(recognition->sex != FLOWER_SEX_FEMALE) {
            s_mission_runtime.motion_owner = MISSION_MOTION_OWNER_NAVIGATION;
            s_mission_runtime.chassis_hold_state = MISSION_CHASSIS_HOLD_NONE;
            s_mission_runtime.poll_sequence.chassis_hold_active = false;
            s_mission_runtime.poll_sequence.state = MISSION_POLL_SEQUENCE_SKIPPED;
            s_mission_runtime.poll_sequence.navigation_release_pending = true;
            s_mission_runtime.poll_sequence.last_decision = MISSION_POLL_DECISION_ADVANCE_NO_POLL;
            decision = MISSION_POLL_DECISION_ADVANCE_NO_POLL;
            break;
        }

        status = mission_request_poll_chassis_hold(&s_mission_runtime);
        if(status != MISSION_OK) {
            return status;
        }

        s_mission_runtime.poll_sequence.state = MISSION_POLL_SEQUENCE_EXECUTING_FEMALE_ACTION;
        s_mission_runtime.poll_sequence.female_action_requested = true;
        s_mission_runtime.poll_sequence.arm_action_timestamp_ms = now_ms;
        s_mission_runtime.poll_sequence.last_decision = MISSION_POLL_DECISION_TRIGGER_FEMALE_ACTION;
        decision = MISSION_POLL_DECISION_TRIGGER_FEMALE_ACTION;
        break;

    case MISSION_POLL_SEQUENCE_STEP_COMPLETE_ARM_ACTION:
        if(s_mission_runtime.poll_sequence.state != MISSION_POLL_SEQUENCE_EXECUTING_FEMALE_ACTION
            || s_mission_runtime.poll_sequence.female_action_requested == false) {
            return MISSION_INVALID_STATE;
        }

        status = mission_request_poll_chassis_hold(&s_mission_runtime);
        if(status != MISSION_OK) {
            return status;
        }

        s_mission_runtime.poll_sequence.female_action_requested = false;
        s_mission_runtime.poll_sequence.female_action_complete = true;
        s_mission_runtime.poll_sequence.retract_requested = true;
        s_mission_runtime.poll_sequence.retract_timestamp_ms = now_ms;
        s_mission_runtime.poll_sequence.state = MISSION_POLL_SEQUENCE_RETRACTING;
        s_mission_runtime.poll_sequence.last_decision = MISSION_POLL_DECISION_HOLD_FOR_RETRACT;
        decision = MISSION_POLL_DECISION_HOLD_FOR_RETRACT;
        break;

    case MISSION_POLL_SEQUENCE_STEP_COMPLETE_RETRACT:
        if(s_mission_runtime.poll_sequence.state != MISSION_POLL_SEQUENCE_RETRACTING
            || s_mission_runtime.poll_sequence.retract_requested == false) {
            return MISSION_INVALID_STATE;
        }

        status = mission_request_poll_chassis_hold(&s_mission_runtime);
        if(status != MISSION_OK) {
            return status;
        }

        s_mission_runtime.poll_sequence.retract_requested = false;
        s_mission_runtime.poll_sequence.retract_complete = true;
        s_mission_runtime.poll_sequence.navigation_release_pending = true;
        s_mission_runtime.poll_sequence.state = MISSION_POLL_SEQUENCE_COMPLETE;
        s_mission_runtime.poll_sequence.last_decision = MISSION_POLL_DECISION_RELEASE_NAVIGATION;
        decision = MISSION_POLL_DECISION_RELEASE_NAVIGATION;
        break;

    case MISSION_POLL_SEQUENCE_STEP_RELEASE_NAVIGATION:
        if(s_mission_runtime.poll_sequence.navigation_release_pending == false) {
            return MISSION_INVALID_STATE;
        }

        status = mission_release_poll_navigation(&s_mission_runtime);
        if(status != MISSION_OK) {
            return status;
        }

        s_mission_runtime.poll_sequence.navigation_release_pending = false;
        s_mission_runtime.poll_sequence.last_decision = MISSION_POLL_DECISION_RELEASE_NAVIGATION;
        decision = MISSION_POLL_DECISION_RELEASE_NAVIGATION;
        break;

    default:
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.freshness.last_runtime_update_ms = now_ms;
    mission_fill_poll_update(
        update,
        &s_mission_runtime,
        decision,
        decision == MISSION_POLL_DECISION_TRIGGER_FEMALE_ACTION,
        step == MISSION_POLL_SEQUENCE_STEP_RELEASE_NAVIGATION);
    return MISSION_OK;
}

MissionStatus mission_touch_dependency(MissionDependencyId dependency, uint32_t now_ms) {
    return mission_set_dependency_freshness(dependency, MISSION_DEPENDENCY_FRESH, now_ms);
}

MissionStatus mission_set_dependency_freshness(MissionDependencyId dependency, MissionDependencyFreshness freshness, uint32_t now_ms) {
    MissionDependencyFreshness* slot = NULL;
    uint32_t* timestamp_slot = NULL;

    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(!mission_dependency_id_is_valid(dependency) || !mission_dependency_freshness_is_valid(freshness)) {
        return MISSION_INVALID_PARAM;
    }

    slot = mission_dependency_slot(&s_mission_runtime.dependency_health, dependency);
    timestamp_slot = mission_dependency_timestamp_slot(&s_mission_runtime.freshness, dependency);
    if(slot == NULL || timestamp_slot == NULL) {
        return MISSION_INVALID_PARAM;
    }

    *slot = freshness;
    *timestamp_slot = now_ms;
    s_mission_runtime.freshness.last_runtime_update_ms = now_ms;
    return MISSION_OK;
}

MissionStatus mission_update_freshness(uint32_t now_ms) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    if(s_mission_runtime.has_latest_recognition) {
        s_mission_runtime.latest_recognition_fresh = mission_recognition_is_fresh(
            &s_mission_runtime.latest_recognition,
            now_ms,
            s_mission_runtime.freshness.latest_recognition_timestamp_ms);
        s_mission_runtime.dependency_health.vision = mission_compute_freshness(
            now_ms,
            s_mission_runtime.freshness.vision_timestamp_ms,
            VISION_STALE_MAX_MS,
            VISION_REPLY_TIMEOUT_MS);
    }

    s_mission_runtime.dependency_health.line_sensor = mission_compute_freshness(
        now_ms,
        s_mission_runtime.freshness.line_sensor_timestamp_ms,
        MISSION_GENERIC_STALE_TIMEOUT_MS,
        MISSION_HEARTBEAT_LOSS_MS);
    s_mission_runtime.dependency_health.remote_link = mission_compute_freshness(
        now_ms,
        s_mission_runtime.freshness.remote_link_timestamp_ms,
        MISSION_GENERIC_STALE_TIMEOUT_MS,
        MISSION_HEARTBEAT_LOSS_MS);
    s_mission_runtime.dependency_health.mission_heartbeat = mission_compute_freshness(
        now_ms,
        s_mission_runtime.freshness.mission_heartbeat_timestamp_ms,
        MISSION_GENERIC_STALE_TIMEOUT_MS,
        MISSION_HEARTBEAT_LOSS_MS);

    if(s_mission_runtime.uav_handoff.pending) {
        s_mission_runtime.dependency_health.uav_handoff = mission_compute_freshness(
            now_ms,
            s_mission_runtime.uav_handoff.request_timestamp_ms,
            UAV_HANDOFF_TIMEOUT_MS,
            UAV_HANDOFF_TIMEOUT_MS);
    } else {
        s_mission_runtime.dependency_health.uav_handoff = mission_compute_freshness(
            now_ms,
            s_mission_runtime.freshness.uav_handoff_timestamp_ms,
            MISSION_GENERIC_STALE_TIMEOUT_MS,
            UAV_HANDOFF_TIMEOUT_MS);
    }

    s_mission_runtime.freshness.last_runtime_update_ms = now_ms;
    return MISSION_OK;
}

MissionStatus mission_record_fault(MissionFaultCause cause) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(cause == MISSION_FAULT_NONE) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.last_fault_cause = cause;
    if(cause == MISSION_FAULT_ESTOP_REQUESTED) {
        s_mission_runtime.current_phase = MISSION_PHASE_ESTOP;
        s_mission_runtime.run_result = MISSION_RUN_RESULT_ESTOP;
    } else {
        s_mission_runtime.current_phase = MISSION_PHASE_FAULT;
        s_mission_runtime.run_result = MISSION_RUN_RESULT_FAULT;
    }

    return MISSION_OK;
}

MissionStatus mission_stop(void) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    s_mission_runtime.current_phase = MISSION_PHASE_STOPPED;
    s_mission_runtime.run_result = MISSION_RUN_RESULT_STOPPED;
    return MISSION_OK;
}

MissionStatus mission_estop(void) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    s_mission_runtime.last_fault_cause = MISSION_FAULT_ESTOP_REQUESTED;
    s_mission_runtime.current_phase = MISSION_PHASE_ESTOP;
    s_mission_runtime.run_result = MISSION_RUN_RESULT_ESTOP;
    return MISSION_OK;
}

MissionStatus mission_finish_success(void) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    s_mission_runtime.current_phase = MISSION_PHASE_FINISH;
    s_mission_runtime.run_result = MISSION_RUN_RESULT_SUCCESS;
    return MISSION_OK;
}

MissionStatus mission_start_uav_handoff(uint8_t handoff_step, uint8_t request_count, uint32_t now_ms) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    s_mission_runtime.current_zone = MISSION_ZONE_D;
    s_mission_runtime.uav_handoff.pending = true;
    s_mission_runtime.uav_handoff.completed = false;
    s_mission_runtime.uav_handoff.has_ack = false;
    s_mission_runtime.uav_handoff.handoff_step = handoff_step;
    s_mission_runtime.uav_handoff.request_count = request_count;
    s_mission_runtime.uav_handoff.request_timestamp_ms = now_ms;
    s_mission_runtime.uav_handoff.ack_timestamp_ms = 0u;
    memset(&s_mission_runtime.uav_handoff.last_ack, 0, sizeof(s_mission_runtime.uav_handoff.last_ack));

    s_mission_runtime.freshness.uav_handoff_timestamp_ms = now_ms;
    s_mission_runtime.dependency_health.uav_handoff = MISSION_DEPENDENCY_FRESH;
    s_mission_runtime.freshness.last_runtime_update_ms = now_ms;
    return MISSION_OK;
}

MissionStatus mission_note_uav_handoff_ack(const MissionUavHandoffAck* ack, uint32_t now_ms) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }
    if(ack == NULL || ack->status > MISSION_UAV_HANDOFF_FAIL_TERMINAL) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime.uav_handoff.has_ack = true;
    s_mission_runtime.uav_handoff.last_ack = *ack;
    s_mission_runtime.uav_handoff.ack_timestamp_ms = now_ms;
    s_mission_runtime.uav_handoff.pending = false;
    s_mission_runtime.uav_handoff.completed = (ack->status == MISSION_UAV_HANDOFF_SUCCESS);
    s_mission_runtime.freshness.uav_handoff_timestamp_ms = now_ms;
    s_mission_runtime.dependency_health.uav_handoff = (ack->status == MISSION_UAV_HANDOFF_FAIL_TERMINAL)
        ? MISSION_DEPENDENCY_LOST
        : MISSION_DEPENDENCY_FRESH;
    s_mission_runtime.freshness.last_runtime_update_ms = now_ms;

    return MISSION_OK;
}

MissionStatus mission_clear_uav_handoff(void) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    memset(&s_mission_runtime.uav_handoff, 0, sizeof(s_mission_runtime.uav_handoff));
    s_mission_runtime.dependency_health.uav_handoff = MISSION_DEPENDENCY_UNKNOWN;
    s_mission_runtime.freshness.uav_handoff_timestamp_ms = 0u;
    return MISSION_OK;
}

const MissionRuntime* mission_get_state(void) {
    if(!s_mission_runtime.initialized) {
        mission_init();
    }

    return &s_mission_runtime;
}

#ifdef COMPETITION_VERIFY_MODE
MissionStatus mission_verify_inject_state(const MissionRuntime* state) {
    if(state == NULL) {
        return MISSION_INVALID_PARAM;
    }

    s_mission_runtime = *state;
    s_mission_runtime.initialized = true;
    return MISSION_OK;
}

void mission_verify_reset(void) {
    mission_init();
}
#endif

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void mission_reset_runtime(MissionRuntime* runtime) {
    memset(runtime, 0, sizeof(*runtime));
    runtime->current_phase = MISSION_PHASE_IDLE;
    runtime->current_zone = MISSION_ZONE_NONE;
    runtime->last_fault_cause = MISSION_FAULT_NONE;
    runtime->run_result = MISSION_RUN_RESULT_NONE;
    runtime->motion_owner = MISSION_MOTION_OWNER_NONE;
    runtime->chassis_hold_state = MISSION_CHASSIS_HOLD_NONE;
    runtime->dependency_health.vision = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.line_sensor = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.remote_link = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.mission_heartbeat = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.uav_handoff = MISSION_DEPENDENCY_UNKNOWN;
    mission_reset_poll_sequence(runtime);
}

static bool mission_phase_is_valid(MissionPhase phase) {
    return phase >= MISSION_PHASE_IDLE && phase <= MISSION_PHASE_ESTOP;
}

static bool mission_zone_is_valid(MissionZoneId zone) {
    return zone >= MISSION_ZONE_NONE && zone <= MISSION_ZONE_HOME;
}

static bool mission_dependency_id_is_valid(MissionDependencyId dependency) {
    return dependency >= MISSION_DEPENDENCY_ID_VISION && dependency <= MISSION_DEPENDENCY_ID_UAV_HANDOFF;
}

static bool mission_dependency_freshness_is_valid(MissionDependencyFreshness freshness) {
    return freshness >= MISSION_DEPENDENCY_UNKNOWN && freshness <= MISSION_DEPENDENCY_LOST;
}

static bool mission_flower_sex_is_valid(FlowerSex sex) {
    return sex >= FLOWER_SEX_UNKNOWN && sex <= FLOWER_SEX_HERMAPHRODITE;
}

static MissionZoneId mission_zone_from_phase(MissionPhase phase) {
    switch(phase) {
    case MISSION_PHASE_A:
        return MISSION_ZONE_A;
    case MISSION_PHASE_B:
        return MISSION_ZONE_B;
    case MISSION_PHASE_C:
        return MISSION_ZONE_C;
    case MISSION_PHASE_D_HANDOFF:
        return MISSION_ZONE_D;
    case MISSION_PHASE_HOME:
    case MISSION_PHASE_FINISH:
        return MISSION_ZONE_HOME;
    default:
        return MISSION_ZONE_NONE;
    }
}

static MissionRunResult mission_run_result_from_phase(MissionPhase phase) {
    switch(phase) {
    case MISSION_PHASE_START:
    case MISSION_PHASE_A:
    case MISSION_PHASE_B:
    case MISSION_PHASE_C:
    case MISSION_PHASE_D_HANDOFF:
    case MISSION_PHASE_HOME:
        return MISSION_RUN_RESULT_ACTIVE;
    case MISSION_PHASE_FINISH:
        return MISSION_RUN_RESULT_SUCCESS;
    case MISSION_PHASE_STOPPED:
        return MISSION_RUN_RESULT_STOPPED;
    case MISSION_PHASE_FAULT:
        return MISSION_RUN_RESULT_FAULT;
    case MISSION_PHASE_ESTOP:
        return MISSION_RUN_RESULT_ESTOP;
    case MISSION_PHASE_IDLE:
    default:
        return MISSION_RUN_RESULT_NONE;
    }
}

static bool mission_poll_sequence_step_is_valid(MissionPollSequenceStep step) {
    return step >= MISSION_POLL_SEQUENCE_STEP_PREPARE_APPROACH
        && step <= MISSION_POLL_SEQUENCE_STEP_RELEASE_NAVIGATION;
}

static bool mission_zone_supports_poll_sequence(MissionZoneId zone) {
    return zone == MISSION_ZONE_A || zone == MISSION_ZONE_B || zone == MISSION_ZONE_C;
}

static MissionDependencyFreshness mission_compute_freshness(uint32_t now_ms, uint32_t timestamp_ms, uint32_t stale_after_ms, uint32_t lost_after_ms) {
    const uint32_t age_ms = now_ms - timestamp_ms;

    if(timestamp_ms == 0u) {
        return MISSION_DEPENDENCY_UNKNOWN;
    }
    if(age_ms > lost_after_ms) {
        return MISSION_DEPENDENCY_LOST;
    }
    if(age_ms > stale_after_ms) {
        return MISSION_DEPENDENCY_STALE;
    }

    return MISSION_DEPENDENCY_FRESH;
}

static MissionDependencyFreshness* mission_dependency_slot(MissionDependencyHealth* health, MissionDependencyId dependency) {
    switch(dependency) {
    case MISSION_DEPENDENCY_ID_VISION:
        return &health->vision;
    case MISSION_DEPENDENCY_ID_LINE_SENSOR:
        return &health->line_sensor;
    case MISSION_DEPENDENCY_ID_REMOTE_LINK:
        return &health->remote_link;
    case MISSION_DEPENDENCY_ID_MISSION_HEARTBEAT:
        return &health->mission_heartbeat;
    case MISSION_DEPENDENCY_ID_UAV_HANDOFF:
        return &health->uav_handoff;
    default:
        return NULL;
    }
}

static uint32_t* mission_dependency_timestamp_slot(MissionFreshnessBookkeeping* freshness, MissionDependencyId dependency) {
    switch(dependency) {
    case MISSION_DEPENDENCY_ID_VISION:
        return &freshness->vision_timestamp_ms;
    case MISSION_DEPENDENCY_ID_LINE_SENSOR:
        return &freshness->line_sensor_timestamp_ms;
    case MISSION_DEPENDENCY_ID_REMOTE_LINK:
        return &freshness->remote_link_timestamp_ms;
    case MISSION_DEPENDENCY_ID_MISSION_HEARTBEAT:
        return &freshness->mission_heartbeat_timestamp_ms;
    case MISSION_DEPENDENCY_ID_UAV_HANDOFF:
        return &freshness->uav_handoff_timestamp_ms;
    default:
        return NULL;
    }
}

static bool mission_recognition_is_valid(const MissionRecognitionResult* result) {
    if(result == NULL) {
        return false;
    }

    return mission_zone_is_valid(result->zone)
        && result->zone != MISSION_ZONE_NONE
        && mission_flower_sex_is_valid(result->sex)
        && (result->flags & 0xF8u) == 0u;
}

static bool mission_recognition_is_fresh(const MissionRecognitionResult* result, uint32_t now_ms, uint32_t timestamp_ms) {
    if(result == NULL) {
        return false;
    }
    if((result->flags & MISSION_RECOGNITION_FLAG_STALE) != 0u) {
        return false;
    }

    return mission_compute_freshness(now_ms, timestamp_ms, VISION_STALE_MAX_MS, VISION_REPLY_TIMEOUT_MS) == MISSION_DEPENDENCY_FRESH;
}

static void mission_reset_poll_sequence(MissionRuntime* runtime) {
    if(runtime == NULL) {
        return;
    }

    memset(&runtime->poll_sequence, 0, sizeof(runtime->poll_sequence));
    runtime->poll_sequence.state = MISSION_POLL_SEQUENCE_IDLE;
    runtime->poll_sequence.decided_sex = FLOWER_SEX_UNKNOWN;
}

static bool mission_poll_sequence_execution_active(const MissionRuntime* runtime) {
    if(runtime == NULL) {
        return false;
    }

    return runtime->motion_owner == MISSION_MOTION_OWNER_POLL_SEQUENCE
        && (runtime->poll_sequence.state == MISSION_POLL_SEQUENCE_EXECUTING_FEMALE_ACTION
            || runtime->poll_sequence.state == MISSION_POLL_SEQUENCE_RETRACTING
            || runtime->poll_sequence.navigation_release_pending);
}

static void mission_fill_poll_update(
    MissionPollSequenceUpdate* update,
    const MissionRuntime* runtime,
    MissionPollDecision decision,
    bool should_trigger_female_action,
    bool navigation_released) {
    if(update == NULL || runtime == NULL) {
        return;
    }

    update->decision = decision;
    update->state = runtime->poll_sequence.state;
    update->motion_owner = runtime->motion_owner;
    update->chassis_hold_state = runtime->chassis_hold_state;
    update->poll_execution_active = mission_poll_sequence_execution_active(runtime);
    update->should_trigger_female_action = should_trigger_female_action;
    update->navigation_released = navigation_released;
}

static MissionStatus mission_request_poll_chassis_hold(MissionRuntime* runtime) {
    ChassisErrorCode hold_status = CHASSIS_INVALID_PARAM;

    if(runtime == NULL) {
        return MISSION_INVALID_PARAM;
    }

    hold_status = chassis.brake();

    runtime->motion_owner = MISSION_MOTION_OWNER_POLL_SEQUENCE;
    runtime->poll_sequence.chassis_hold_active = true;
    runtime->chassis_hold_state = (hold_status == CHASSIS_OK)
        ? MISSION_CHASSIS_HOLD_HELD
        : MISSION_CHASSIS_HOLD_BRAKE_REQUESTED;

    return (hold_status == CHASSIS_OK) ? MISSION_OK : MISSION_NOT_READY;
}

static MissionStatus mission_release_poll_navigation(MissionRuntime* runtime) {
    ChassisErrorCode stop_status = CHASSIS_INVALID_PARAM;

    if(runtime == NULL) {
        return MISSION_INVALID_PARAM;
    }

    stop_status = chassis.stop();
    if(stop_status != CHASSIS_OK) {
        return MISSION_NOT_READY;
    }

    runtime->motion_owner = MISSION_MOTION_OWNER_NAVIGATION;
    runtime->chassis_hold_state = MISSION_CHASSIS_HOLD_NONE;
    runtime->poll_sequence.chassis_hold_active = false;
    return MISSION_OK;
}
