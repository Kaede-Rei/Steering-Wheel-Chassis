#include "mission.h"

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
static MissionDependencyFreshness mission_compute_freshness(uint32_t now_ms, uint32_t timestamp_ms, uint32_t stale_after_ms, uint32_t lost_after_ms);
static MissionDependencyFreshness* mission_dependency_slot(MissionDependencyHealth* health, MissionDependencyId dependency);
static uint32_t* mission_dependency_timestamp_slot(MissionFreshnessBookkeeping* freshness, MissionDependencyId dependency);
static bool mission_recognition_is_valid(const MissionRecognitionResult* result);
static bool mission_recognition_is_fresh(const MissionRecognitionResult* result, uint32_t now_ms, uint32_t timestamp_ms);

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
    runtime->dependency_health.vision = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.line_sensor = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.remote_link = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.mission_heartbeat = MISSION_DEPENDENCY_UNKNOWN;
    runtime->dependency_health.uav_handoff = MISSION_DEPENDENCY_UNKNOWN;
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
