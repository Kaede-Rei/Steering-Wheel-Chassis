#ifndef _APP_COMPETITION_H_
#define _APP_COMPETITION_H_

/**
 * @file competition.h
 * @brief 比赛自主任务 HFSM 应用层接口
 *
 * `app/competition.*` 是自动比赛流程在 app 层的唯一拥有者，负责把任务控制命令、
 * 识别结果分支、D 区交接边界与终止型 stop/fault/e-stop 语义组织成显式 HFSM。
 *
 * 本层不实现遥控接管策略、不实现 UAV 飞控、不实现视觉推理或语音合成，只编排状态。
 */

#include "mission.h"
#include "line_sensor.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 比赛自主应用层别名，业务代码可通过 `competition.xxx(...)` 调用
 */
#define competition competition_interface

/**
 * @brief 比赛自主应用层状态码表
 */
#define COMPETITION_STATUS_TABLE         \
    X(OK, "OK")                        \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(NOT_INITIALIZED, "Not Initialized") \
    X(NO_SPACE, "No Event Space")

/**
 * @brief 比赛自主应用层状态码
 */
#define X(name, str) COMPETITION_##name,
typedef enum {
    COMPETITION_STATUS_TABLE
} CompetitionStatus;
#undef X

/**
 * @brief 比赛 HFSM 显式状态枚举
 */
typedef enum {
    COMPETITION_STATE_IDLE = 0,
    COMPETITION_STATE_START_BROADCAST,
    COMPETITION_STATE_GO_A,
    COMPETITION_STATE_A_SCAN,
    COMPETITION_STATE_A_POLLEN,
    COMPETITION_STATE_GO_B,
    COMPETITION_STATE_B_SCAN,
    COMPETITION_STATE_B_POLLEN,
    COMPETITION_STATE_GO_C,
    COMPETITION_STATE_C_SCAN,
    COMPETITION_STATE_C_POLLEN,
    COMPETITION_STATE_GO_D_HANDOFF,
    COMPETITION_STATE_GO_HOME,
    COMPETITION_STATE_FINISH,
    COMPETITION_STATE_STOPPED,
    COMPETITION_STATE_FAULT,
    COMPETITION_STATE_ESTOP,
} CompetitionState;

/**
 * @brief 比赛 HFSM 显式业务事件枚举
 */
typedef enum {
    COMPETITION_EVENT_START = 1,
    COMPETITION_EVENT_STOP,
    COMPETITION_EVENT_ESTOP,
    COMPETITION_EVENT_RESET,
    COMPETITION_EVENT_ZONE_REACHED,
    COMPETITION_EVENT_FEMALE_RESULT,
    COMPETITION_EVENT_MALE_RESULT,
    COMPETITION_EVENT_RETRY_SCAN,
    COMPETITION_EVENT_SKIP_TARGET,
    COMPETITION_EVENT_ACTION_COMPLETE,
    COMPETITION_EVENT_TIMEOUT,
    COMPETITION_EVENT_TERMINAL_FAULT,
} CompetitionEvent;

/**
 * @brief 比赛自主应用层只读状态快照
 */
typedef struct {
    CompetitionState current_state;
    MissionPhase current_phase;
    MissionZoneId current_zone;
    MissionRecognitionResult last_vision_result;
    bool has_last_vision_result;
    bool last_line_sensor_valid;
    MissionFaultCause last_fault_cause;
    MissionRunResult final_run_result;
    MissionFaultCause pending_fault_cause;
    MissionAttemptCounters attempts;
    bool duplicate_start_logged;
    bool initialized;
} Competition;

/**
 * @brief 比赛自主应用层统一接口表
 */
extern const struct CompetitionInterface {
#define X(name, str) CompetitionStatus name;
    struct {
        COMPETITION_STATUS_TABLE
    };
#undef X

    CompetitionStatus (*init)(void);
    CompetitionStatus (*process)(void);
    CompetitionStatus (*process_all)(void);
    CompetitionStatus (*post_event)(CompetitionEvent event_id, uint32_t argument);
    CompetitionStatus (*handle_command)(MissionCommand command);
    CompetitionStatus (*handle_recognition)(const MissionRecognitionResult* result, uint32_t now_ms);
    CompetitionStatus (*handle_uav_handoff_ack)(const MissionUavHandoffAck* ack, uint32_t now_ms);
    CompetitionStatus (*report_fault)(MissionFaultCause cause);
    CompetitionState (*get_state_id)(void);
    const Competition* (*get_state)(void);
    const char* (*status_str)(CompetitionStatus status);
    const char* (*state_str)(CompetitionState state);
    const char* (*event_str)(CompetitionEvent event_id);
} competition_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CompetitionStatus competition_init(void);
CompetitionStatus competition_process(void);
CompetitionStatus competition_process_all(void);
CompetitionStatus competition_post_event(CompetitionEvent event_id, uint32_t argument);
CompetitionStatus competition_handle_command(MissionCommand command);
CompetitionStatus competition_handle_recognition(const MissionRecognitionResult* result, uint32_t now_ms);
CompetitionStatus competition_handle_uav_handoff_ack(const MissionUavHandoffAck* ack, uint32_t now_ms);
CompetitionStatus competition_report_fault(MissionFaultCause cause);
CompetitionState competition_get_state_id(void);
const Competition* competition_get_state(void);
const char* competition_status_str(CompetitionStatus status);
const char* competition_state_str(CompetitionState state);
const char* competition_event_str(CompetitionEvent event_id);

#endif
