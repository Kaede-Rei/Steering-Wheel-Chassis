#ifndef _visual_comms_h_
#define _visual_comms_h_

#include <stdbool.h>

#include "main.h" // IWYU pragma: keep
#include "mission.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 视觉通信服务接口单例别名，业务代码可通过 `visual_comms.xxx(...)` 调用
 */
#define visual_comms visual_comms_interface

/**
 * @brief 视觉通信服务状态码表
 */
#define VISUAL_COMMS_STATUS_TABLE             \
    X(OK, "OK")                               \
    X(INVALID_PARAM, "Invalid Parameter")     \
    X(NOT_INITIALIZED, "Not Initialized")     \
    X(RING_BUFFER_ERROR, "Ring Buffer Error") \
    X(UART_ERROR, "UART Error")               \
    X(FRAME_ERROR, "Frame Error")

/**
 * @brief 视觉通信服务状态码
 */
#define X(name, str) VISUAL_COMMS_##name,
typedef enum {
    VISUAL_COMMS_STATUS_TABLE
} VisualCommsStatus;
#undef X

/**
 * @brief 视觉通信服务只读状态快照
 * @param pending_command 尚未被上层取走的任务控制命令
 * @param current_command 最近一次被上层确认消费的任务控制命令
 * @param latest_recognition 最近一次成功解析的识别结果
 * @param latest_uav_handoff_ack 最近一次成功解析的 UAV 交接 ACK
 * @param has_new_recognition 是否存在尚未被上层取走的新识别结果
 * @param has_new_uav_handoff_ack 是否存在尚未被上层取走的新 UAV 交接 ACK
 * @param malformed_frame_count 被拒绝的畸形帧累计次数
 * @param initialized 服务是否已经初始化完成
 */
typedef struct {
    MissionCommand pending_command;
    MissionCommand current_command;
    MissionRecognitionResult latest_recognition;
    MissionUavHandoffAck latest_uav_handoff_ack;
    bool has_new_recognition;
    bool has_new_uav_handoff_ack;
    uint32_t malformed_frame_count;
    bool initialized;
} VisualComms;

/**
 * @brief 视觉通信服务统一接口表
 */
extern const struct VisualCommsInterface {
#define X(name, str) VisualCommsStatus name;
    struct {
        VISUAL_COMMS_STATUS_TABLE
    };
#undef X

    VisualCommsStatus (*init)(UART_HandleTypeDef* huart);
    VisualCommsStatus (*process)(void);
    bool (*consume_command)(MissionCommand* command);
    bool (*consume_recognition)(MissionRecognitionResult* result);
    bool (*consume_uav_handoff_ack)(MissionUavHandoffAck* ack);
    VisualCommsStatus (*send_scan_request)(MissionZoneId zone, uint8_t target_index, uint8_t retry_count);
    VisualCommsStatus (*send_voice_event)(MissionVoiceEventId event_id, MissionZoneId zone, uint8_t sex_or_result);
    VisualCommsStatus (*send_mission_event)(MissionPhase phase_id, MissionZoneId zone, MissionRunResult run_status);
    VisualCommsStatus (*send_uav_handoff_request)(uint8_t handoff_step);
    VisualCommsStatus (*send_ack_ok)(void);
    VisualCommsStatus (*send_ack_err)(void);
    bool (*recognition_is_stale)(const MissionRecognitionResult* result);
    bool (*recognition_has_pose)(const MissionRecognitionResult* result);
    bool (*recognition_retry_suggested)(const MissionRecognitionResult* result);
    bool (*is_ready)(void);
    const VisualComms* (*get_state)(void);
    const char* (*status_str)(VisualCommsStatus status);
} visual_comms_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

VisualCommsStatus visual_comms_init(UART_HandleTypeDef* huart);
VisualCommsStatus visual_comms_process(void);
bool visual_comms_consume_command(MissionCommand* command);
bool visual_comms_consume_recognition(MissionRecognitionResult* result);
bool visual_comms_consume_uav_handoff_ack(MissionUavHandoffAck* ack);
VisualCommsStatus visual_comms_send_scan_request(MissionZoneId zone, uint8_t target_index, uint8_t retry_count);
VisualCommsStatus visual_comms_send_voice_event(MissionVoiceEventId event_id, MissionZoneId zone, uint8_t sex_or_result);
VisualCommsStatus visual_comms_send_mission_event(MissionPhase phase_id, MissionZoneId zone, MissionRunResult run_status);
VisualCommsStatus visual_comms_send_uav_handoff_request(uint8_t handoff_step);
VisualCommsStatus visual_comms_send_ack_ok(void);
VisualCommsStatus visual_comms_send_ack_err(void);
bool visual_comms_recognition_is_stale(const MissionRecognitionResult* result);
bool visual_comms_recognition_has_pose(const MissionRecognitionResult* result);
bool visual_comms_recognition_retry_suggested(const MissionRecognitionResult* result);
bool visual_comms_is_ready(void);
const VisualComms* visual_comms_get_state(void);
const char* visual_comms_status_str(VisualCommsStatus status);

#endif
