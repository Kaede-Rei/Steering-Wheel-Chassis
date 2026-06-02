#ifndef _MISSION_H_
#define _MISSION_H_

/**
 * @file mission.h
 * @brief 比赛任务共享契约与边界声明
 *
 * `service/mission.h` 是比赛任务编排的单一公共契约源，供 `visual_comms.*`、
 * `mission.*`、`competition.*` 与后续 UAV 交接占位逻辑共享使用。
 *
 * 本仓库只负责地面机器人任务编排、阶段状态、对外接口语义与超时策略；
 * 不负责 UAV 飞行控制、不负责视觉模型推理实现、也不负责语音合成内部实现。
 * Public contract 必须保持 HAL/UART/FDCAN/platform 无关，便于后续 mock/验证复用。
 */

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 视觉识别等待超时，单位 ms
 */
#define VISION_REPLY_TIMEOUT_MS 1500U

/**
 * @brief 视觉识别结果允许的最大新鲜度，单位 ms
 */
#define VISION_STALE_MAX_MS 300U

/**
 * @brief 机械臂动作超时，单位 ms
 */
#define ARM_ACTION_TIMEOUT_MS 4000U

/**
 * @brief 任务心跳丢失判定阈值，单位 ms
 */
#define MISSION_HEARTBEAT_LOSS_MS 500U

/**
 * @brief D 区 UAV 交接等待超时，单位 ms
 */
#define UAV_HANDOFF_TIMEOUT_MS 8000U

/**
 * @brief unknown/stale 识别结果最多允许补扫次数
 */
#define UNKNOWN_OR_STALE_SCAN_MAX_RETRIES 1U

/**
 * @brief 任务公共状态码表
 */
#define MISSION_STATUS_TABLE             \
    X(OK, "OK")                        \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_STATE, "Invalid State")  \
    X(NOT_READY, "Not Ready")          \
    X(TIMEOUT, "Timeout")              \
    X(STALE_DATA, "Stale Data")

/**
 * @brief 任务公共状态码
 */
#define X(name, str) MISSION_##name,
typedef enum {
    MISSION_STATUS_TABLE
} MissionStatus;
#undef X

/**
 * @brief 比赛分区 ID
 *
 * frozen packet contract 要求 `A=1`、`B=2`、`C=3`、`D=4`、`HOME=5`
 */
typedef enum {
    MISSION_ZONE_NONE = 0,
    MISSION_ZONE_A = 1,
    MISSION_ZONE_B = 2,
    MISSION_ZONE_C = 3,
    MISSION_ZONE_D = 4,
    MISSION_ZONE_HOME = 5,
} MissionZoneId;

/**
 * @brief 花朵性别枚举
 *
 * frozen packet contract 要求 `UNKNOWN=0`、`FEMALE=1`、`MALE=2`、`HERMAPHRODITE=3`
 */
typedef enum {
    FLOWER_SEX_UNKNOWN = 0,
    FLOWER_SEX_FEMALE = 1,
    FLOWER_SEX_MALE = 2,
    FLOWER_SEX_HERMAPHRODITE = 3,
} FlowerSex;

/**
 * @brief 比赛任务阶段
 *
 * frozen 默认阶段顺序：`START -> A -> B -> C -> D_HANDOFF -> HOME -> FINISH`
 * 终止阶段为 `STOPPED`、`FAULT`、`ESTOP`
 */
typedef enum {
    MISSION_PHASE_IDLE = 0,
    MISSION_PHASE_START,
    MISSION_PHASE_A,
    MISSION_PHASE_B,
    MISSION_PHASE_C,
    MISSION_PHASE_D_HANDOFF,
    MISSION_PHASE_HOME,
    MISSION_PHASE_FINISH,
    MISSION_PHASE_STOPPED,
    MISSION_PHASE_FAULT,
    MISSION_PHASE_ESTOP,
} MissionPhase;

/**
 * @brief 任务运行结果/状态
 *
 * 用于 mission event、语音播报和最终运行归档的共享枚举。
 */
typedef enum {
    MISSION_RUN_RESULT_NONE = 0,
    MISSION_RUN_RESULT_ACTIVE,
    MISSION_RUN_RESULT_SUCCESS,
    MISSION_RUN_RESULT_STOPPED,
    MISSION_RUN_RESULT_FAULT,
    MISSION_RUN_RESULT_ESTOP,
} MissionRunResult;

/**
 * @brief 对外控制帧 type 值
 */
typedef enum {
    MISSION_FRAME_TYPE_SCAN_REQUEST = 0x01,
    MISSION_FRAME_TYPE_VOICE_EVENT = 0x02,
    MISSION_FRAME_TYPE_MISSION_EVENT = 0x03,
    MISSION_FRAME_TYPE_UAV_HANDOFF_REQUEST = 0x04,
    MISSION_FRAME_TYPE_ACK_OK = 0x05,
    MISSION_FRAME_TYPE_ACK_ERR = 0x06,
    MISSION_FRAME_TYPE_START = 0x11,
    MISSION_FRAME_TYPE_STOP = 0x12,
    MISSION_FRAME_TYPE_ESTOP = 0x13,
    MISSION_FRAME_TYPE_RESET = 0x14,
    MISSION_FRAME_TYPE_UAV_HANDOFF_ACK = 0x15,
} MissionFrameType;

/**
 * @brief 任务控制命令
 *
 * `RESET` 只允许从 `IDLE`、`STOPPED`、`FAULT`、`ESTOP` 接受。
 */
typedef enum {
    MISSION_COMMAND_NONE = 0,
    MISSION_COMMAND_START = MISSION_FRAME_TYPE_START,
    MISSION_COMMAND_STOP = MISSION_FRAME_TYPE_STOP,
    MISSION_COMMAND_ESTOP = MISSION_FRAME_TYPE_ESTOP,
    MISSION_COMMAND_RESET = MISSION_FRAME_TYPE_RESET,
} MissionCommand;

/**
 * @brief 语音播报事件 ID
 *
 * 本枚举只定义地面机器人对外请求的事件语义，不定义语音合成实现细节。
 */
typedef enum {
    MISSION_VOICE_EVENT_NONE = 0,
    MISSION_VOICE_EVENT_FLOWER_FEMALE,
    MISSION_VOICE_EVENT_FLOWER_MALE,
    MISSION_VOICE_EVENT_FLOWER_UNKNOWN_OR_STALE,
    MISSION_VOICE_EVENT_POLLINATION_START,
    MISSION_VOICE_EVENT_POLLINATION_DONE,
    MISSION_VOICE_EVENT_ZONE_COMPLETE,
    MISSION_VOICE_EVENT_D_HANDOFF_READY,
    MISSION_VOICE_EVENT_D_HANDOFF_DONE,
    MISSION_VOICE_EVENT_RUN_FINISHED,
    MISSION_VOICE_EVENT_RUN_STOPPED,
    MISSION_VOICE_EVENT_RUN_FAULT,
    MISSION_VOICE_EVENT_RUN_ESTOP,
} MissionVoiceEventId;

/**
 * @brief UAV 交接 ACK 状态
 *
 * frozen packet contract 要求：`success=0`、`busy/retryable=1`、`fail-terminal=2`
 */
typedef enum {
    MISSION_UAV_HANDOFF_SUCCESS = 0,
    MISSION_UAV_HANDOFF_BUSY_RETRYABLE = 1,
    MISSION_UAV_HANDOFF_FAIL_TERMINAL = 2,
} MissionUavHandoffStatus;

/**
 * @brief 外部依赖新鲜度状态
 */
typedef enum {
    MISSION_DEPENDENCY_UNKNOWN = 0,
    MISSION_DEPENDENCY_FRESH,
    MISSION_DEPENDENCY_STALE,
    MISSION_DEPENDENCY_LOST,
} MissionDependencyFreshness;

/**
 * @brief 识别结果 flags 位定义
 *
 * frozen packet contract 要求：
 * - bit0 = stale result
 * - bit1 = pose valid
 * - bit2 = retry suggested by external side
 * - bit3-bit7 保留且必须为 0
 */
typedef enum {
    MISSION_RECOGNITION_FLAG_STALE = 1u << 0,
    MISSION_RECOGNITION_FLAG_POSE_VALID = 1u << 1,
    MISSION_RECOGNITION_FLAG_RETRY_SUGGESTED = 1u << 2,
} MissionRecognitionFlag;

/**
 * @brief 任务故障原因
 *
 * 终止型故障遵循 frozen appendix：碰撞、出界、e-stop、依赖冲突/丢失、机械臂超时、
 * D 区交接超时或终止失败等。
 */
typedef enum {
    MISSION_FAULT_NONE = 0,
    MISSION_FAULT_COLLISION,
    MISSION_FAULT_OUT_OF_BOUNDS,
    MISSION_FAULT_LINE_SENSOR_CONTRADICTION,
    MISSION_FAULT_VISION_REPLY_TIMEOUT,
    MISSION_FAULT_WRONG_ZONE_REPEATED,
    MISSION_FAULT_ARM_ACTION_TIMEOUT,
    MISSION_FAULT_MISSION_HEARTBEAT_LOSS,
    MISSION_FAULT_UAV_HANDOFF_TIMEOUT,
    MISSION_FAULT_UAV_HANDOFF_FAIL_TERMINAL,
    MISSION_FAULT_MALFORMED_EXTERNAL_COMMAND,
    MISSION_FAULT_INVALID_RESET,
    MISSION_FAULT_ESTOP_REQUESTED,
} MissionFaultCause;

/**
 * @brief 三维识别位姿
 */
typedef struct {
    float x;
    float y;
    float z;
} MissionPose3;

/**
 * @brief 视觉识别共享结果
 */
typedef struct {
    MissionZoneId zone;
    FlowerSex sex;
    uint8_t confidence;
    uint8_t flags;
    MissionPose3 pose;
} MissionRecognitionResult;

/**
 * @brief 对外控制/事件帧参数
 *
 * fixed 8-byte frame 语义：`AA 55 type arg0 arg1 arg2 55 AA`
 */
typedef struct {
    MissionFrameType type;
    uint8_t arg0;
    uint8_t arg1;
    uint8_t arg2;
} MissionFrameArgs;

/**
 * @brief UAV 交接 ACK 共享结果
 */
typedef struct {
    MissionUavHandoffStatus status;
    uint8_t detail;
} MissionUavHandoffAck;

/**
 * @brief 外部依赖健康度快照
 */
typedef struct {
    MissionDependencyFreshness vision;
    MissionDependencyFreshness line_sensor;
    MissionDependencyFreshness remote_link;
    MissionDependencyFreshness mission_heartbeat;
    MissionDependencyFreshness uav_handoff;
} MissionDependencyHealth;

/**
 * @brief 外部依赖 ID，用于 mission runtime 的通用新鲜度更新接口
 */
typedef enum {
    MISSION_DEPENDENCY_ID_VISION = 0,
    MISSION_DEPENDENCY_ID_LINE_SENSOR,
    MISSION_DEPENDENCY_ID_REMOTE_LINK,
    MISSION_DEPENDENCY_ID_MISSION_HEARTBEAT,
    MISSION_DEPENDENCY_ID_UAV_HANDOFF,
} MissionDependencyId;

/**
 * @brief 任务运行中需要跨阶段保留的尝试计数
 */
typedef struct {
    uint8_t target_index;
    uint8_t scan_retry_count;
    uint8_t wrong_zone_count;
    uint8_t uav_handoff_retry_count;
} MissionAttemptCounters;

/**
 * @brief 新鲜度时间戳账本，统一由 mission runtime 持有
 */
typedef struct {
    uint32_t latest_recognition_timestamp_ms;
    uint32_t last_valid_recognition_timestamp_ms;
    uint32_t vision_timestamp_ms;
    uint32_t line_sensor_timestamp_ms;
    uint32_t remote_link_timestamp_ms;
    uint32_t mission_heartbeat_timestamp_ms;
    uint32_t uav_handoff_timestamp_ms;
    uint32_t last_runtime_update_ms;
} MissionFreshnessBookkeeping;

/**
 * @brief D 区 UAV 交接运行时快照
 */
typedef struct {
    bool pending;
    bool completed;
    bool has_ack;
    uint8_t handoff_step;
    uint8_t request_count;
    uint32_t request_timestamp_ms;
    uint32_t ack_timestamp_ms;
    MissionUavHandoffAck last_ack;
} MissionUavHandoffState;

/**
 * @brief 比赛任务运行时上下文快照
 *
 * 该结构由 `mission.c` 独占写入；上层通过只读 getter 观察当前上下文，
 * 避免直接依赖原始 UART 数据或灰度位掩码细节。
 */
typedef struct {
    MissionPhase current_phase;
    MissionZoneId current_zone;
    MissionAttemptCounters attempts;
    MissionRecognitionResult latest_recognition;
    MissionRecognitionResult last_valid_recognition;
    MissionDependencyHealth dependency_health;
    MissionFreshnessBookkeeping freshness;
    MissionUavHandoffState uav_handoff;
    MissionFaultCause last_fault_cause;
    MissionRunResult run_result;
    bool has_latest_recognition;
    bool latest_recognition_fresh;
    bool has_last_valid_recognition;
    bool initialized;
} MissionRuntime;

/**
 * @brief mission runtime 服务接口单例别名
 */
#define mission mission_interface

/**
 * @brief mission runtime 服务接口表
 */
extern const struct MissionInterface {
#define X(name, str) MissionStatus name;
    struct {
        MISSION_STATUS_TABLE
    };
#undef X

    void (*init)(void);
    MissionStatus (*reset)(void);
    MissionStatus (*set_phase)(MissionPhase phase);
    MissionStatus (*set_zone)(MissionZoneId zone);
    MissionStatus (*set_attempt_counters)(const MissionAttemptCounters* counters);
    MissionStatus (*note_recognition)(const MissionRecognitionResult* result, uint32_t now_ms);
    MissionStatus (*touch_dependency)(MissionDependencyId dependency, uint32_t now_ms);
    MissionStatus (*set_dependency_freshness)(MissionDependencyId dependency, MissionDependencyFreshness freshness, uint32_t now_ms);
    MissionStatus (*update_freshness)(uint32_t now_ms);
    MissionStatus (*record_fault)(MissionFaultCause cause);
    MissionStatus (*stop)(void);
    MissionStatus (*estop)(void);
    MissionStatus (*finish_success)(void);
    MissionStatus (*start_uav_handoff)(uint8_t handoff_step, uint8_t request_count, uint32_t now_ms);
    MissionStatus (*note_uav_handoff_ack)(const MissionUavHandoffAck* ack, uint32_t now_ms);
    MissionStatus (*clear_uav_handoff)(void);
    const MissionRuntime* (*get_state)(void);
} mission_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void mission_init(void);
MissionStatus mission_reset(void);
MissionStatus mission_set_phase(MissionPhase phase);
MissionStatus mission_set_zone(MissionZoneId zone);
MissionStatus mission_set_attempt_counters(const MissionAttemptCounters* counters);
MissionStatus mission_note_recognition(const MissionRecognitionResult* result, uint32_t now_ms);
MissionStatus mission_touch_dependency(MissionDependencyId dependency, uint32_t now_ms);
MissionStatus mission_set_dependency_freshness(MissionDependencyId dependency, MissionDependencyFreshness freshness, uint32_t now_ms);
MissionStatus mission_update_freshness(uint32_t now_ms);
MissionStatus mission_record_fault(MissionFaultCause cause);
MissionStatus mission_stop(void);
MissionStatus mission_estop(void);
MissionStatus mission_finish_success(void);
MissionStatus mission_start_uav_handoff(uint8_t handoff_step, uint8_t request_count, uint32_t now_ms);
MissionStatus mission_note_uav_handoff_ack(const MissionUavHandoffAck* ack, uint32_t now_ms);
MissionStatus mission_clear_uav_handoff(void);
const MissionRuntime* mission_get_state(void);

#ifdef COMPETITION_VERIFY_MODE
/**
 * @brief 为仓库级验证直接注入完整 mission runtime 快照
 * @param state 目标运行时快照
 * @return MissionStatus 注入结果
 */
MissionStatus mission_verify_inject_state(const MissionRuntime* state);

/**
 * @brief 清空 mission runtime 到默认初始态，便于确定性验证
 */
void mission_verify_reset(void);
#endif

/**
 * @brief frozen 运行语义边界说明
 *
 * - 任务激活期间重复收到 `START`：忽略，仅允许记录日志
 * - 激活期间收到 `STOP`：进入显式终止阶段 `STOPPED`，停止/刹车执行器，必须显式 `RESET`
 * - 任意阶段收到 `ESTOP`：立即进入 `ESTOP` 并停止/刹车执行器，仅在显式 `RESET` 后允许新任务
 * - 本仓库的 `mission.*` / `competition.*` 仅编排这些语义，不承担 UAV 飞控、视觉模型推理或语音合成内部实现
 */

#endif
