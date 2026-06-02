#include "visual_comms.h"

#include <math.h>
#include <string.h>

#include "log.h"
#include "protocol_parser.h"
#include "stm32_hal_uart.h"

// ! ========================= 常 量 / 宏 声 明 ========================= ! //

#define VISUAL_SHORT_FRAME_LENGTH 8u
#define VISUAL_RECOGNITION_FRAME_LENGTH 20u
#define VISUAL_RX_RING_BUFFER_SIZE 64u
#define VISUAL_MAX_FRAME_LENGTH VISUAL_RECOGNITION_FRAME_LENGTH
#define VISUAL_RECOGNITION_RESERVED_FLAG_MASK 0xF8u

// ! ========================= 私 有 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    VISUAL_FRAME_NONE = 0,
    VISUAL_FRAME_SHORT,
    VISUAL_FRAME_RECOGNITION,
} VisualFrameType;

static const uint8_t s_short_header[2] = { 0xAAu, 0x55u };
static const uint8_t s_short_tail[2] = { 0x55u, 0xAAu };
static const uint8_t s_recognition_header[2] = { 0xBBu, 0x66u };
static const uint8_t s_recognition_tail[2] = { 0x66u, 0xBBu };

static VisualComms s_visual_comms = { 0 };
static VisualComms s_visual_comms_view = { 0 };
static UART_HandleTypeDef* s_visual_uart = NULL;
static RingBuf s_visual_rx_ring = { 0 };
static uint8_t s_visual_rx_ring_storage[VISUAL_RX_RING_BUFFER_SIZE] = { 0 };
static uint8_t s_visual_rx_byte = 0u;
static uint8_t s_frame_buffer[VISUAL_MAX_FRAME_LENGTH] = { 0 };
static uint8_t s_frame_length = 0u;
static uint8_t s_expected_length = 0u;
static VisualFrameType s_frame_type = VISUAL_FRAME_NONE;
static uint32_t s_protocol_parser_primask = 0u;

const struct VisualCommsInterface visual_comms_interface = {
#define X(name, str) .name = VISUAL_COMMS_##name,
    { VISUAL_COMMS_STATUS_TABLE },
#undef X
    .init = visual_comms_init,
    .process = visual_comms_process,
    .consume_command = visual_comms_consume_command,
    .consume_recognition = visual_comms_consume_recognition,
    .consume_uav_handoff_ack = visual_comms_consume_uav_handoff_ack,
    .send_scan_request = visual_comms_send_scan_request,
    .send_voice_event = visual_comms_send_voice_event,
    .send_mission_event = visual_comms_send_mission_event,
    .send_uav_handoff_request = visual_comms_send_uav_handoff_request,
    .send_ack_ok = visual_comms_send_ack_ok,
    .send_ack_err = visual_comms_send_ack_err,
    .recognition_is_stale = visual_comms_recognition_is_stale,
    .recognition_has_pose = visual_comms_recognition_has_pose,
    .recognition_retry_suggested = visual_comms_recognition_retry_suggested,
    .is_ready = visual_comms_is_ready,
    .get_state = visual_comms_get_state,
    .status_str = visual_comms_status_str,
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void s_uart_rx_complete(void);
static void s_uart_error(void);
static bool s_restart_receive(void);
static VisualCommsStatus s_write_byte_to_ring(uint8_t byte);
static VisualCommsStatus s_process_byte(uint8_t byte);
static bool s_try_start_frame(uint8_t byte);
static void s_reset_frame(void);
static bool s_tail_matches(const uint8_t* frame, const uint8_t* tail, uint8_t frame_length);
static VisualCommsStatus s_handle_complete_frame(void);
static VisualCommsStatus s_handle_short_frame(void);
static VisualCommsStatus s_handle_recognition_frame(void);
static bool s_is_valid_zone(uint8_t zone);
static bool s_is_valid_flower_sex(uint8_t sex);
static bool s_is_valid_uav_handoff_status(uint8_t status);
static float s_decode_float_le(const uint8_t* data);
static VisualCommsStatus s_send_args_frame(MissionFrameType type, uint8_t arg0, uint8_t arg1, uint8_t arg2);
static VisualCommsStatus s_send_frame(const uint8_t* data, uint16_t len);
static void s_mark_malformed_frame(const char* reason, uint8_t type);
static void s_sync_view(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void protocol_parser_enter_critical(void) {
    s_protocol_parser_primask = __get_PRIMASK();
    __disable_irq();
}

void protocol_parser_exit_critical(void) {
    if(s_protocol_parser_primask == 0u) {
        __enable_irq();
    }
}

VisualCommsStatus visual_comms_init(UART_HandleTypeDef* huart) {
    if(huart == NULL) {
        return VISUAL_COMMS_INVALID_PARAM;
    }

    memset(&s_visual_comms, 0, sizeof(s_visual_comms));
    memset(&s_visual_comms_view, 0, sizeof(s_visual_comms_view));
    s_visual_uart = huart;
    s_reset_frame();

    if(ring_buf.create(&s_visual_rx_ring, s_visual_rx_ring_storage, VISUAL_RX_RING_BUFFER_SIZE, false) != RING_BUF_SUCCESS) {
        return VISUAL_COMMS_RING_BUFFER_ERROR;
    }

    uart_register_rx_complete_callback(s_visual_uart, s_uart_rx_complete);
    uart_register_error_callback(s_visual_uart, s_uart_error);

    if(s_restart_receive() == false) {
        s_visual_uart = NULL;
        return VISUAL_COMMS_UART_ERROR;
    }

    s_visual_comms.initialized = true;
    s_sync_view();
    return VISUAL_COMMS_OK;
}

VisualCommsStatus visual_comms_process(void) {
    uint8_t byte = 0u;
    VisualCommsStatus status = VISUAL_COMMS_OK;

    if(s_visual_comms.initialized == false) {
        return VISUAL_COMMS_NOT_INITIALIZED;
    }

    while(ring_buf.read(&s_visual_rx_ring, &byte) == RING_BUF_SUCCESS) {
        status = s_process_byte(byte);
        if(status != VISUAL_COMMS_OK) {
            return status;
        }
    }

    return VISUAL_COMMS_OK;
}

bool visual_comms_consume_command(MissionCommand* command) {
    if(s_visual_comms.initialized == false || command == NULL) {
        return false;
    }
    if(s_visual_comms.pending_command == MISSION_COMMAND_NONE) {
        return false;
    }

    *command = s_visual_comms.pending_command;
    s_visual_comms.current_command = s_visual_comms.pending_command;
    s_visual_comms.pending_command = MISSION_COMMAND_NONE;
    s_sync_view();
    return true;
}

bool visual_comms_consume_recognition(MissionRecognitionResult* result) {
    if(s_visual_comms.initialized == false || result == NULL) {
        return false;
    }
    if(s_visual_comms.has_new_recognition == false) {
        return false;
    }

    *result = s_visual_comms.latest_recognition;
    s_visual_comms.has_new_recognition = false;
    s_sync_view();
    return true;
}

bool visual_comms_consume_uav_handoff_ack(MissionUavHandoffAck* ack) {
    if(s_visual_comms.initialized == false || ack == NULL) {
        return false;
    }
    if(s_visual_comms.has_new_uav_handoff_ack == false) {
        return false;
    }

    *ack = s_visual_comms.latest_uav_handoff_ack;
    s_visual_comms.has_new_uav_handoff_ack = false;
    s_sync_view();
    return true;
}

VisualCommsStatus visual_comms_send_scan_request(MissionZoneId zone, uint8_t target_index, uint8_t retry_count) {
    if(s_is_valid_zone((uint8_t)zone) == false || zone == MISSION_ZONE_NONE) {
        return VISUAL_COMMS_INVALID_PARAM;
    }

    return s_send_args_frame(MISSION_FRAME_TYPE_SCAN_REQUEST, (uint8_t)zone, target_index, retry_count);
}

VisualCommsStatus visual_comms_send_voice_event(MissionVoiceEventId event_id, MissionZoneId zone, uint8_t sex_or_result) {
    if(zone != MISSION_ZONE_NONE && s_is_valid_zone((uint8_t)zone) == false) {
        return VISUAL_COMMS_INVALID_PARAM;
    }

    return s_send_args_frame(MISSION_FRAME_TYPE_VOICE_EVENT, (uint8_t)event_id, (uint8_t)zone, sex_or_result);
}

VisualCommsStatus visual_comms_send_mission_event(MissionPhase phase_id, MissionZoneId zone, MissionRunResult run_status) {
    if(zone != MISSION_ZONE_NONE && s_is_valid_zone((uint8_t)zone) == false) {
        return VISUAL_COMMS_INVALID_PARAM;
    }

    return s_send_args_frame(MISSION_FRAME_TYPE_MISSION_EVENT, (uint8_t)phase_id, (uint8_t)zone, (uint8_t)run_status);
}

VisualCommsStatus visual_comms_send_uav_handoff_request(uint8_t handoff_step) {
    return s_send_args_frame(MISSION_FRAME_TYPE_UAV_HANDOFF_REQUEST, (uint8_t)MISSION_ZONE_D, handoff_step, 0u);
}

VisualCommsStatus visual_comms_send_ack_ok(void) {
    return s_send_args_frame(MISSION_FRAME_TYPE_ACK_OK, 0u, 0u, 0u);
}

VisualCommsStatus visual_comms_send_ack_err(void) {
    return s_send_args_frame(MISSION_FRAME_TYPE_ACK_ERR, 0u, 0u, 0u);
}

bool visual_comms_recognition_is_stale(const MissionRecognitionResult* result) {
    if(result == NULL) {
        return false;
    }

    return (result->flags & MISSION_RECOGNITION_FLAG_STALE) != 0u;
}

bool visual_comms_recognition_has_pose(const MissionRecognitionResult* result) {
    if(result == NULL) {
        return false;
    }

    return (result->flags & MISSION_RECOGNITION_FLAG_POSE_VALID) != 0u;
}

bool visual_comms_recognition_retry_suggested(const MissionRecognitionResult* result) {
    if(result == NULL) {
        return false;
    }

    return (result->flags & MISSION_RECOGNITION_FLAG_RETRY_SUGGESTED) != 0u;
}

bool visual_comms_is_ready(void) {
    return s_visual_comms.initialized;
}

const VisualComms* visual_comms_get_state(void) {
    s_sync_view();
    return &s_visual_comms_view;
}

const char* visual_comms_status_str(VisualCommsStatus status) {
    switch(status) {
#define X(name, str)          \
    case VISUAL_COMMS_##name: \
        return str;
        VISUAL_COMMS_STATUS_TABLE
#undef X
        default:
            return "Unknown Visual Comms Status";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void s_uart_rx_complete(void) {
    if(s_visual_comms.initialized == false) {
        return;
    }

    if(s_write_byte_to_ring(s_visual_rx_byte) != VISUAL_COMMS_OK) {
        s_mark_malformed_frame("rx-ring-overflow", 0u);
    }

    (void)s_restart_receive();
}

static void s_uart_error(void) {
    if(s_visual_uart == NULL) {
        return;
    }

    (void)uart_abort_receive_it(s_visual_uart);
    (void)s_restart_receive();
}

static bool s_restart_receive(void) {
    if(s_visual_uart == NULL) {
        return false;
    }

    return uart_receive_it(s_visual_uart, &s_visual_rx_byte, 1u);
}

static VisualCommsStatus s_write_byte_to_ring(uint8_t byte) {
    if(ring_buf.write(&s_visual_rx_ring, byte) != RING_BUF_SUCCESS) {
        return VISUAL_COMMS_RING_BUFFER_ERROR;
    }

    return VISUAL_COMMS_OK;
}

static VisualCommsStatus s_process_byte(uint8_t byte) {
    if(s_frame_length == 0u) {
        (void)s_try_start_frame(byte);
        return VISUAL_COMMS_OK;
    }

    if(s_frame_length == 1u) {
        const uint8_t expected_header_byte = (s_frame_type == VISUAL_FRAME_SHORT) ? s_short_header[1] : s_recognition_header[1];
        if(byte != expected_header_byte) {
            s_reset_frame();
            (void)s_try_start_frame(byte);
            return VISUAL_COMMS_OK;
        }
    }

    s_frame_buffer[s_frame_length++] = byte;
    if(s_frame_length < s_expected_length) {
        return VISUAL_COMMS_OK;
    }

    return s_handle_complete_frame();
}

static bool s_try_start_frame(uint8_t byte) {
    if(byte == s_short_header[0]) {
        s_frame_buffer[0] = byte;
        s_frame_length = 1u;
        s_expected_length = VISUAL_SHORT_FRAME_LENGTH;
        s_frame_type = VISUAL_FRAME_SHORT;
        return true;
    }

    if(byte == s_recognition_header[0]) {
        s_frame_buffer[0] = byte;
        s_frame_length = 1u;
        s_expected_length = VISUAL_RECOGNITION_FRAME_LENGTH;
        s_frame_type = VISUAL_FRAME_RECOGNITION;
        return true;
    }

    return false;
}

static void s_reset_frame(void) {
    memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
    s_frame_length = 0u;
    s_expected_length = 0u;
    s_frame_type = VISUAL_FRAME_NONE;
}

static bool s_tail_matches(const uint8_t* frame, const uint8_t* tail, uint8_t frame_length) {
    return frame[frame_length - 2u] == tail[0] && frame[frame_length - 1u] == tail[1];
}

static VisualCommsStatus s_handle_complete_frame(void) {
    VisualCommsStatus status = VISUAL_COMMS_OK;

    if(s_frame_type == VISUAL_FRAME_SHORT) {
        if(s_frame_buffer[0] != s_short_header[0] || s_frame_buffer[1] != s_short_header[1] ||
           s_tail_matches(s_frame_buffer, s_short_tail, VISUAL_SHORT_FRAME_LENGTH) == false) {
            s_mark_malformed_frame("bad-short-frame", s_frame_buffer[2]);
            s_reset_frame();
            return VISUAL_COMMS_FRAME_ERROR;
        }

        status = s_handle_short_frame();
    }
    else if(s_frame_type == VISUAL_FRAME_RECOGNITION) {
        if(s_frame_buffer[0] != s_recognition_header[0] || s_frame_buffer[1] != s_recognition_header[1] ||
           s_tail_matches(s_frame_buffer, s_recognition_tail, VISUAL_RECOGNITION_FRAME_LENGTH) == false) {
            s_mark_malformed_frame("bad-recognition-frame", s_frame_buffer[2]);
            s_reset_frame();
            return VISUAL_COMMS_FRAME_ERROR;
        }

        status = s_handle_recognition_frame();
    }
    else {
        s_mark_malformed_frame("unknown-frame-type", 0u);
        s_reset_frame();
        return VISUAL_COMMS_FRAME_ERROR;
    }

    s_reset_frame();
    return status;
}

static VisualCommsStatus s_handle_short_frame(void) {
    const MissionFrameType type = (MissionFrameType)s_frame_buffer[2];

    switch(type) {
        case MISSION_FRAME_TYPE_START:
        case MISSION_FRAME_TYPE_STOP:
        case MISSION_FRAME_TYPE_ESTOP:
        case MISSION_FRAME_TYPE_RESET:
            if(s_frame_buffer[3] != 0u || s_frame_buffer[4] != 0u || s_frame_buffer[5] != 0u) {
                s_mark_malformed_frame("control-command-payload", (uint8_t)type);
                return VISUAL_COMMS_FRAME_ERROR;
            }

            s_visual_comms.pending_command = (MissionCommand)type;
            s_sync_view();
            return VISUAL_COMMS_OK;

        case MISSION_FRAME_TYPE_UAV_HANDOFF_ACK:
            if(s_frame_buffer[5] != 0u || s_is_valid_uav_handoff_status(s_frame_buffer[3]) == false) {
                s_mark_malformed_frame("uav-handoff-ack", (uint8_t)type);
                return VISUAL_COMMS_FRAME_ERROR;
            }

            s_visual_comms.latest_uav_handoff_ack.status = (MissionUavHandoffStatus)s_frame_buffer[3];
            s_visual_comms.latest_uav_handoff_ack.detail = s_frame_buffer[4];
            s_visual_comms.has_new_uav_handoff_ack = true;
            s_sync_view();
            return VISUAL_COMMS_OK;

        default:
            s_mark_malformed_frame("unsupported-short-frame-type", (uint8_t)type);
            return VISUAL_COMMS_FRAME_ERROR;
    }
}

static VisualCommsStatus s_handle_recognition_frame(void) {
    MissionRecognitionResult result;

    result.zone = (MissionZoneId)s_frame_buffer[2];
    result.sex = (FlowerSex)s_frame_buffer[3];
    result.confidence = s_frame_buffer[4];
    result.flags = s_frame_buffer[5];
    result.pose.x = s_decode_float_le(&s_frame_buffer[6]);
    result.pose.y = s_decode_float_le(&s_frame_buffer[10]);
    result.pose.z = s_decode_float_le(&s_frame_buffer[14]);

    if(s_is_valid_zone((uint8_t)result.zone) == false ||
       s_is_valid_flower_sex((uint8_t)result.sex) == false ||
       (result.flags & VISUAL_RECOGNITION_RESERVED_FLAG_MASK) != 0u ||
       isfinite(result.pose.x) == 0 || isfinite(result.pose.y) == 0 || isfinite(result.pose.z) == 0) {
        s_mark_malformed_frame("recognition-payload", (uint8_t)MISSION_FRAME_TYPE_SCAN_REQUEST);
        return VISUAL_COMMS_FRAME_ERROR;
    }

    s_visual_comms.latest_recognition = result;
    s_visual_comms.has_new_recognition = true;
    s_sync_view();
    return VISUAL_COMMS_OK;
}

static bool s_is_valid_zone(uint8_t zone) {
    return zone == (uint8_t)MISSION_ZONE_A ||
           zone == (uint8_t)MISSION_ZONE_B ||
           zone == (uint8_t)MISSION_ZONE_C ||
           zone == (uint8_t)MISSION_ZONE_D ||
           zone == (uint8_t)MISSION_ZONE_HOME;
}

static bool s_is_valid_flower_sex(uint8_t sex) {
    return sex == (uint8_t)FLOWER_SEX_UNKNOWN ||
           sex == (uint8_t)FLOWER_SEX_FEMALE ||
           sex == (uint8_t)FLOWER_SEX_MALE ||
           sex == (uint8_t)FLOWER_SEX_HERMAPHRODITE;
}

static bool s_is_valid_uav_handoff_status(uint8_t status) {
    return status == (uint8_t)MISSION_UAV_HANDOFF_SUCCESS ||
           status == (uint8_t)MISSION_UAV_HANDOFF_BUSY_RETRYABLE ||
           status == (uint8_t)MISSION_UAV_HANDOFF_FAIL_TERMINAL;
}

static float s_decode_float_le(const uint8_t* data) {
    union {
        uint32_t u32;
        float f32;
    } converter;

    converter.u32 = ((uint32_t)data[0]) |
                    ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[3] << 24);
    return converter.f32;
}

static VisualCommsStatus s_send_args_frame(MissionFrameType type, uint8_t arg0, uint8_t arg1, uint8_t arg2) {
    const uint8_t frame[VISUAL_SHORT_FRAME_LENGTH] = {
        s_short_header[0],
        s_short_header[1],
        (uint8_t)type,
        arg0,
        arg1,
        arg2,
        s_short_tail[0],
        s_short_tail[1],
    };

    return s_send_frame(frame, VISUAL_SHORT_FRAME_LENGTH);
}

static VisualCommsStatus s_send_frame(const uint8_t* data, uint16_t len) {
    if(s_visual_comms.initialized == false) {
        return VISUAL_COMMS_NOT_INITIALIZED;
    }
    if(data == NULL || len == 0u) {
        return VISUAL_COMMS_INVALID_PARAM;
    }
    if(uart10_write_blocking((const char*)data, len) == false) {
        return VISUAL_COMMS_UART_ERROR;
    }

    return VISUAL_COMMS_OK;
}

static void s_mark_malformed_frame(const char* reason, uint8_t type) {
    s_visual_comms.malformed_frame_count++;
    s_sync_view();
    (void)log_warn("VISUAL COMMS reject frame reason=%s type=0x%02X count=%lu",
                   reason,
                   (unsigned int)type,
                   (unsigned long)s_visual_comms.malformed_frame_count);
}

static void s_sync_view(void) {
    s_visual_comms_view = s_visual_comms;
}
