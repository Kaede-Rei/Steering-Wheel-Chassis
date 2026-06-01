#include "visual_comms.h"

#include <math.h>
#include <string.h>

#include "protocol_parser.h"
#include "stm32_hal_uart.h"

// ! ========================= 常 量 / 宏 声 明 ========================= ! //

#define VISUAL_CTRL_FRAME_LENGTH 8u
#define VISUAL_COORD_FRAME_LENGTH 16u
#define VISUAL_RX_RING_BUFFER_SIZE 64u

// ! ========================= 私 有 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    VISUAL_FRAME_NONE = 0,
    VISUAL_FRAME_CONTROL,
    VISUAL_FRAME_COORDINATE,
} VisualFrameType;

static const uint8_t s_control_header[2] = { 0xAAu, 0x55u };
static const uint8_t s_control_tail[2] = { 0x55u, 0xAAu };
static const uint8_t s_coordinate_header[2] = { 0xBBu, 0x66u };
static const uint8_t s_coordinate_tail[2] = { 0x66u, 0xBBu };
static const uint8_t s_coordinate_ack_success[VISUAL_CTRL_FRAME_LENGTH] = { 0xAAu, 0x55u, 0x02u, 0x00u, 0x00u, 0x00u, 0x55u, 0xAAu };
static const uint8_t s_coordinate_ack_failed[VISUAL_CTRL_FRAME_LENGTH] = { 0xAAu, 0x55u, 0x02u, 0x01u, 0x00u, 0x00u, 0x55u, 0xAAu };
static const uint8_t s_end_task_notify[VISUAL_CTRL_FRAME_LENGTH] = { 0xAAu, 0x55u, 0x03u, 0x00u, 0x00u, 0x00u, 0x55u, 0xAAu };

static VisualComms s_visual_comms = { 0 };
static VisualComms s_visual_comms_view = { 0 };
static UART_HandleTypeDef* s_visual_uart = NULL;
static RingBuf s_visual_rx_ring = { 0 };
static uint8_t s_visual_rx_ring_storage[VISUAL_RX_RING_BUFFER_SIZE] = { 0 };
static uint8_t s_visual_rx_byte = 0u;
static uint8_t s_frame_buffer[VISUAL_COORD_FRAME_LENGTH] = { 0 };
static uint8_t s_frame_length = 0u;
static uint8_t s_expected_length = 0u;
static VisualFrameType s_frame_type = VISUAL_FRAME_NONE;
static volatile bool s_pending_ack = false;
static volatile bool s_pending_ack_success = false;
static uint32_t s_protocol_parser_primask = 0u;

const struct VisualCommsInterface visual_comms_interface = {
#define X(name, str) .name = VISUAL_COMMS_##name,
    { VISUAL_COMMS_STATUS_TABLE },
#undef X
    .init = visual_comms_init,
    .process = visual_comms_process,
    .consume_command = visual_comms_consume_command,
    .consume_coordinate = visual_comms_consume_coordinate,
    .send_coordinate_ack = visual_comms_send_coordinate_ack,
    .send_end_task = visual_comms_send_end_task,
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
static VisualCommsStatus s_handle_control_frame(void);
static VisualCommsStatus s_handle_coordinate_frame(void);
static float s_decode_float_le(const uint8_t* data);
static VisualCommsStatus s_send_frame(const uint8_t* data, uint16_t len);
static void s_sync_view(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 为 `protocol_parser` 提供临界区入口实现
 *
 * 当前视觉通信服务复用了 `infra/protocol_parser.*` 中的环形缓冲区能力，
 * 因此在本文件内提供该模块要求的临界区钩子
 */
void protocol_parser_enter_critical(void) {
    s_protocol_parser_primask = __get_PRIMASK();
    __disable_irq();
}

/**
 * @brief 为 `protocol_parser` 提供临界区出口实现
 */
void protocol_parser_exit_critical(void) {
    if(s_protocol_parser_primask == 0u) {
        __enable_irq();
    }
}

/**
 * @brief 初始化视觉通信服务
 * @param huart 视觉模块使用的串口句柄，当前预期传入 `&huart10`
 * @return VisualCommsStatus 状态码
 */
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

/**
 * @brief 处理当前已接收的视觉串口数据
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_process(void) {
    uint8_t byte = 0u;
    VisualCommsStatus status = VISUAL_COMMS_OK;

    if(s_visual_comms.initialized == false) {
        return VISUAL_COMMS_NOT_INITIALIZED;
    }

    if(s_pending_ack) {
        const bool success = s_pending_ack_success;
        s_pending_ack = false;
        s_pending_ack_success = false;
        status = visual_comms_send_coordinate_ack(success);
        if(status != VISUAL_COMMS_OK) {
            return status;
        }
    }

    while(ring_buf.read(&s_visual_rx_ring, &byte) == RING_BUF_SUCCESS) {
        status = s_process_byte(byte);
        if(status != VISUAL_COMMS_OK) {
            return status;
        }
    }

    return VISUAL_COMMS_OK;
}

/**
 * @brief 取走一条待处理控制命令
 * @param command 输出参数，用于接收命令类型
 * @return true 成功取到一条新命令
 * @return false 当前无新命令或参数无效
 */
bool visual_comms_consume_command(VisualCommand* command) {
    if(s_visual_comms.initialized == false || command == NULL) {
        return false;
    }
    if(s_visual_comms.pending_command == VISUAL_CMD_NONE) {
        return false;
    }

    *command = s_visual_comms.pending_command;
    s_visual_comms.current_command = s_visual_comms.pending_command;
    s_visual_comms.pending_command = VISUAL_CMD_NONE;
    s_sync_view();
    return true;
}

/**
 * @brief 取走一组新坐标
 * @param coordinate 输出参数，用于接收坐标值
 * @return true 成功取到一组新坐标
 * @return false 当前无新坐标或参数无效
 */
bool visual_comms_consume_coordinate(VisualCoordinate* coordinate) {
    if(s_visual_comms.initialized == false || coordinate == NULL) {
        return false;
    }
    if(s_visual_comms.has_new_coordinate == false) {
        return false;
    }

    *coordinate = s_visual_comms.latest_coordinate;
    s_visual_comms.has_new_coordinate = false;
    s_sync_view();
    return true;
}

/**
 * @brief 发送坐标接收应答
 * @param success true 发送成功应答，false 发送失败应答
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_send_coordinate_ack(bool success) {
    const uint8_t* frame = success ? s_coordinate_ack_success : s_coordinate_ack_failed;
    return s_send_frame(frame, VISUAL_CTRL_FRAME_LENGTH);
}

/**
 * @brief 发送任务结束通知
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_send_end_task(void) {
    return s_send_frame(s_end_task_notify, VISUAL_CTRL_FRAME_LENGTH);
}

/**
 * @brief 查询视觉通信服务是否已经完成初始化
 * @return true 服务可用
 * @return false 服务尚未初始化
 */
bool visual_comms_is_ready(void) {
    return s_visual_comms.initialized;
}

/**
 * @brief 获取视觉通信服务只读状态视图
 * @return const VisualComms* 只读状态快照指针
 */
const VisualComms* visual_comms_get_state(void) {
    s_sync_view();
    return &s_visual_comms_view;
}

/**
 * @brief 将视觉通信服务状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
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

/**
 * @brief UART 单字节接收完成回调
 *
 * 该回调仅负责把字节写入内部环形缓冲区，并重新启动下一字节接收；
 * 不在中断中执行阻塞串口发送
 */
static void s_uart_rx_complete(void) {
    if(s_visual_comms.initialized == false) {
        return;
    }

    if(s_write_byte_to_ring(s_visual_rx_byte) != VISUAL_COMMS_OK) {
        s_pending_ack = true;
        s_pending_ack_success = false;
    }

    (void)s_restart_receive();
}

/**
 * @brief UART 错误回调
 *
 * 当前策略为中止接收并立即重启单字节中断接收
 */
static void s_uart_error(void) {
    if(s_visual_uart == NULL) {
        return;
    }

    (void)uart_abort_receive_it(s_visual_uart);
    (void)s_restart_receive();
}

/**
 * @brief 重新启动 UART 单字节中断接收
 * @return true 启动成功
 * @return false 启动失败
 */
static bool s_restart_receive(void) {
    if(s_visual_uart == NULL) {
        return false;
    }

    return uart_receive_it(s_visual_uart, &s_visual_rx_byte, 1u);
}

/**
 * @brief 将一个接收到的字节写入内部环形缓冲区
 * @param byte 待写入字节
 * @return VisualCommsStatus 状态码
 */
static VisualCommsStatus s_write_byte_to_ring(uint8_t byte) {
    if(ring_buf.write(&s_visual_rx_ring, byte) != RING_BUF_SUCCESS) {
        return VISUAL_COMMS_RING_BUFFER_ERROR;
    }

    return VISUAL_COMMS_OK;
}

/**
 * @brief 处理单个串口字节并推进内部帧状态机
 * @param byte 待处理字节
 * @return VisualCommsStatus 状态码
 */
static VisualCommsStatus s_process_byte(uint8_t byte) {
    if(s_frame_length == 0u) {
        (void)s_try_start_frame(byte);
        return VISUAL_COMMS_OK;
    }

    if(s_frame_length == 1u) {
        const uint8_t expected_header_byte = (s_frame_type == VISUAL_FRAME_CONTROL) ? s_control_header[1] : s_coordinate_header[1];
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

/**
 * @brief 尝试以当前字节作为新帧帧头开始解析
 * @param byte 当前输入字节
 * @return true 已识别为某类帧头首字节并启动收帧
 * @return false 当前字节不构成已知帧头
 */
static bool s_try_start_frame(uint8_t byte) {
    if(byte == s_control_header[0]) {
        s_frame_buffer[0] = byte;
        s_frame_length = 1u;
        s_expected_length = VISUAL_CTRL_FRAME_LENGTH;
        s_frame_type = VISUAL_FRAME_CONTROL;
        return true;
    }

    if(byte == s_coordinate_header[0]) {
        s_frame_buffer[0] = byte;
        s_frame_length = 1u;
        s_expected_length = VISUAL_COORD_FRAME_LENGTH;
        s_frame_type = VISUAL_FRAME_COORDINATE;
        return true;
    }

    return false;
}

/**
 * @brief 复位当前正在接收的临时帧状态
 */
static void s_reset_frame(void) {
    memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
    s_frame_length = 0u;
    s_expected_length = 0u;
    s_frame_type = VISUAL_FRAME_NONE;
}

/**
 * @brief 检查一帧数据的帧尾是否符合预期
 * @param frame 完整帧缓冲区
 * @param tail 目标帧尾
 * @param frame_length 帧长
 * @return true 帧尾匹配
 * @return false 帧尾不匹配
 */
static bool s_tail_matches(const uint8_t* frame, const uint8_t* tail, uint8_t frame_length) {
    return frame[frame_length - 2u] == tail[0] && frame[frame_length - 1u] == tail[1];
}

/**
 * @brief 处理一帧已收满的完整数据
 * @return VisualCommsStatus 状态码
 */
static VisualCommsStatus s_handle_complete_frame(void) {
    VisualCommsStatus status;

    if(s_frame_type == VISUAL_FRAME_CONTROL) {
        if(s_frame_buffer[0] != s_control_header[0] || s_frame_buffer[1] != s_control_header[1] ||
           s_tail_matches(s_frame_buffer, s_control_tail, VISUAL_CTRL_FRAME_LENGTH) == false) {
            s_reset_frame();
            return VISUAL_COMMS_FRAME_ERROR;
        }
    }
    else if(s_frame_type == VISUAL_FRAME_COORDINATE) {
        if(s_frame_buffer[0] != s_coordinate_header[0] || s_frame_buffer[1] != s_coordinate_header[1] ||
           s_tail_matches(s_frame_buffer, s_coordinate_tail, VISUAL_COORD_FRAME_LENGTH) == false) {
            s_reset_frame();
            s_pending_ack = true;
            s_pending_ack_success = false;
            return VISUAL_COMMS_FRAME_ERROR;
        }
    }
    else {
        s_reset_frame();
        return VISUAL_COMMS_FRAME_ERROR;
    }

    if(s_frame_type == VISUAL_FRAME_CONTROL) {
        status = s_handle_control_frame();
    }
    else {
        status = s_handle_coordinate_frame();
    }

    s_reset_frame();
    return status;
}

/**
 * @brief 解析并处理 8 字节控制帧
 * @return VisualCommsStatus 状态码
 */
static VisualCommsStatus s_handle_control_frame(void) {
    if(s_frame_buffer[2] != 0x01u || s_frame_buffer[4] != 0x00u || s_frame_buffer[5] != 0x00u) {
        return VISUAL_COMMS_FRAME_ERROR;
    }

    if(s_frame_buffer[3] == 0x01u) {
        s_visual_comms.pending_command = VISUAL_CMD_START_RECOGNIZE_A;
    }
    else if(s_frame_buffer[3] == 0x02u) {
        s_visual_comms.pending_command = VISUAL_CMD_START_RECOGNIZE_B;
    }
    else {
        return VISUAL_COMMS_FRAME_ERROR;
    }

    s_sync_view();
    return VISUAL_COMMS_OK;
}

/**
 * @brief 解析并处理 16 字节坐标帧
 * @return VisualCommsStatus 状态码
 */
static VisualCommsStatus s_handle_coordinate_frame(void) {
    VisualCoordinate coordinate;

    coordinate.x = s_decode_float_le(&s_frame_buffer[2]);
    coordinate.y = s_decode_float_le(&s_frame_buffer[6]);
    coordinate.z = s_decode_float_le(&s_frame_buffer[10]);

    if(isfinite(coordinate.x) == 0 || isfinite(coordinate.y) == 0 || isfinite(coordinate.z) == 0) {
        s_pending_ack = true;
        s_pending_ack_success = false;
        return VISUAL_COMMS_FRAME_ERROR;
    }

    s_visual_comms.latest_coordinate = coordinate;
    s_visual_comms.has_new_coordinate = true;
    s_sync_view();

    return visual_comms_send_coordinate_ack(true);
}

/**
 * @brief 将 4 字节小端数据解码为 `float`
 * @param data 指向 4 字节小端浮点数据
 * @return float 解码后的浮点值
 */
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

/**
 * @brief 通过视觉串口发送一帧完整数据
 * @param data 待发送帧缓冲区
 * @param len 帧长度
 * @return VisualCommsStatus 状态码
 */
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

/**
 * @brief 将内部可变服务状态同步到对外只读视图
 */
static void s_sync_view(void) {
    s_visual_comms_view = s_visual_comms;
}
