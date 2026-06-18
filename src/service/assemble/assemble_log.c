#include "assemble.h"

#include "log.h"
#include "stm32_hal_uart.h"
#include "task/task.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 日志串口接收命令最大长度
 */
#define LOG_RX_CMD_MAX_LEN 32u

static const LogPortOps log_ops = {
    .write = uart1_write_blocking,
};

static bool s_rx_active = false;
static uint8_t s_rx_byte = 0u;
static uint8_t s_rx_len = 0u;
static char s_rx_cmd[LOG_RX_CMD_MAX_LEN] = { 0 };
static volatile bool s_rx_cmd_ready = false;
static char s_pending_cmd[LOG_RX_CMD_MAX_LEN] = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void log_rx_complete_callback(void);
static void log_error_callback(void);
static void log_feed_byte(uint8_t byte);
static bool log_restart_receive(void);
static bool log_take_pending_command(char* out, uint32_t out_size);
static void log_parse_command(const char* cmd);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = false,
    };

    if(log_init(&log_config) != LOG_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_rx_complete_callback(&huart1, log_rx_complete_callback);
    uart_register_error_callback(&huart1, log_error_callback);

    if(!log_restart_receive()) {
        return SYSTEM_STATUS_ERROR;
    }

    return SYSTEM_STATUS_OK;
}

void assemble_log_process(void) {
    char cmd[LOG_RX_CMD_MAX_LEN];

    if(!log_take_pending_command(cmd, sizeof(cmd))) {
        return;
    }

    log_parse_command(cmd);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void log_rx_complete_callback(void) {
    s_rx_active = false;
    log_feed_byte(s_rx_byte);
    (void)log_restart_receive();
}

static void log_error_callback(void) {
    s_rx_active = false;
    s_rx_len = 0u;
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    (void)uart_abort_receive_it(&huart1);
    (void)log_restart_receive();
}

static void log_feed_byte(uint8_t byte) {
    if(byte == '\r' || byte == '\n') {
        if(s_rx_len > 0u) {
            s_rx_cmd[s_rx_len] = '\0';
            memcpy(s_pending_cmd, s_rx_cmd, s_rx_len + 1u);
            s_rx_cmd_ready = true;
            s_rx_len = 0u;
        }
        return;
    }

    if(s_rx_len >= (LOG_RX_CMD_MAX_LEN - 1u)) {
        s_rx_len = 0u;
        return;
    }

    s_rx_cmd[s_rx_len++] = (char)byte;
}

static bool log_restart_receive(void) {
    s_rx_byte = 0u;
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    s_rx_active = uart_receive_it(&huart1, &s_rx_byte, 1u);
    return s_rx_active;
}

static bool log_take_pending_command(char* out, uint32_t out_size) {
    uint32_t primask;

    if(out == NULL || out_size == 0u) {
        return false;
    }

    if(!s_rx_cmd_ready) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if(!s_rx_cmd_ready) {
        if(primask == 0u) {
            __enable_irq();
        }
        return false;
    }

    strncpy(out, s_pending_cmd, out_size - 1u);
    out[out_size - 1u] = '\0';
    s_rx_cmd_ready = false;
    if(primask == 0u) {
        __enable_irq();
    }

    return true;
}

static void log_parse_command(const char* cmd) {
    if(cmd == NULL) {
        return;
    }

    if(strcmp(cmd, "CHASSIS:START") == 0) {
        (void)task_post(TASK_EVENT_START);
        log_info("LOG CMD: CHASSIS:START");
    }
}
