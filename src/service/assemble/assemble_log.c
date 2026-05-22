#include "assemble.h"

#include "delay.h"
#include "log.h"
#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

#define ASSEMBLE_BOOT_SETTLE_DELAY_MS 2000u

static const LogPortOps log_ops = {
    .write = uart1_write,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = true,
    };

    delay_ms_init(HAL_GetTick);
    delay_ms(ASSEMBLE_BOOT_SETTLE_DELAY_MS);

    if(log_init(&log_config) != LOG_STATUS_OK) {
        return SYSTEM_STATUS_ERROR;
    }

    uart_register_tx_complete_callback(&huart1, log_write_complete);
    return SYSTEM_STATUS_OK;
}
