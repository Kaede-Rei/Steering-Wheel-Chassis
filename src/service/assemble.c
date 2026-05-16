#include "assemble.h"

// ! system ! //
#include <assert.h>

// ! device ! //
#include "rgb_led/rgb_led.h"
#include "rgb_led/ws2812_rgb_led.h"

// ! service ! //
#include "chassis.h"

// ! infra ! //
#include "delay.h"
#include "log.h"

// ! platform ! //
#include "stm32_hal_can.h"
#include "stm32_hal_spi.h"
#include "stm32_hal_uart.h"
#include "fdcan.h"

// ! ========================= 变 量 声 明 ========================= ! //

static uint8_t rgb_color_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * RGB_LED_COLOR_BYTES];
static uint8_t rgb_tx_buffer[WS2812_RGB_LED_DEFAULT_PIXEL_COUNT * WS2812_RGB_LED_BITS_PER_PIXEL + WS2812_RGB_LED_DEFAULT_RESET_BYTES]
__attribute__((section(".ram_d3"), aligned(32)));

static const RgbLedPortOps rgb_ops = {
    .write = spi_write,
};

static const LogPortOps log_ops = {
    .write = uart1_write,
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void assemble_log(void);
static void assemble_rgb_led(void);
static void assemble_chassis(void);
static void assemble_rgb_led_write_complete(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void assemble_init(void) {
    delay_ms_init(HAL_GetTick);
    assemble_log();
    assemble_rgb_led();
    assemble_chassis();

    delay_ms(500);
    log_info("System initialized successfully");
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void assemble_log(void) {
    LogConfig log_config = {
        .ops = &log_ops,
        .level = LOG_LEVEL_INFO,
        .enable_color = true,
        .async_write = true,
    };

    assert(log_init(&log_config) == LOG_STATUS_OK);
    uart_register_tx_complete_callback(&huart1, log_write_complete);
}

static void assemble_rgb_led(void) {
    RgbLedConfig rgb_config;

    rgb_led_set_instance(&ws2812_rgb_led_instance);
    assert(ws2812_rgb_led_make_config(&rgb_config, &rgb_ops,
        rgb_color_buffer, sizeof(rgb_color_buffer),
        rgb_tx_buffer, sizeof(rgb_tx_buffer)) == RGB_LED_STATUS_OK);
    rgb_config.async_write = true;

    assert(rgb_led.init(&rgb_config) == RGB_LED_STATUS_OK);
    spi_register_tx_complete_callback(&hspi6, assemble_rgb_led_write_complete);
    rgb_led.fill(0u, 255u, 0u);
    rgb_led.show();
}

static void assemble_chassis(void) {
    assert(can_filter_init() == STM32_HAL_CAN_OK);
    assert(can_start(&hfdcan1) == STM32_HAL_CAN_OK);
    assert(can_start(&hfdcan2) == STM32_HAL_CAN_OK);

    delay_ms(1000);

    assert(chassis_init() == chassis.OK);
}

static void assemble_rgb_led_write_complete(void) {
    (void)rgb_led_write_complete();
}
