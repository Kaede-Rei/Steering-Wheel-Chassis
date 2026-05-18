#include "assemble.h"

// ! system ! //
#include <assert.h>

// ! service ! //
#include "chassis.h"
#include "remote.h"

// ! device ! //
#include "rgb_led/rgb_led.h"
#include "rgb_led/ws2812_rgb_led.h"
#include "imu/imu.h"
#include "imu/bmi088.h"
#include "fs_ia10b.h"

// ! infra ! //
#include "delay.h"
#include "log.h"

// ! platform ! //
#include "stm32_hal_can.h"
#include "stm32_hal_exti.h"
#include "stm32_hal_spi.h"
#include "stm32_hal_tim.h"
#include "stm32_hal_uart.h"
#include "stm32_hal_tim.h"

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

bool chassis_control_flag = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void assemble_log(void);
static void assemble_imu(void);
static void assemble_rgb_led(void);
static void assemble_chassis(void);
static void assemble_remote(void);

static void rgb_led_write_complete_callback(SPI_HandleTypeDef* hspi);
static void chassis_control_callback(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void assemble_init(void) {
    delay_ms_init(HAL_GetTick);
    delay_ms(500);

    assemble_log();
    assemble_imu();
    assemble_rgb_led();
    assemble_remote();
    assemble_chassis();

    delay_ms(500);
    log_info("System initialized successfully");
}

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

static void assemble_imu(void) {
    ImuStatus status = IMU_STATUS_OK;

    assert(imu_set_instance(&bmi088_instance) == IMU_STATUS_OK);

    status = imu.init();
    if(status != IMU_STATUS_OK) {
        log_error(
            "BMI088 initialization failed: %s (%s)",
            imu.status_str(status),
            bmi088_error_str(bmi088_get_init_error()));
    }

    exti_register_callback(ACC_INT_Pin, bmi088_exti_callback);
    exti_register_callback(GYRO_INT_Pin, bmi088_exti_callback);
    spi_register_txrx_complete_callback(&hspi2, bmi088_spi_txrx_cplt_callback);
    spi_register_error_callback(&hspi2, bmi088_spi_error_callback);
}

static void assemble_rgb_led(void) {
    RgbLedConfig rgb_config;

    rgb_led_set_instance(&ws2812_rgb_led_instance);
    assert(ws2812_rgb_led_make_config(
        &rgb_config,
        &rgb_ops,
        rgb_color_buffer,
        sizeof(rgb_color_buffer),
        rgb_tx_buffer,
        sizeof(rgb_tx_buffer)) == RGB_LED_STATUS_OK);

    rgb_config.async_write = true;

    assert(rgb_led.init(&rgb_config) == RGB_LED_STATUS_OK);
    spi_register_tx_complete_callback(&hspi6, rgb_led_write_complete_callback);
    rgb_led.fill(0U, 255U, 0U);
    rgb_led.show();
}

static void assemble_remote(void) {
    ibus_init();
    remote_init();
}

static void assemble_chassis(void) {
    assert(can_filter_init() == STM32_HAL_CAN_OK);
    assert(can_start(&hfdcan1) == STM32_HAL_CAN_OK);
    assert(can_start(&hfdcan2) == STM32_HAL_CAN_OK);

    delay_ms(1000);

    assert(chassis_init() == chassis.OK);
    tim_register_callback(&htim6, chassis_control_callback);
    tim_start();
}

static void rgb_led_write_complete_callback(SPI_HandleTypeDef* hspi) {
    (void)hspi;
    (void)rgb_led_write_complete();
}

static void chassis_control_callback(void) {
    chassis_control_flag = true;
}
