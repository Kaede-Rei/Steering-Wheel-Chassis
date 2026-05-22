#ifndef _stm32_hal_bmi088_h_
#define _stm32_hal_bmi088_h_

#include <stdint.h>

#include "imu/bmi088.h"
#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 函 数 声 明 ========================= ! //

const Bmi088PortOps* stm32_bmi088_get_ops(void);
uint16_t stm32_bmi088_get_accel_int_pin(void);
uint16_t stm32_bmi088_get_gyro_int_pin(void);
void stm32_bmi088_spi_txrx_complete_callback(SPI_HandleTypeDef* hspi);
void stm32_bmi088_spi_error_callback(SPI_HandleTypeDef* hspi);

#endif
