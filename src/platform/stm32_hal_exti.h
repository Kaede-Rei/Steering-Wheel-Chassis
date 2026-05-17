#ifndef _stm32_hal_exti_h_
#define _stm32_hal_exti_h_

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //



// ! ========================= 接 口 函 数 声 明 ========================= ! //

void exti_register_callback(uint16_t gpio_pin, void(*callback)(uint16_t gpio_pin));

#endif
