#ifndef _stm32_hal_tim_h_
#define _stm32_hal_tim_h_

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

extern TIM_HandleTypeDef htim6;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void tim_start(void);
void tim_stop(void);
void tim_register_callback(TIM_HandleTypeDef* htim, void (*callback)(void));

#endif
