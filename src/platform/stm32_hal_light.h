#ifndef _stm32_hal_light_h_
#define _stm32_hal_light_h_

#include <stdbool.h>
#include <stdint.h>

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 光电传感器触发方向
 */
typedef enum {
    STM32_HAL_LIGHT_NONE = 0x00u,
    STM32_HAL_LIGHT_LEFT = 0x01u,
    STM32_HAL_LIGHT_RIGHT = 0x02u,
    STM32_HAL_LIGHT_BOTH = STM32_HAL_LIGHT_LEFT | STM32_HAL_LIGHT_RIGHT,
} LightSide;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化光电传感器 EXTI 回调
 */
void light_init(void);

/**
 * @brief 取出并清除已触发的光电事件
 * @return LightSide 本次取出的触发方向
 */
LightSide light_take_triggered(void);

/**
 * @brief 获取已触发但尚未清除的光电事件
 * @return LightSide 已触发方向
 */
LightSide light_get_triggered(void);

/**
 * @brief 清除指定方向的光电触发事件
 * @param side 需要清除的方向
 */
void light_clear_triggered(LightSide side);

#endif
