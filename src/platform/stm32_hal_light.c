#include "stm32_hal_light.h"

#include <stddef.h>

#include "stm32_hal_exti.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 光电传感器消抖时间
 */
#define LIGHT_DEBOUNCE_MS 50u

/**
 * @brief 已触发但尚未被业务层取走的光电事件
 */
static volatile uint8_t s_triggered_side = STM32_HAL_LIGHT_NONE;

/**
 * @brief 最近一次左右光电触发的时间戳
 */
static volatile uint32_t s_left_trigger_tick = 0u;
static volatile uint32_t s_right_trigger_tick = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 光电传感器 EXTI 回调入口
 * @param gpio_pin 触发中断的 GPIO 引脚掩码
 * @param user 用户指针
 */
static void light_exti_callback(uint16_t gpio_pin, void* user);

/**
 * @brief 根据 GPIO 引脚获取光电方向
 * @param gpio_pin GPIO 引脚掩码
 * @return LightSide 光电方向
 */
static LightSide light_get_side_by_pin(uint16_t gpio_pin);

/**
 * @brief 根据光电方向读取 GPIO 电平
 * @param side 光电方向
 * @return GPIO_PinState GPIO 电平
 */
static GPIO_PinState light_read_pin(LightSide side);

/**
 * @brief 获取光电方向对应的上次触发时间戳
 * @param side 光电方向
 * @return volatile uint32_t* 时间戳指针
 */
static volatile uint32_t* light_get_last_tick(LightSide side);

/**
 * @brief 进入临界区并返回之前的中断状态
 * @return uint32_t PRIMASK 状态
 */
static uint32_t light_enter_critical(void);

/**
 * @brief 退出临界区
 * @param primask 进入临界区前的 PRIMASK 状态
 */
static void light_exit_critical(uint32_t primask);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化光电传感器 EXTI 回调
 */
void light_init(void) {
    s_triggered_side = STM32_HAL_LIGHT_NONE;
    s_left_trigger_tick = 0u;
    s_right_trigger_tick = 0u;

    (void)exti_register_callback(LIGHT_LEFT_Pin, light_exti_callback, NULL);
    (void)exti_register_callback(LIGHT_RIGHT_Pin, light_exti_callback, NULL);
}

/**
 * @brief 取出并清除已触发的光电事件
 * @return LightSide 本次取出的触发方向
 */
LightSide light_take_triggered(void) {
    uint32_t primask = light_enter_critical();
    LightSide side = (LightSide)s_triggered_side;

    s_triggered_side = STM32_HAL_LIGHT_NONE;
    light_exit_critical(primask);

    return side;
}

/**
 * @brief 获取已触发但尚未清除的光电事件
 * @return LightSide 已触发方向
 */
LightSide light_get_triggered(void) {
    return (LightSide)s_triggered_side;
}

/**
 * @brief 清除指定方向的光电触发事件
 * @param side 需要清除的方向
 */
void light_clear_triggered(LightSide side) {
    uint32_t primask = light_enter_critical();

    s_triggered_side &= (uint8_t)(~(uint8_t)side);
    light_exit_critical(primask);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 光电传感器 EXTI 回调入口
 * @param gpio_pin 触发中断的 GPIO 引脚掩码
 * @param user 用户指针
 */
static void light_exti_callback(uint16_t gpio_pin, void* user) {
    LightSide side = light_get_side_by_pin(gpio_pin);
    volatile uint32_t* last_tick = light_get_last_tick(side);
    uint32_t now = HAL_GetTick();

    (void)user;

    if(side == STM32_HAL_LIGHT_NONE || last_tick == NULL) {
        return;
    }

    if(light_read_pin(side) != GPIO_PIN_RESET) {
        return;
    }

    if((*last_tick != 0u) && ((now - *last_tick) < LIGHT_DEBOUNCE_MS)) {
        return;
    }

    *last_tick = now;
    s_triggered_side |= (uint8_t)side;
}

/**
 * @brief 根据 GPIO 引脚获取光电方向
 * @param gpio_pin GPIO 引脚掩码
 * @return LightSide 光电方向
 */
static LightSide light_get_side_by_pin(uint16_t gpio_pin) {
    if(gpio_pin == LIGHT_LEFT_Pin) {
        return STM32_HAL_LIGHT_LEFT;
    }

    if(gpio_pin == LIGHT_RIGHT_Pin) {
        return STM32_HAL_LIGHT_RIGHT;
    }

    return STM32_HAL_LIGHT_NONE;
}

/**
 * @brief 根据光电方向读取 GPIO 电平
 * @param side 光电方向
 * @return GPIO_PinState GPIO 电平
 */
static GPIO_PinState light_read_pin(LightSide side) {
    if(side == STM32_HAL_LIGHT_LEFT) {
        return HAL_GPIO_ReadPin(LIGHT_LEFT_GPIO_Port, LIGHT_LEFT_Pin);
    }

    if(side == STM32_HAL_LIGHT_RIGHT) {
        return HAL_GPIO_ReadPin(LIGHT_RIGHT_GPIO_Port, LIGHT_RIGHT_Pin);
    }

    return GPIO_PIN_SET;
}

/**
 * @brief 获取光电方向对应的上次触发时间戳
 * @param side 光电方向
 * @return volatile uint32_t* 时间戳指针
 */
static volatile uint32_t* light_get_last_tick(LightSide side) {
    if(side == STM32_HAL_LIGHT_LEFT) {
        return &s_left_trigger_tick;
    }

    if(side == STM32_HAL_LIGHT_RIGHT) {
        return &s_right_trigger_tick;
    }

    return NULL;
}

/**
 * @brief 进入临界区并返回之前的中断状态
 * @return uint32_t PRIMASK 状态
 */
static uint32_t light_enter_critical(void) {
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出临界区
 * @param primask 进入临界区前的 PRIMASK 状态
 */
static void light_exit_critical(uint32_t primask) {
    if(primask == 0u) {
        __enable_irq();
    }
}
