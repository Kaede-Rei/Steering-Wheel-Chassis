#include "stm32_hal_exti.h"

// ! ========================= 变 量 声 明 ========================= ! //

static void(*exti_callbacks[16])(uint16_t gpio_pin) = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static int8_t exti_gpio_pin_to_index(uint16_t gpio_pin);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void exti_register_callback(uint16_t gpio_pin, void(*callback)(uint16_t gpio_pin)) {
    int8_t pin_index = exti_gpio_pin_to_index(gpio_pin);

    if(pin_index < 0) {
        return;
    }

    exti_callbacks[pin_index] = callback;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    int8_t pin_index = exti_gpio_pin_to_index(GPIO_Pin);

    if(pin_index < 0) {
        return;
    }

    if(exti_callbacks[pin_index] != NULL) {
        exti_callbacks[pin_index](GPIO_Pin);
    }
}

static int8_t exti_gpio_pin_to_index(uint16_t gpio_pin) {
    if((gpio_pin == 0U) || ((gpio_pin & (gpio_pin - 1U)) != 0U)) {
        return -1;
    }

    for(int8_t pin_index = 0; pin_index < 16; pin_index++) {
        if(gpio_pin == (uint16_t)(1U << pin_index)) {
            return pin_index;
        }
    }

    return -1;
}
