#include "bsp_led.h"

#include "main.h"

// ! ========================= 变 量 声 明 ========================= ! //

const struct LedInterface led_instance = {
    .on = led_on,
    .off = led_off,
    .toggle = led_toggle
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void led_on(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

void led_off(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
}

void led_toggle(void) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
}


