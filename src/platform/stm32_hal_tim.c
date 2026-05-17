#include "stm32_hal_tim.h"

// ! ========================= 变 量 声 明 ========================= ! //

static void (*tim_callback)(void) = NULL;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

void tim_start(void) {
    HAL_TIM_Base_Start_IT(&htim6);
}

void tim_stop(void) {
    HAL_TIM_Base_Stop_IT(&htim6);
}

void tim_register_callback(TIM_HandleTypeDef* htim, void (*callback)(void)) {
    tim_callback = callback;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if(htim->Instance == htim6.Instance) {
        if(tim_callback != NULL) {
            tim_callback();
        }
    }
}
