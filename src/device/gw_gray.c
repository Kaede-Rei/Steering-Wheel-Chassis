#include "gw_gray.h"

#include "main.h"
#include "delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

#define GW_GRAY_CLK_HIGH_US       5u
#define GW_GRAY_FRAME_GAP_US      1000u

#define GW_GRAY_BLACK_ACTIVE_LOW  1u

#define GW_GRAY_FRONT_REVERSE     0u
#define GW_GRAY_BACK_REVERSE      0u

static GwGrayState s_state;
static uint32_t s_last_frame_us = 0u;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void gw_gray_front_clk(uint8_t high);
static void gw_gray_back_clk(uint8_t high);
static uint8_t gw_gray_front_data(void);
static uint8_t gw_gray_back_data(void);

static uint8_t gw_gray_reverse8(uint8_t value);
static uint8_t gw_gray_make_black(uint8_t raw);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void gw_gray_init(void) {
    gw_gray_front_clk(0u);
    gw_gray_back_clk(0u);

    s_state.front_raw = 0u;
    s_state.back_raw = 0u;
    s_state.front_black = 0u;
    s_state.back_black = 0u;
    s_state.timestamp_us = 0u;
    s_state.update_count = 0u;
    s_state.not_ready_count = 0u;

    s_last_frame_us = delay_now_us();
}

GwGrayStatus gw_gray_update(void) {
    uint8_t front_raw = 0u;
    uint8_t back_raw = 0u;
    uint32_t now = delay_now_us();

    if((uint32_t)(now - s_last_frame_us) < GW_GRAY_FRAME_GAP_US) {
        s_state.not_ready_count++;
        return GW_GRAY_STATUS_NOT_READY;
    }

    gw_gray_front_clk(0u);
    gw_gray_back_clk(0u);

    for(uint8_t i = 0u; i < 8u; ++i) {
        if(gw_gray_front_data() != 0u) {
            front_raw |= (uint8_t)(1u << i);
        }

        if(gw_gray_back_data() != 0u) {
            back_raw |= (uint8_t)(1u << i);
        }

        gw_gray_front_clk(1u);
        gw_gray_back_clk(1u);

        delay_us(GW_GRAY_CLK_HIGH_US);

        gw_gray_front_clk(0u);
        gw_gray_back_clk(0u);
    }

    s_last_frame_us = delay_now_us();

    if(GW_GRAY_FRONT_REVERSE) {
        front_raw = gw_gray_reverse8(front_raw);
    }

    if(GW_GRAY_BACK_REVERSE) {
        back_raw = gw_gray_reverse8(back_raw);
    }

    s_state.front_raw = front_raw;
    s_state.back_raw = back_raw;
    s_state.front_black = gw_gray_make_black(front_raw);
    s_state.back_black = gw_gray_make_black(back_raw);
    s_state.timestamp_us = s_last_frame_us;
    s_state.update_count++;

    return GW_GRAY_STATUS_OK;
}

uint8_t gw_gray_get_front_raw(void) {
    return s_state.front_raw;
}

uint8_t gw_gray_get_back_raw(void) {
    return s_state.back_raw;
}

uint8_t gw_gray_get_front_black(void) {
    return s_state.front_black;
}

uint8_t gw_gray_get_back_black(void) {
    return s_state.back_black;
}

const GwGrayState* gw_gray_get_state(void) {
    return &s_state;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void gw_gray_front_clk(uint8_t high) {
    HAL_GPIO_WritePin(TRACE_FORWARD_CLK_GPIO_Port, TRACE_FORWARD_CLK_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void gw_gray_back_clk(uint8_t high) {
    HAL_GPIO_WritePin(TRACE_BACK_CLK_GPIO_Port, TRACE_BACK_CLK_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t gw_gray_front_data(void) {
    return HAL_GPIO_ReadPin(TRACE_FORWARD_DATA_GPIO_Port, TRACE_FORWARD_DATA_Pin) == GPIO_PIN_SET ? 1u : 0u;
}

static uint8_t gw_gray_back_data(void) {
    return HAL_GPIO_ReadPin(TRACE_BACK_DATA_GPIO_Port, TRACE_BACK_DATA_Pin) == GPIO_PIN_SET ? 1u : 0u;
}

static uint8_t gw_gray_make_black(uint8_t raw) {
    if(GW_GRAY_BLACK_ACTIVE_LOW) return (uint8_t)(~raw);
    else return raw;
}

static uint8_t gw_gray_reverse8(uint8_t value) {
    value = (uint8_t)(((value & 0xF0u) >> 4u) | ((value & 0x0Fu) << 4u));
    value = (uint8_t)(((value & 0xCCu) >> 2u) | ((value & 0x33u) << 2u));
    value = (uint8_t)(((value & 0xAAu) >> 1u) | ((value & 0x55u) << 1u));
    return value;
}
