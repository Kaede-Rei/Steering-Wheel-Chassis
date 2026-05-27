#ifndef _gw_gray_h_
#define _gw_gray_h_

#include <stdint.h>
#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    GW_GRAY_STATUS_OK = 0,
    GW_GRAY_STATUS_ERROR,
    GW_GRAY_STATUS_NOT_READY,
} GwGrayStatus;

typedef struct {
    uint8_t front_raw;
    uint8_t back_raw;

    /*
     * black_mask 中：
     * bit = 1 表示检测到黑线
     * bit = 0 表示白场
     */
    uint8_t front_black;
    uint8_t back_black;

    uint32_t timestamp_us;
    uint32_t update_count;
    uint32_t not_ready_count;
} GwGrayState;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void gw_gray_init(void);
GwGrayStatus gw_gray_update(void);

uint8_t gw_gray_get_front_raw(void);
uint8_t gw_gray_get_back_raw(void);
uint8_t gw_gray_get_front_black(void);
uint8_t gw_gray_get_back_black(void);

const GwGrayState* gw_gray_get_state(void);

#endif
