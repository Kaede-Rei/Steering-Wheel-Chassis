#ifndef _fs_ia10b_h_
#define _fs_ia10b_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define FS_IA10B_IBUS_FRAME_LEN 32u
#define FS_IA10B_CHANNEL_COUNT 14u

typedef struct {
    uint16_t channel[FS_IA10B_CHANNEL_COUNT];
    uint32_t frame_count;
    uint32_t error_count;
    uint32_t last_update_ms;
    bool valid;
} FsIa10bData;

typedef struct {
    uint8_t bytes[FS_IA10B_IBUS_FRAME_LEN];
    uint32_t rx_byte_count;
    uint32_t header0_count;
    uint32_t header01_count;
    uint8_t latest_byte;
} FsIa10bDebug;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void ibus_init(void);
bool ibus_get_data(FsIa10bData* out);
bool ibus_get_debug(FsIa10bDebug* out);
bool ibus_is_online(uint32_t timeout_ms);
uint16_t ibus_get_channel(uint8_t index);

#endif
