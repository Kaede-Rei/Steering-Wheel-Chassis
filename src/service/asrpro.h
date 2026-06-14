#ifndef _asrpro_h_
#define _asrpro_h_

#include <stdbool.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief AsrPro 语音播报指令
 */
typedef enum {
    START,


    MAX,
} AsrProCmd;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool asrpro_broadcast(AsrProCmd cmd);

#endif
