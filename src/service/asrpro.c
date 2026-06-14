#include "asrpro.h"

#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 语音播报函数
 * @param cmd 语音播报指令
 * @return 是否成功发送播报指令
 */
bool asrpro_broadcast(AsrProCmd cmd) {
    switch(cmd) {
        case START: {
            char data[] = { 0xAA, 0x55, 0x02, 0x01, 0x01, 0x03, 0xFF };
            uart10_write_blocking(data, sizeof(data));
            break;
        }
        default:
            return false;
    }

    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //
