#include "assemble.h"

#include "log.h"
#include "usart.h"
#include "visual_comms.h"

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_comms(void) {
    VisualCommsStatus status = visual_comms.init(&huart10);
    if(status != VISUAL_COMMS_OK) {
        log_error("VISUAL COMMS init failed: %s", visual_comms.status_str(status));
        return SYSTEM_STATUS_ERROR;
    }

    log_info("VISUAL COMMS init done on UART10");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //
