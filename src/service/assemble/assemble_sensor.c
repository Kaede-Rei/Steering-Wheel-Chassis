#include "assemble.h"

#include "stm32_hal_light.h"
#include "gw_gray.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_light(void) {
    light_init();
    gw_gray_init();

    return SYSTEM_STATUS_OK;
}
