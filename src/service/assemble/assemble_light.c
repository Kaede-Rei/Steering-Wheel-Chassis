#include "assemble.h"

#include "stm32_hal_light.h"

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_light(void) {
    light_init();

    return SYSTEM_STATUS_OK;
}
