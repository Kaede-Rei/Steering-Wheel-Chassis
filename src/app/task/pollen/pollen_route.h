#ifndef _pollen_route_h_
#define _pollen_route_h_

#include "arm.h" // IWYU pragma: keep

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

#define POLLEN_ROUTE_MAX_STEPS 6u

/**
 * @brief 单个授粉点的机械臂动作序列
 * @param steps 动作序列
 * @param step_count 动作数量
 */
typedef struct {
    FiveDofArmJointArray steps[POLLEN_ROUTE_MAX_STEPS];
    uint8_t step_count;
} PollenActionSequence;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool pollen_route_get(uint8_t nav_index, PollenActionSequence* out);

#endif
