#include "navigation_map.h"

#include <stdint.h>
#include <stdbool.h>

// ! ========================= 变 量 声 明 ========================= ! //

NavPoint nav_map[10] = { 0 };
static uint8_t current_point = 0;
static bool finish_current_point = false;

// ! ========================= 私 有 函 数 声 明 ========================= ! //



// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化导航地图
 */
void nav_map_init(void) {
    nav_map[0].x = 0.0f;
    nav_map[0].y = 0.0f;
    nav_map[0].area_type = START_END;

    finish_current_point = false;
    current_point = 0;
}

/**
 * @brief 标记当前导航点已完成
 */
void finish_current_nav_point(void) {
    finish_current_point = true;
}

/**
 * @brief 获取下一个导航点
 * @return 下一个导航点
 * @note 如果当前导航点未完成，则返回当前导航点；如果当前导航点已完成，则返回下一个导航点并将完成标志重置为 false
 */
NavPoint get_next_nav_point(void) {
    if(finish_current_point == true) {
        finish_current_point = false;
        return nav_map[++current_point];
    }
    else
        return nav_map[current_point];
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //
