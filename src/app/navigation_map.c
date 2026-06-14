#include "navigation_map.h"

// ! ========================= 变 量 声 明 ========================= ! //

#define NAV_MAP_POINT_MAX 15u
#define BACK_HOME_POINT_COUNT 2u

static NavPoint nav_map[NAV_MAP_POINT_MAX] = { 0 };
static uint8_t current_point = 1u;
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

    nav_map[1].x = 0.0f;
    nav_map[1].y = -0.04f;
    nav_map[1].area_type = PASS_BY;

    nav_map[2].x = 0.64;
    nav_map[2].y = -0.05;
    nav_map[2].area_type = AREA_A;

    nav_map[3].x = 1.15;
    nav_map[3].y = -0.05;
    nav_map[3].area_type = AREA_A;

    nav_map[4].x = 1.65;
    nav_map[4].y = -0.05;
    nav_map[4].area_type = AREA_A;

    nav_map[5].x = 2.07;
    nav_map[5].y = -0.05;
    nav_map[5].area_type = PASS_BY;

    nav_map[6].x = 2.07;
    nav_map[6].y = -0.79;
    nav_map[6].area_type = PASS_BY;

    nav_map[7].x = 1.64;
    nav_map[7].y = -0.79;
    nav_map[7].area_type = AREA_B;

    nav_map[8].x = 1.15;
    nav_map[8].y = -0.79;
    nav_map[8].area_type = AREA_B;

    nav_map[9].x = 0.65;
    nav_map[9].y = -0.79;
    nav_map[9].area_type = AREA_B;

    nav_map[10].x = 0.35;
    nav_map[10].y = -0.79;
    nav_map[10].area_type = PASS_BY;

    nav_map[11].x = 0.35;
    nav_map[11].y = -1.55;
    nav_map[11].area_type = PASS_BY;

    nav_map[12].x = 0.78;
    nav_map[12].y = -1.55;
    nav_map[12].area_type = AREA_C;

    nav_map[13].x = 1.16;
    nav_map[13].y = -1.55;
    nav_map[13].area_type = AREA_C;

    nav_map[14].x = 1.58;
    nav_map[14].y = -1.55;
    nav_map[14].area_type = AREA_C;

    finish_current_point = false;
    current_point = 1u;
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

        if(current_point + 1u < NAV_MAP_POINT_MAX)
            current_point++;
    }

    return nav_map[current_point];
}

/**
 * @brief 获取返回路径的导航点
 * @return 返回路径的导航点数组
 */
NavPoint* get_back_home_points(void) {
    static NavPoint back_home_points[BACK_HOME_POINT_COUNT];

    back_home_points[0] = nav_map[NAV_MAP_POINT_MAX - 4u];
    back_home_points[1] = nav_map[0];
    back_home_points[1].x -= 0.02f;
    back_home_points[1].y -= 0.02f;

    return back_home_points;
}

/**
 * @brief 获取当前导航点索引
 * @return 当前导航点索引
 */
uint8_t get_current_nav_point_index(void) {
    return current_point;
}

/**
 * @brief 获取导航点总数
 * @return 导航点总数
 */
uint8_t get_nav_point_max(void) {
    return NAV_MAP_POINT_MAX;
}

/**
 * @brief 获取返回路径导航点数量
 * @return 返回路径导航点数量
 */
uint8_t get_back_home_point_count(void) {
    return BACK_HOME_POINT_COUNT;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //
