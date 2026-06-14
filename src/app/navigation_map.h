#ifndef _navigation_map_h_
#define _navigation_map_h_


// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 当前区域类型枚举
 * @param START_END 起点/终点
 * @param AREA_A 区域 A
 * @param AREA_B 区域 B
 * @param AREA_C 区域 C
 */
typedef enum {
    START_END,
    AREA_A,
    AREA_B,
    AREA_C
} AreaType;

/**
 * @brief 导航点结构体
 * @param x 导航点 x 坐标
 * @param y 导航点 y 坐标
 * @param area_type 导航点所在区域类型
 */
typedef struct {
    float x;
    float y;
    AreaType area_type;
} NavPoint;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

void nav_map_init(void);
void finish_current_nav_point(void);
NavPoint get_next_nav_point(void);

#endif
