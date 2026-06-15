#include "navigation_route.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define NAV_ROUTE_POINT_COUNT 21u
#define NAV_RETURN_ROUTE_POINT_COUNT 2u

static const NavPoint nav_route_points[NAV_ROUTE_POINT_COUNT] = {
    { .x = 0.0f, .y = 0.0f, .area_type = START_END },
    { .x = 0.0f, .y = -0.04f, .area_type = PASS_BY },
    { .x = 0.64f, .y = -0.05f, .area_type = AREA_A, .y_flowers = { .up = true, .mid = true, .down = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 1.606f, 2.315f, 5.875f, 2.152f, 3.141f } } } },
    { .x = 0.64f, .y = -0.05f, .area_type = AREA_A, .y_flowers = { .up = false, .mid = false, .down = true }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 4.714f, 2.315f, 5.875f, 2.152f, 3.141f } } } },
    { .x = 1.15f, .y = -0.05f, .area_type = AREA_A, .y_flowers = { .up = true, .mid = true, .down = true }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 1.606f, 2.315f, 5.875f, 2.152f, 3.141f } } } },
    { .x = 1.15f, .y = -0.05f, .area_type = AREA_A, .y_flowers = { .up = true, .mid = true, .down = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 4.714f, 2.315f, 5.875f, 2.152f, 3.141f } } } },
    { .x = 1.65f, .y = -0.05f, .area_type = AREA_A, .y_flowers = { .up = true, .mid = true, .down = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 1.606f, 2.315f, 5.875f, 2.152f, 3.141f } } } },
    { .x = 1.65f, .y = -0.05f, .area_type = AREA_A, .y_flowers = { .up = false, .mid = false, .down = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 4.714f, 2.315f, 5.875f, 2.152f, 3.141f } } } },
    { .x = 2.07f, .y = -0.05f, .area_type = PASS_BY },
    { .x = 2.07f, .y = -0.79f, .area_type = PASS_BY },
    { .x = 1.64f, .y = -0.79f, .area_type = AREA_B, .x_flowers = { .left = true, .mid = true, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 0.206f, 1.591f, 5.380f, 2.172f, 3.141f } } } },
    { .x = 1.64f, .y = -0.79f, .area_type = AREA_B, .x_flowers = { .left = false, .mid = true, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 3.141f, 1.591f, 5.380f, 2.172f, 3.141f } } } },
    { .x = 1.15f, .y = -0.79f, .area_type = AREA_B, .x_flowers = { .left = true, .mid = true, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 0.206f, 1.591f, 5.380f, 2.172f, 3.141f } } } },
    { .x = 1.15f, .y = -0.79f, .area_type = AREA_B, .x_flowers = { .left = false, .mid = true, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 3.141f, 1.591f, 5.380f, 2.172f, 3.141f } } } },
    { .x = 0.65f, .y = -0.79f, .area_type = AREA_B, .x_flowers = { .left = true, .mid = true, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 0.206f, 1.591f, 5.380f, 2.172f, 3.141f } } } },
    { .x = 0.65f, .y = -0.79f, .area_type = AREA_B, .x_flowers = { .left = false, .mid = true, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 3.141f, 1.591f, 5.380f, 2.172f, 3.141f } } } },
    { .x = 0.35f, .y = -0.79f, .area_type = PASS_BY },
    { .x = 0.35f, .y = -1.55f, .area_type = PASS_BY },
    { .x = 0.78f, .y = -1.55f, .area_type = AREA_C, .x_flowers = { .left = true, .mid = false, .right = true }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 1.536f, 2.511f, 5.660f, 2.424f, 3.141f } } } },
    { .x = 1.16f, .y = -1.55f, .area_type = AREA_C, .x_flowers = { .left = true, .mid = false, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 1.536f, 2.511f, 5.660f, 2.424f, 3.141f } } } },
    { .x = 1.58f, .y = -1.55f, .area_type = AREA_C, .x_flowers = { .left = false, .mid = false, .right = false }, .pre_detect_joints = { .exist = true, .joints = { .dof = ARM_DOF, .q = { 1.536f, 2.511f, 5.660f, 2.424f, 3.141f } } } },
};

static const NavPoint nav_return_route_points[NAV_RETURN_ROUTE_POINT_COUNT] = {
    { .x = 0.35f, .y = -1.55f, .area_type = PASS_BY },
    { .x = -0.02f, .y = -0.01f, .area_type = START_END },
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取导航路径点
 * @param index 导航点索引
 * @param out 导航点输出
 * @return bool `true` 表示获取成功
 */
bool nav_route_get(uint8_t index, NavPoint* out) {
    if(out == NULL || index >= NAV_ROUTE_POINT_COUNT)
        return false;

    memcpy(out, &nav_route_points[index], sizeof(*out));
    return true;
}

/**
 * @brief 获取导航路径点数量
 * @return 导航路径点数量
 */
uint8_t nav_route_count(void) {
    return NAV_ROUTE_POINT_COUNT;
}

/**
 * @brief 获取返回路径点
 * @param index 返回路径点索引
 * @param out 导航点输出
 * @return bool `true` 表示获取成功
 */
bool nav_return_route_get(uint8_t index, NavPoint* out) {
    if(out == NULL || index >= NAV_RETURN_ROUTE_POINT_COUNT)
        return false;

    memcpy(out, &nav_return_route_points[index], sizeof(*out));
    return true;
}

/**
 * @brief 获取返回路径点数量
 * @return 返回路径点数量
 */
uint8_t nav_return_route_count(void) {
    return NAV_RETURN_ROUTE_POINT_COUNT;
}
