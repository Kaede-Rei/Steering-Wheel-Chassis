#ifndef _navigation_route_h_
#define _navigation_route_h_

#include <stdbool.h>
#include <stdint.h>

#include "arm.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 当前区域类型枚举
 * @param START_END 起点/终点
 * @param PASS_BY 过渡点
 * @param AREA_A 区域 A
 * @param AREA_B 区域 B
 * @param AREA_C 区域 C
 */
typedef enum {
    START_END,
    PASS_BY,
    AREA_A,
    AREA_B,
    AREA_C
} AreaType;

/**
 * @brief 从左到右花的类型
 * @param left 左
 * @param mid 中
 * @param right 右
 */
typedef struct {
    bool left;
    bool mid;
    bool right;
} XFlowerType;

/**
 * @brief 从上到下花的类型
 * @param up 上
 * @param mid 中
 * @param down 下
 */
typedef struct {
    bool up;
    bool mid;
    bool down;
} YFlowerType;

/**
 * @brief 预识别机械臂关节角
 * @param exist 是否存在预识别动作
 * @param joints 预识别动作关节角
 */
typedef struct {
    bool exist;
    FiveDofArmJointArray joints;
} PreDetectJoints;

/**
 * @brief 导航点结构体
 * @param x 导航点 x 坐标
 * @param y 导航点 y 坐标
 * @param area_type 导航点所在区域类型
 * @param x_flowers 横向花型
 * @param y_flowers 纵向花型
 * @param pre_detect_joints 预识别机械臂关节角
 */
typedef struct {
    float x;
    float y;
    AreaType area_type;
    XFlowerType x_flowers;
    YFlowerType y_flowers;
    PreDetectJoints pre_detect_joints;
} NavPoint;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

bool nav_route_get(uint8_t index, NavPoint* out);
uint8_t nav_route_count(void);
bool nav_return_route_get(uint8_t index, NavPoint* out);
uint8_t nav_return_route_count(void);

#endif
