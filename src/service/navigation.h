#ifndef _navigation_h_
#define _navigation_h_

/**
 * @file navigation.h
 * @brief 比赛固定场地导航契约
 *
 * `service/navigation.*` 只负责冻结场地模型、默认路线顺序、分区接近点和边界辅助查询。
 * 本层不实现 SLAM、动态全局规划、定位融合，也不实现 D 区 UAV 飞控；
 * D 区仅作为地面机器人可导航到的交接目标与退出目标存在。
 */

#include "mission.h"

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 场地二维位姿，单位为 m / rad
 * @param x 场地坐标系 x，单位 m
 * @param y 场地坐标系 y，单位 m
 * @param yaw 朝向角，单位 rad
 */
typedef struct {
    float x;
    float y;
    float yaw;
} FieldPose;

/**
 * @brief 场地矩形边界，单位为 m
 */
typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
} FieldBounds;

/**
 * @brief 默认固定路线停靠点顺序
 *
 * frozen 默认顺序：`START -> A -> B -> C -> D_HANDOFF -> HOME`
 */
typedef enum {
    NAV_ROUTE_STOP_START = 0,
    NAV_ROUTE_STOP_A,
    NAV_ROUTE_STOP_B,
    NAV_ROUTE_STOP_C,
    NAV_ROUTE_STOP_D_HANDOFF,
    NAV_ROUTE_STOP_HOME,
    NAV_ROUTE_STOP_COUNT,
} NavigationRouteStop;

/**
 * @brief D 区导航目标类型
 *
 * 仅表达地面机器人在 D 区的导航目标，不表达任何 UAV 控制实现。
 */
typedef enum {
    NAV_D_TARGET_HANDOFF = 0,
    NAV_D_TARGET_EXIT,
} NavigationDTarget;

#define NAV_FIELD_WIDTH_M  3.79f
#define NAV_FIELD_HEIGHT_M 3.00f

#define NAV_A_APPROACH_COUNT 6u
#define NAV_B_APPROACH_COUNT 3u
#define NAV_C_APPROACH_COUNT 3u

extern const FieldPose START_POSE;
extern const FieldPose HOME_POSE;
extern const FieldPose A_APPROACH_POSE[NAV_A_APPROACH_COUNT];
extern const FieldPose B_APPROACH_POSE[NAV_B_APPROACH_COUNT];
extern const FieldPose C_APPROACH_POSE[NAV_C_APPROACH_COUNT];
extern const FieldPose D_HANDOFF_POSE;
extern const FieldPose D_EXIT_POSE;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 获取固定场地边界
 * @return const FieldBounds* 只读边界视图
 */
const FieldBounds* navigation_get_field_bounds(void);

/**
 * @brief 判断给定位姿是否落在场地矩形边界内
 * @param pose 待检查位姿
 * @return true 位姿在边界内
 * @return false 参数无效或位姿越界
 */
bool navigation_is_pose_in_field(const FieldPose* pose);

/**
 * @brief 获取 frozen 默认路线停靠点数量
 * @return uint8_t 固定为 `NAV_ROUTE_STOP_COUNT`
 */
uint8_t navigation_get_default_route_count(void);

/**
 * @brief 按索引获取默认路线停靠点
 * @param index 默认路线索引
 * @return NavigationRouteStop 路线停靠点；越界返回 `NAV_ROUTE_STOP_COUNT`
 */
NavigationRouteStop navigation_get_default_route_stop(uint8_t index);

/**
 * @brief 获取路线停靠点对应的任务阶段
 * @param stop 路线停靠点
 * @return MissionPhase 对应任务阶段；未知返回 `MISSION_PHASE_IDLE`
 */
MissionPhase navigation_get_route_stop_phase(NavigationRouteStop stop);

/**
 * @brief 获取路线停靠点对应的主要目标位姿
 * @param stop 路线停靠点
 * @return const FieldPose* 只读位姿；未知返回 `NULL`
 */
const FieldPose* navigation_get_route_stop_pose(NavigationRouteStop stop);

/**
 * @brief 获取指定分区的接近点数量
 * @param zone 分区 ID
 * @return uint8_t A/B/C 返回固定数量；其他分区返回 0
 */
uint8_t navigation_get_zone_approach_count(MissionZoneId zone);

/**
 * @brief 获取指定分区接近点位姿
 * @param zone 分区 ID
 * @param index 接近点索引
 * @return const FieldPose* 只读位姿；参数无效时返回 `NULL`
 */
const FieldPose* navigation_get_zone_approach_pose(MissionZoneId zone, uint8_t index);

/**
 * @brief 获取 D 区地面机器人导航目标位姿
 * @param target D 区目标类型
 * @return const FieldPose* 只读位姿；参数无效时返回 `NULL`
 */
const FieldPose* navigation_get_d_target_pose(NavigationDTarget target);

#endif
