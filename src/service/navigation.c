#include "navigation.h"

#include <stddef.h>

// ! ========================= 私 有 Typedef 声 明 ========================= ! //

typedef struct {
    NavigationRouteStop stop;
    MissionPhase phase;
    const FieldPose* pose;
} NavigationRouteEntry;

// ! ========================= 变 量 声 明 ========================= ! //

const FieldPose START_POSE = { .x = 0.45f, .y = 0.20f, .yaw = 0.0f };
const FieldPose HOME_POSE = { .x = 0.45f, .y = 0.20f, .yaw = 0.0f };

const FieldPose A_APPROACH_POSE[NAV_A_APPROACH_COUNT] = {
    { .x = 0.225f, .y = 1.225f, .yaw = 0.0f },
    { .x = 0.850f, .y = 1.225f, .yaw = 0.0f },
    { .x = 0.225f, .y = 1.775f, .yaw = 0.0f },
    { .x = 0.850f, .y = 1.775f, .yaw = 0.0f },
    { .x = 0.225f, .y = 2.325f, .yaw = 0.0f },
    { .x = 0.850f, .y = 2.325f, .yaw = 0.0f },
};

const FieldPose B_APPROACH_POSE[NAV_B_APPROACH_COUNT] = {
    { .x = 1.750f, .y = 2.325f, .yaw = 0.0f },
    { .x = 1.750f, .y = 1.775f, .yaw = 0.0f },
    { .x = 1.750f, .y = 1.225f, .yaw = 0.0f },
};

const FieldPose C_APPROACH_POSE[NAV_C_APPROACH_COUNT] = {
    { .x = 2.635f, .y = 2.325f, .yaw = 0.0f },
    { .x = 2.635f, .y = 1.775f, .yaw = 0.0f },
    { .x = 2.635f, .y = 1.225f, .yaw = 0.0f },
};

const FieldPose D_HANDOFF_POSE = { .x = 3.170f, .y = 1.500f, .yaw = 0.0f };
const FieldPose D_EXIT_POSE = { .x = 3.170f, .y = 0.400f, .yaw = 0.0f };

static const FieldBounds s_field_bounds = {
    .min_x = 0.0f,
    .max_x = NAV_FIELD_WIDTH_M,
    .min_y = 0.0f,
    .max_y = NAV_FIELD_HEIGHT_M,
};

static const NavigationRouteEntry s_default_route[NAV_ROUTE_STOP_COUNT] = {
    { .stop = NAV_ROUTE_STOP_START, .phase = MISSION_PHASE_START, .pose = &START_POSE },
    { .stop = NAV_ROUTE_STOP_A, .phase = MISSION_PHASE_A, .pose = &A_APPROACH_POSE[0] },
    { .stop = NAV_ROUTE_STOP_B, .phase = MISSION_PHASE_B, .pose = &B_APPROACH_POSE[0] },
    { .stop = NAV_ROUTE_STOP_C, .phase = MISSION_PHASE_C, .pose = &C_APPROACH_POSE[0] },
    { .stop = NAV_ROUTE_STOP_D_HANDOFF, .phase = MISSION_PHASE_D_HANDOFF, .pose = &D_HANDOFF_POSE },
    { .stop = NAV_ROUTE_STOP_HOME, .phase = MISSION_PHASE_HOME, .pose = &HOME_POSE },
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

const FieldBounds* navigation_get_field_bounds(void) {
    return &s_field_bounds;
}

bool navigation_is_pose_in_field(const FieldPose* pose) {
    if(pose == NULL) {
        return false;
    }

    return pose->x >= s_field_bounds.min_x && pose->x <= s_field_bounds.max_x && pose->y >= s_field_bounds.min_y && pose->y <= s_field_bounds.max_y;
}

uint8_t navigation_get_default_route_count(void) {
    return (uint8_t)NAV_ROUTE_STOP_COUNT;
}

NavigationRouteStop navigation_get_default_route_stop(uint8_t index) {
    if(index >= (uint8_t)NAV_ROUTE_STOP_COUNT) {
        return NAV_ROUTE_STOP_COUNT;
    }

    return s_default_route[index].stop;
}

MissionPhase navigation_get_route_stop_phase(NavigationRouteStop stop) {
    if(stop >= NAV_ROUTE_STOP_COUNT) {
        return MISSION_PHASE_IDLE;
    }

    return s_default_route[stop].phase;
}

const FieldPose* navigation_get_route_stop_pose(NavigationRouteStop stop) {
    if(stop >= NAV_ROUTE_STOP_COUNT) {
        return NULL;
    }

    return s_default_route[stop].pose;
}

uint8_t navigation_get_zone_approach_count(MissionZoneId zone) {
    switch(zone) {
        case MISSION_ZONE_A:
            return NAV_A_APPROACH_COUNT;
        case MISSION_ZONE_B:
            return NAV_B_APPROACH_COUNT;
        case MISSION_ZONE_C:
            return NAV_C_APPROACH_COUNT;
        default:
            return 0u;
    }
}

const FieldPose* navigation_get_zone_approach_pose(MissionZoneId zone, uint8_t index) {
    switch(zone) {
        case MISSION_ZONE_A:
            return index < NAV_A_APPROACH_COUNT ? &A_APPROACH_POSE[index] : NULL;
        case MISSION_ZONE_B:
            return index < NAV_B_APPROACH_COUNT ? &B_APPROACH_POSE[index] : NULL;
        case MISSION_ZONE_C:
            return index < NAV_C_APPROACH_COUNT ? &C_APPROACH_POSE[index] : NULL;
        default:
            return NULL;
    }
}

const FieldPose* navigation_get_d_target_pose(NavigationDTarget target) {
    switch(target) {
        case NAV_D_TARGET_HANDOFF:
            return &D_HANDOFF_POSE;
        case NAV_D_TARGET_EXIT:
            return &D_EXIT_POSE;
        default:
            return NULL;
    }
}
