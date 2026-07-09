#include "pollen_route.h"

#include <string.h>

// ! ========================= Typedef 声 明 ========================= ! //

typedef struct {
    uint8_t nav_index;
    PollenActionSequence sequence;
} PollenRouteItem;

// ! ========================= 变 量 声 明 ========================= ! //

#define JOINTS(q0, q1, q2, q3, q4)   \
    {                                \
        .dof = ARM_DOF, .q = {(q0),  \
                              (q1),  \
                              (q2),  \
                              (q3),  \
                              (q4) } \
    }

static const PollenRouteItem pollen_route_items[] = {
    /** A 区 */
    { .nav_index = 2u,
      .sequence = { .step_count = 4u,
                    .steps[0] = JOINTS(1.485f, 3.106f, 5.410f, 1.930f, 3.141f),
                    .steps[1] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f),
                    .steps[2] = JOINTS(1.480f, 3.037f, 5.645f, 1.930f, 3.141f),
                    .steps[3] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f) } },
    { .nav_index = 3u,
      .sequence = { .step_count = 2u,
                    .steps[0] = JOINTS(4.608f, 2.810f, 6.114f, 1.927f, 3.141f),
                    .steps[1] = JOINTS(4.714f, 2.315f, 5.875f, 2.152f, 3.141f) } },

    { .nav_index = 4u,
      .sequence = { .step_count = 6u,
                    .steps[0] = JOINTS(1.459f, 3.195f, 5.381f, 1.928f, 3.141f),
                    .steps[1] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f),
                    .steps[2] = JOINTS(1.453f, 3.114f, 5.613f, 1.930f, 3.141f),
                    .steps[3] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f),
                    .steps[4] = JOINTS(1.411f, 3.109f, 5.800f, 1.930f, 3.141f),
                    .steps[5] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f) } },
    { .nav_index = 5u,
      .sequence = { .step_count = 4u,
                    .steps[0] = JOINTS(4.737f, 2.873f, 5.610f, 1.930f, 3.141f),
                    .steps[1] = JOINTS(4.714f, 2.315f, 5.875f, 2.152f, 3.141f),
                    .steps[2] = JOINTS(4.738f, 2.777f, 5.860f, 1.930f, 3.141f),
                    .steps[3] = JOINTS(4.714f, 2.315f, 5.875f, 2.152f, 3.141f) } },

    { .nav_index = 6u,
      .sequence = { .step_count = 4u,
                    .steps[0] = JOINTS(1.451f, 3.235f, 5.283f, 1.924f, 3.141f),
                    .steps[1] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f),
                    .steps[2] = JOINTS(1.445f, 3.148f, 5.539f, 1.928f, 3.141f),
                    .steps[3] = JOINTS(1.606f, 2.315f, 5.875f, 2.152f, 3.141f) } },
    { .nav_index = 7u, .sequence = { .step_count = 0u } },

    /** B 区 */
    { .nav_index = 10u,
      .sequence = { .step_count = 4u,
                    .steps[0] = JOINTS(0.904f, 1.910f, 4.705f, 2.888f, 2.229f),
                    .steps[1] = JOINTS(0.206f, 1.591f, 5.380f, 2.172f, 3.141f),
                    .steps[2] = JOINTS(0.043f, 1.994f, 5.127f, 2.338f, 3.625f),
                    .steps[3] = JOINTS(0.206f, 1.591f, 5.380f, 2.172f, 3.141f) } },
    { .nav_index = 11u,
      .sequence = { .step_count = 2u,
                    .steps[0] = JOINTS(3.198f, 2.043f, 5.008f, 2.526f, 2.471f),
                    .steps[1] = JOINTS(3.141f, 1.591f, 5.380f, 2.172f, 3.141f) } },

    { .nav_index = 12u,
      .sequence = { .step_count = 5u,
                    .steps[0] = JOINTS(0.445f, 2.005f, 4.854f, 2.603f, 2.164f),
                    .steps[1] = JOINTS(0.206f, 1.591f, 5.380f, 2.172f, 3.141f),
                    .steps[2] = JOINTS(5.944f, 1.591f, 5.380f, 2.172f, 3.141f),
                    .steps[3] = JOINTS(5.944f, 2.054f, 4.660f, 3.112f, 4.120f),
                    .steps[4] = JOINTS(5.944f, 1.591f, 5.380f, 2.172f, 3.141f) } },
    { .nav_index = 13u,
      .sequence = { .step_count = 2u,
                    .steps[0] = JOINTS(3.163f, 2.128f, 4.665f, 3.123f, 2.077f),
                    .steps[1] = JOINTS(3.141f, 1.591f, 5.380f, 2.172f, 3.141f) } },

    { .nav_index = 14u,
      .sequence = { .step_count = 5u,
                    .steps[0] = JOINTS(0.494f, 1.916f, 4.909f, 2.622f, 2.240f),
                    .steps[1] = JOINTS(0.206f, 1.591f, 5.380f, 2.172f, 3.141f),
                    .steps[2] = JOINTS(6.116f, 1.591f, 5.380f, 2.172f, 3.141f),
                    .steps[3] = JOINTS(6.116f, 2.066f, 4.837f, 2.628f, 4.326f),
                    .steps[4] = JOINTS(6.116f, 1.591f, 5.380f, 2.172f, 3.141f) } },
    { .nav_index = 15u,
      .sequence = { .step_count = 2u,
                    .steps[0] = JOINTS(3.853f, 1.930f, 4.877f, 2.622f, 2.187f),
                    .steps[1] = JOINTS(3.141f, 1.591f, 5.380f, 2.172f, 3.141f) } },

    /** C 区 */
    { .nav_index = 18u,
      .sequence = { .step_count = 4u,
                    .steps[0] = JOINTS(1.048f, 4.162f, 4.742f, 2.051f, 3.335f),
                    .steps[1] = JOINTS(1.536f, 2.511f, 5.660f, 2.424f, 3.141f),
                    .steps[2] = JOINTS(1.589f, 3.964f, 5.136f, 2.052f, 3.336f),
                    .steps[3] = JOINTS(1.536f, 2.511f, 5.660f, 2.424f, 3.141f) } },
    { .nav_index = 19u,
      .sequence = { .step_count = 2u,
                    .steps[0] = JOINTS(1.256f, 3.953f, 5.143f, 2.063f, 3.335f),
                    .steps[1] = JOINTS(1.536f, 2.511f, 5.660f, 2.424f, 3.141f) } },
    { .nav_index = 20u, .sequence = { .step_count = 0u } },
};

/**







 */

#undef JOINTS

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 获取指定导航点的授粉动作序列
 * @param nav_index 导航点索引
 * @param out 授粉动作序列输出
 * @return bool `true` 表示获取成功
 */
bool pollen_route_get(uint8_t nav_index, PollenActionSequence* out) {
    uint8_t i;

    if(out == NULL)
        return false;

    for(i = 0u; i < (uint8_t)(sizeof(pollen_route_items) / sizeof(pollen_route_items[0])); i++) {
        if(pollen_route_items[i].nav_index == nav_index) {
            memcpy(out, &pollen_route_items[i].sequence, sizeof(*out));
            return true;
        }
    }

    memset(out, 0, sizeof(*out));
    return false;
}
