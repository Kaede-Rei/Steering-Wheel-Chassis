#include "navigation_map.h"

// ! ========================= 变 量 声 明 ========================= ! //

#define NAV_MAP_POINT_MAX 21u
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
    nav_map[0].pre_detect_joints.exist = false;

    nav_map[1].x = 0.0f;
    nav_map[1].y = -0.04f;
    nav_map[1].area_type = PASS_BY;
    nav_map[1].pre_detect_joints.exist = false;

    nav_map[2].x = 0.64f;
    nav_map[2].y = -0.05f;
    nav_map[2].area_type = AREA_A;
    nav_map[2].y_flowers.up = true;
    nav_map[2].y_flowers.mid = true;
    nav_map[2].y_flowers.down = false;
    nav_map[2].pre_detect_joints.exist = true;
    nav_map[2].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[2].pre_detect_joints.joints.q[0] = 1.606f;
    nav_map[2].pre_detect_joints.joints.q[1] = 2.315f;
    nav_map[2].pre_detect_joints.joints.q[2] = 5.875f;
    nav_map[2].pre_detect_joints.joints.q[3] = 2.152f;
    nav_map[2].pre_detect_joints.joints.q[4] = 3.141f;
    nav_map[3].x = 0.64f;
    nav_map[3].y = -0.05f;
    nav_map[3].area_type = AREA_A;
    nav_map[3].y_flowers.up = false;
    nav_map[3].y_flowers.mid = false;
    nav_map[3].y_flowers.down = true;
    nav_map[3].pre_detect_joints.exist = true;
    nav_map[3].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[3].pre_detect_joints.joints.q[0] = 4.714f;
    nav_map[3].pre_detect_joints.joints.q[1] = 2.315f;
    nav_map[3].pre_detect_joints.joints.q[2] = 5.875f;
    nav_map[3].pre_detect_joints.joints.q[3] = 2.152f;
    nav_map[3].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[4].x = 1.15f;
    nav_map[4].y = -0.05f;
    nav_map[4].area_type = AREA_A;
    nav_map[4].y_flowers.up = true;
    nav_map[4].y_flowers.mid = true;
    nav_map[4].y_flowers.down = true;
    nav_map[4].pre_detect_joints.exist = true;
    nav_map[4].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[4].pre_detect_joints.joints.q[0] = 1.606f;
    nav_map[4].pre_detect_joints.joints.q[1] = 2.315f;
    nav_map[4].pre_detect_joints.joints.q[2] = 5.875f;
    nav_map[4].pre_detect_joints.joints.q[3] = 2.152f;
    nav_map[4].pre_detect_joints.joints.q[4] = 3.141f;
    nav_map[5].x = 1.15f;
    nav_map[5].y = -0.05f;
    nav_map[5].area_type = AREA_A;
    nav_map[5].y_flowers.up = true;
    nav_map[5].y_flowers.mid = true;
    nav_map[5].y_flowers.down = false;
    nav_map[5].pre_detect_joints.exist = true;
    nav_map[5].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[5].pre_detect_joints.joints.q[0] = 4.714f;
    nav_map[5].pre_detect_joints.joints.q[1] = 2.315f;
    nav_map[5].pre_detect_joints.joints.q[2] = 5.875f;
    nav_map[5].pre_detect_joints.joints.q[3] = 2.152f;
    nav_map[5].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[6].x = 1.65f;
    nav_map[6].y = -0.05f;
    nav_map[6].area_type = AREA_A;
    nav_map[6].y_flowers.up = true;
    nav_map[6].y_flowers.mid = true;
    nav_map[6].y_flowers.down = false;
    nav_map[6].pre_detect_joints.exist = true;
    nav_map[6].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[6].pre_detect_joints.joints.q[0] = 1.606f;
    nav_map[6].pre_detect_joints.joints.q[1] = 2.315f;
    nav_map[6].pre_detect_joints.joints.q[2] = 5.875f;
    nav_map[6].pre_detect_joints.joints.q[3] = 2.152f;
    nav_map[6].pre_detect_joints.joints.q[4] = 3.141f;
    nav_map[7].x = 1.65f;
    nav_map[7].y = -0.05f;
    nav_map[7].area_type = AREA_A;
    nav_map[7].y_flowers.up = false;
    nav_map[7].y_flowers.mid = false;
    nav_map[7].y_flowers.down = false;
    nav_map[7].pre_detect_joints.exist = true;
    nav_map[7].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[7].pre_detect_joints.joints.q[0] = 4.714f;
    nav_map[7].pre_detect_joints.joints.q[1] = 2.315f;
    nav_map[7].pre_detect_joints.joints.q[2] = 5.875f;
    nav_map[7].pre_detect_joints.joints.q[3] = 2.152f;
    nav_map[7].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[8].x = 2.07f;
    nav_map[8].y = -0.05f;
    nav_map[8].area_type = PASS_BY;
    nav_map[8].pre_detect_joints.exist = false;

    nav_map[9].x = 2.07f;
    nav_map[9].y = -0.79f;
    nav_map[9].area_type = PASS_BY;
    nav_map[9].pre_detect_joints.exist = false;

    nav_map[10].x = 1.64f;
    nav_map[10].y = -0.79f;
    nav_map[10].area_type = AREA_B;
    nav_map[10].x_flowers.left = true;
    nav_map[10].x_flowers.mid = true;
    nav_map[10].x_flowers.right = false;
    nav_map[10].pre_detect_joints.exist = true;
    nav_map[10].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[10].pre_detect_joints.joints.q[0] = 0.206f;
    nav_map[10].pre_detect_joints.joints.q[1] = 1.591f;
    nav_map[10].pre_detect_joints.joints.q[2] = 5.380f;
    nav_map[10].pre_detect_joints.joints.q[3] = 2.172f;
    nav_map[10].pre_detect_joints.joints.q[4] = 3.141f;
    nav_map[11].x = 1.64f;
    nav_map[11].y = -0.79f;
    nav_map[11].area_type = AREA_B;
    nav_map[11].x_flowers.left = false;
    nav_map[11].x_flowers.mid = true;
    nav_map[11].x_flowers.right = false;
    nav_map[11].pre_detect_joints.exist = true;
    nav_map[11].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[11].pre_detect_joints.joints.q[0] = 3.141f;
    nav_map[11].pre_detect_joints.joints.q[1] = 1.591f;
    nav_map[11].pre_detect_joints.joints.q[2] = 5.380f;
    nav_map[11].pre_detect_joints.joints.q[3] = 2.172f;
    nav_map[11].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[12].x = 1.15f;
    nav_map[12].y = -0.79f;
    nav_map[12].area_type = AREA_B;
    nav_map[12].x_flowers.left = true;
    nav_map[12].x_flowers.mid = true;
    nav_map[12].x_flowers.right = false;
    nav_map[12].pre_detect_joints.exist = true;
    nav_map[12].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[12].pre_detect_joints.joints.q[0] = 0.206f;
    nav_map[12].pre_detect_joints.joints.q[1] = 1.591f;
    nav_map[12].pre_detect_joints.joints.q[2] = 5.380f;
    nav_map[12].pre_detect_joints.joints.q[3] = 2.172f;
    nav_map[12].pre_detect_joints.joints.q[4] = 3.141f;
    nav_map[13].x = 1.15f;
    nav_map[13].y = -0.79f;
    nav_map[13].area_type = AREA_B;
    nav_map[13].x_flowers.left = false;
    nav_map[13].x_flowers.mid = true;
    nav_map[13].x_flowers.right = false;
    nav_map[13].pre_detect_joints.exist = true;
    nav_map[13].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[13].pre_detect_joints.joints.q[0] = 3.141f;
    nav_map[13].pre_detect_joints.joints.q[1] = 1.591f;
    nav_map[13].pre_detect_joints.joints.q[2] = 5.380f;
    nav_map[13].pre_detect_joints.joints.q[3] = 2.172f;
    nav_map[13].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[14].x = 0.65f;
    nav_map[14].y = -0.79f;
    nav_map[14].area_type = AREA_B;
    nav_map[14].x_flowers.left = true;
    nav_map[14].x_flowers.mid = true;
    nav_map[14].x_flowers.right = false;
    nav_map[14].pre_detect_joints.exist = true;
    nav_map[14].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[14].pre_detect_joints.joints.q[0] = 0.206f;
    nav_map[14].pre_detect_joints.joints.q[1] = 1.591f;
    nav_map[14].pre_detect_joints.joints.q[2] = 5.380f;
    nav_map[14].pre_detect_joints.joints.q[3] = 2.172f;
    nav_map[14].pre_detect_joints.joints.q[4] = 3.141f;
    nav_map[15].x = 0.65f;
    nav_map[15].y = -0.79f;
    nav_map[15].area_type = AREA_B;
    nav_map[15].x_flowers.left = false;
    nav_map[15].x_flowers.mid = true;
    nav_map[15].x_flowers.right = false;
    nav_map[15].pre_detect_joints.exist = true;
    nav_map[15].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[15].pre_detect_joints.joints.q[0] = 3.141f;
    nav_map[15].pre_detect_joints.joints.q[1] = 1.591f;
    nav_map[15].pre_detect_joints.joints.q[2] = 5.380f;
    nav_map[15].pre_detect_joints.joints.q[3] = 2.172f;
    nav_map[15].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[16].x = 0.35f;
    nav_map[16].y = -0.79f;
    nav_map[16].area_type = PASS_BY;
    nav_map[16].pre_detect_joints.exist = false;

    nav_map[17].x = 0.35f;
    nav_map[17].y = -1.55f;
    nav_map[17].area_type = PASS_BY;
    nav_map[17].pre_detect_joints.exist = false;

    nav_map[18].x = 0.78f;
    nav_map[18].y = -1.55f;
    nav_map[18].area_type = AREA_C;
    nav_map[18].x_flowers.left = true;
    nav_map[18].x_flowers.mid = false;
    nav_map[18].x_flowers.right = true;
    nav_map[18].pre_detect_joints.exist = true;
    nav_map[18].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[18].pre_detect_joints.joints.q[0] = 1.536f;
    nav_map[18].pre_detect_joints.joints.q[1] = 2.511f;
    nav_map[18].pre_detect_joints.joints.q[2] = 5.660f;
    nav_map[18].pre_detect_joints.joints.q[3] = 2.424f;
    nav_map[18].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[19].x = 1.16f;
    nav_map[19].y = -1.55f;
    nav_map[19].area_type = AREA_C;
    nav_map[19].x_flowers.left = true;
    nav_map[19].x_flowers.mid = false;
    nav_map[19].x_flowers.right = false;
    nav_map[19].pre_detect_joints.exist = true;
    nav_map[19].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[19].pre_detect_joints.joints.q[0] = 1.536f;
    nav_map[19].pre_detect_joints.joints.q[1] = 2.511f;
    nav_map[19].pre_detect_joints.joints.q[2] = 5.660f;
    nav_map[19].pre_detect_joints.joints.q[3] = 2.424f;
    nav_map[19].pre_detect_joints.joints.q[4] = 3.141f;

    nav_map[20].x = 1.58f;
    nav_map[20].y = -1.55f;
    nav_map[20].area_type = AREA_C;
    nav_map[20].x_flowers.left = false;
    nav_map[20].x_flowers.mid = false;
    nav_map[20].x_flowers.right = false;
    nav_map[20].pre_detect_joints.exist = true;
    nav_map[20].pre_detect_joints.joints.dof = ARM_DOF;
    nav_map[20].pre_detect_joints.joints.q[0] = 1.536f;
    nav_map[20].pre_detect_joints.joints.q[1] = 2.511f;
    nav_map[20].pre_detect_joints.joints.q[2] = 5.660f;
    nav_map[20].pre_detect_joints.joints.q[3] = 2.424f;
    nav_map[20].pre_detect_joints.joints.q[4] = 3.141f;

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
    back_home_points[1].y -= 0.01f;

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
