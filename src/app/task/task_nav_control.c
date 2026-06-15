#include "task_nav_control.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "delay.h"

#include <math.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define REACH_TIME_S 1.0f
#define TASK_CONTROL_PERIOD_S 0.002f
#define TASK_NAV_ACCEL_RATIO 0.05f
#define TASK_NAV_TRACK_KP 4.0f
#define TASK_NAV_TRACK_SPEED_MARGIN_M_S 0.04f

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void navigation_s_curve_profile(float progress, float* position_ratio, float* speed_ratio);
static void navigation_clamp_velocity(float* vx, float* vy, float max_speed);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 按 S 曲线轨迹跟踪当前导航点
 * @param nav 导航上下文
 * @param od 当前里程计位置
 */
void task_nav_control_follow_s_curve(const TaskNavigationContext* nav, const Vector3* od) {
    Vector3 angle = { 0 };
    Vector3 gyro_corrected = { 0 };
    float elapsed_s;
    float progress;
    float profile_pos;
    float profile_speed;
    float delta_x;
    float delta_y;
    float expect_x;
    float expect_y;
    float vx;
    float vy;
    float segment_len;
    float cruise_speed;
    float speed_limit;

    if(nav == NULL || od == NULL)
        return;

    elapsed_s = (float)(delay_now_ms() - nav->segment_start_ms) / 1000.0f;
    progress = elapsed_s / REACH_TIME_S;
    delta_x = nav->target_point.x - nav->start_point.x;
    delta_y = nav->target_point.y - nav->start_point.y;

    if(progress > 1.0f)
        progress = 1.0f;
    else if(progress < 0.0f)
        progress = 0.0f;

    navigation_s_curve_profile(progress, &profile_pos, &profile_speed);
    expect_x = nav->start_point.x + delta_x * profile_pos;
    expect_y = nav->start_point.y + delta_y * profile_pos;
    vx = delta_x * profile_speed / REACH_TIME_S + (expect_x - od->x) * TASK_NAV_TRACK_KP;
    vy = delta_y * profile_speed / REACH_TIME_S + (expect_y - od->y) * TASK_NAV_TRACK_KP;
    segment_len = sqrtf(delta_x * delta_x + delta_y * delta_y);
    cruise_speed = segment_len / (REACH_TIME_S * (1.0f - TASK_NAV_ACCEL_RATIO));
    speed_limit = cruise_speed + TASK_NAV_TRACK_SPEED_MARGIN_M_S;
    navigation_clamp_velocity(&vx, &vy, speed_limit);

    (void)odom.get_angle(&angle);
    (void)odom.get_gyro_corrected(&gyro_corrected);
    (void)chassis.set_velocity(
        vx,
        vy,
        chassis_yaw_hold_apply(vx, vy, 0.0f, angle.z, gyro_corrected.z, TASK_CONTROL_PERIOD_S));
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 根据进度生成 S 曲线位置与速度比例
 * @param progress 当前归一化进度
 * @param position_ratio 输出位置比例
 * @param speed_ratio 输出速度比例
 */
static void navigation_s_curve_profile(float progress, float* position_ratio, float* speed_ratio) {
    const float accel_ratio = TASK_NAV_ACCEL_RATIO;
    const float cruise_speed_ratio = 1.0f / (1.0f - accel_ratio);
    float p = progress;

    if(position_ratio == NULL || speed_ratio == NULL)
        return;

    if(p <= 0.0f) {
        *position_ratio = 0.0f;
        *speed_ratio = 0.0f;
        return;
    }
    if(p >= 1.0f) {
        *position_ratio = 1.0f;
        *speed_ratio = 0.0f;
        return;
    }

    if(p < accel_ratio) {
        float u = p / accel_ratio;
        *position_ratio = cruise_speed_ratio * accel_ratio * (u * u * u - 0.5f * u * u * u * u);
        *speed_ratio = cruise_speed_ratio * u * u * (3.0f - 2.0f * u);
        return;
    }

    if(p <= 1.0f - accel_ratio) {
        *position_ratio = cruise_speed_ratio * (p - accel_ratio * 0.5f);
        *speed_ratio = cruise_speed_ratio;
        return;
    }

    p = (1.0f - p) / accel_ratio;
    *position_ratio = 1.0f - cruise_speed_ratio * accel_ratio * (p * p * p - 0.5f * p * p * p * p);
    *speed_ratio = cruise_speed_ratio * p * p * (3.0f - 2.0f * p);
}

/**
 * @brief 对平面速度进行幅值限幅
 * @param vx x 方向速度指针
 * @param vy y 方向速度指针
 * @param max_speed 最大允许速度
 */
static void navigation_clamp_velocity(float* vx, float* vy, float max_speed) {
    float speed;
    float scale;

    if(vx == NULL || vy == NULL || max_speed <= 0.0f)
        return;

    speed = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if(speed <= max_speed || speed <= 0.0f)
        return;

    scale = max_speed / speed;
    *vx *= scale;
    *vy *= scale;
}
