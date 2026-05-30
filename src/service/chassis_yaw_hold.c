#include "chassis_yaw_hold.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define CHASSIS_YAW_HOLD_PI     3.14159265358979323846f
#define CHASSIS_YAW_HOLD_2PI    (2.0f * CHASSIS_YAW_HOLD_PI)

typedef struct {
    ChassisYawHoldConfig config;
    bool initialized;
    bool active;
    float yaw_ref;
    float last_yaw_error;
    float last_ff_wz;
    float last_fb_wz;
    float last_output_wz;
} ChassisYawHoldState;

static ChassisYawHoldState s_yaw_hold = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static float chassis_yaw_hold_wrap_pi(float angle);
static float chassis_yaw_hold_clamp(float value, float min_value, float max_value);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ChassisYawHoldConfig chassis_yaw_hold_default_config(void) {
    ChassisYawHoldConfig config;

    config.enabled = true;
    config.kp = 3.0f;
    config.kd = 0.08f;
    config.k_vx = 0.0f;
    config.k_vy = 0.0f;
    config.v_deadband = 0.02f;
    config.wz_deadband = 0.03f;
    config.wz_limit = 0.80f;
    return config;
}

void chassis_yaw_hold_init(const ChassisYawHoldConfig* config) {
    ChassisYawHoldConfig default_config = chassis_yaw_hold_default_config();

    memset(&s_yaw_hold, 0, sizeof(s_yaw_hold));
    s_yaw_hold.config = (config != 0) ? *config : default_config;
    s_yaw_hold.initialized = true;
}

void chassis_yaw_hold_reset(void) {
    s_yaw_hold.active = false;
    s_yaw_hold.yaw_ref = 0.0f;
    s_yaw_hold.last_yaw_error = 0.0f;
    s_yaw_hold.last_ff_wz = 0.0f;
    s_yaw_hold.last_fb_wz = 0.0f;
    s_yaw_hold.last_output_wz = 0.0f;
}

float chassis_yaw_hold_apply(float vx_cmd, float vy_cmd, float wz_cmd, float yaw, float gyro_z_corrected) {
    const ChassisYawHoldConfig* config = &s_yaw_hold.config;
    bool user_rotating;
    bool user_translating;
    float yaw_error;
    float ff_wz;
    float fb_wz;
    float output_wz;

    if(!s_yaw_hold.initialized) {
        chassis_yaw_hold_init(0);
    }

    if(!config->enabled || !isfinite(yaw) || !isfinite(gyro_z_corrected)) {
        chassis_yaw_hold_reset();
        return wz_cmd;
    }

    user_rotating = fabsf(wz_cmd) > config->wz_deadband;
    user_translating = fabsf(vx_cmd) > config->v_deadband || fabsf(vy_cmd) > config->v_deadband;

    if(user_rotating) {
        chassis_yaw_hold_reset();
        return wz_cmd;
    }

    if(!user_translating) {
        s_yaw_hold.last_yaw_error = 0.0f;
        s_yaw_hold.last_ff_wz = 0.0f;
        s_yaw_hold.last_fb_wz = 0.0f;
        s_yaw_hold.last_output_wz = 0.0f;
        return 0.0f;
    }

    if(!s_yaw_hold.active) {
        s_yaw_hold.yaw_ref = 0.0f;
        s_yaw_hold.active = true;
    }

    yaw_error = chassis_yaw_hold_wrap_pi(s_yaw_hold.yaw_ref - yaw);

    /**
     * 机械零位导致的经验漂移前馈：
     *   vx > 0 时底盘倾向产生 +wz，因此 ff_wz = +k_vx * vx，控制输出中要减掉它；
     *   vy > 0 时底盘倾向产生 -wz，因此 ff_wz = -k_vy * vy，控制输出中也要减掉它
     */
    ff_wz = config->k_vx * vx_cmd - config->k_vy * vy_cmd;
    fb_wz = config->kp * yaw_error - config->kd * gyro_z_corrected;
    output_wz = wz_cmd - ff_wz + fb_wz;
    output_wz = chassis_yaw_hold_clamp(output_wz, -config->wz_limit, config->wz_limit);

    s_yaw_hold.last_yaw_error = yaw_error;
    s_yaw_hold.last_ff_wz = ff_wz;
    s_yaw_hold.last_fb_wz = fb_wz;
    s_yaw_hold.last_output_wz = output_wz;
    return output_wz;
}

bool chassis_yaw_hold_is_active(void) {
    return s_yaw_hold.active;
}

float chassis_yaw_hold_get_yaw_ref(void) {
    return s_yaw_hold.yaw_ref;
}

float chassis_yaw_hold_get_yaw_error(void) {
    return s_yaw_hold.last_yaw_error;
}

float chassis_yaw_hold_get_ff_wz(void) {
    return s_yaw_hold.last_ff_wz;
}

float chassis_yaw_hold_get_fb_wz(void) {
    return s_yaw_hold.last_fb_wz;
}

float chassis_yaw_hold_get_output_wz(void) {
    return s_yaw_hold.last_output_wz;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static float chassis_yaw_hold_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, CHASSIS_YAW_HOLD_2PI);
    if(angle >= CHASSIS_YAW_HOLD_PI) {
        angle -= CHASSIS_YAW_HOLD_2PI;
    }
    else if(angle < -CHASSIS_YAW_HOLD_PI) {
        angle += CHASSIS_YAW_HOLD_2PI;
    }
    return angle;
}

static float chassis_yaw_hold_clamp(float value, float min_value, float max_value) {
    if(value < min_value) {
        return min_value;
    }
    if(value > max_value) {
        return max_value;
    }
    return value;
}
