#include "imu_attitude.h"

#include <math.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define IMU_ATTITUDE_PI        3.14159265358979323846f
#define IMU_ATTITUDE_2PI       (2.0f * IMU_ATTITUDE_PI)
#define IMU_ATTITUDE_EPS       1.0e-6f

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void imu_attitude_apply_default_config(ImuAttitudeConfig* config);
static void imu_attitude_reset_calib(ImuAttitude* attitude);
static ImuAttitudeStatus imu_attitude_calibrate(ImuAttitude* attitude, const ImuSample* sample);

static float imu_attitude_acc_norm(const ImuAcc* acc);
static uint8_t imu_attitude_acc_reliable(const ImuAttitude* attitude, const ImuSample* sample);

static float imu_attitude_wrap_pi(float angle);
static float imu_attitude_clampf(float value, float min_value, float max_value);

static void imu_attitude_quat_normalize(ImuQuat* quat);
static void imu_attitude_quat_from_euler(float roll, float pitch, float yaw, ImuQuat* quat);
static void imu_attitude_quat_to_euler(const ImuQuat* quat, ImuAngle* angle);
static void imu_attitude_quat_integrate_gyro(ImuQuat* quat, const ImuGyro* gyro, float dt);

static void imu_attitude_update_complementary(ImuAttitude* attitude, const ImuSample* sample, const ImuGyro* gyro, float dt);
static void imu_attitude_update_mahony(ImuAttitude* attitude, const ImuSample* sample, const ImuGyro* gyro, float dt);
static uint8_t imu_attitude_is_static(const ImuAttitude* attitude, const ImuSample* sample, const ImuGyro* gyro_corrected);
static void imu_attitude_update_static_bias(ImuAttitude* attitude, const ImuSample* sample, ImuGyro* gyro_corrected);
static void imu_attitude_force_zero_if_static(const ImuAttitude* attitude, ImuGyro* gyro_corrected);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ImuAttitudeStatus imu_attitude_init(ImuAttitude* attitude, const ImuAttitudeConfig* config) {
    if(attitude == 0 || config == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    memset(attitude, 0, sizeof(*attitude));

    attitude->config = *config;
    imu_attitude_apply_default_config(&attitude->config);

    attitude->quat.w = 1.0f;
    attitude->quat.x = 0.0f;
    attitude->quat.y = 0.0f;
    attitude->quat.z = 0.0f;

    attitude->angle.roll = 0.0f;
    attitude->angle.pitch = 0.0f;
    attitude->angle.yaw = 0.0f;

    imu_attitude_reset_calib(attitude);

    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_update(ImuAttitude* attitude, const ImuSample* sample) {
    ImuGyro gyro_corrected;
    uint32_t dt_us;
    float dt;

    if(attitude == 0 || sample == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if((sample->flags & IMU_SAMPLE_GYRO_VALID) == 0U) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    if(attitude->calibrated == 0U) {
        return imu_attitude_calibrate(attitude, sample);
    }

    if(attitude->last_update_us == 0U) {
        attitude->last_update_us = sample->gyro_timestamp_us;
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    dt_us = sample->gyro_timestamp_us - attitude->last_update_us;
    attitude->last_update_us = sample->gyro_timestamp_us;

    if(dt_us == 0U || dt_us > 100000U) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    dt = (float)dt_us * 1.0e-6f;

    gyro_corrected.x = sample->gyro.x - attitude->gyro_bias.x;
    gyro_corrected.y = sample->gyro.y - attitude->gyro_bias.y;
    gyro_corrected.z = sample->gyro.z - attitude->gyro_bias.z;


    imu_attitude_update_static_bias(attitude, sample, &gyro_corrected);
    imu_attitude_force_zero_if_static(attitude, &gyro_corrected);

    attitude->gyro_filtered = gyro_corrected;

    if(attitude->config.mode == IMU_ATTITUDE_COMPLEMENTARY) {
        imu_attitude_update_complementary(attitude, sample, &gyro_corrected, dt);
    }
    else if(attitude->config.mode == IMU_ATTITUDE_MAHONY_6AXIS) {
        imu_attitude_update_mahony(attitude, sample, &gyro_corrected, dt);
    }
    else {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    attitude->has_angle = 1U;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_get_angle(const ImuAttitude* attitude, ImuAngle* angle) {
    if(attitude == 0 || angle == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(attitude->has_angle == 0U) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    *angle = attitude->angle;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_get_quat(const ImuAttitude* attitude, ImuQuat* quat) {
    if(attitude == 0 || quat == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(attitude->has_angle == 0U) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    *quat = attitude->quat;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_get_gyro(const ImuAttitude* attitude, ImuGyro* gyro) {
    if(attitude == 0 || gyro == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if(attitude->calibrated == 0U) {
        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    *gyro = attitude->gyro_filtered;
    return IMU_ATTITUDE_STATUS_OK;
}

ImuAttitudeStatus imu_attitude_reset_yaw(ImuAttitude* attitude, float yaw) {
    if(attitude == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    yaw = imu_attitude_wrap_pi(yaw);

    imu_attitude_quat_from_euler(
        attitude->angle.roll,
        attitude->angle.pitch,
        yaw,
        &attitude->quat
    );

    imu_attitude_quat_to_euler(&attitude->quat, &attitude->angle);
    attitude->has_angle = 1U;

    return IMU_ATTITUDE_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void imu_attitude_apply_default_config(ImuAttitudeConfig* config) {
    if(config == 0) {
        return;
    }

    if(config->mode == IMU_ATTITUDE_NONE) {
        config->mode = IMU_ATTITUDE_MAHONY_6AXIS;
    }

    if(config->gyro_calib_samples == 0U) {
        config->gyro_calib_samples = 1000U;
    }

    if(config->gyro_calib_var_threshold <= 0.0f ||
        !isfinite(config->gyro_calib_var_threshold)) {
        config->gyro_calib_var_threshold = 0.0001f;
    }

    if(config->acc_norm <= 0.0f || !isfinite(config->acc_norm)) {
        config->acc_norm = 9.80665f;
    }

    if(config->acc_norm_tolerance <= 0.0f ||
        !isfinite(config->acc_norm_tolerance)) {
        config->acc_norm_tolerance = 1.5f;
    }

    if(config->max_acc_age_us == 0U) {
        config->max_acc_age_us = 20000U;
    }

    if(config->complementary_tau <= 0.0f ||
        !isfinite(config->complementary_tau)) {
        config->complementary_tau = 0.5f;
    }

    if(config->mahony_kp <= 0.0f || !isfinite(config->mahony_kp)) {
        config->mahony_kp = 2.0f;
    }

    if(config->mahony_ki < 0.0f || !isfinite(config->mahony_ki)) {
        config->mahony_ki = 0.0f;
    }

    if(config->mahony_ki_z < 0.0f || !isfinite(config->mahony_ki_z)) {
        config->mahony_ki_z = 0.0f;
    }

    if(config->static_bias_samples == 0U) {
        config->static_bias_samples = 250U;
    }

    if(config->static_gyro_threshold <= 0.0f ||
        !isfinite(config->static_gyro_threshold)) {
        config->static_gyro_threshold = 0.03f;
    }

    if(config->static_acc_tolerance <= 0.0f ||
        !isfinite(config->static_acc_tolerance)) {
        config->static_acc_tolerance = 0.5f;
    }

    if(config->static_bias_alpha <= 0.0f ||
        config->static_bias_alpha > 1.0f ||
        !isfinite(config->static_bias_alpha)) {
        config->static_bias_alpha = 0.001f;
    }

    if(config->static_force_zero_threshold <= 0.0f ||
        !isfinite(config->static_force_zero_threshold)) {
        config->static_force_zero_threshold = 0.006f;
    }
}

static void imu_attitude_reset_calib(ImuAttitude* attitude) {
    if(attitude == 0) {
        return;
    }

    attitude->calib_sum.x = 0.0f;
    attitude->calib_sum.y = 0.0f;
    attitude->calib_sum.z = 0.0f;

    attitude->calib_sq_sum.x = 0.0f;
    attitude->calib_sq_sum.y = 0.0f;
    attitude->calib_sq_sum.z = 0.0f;

    attitude->calib_count = 0U;
}

static ImuAttitudeStatus imu_attitude_calibrate(ImuAttitude* attitude, const ImuSample* sample) {
    float inv_n;
    ImuGyro mean;
    ImuGyro var;

    if(attitude == 0 || sample == 0) {
        return IMU_ATTITUDE_STATUS_INVALID_PARAM;
    }

    if((sample->flags & IMU_SAMPLE_GYRO_VALID) == 0U) {
        return IMU_ATTITUDE_STATUS_NOT_READY;
    }

    attitude->calib_sum.x += sample->gyro.x;
    attitude->calib_sum.y += sample->gyro.y;
    attitude->calib_sum.z += sample->gyro.z;

    attitude->calib_sq_sum.x += sample->gyro.x * sample->gyro.x;
    attitude->calib_sq_sum.y += sample->gyro.y * sample->gyro.y;
    attitude->calib_sq_sum.z += sample->gyro.z * sample->gyro.z;

    attitude->calib_count++;

    if(attitude->calib_count < attitude->config.gyro_calib_samples) {
        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    inv_n = 1.0f / (float)attitude->calib_count;

    mean.x = attitude->calib_sum.x * inv_n;
    mean.y = attitude->calib_sum.y * inv_n;
    mean.z = attitude->calib_sum.z * inv_n;

    var.x = attitude->calib_sq_sum.x * inv_n - mean.x * mean.x;
    var.y = attitude->calib_sq_sum.y * inv_n - mean.y * mean.y;
    var.z = attitude->calib_sq_sum.z * inv_n - mean.z * mean.z;

    if(var.x > attitude->config.gyro_calib_var_threshold ||
        var.y > attitude->config.gyro_calib_var_threshold ||
        var.z > attitude->config.gyro_calib_var_threshold) {
        imu_attitude_reset_calib(attitude);

        return IMU_ATTITUDE_STATUS_CALIBRATING;
    }

    attitude->gyro_bias = mean;
    attitude->gyro_filtered.x = 0.0f;
    attitude->gyro_filtered.y = 0.0f;
    attitude->gyro_filtered.z = 0.0f;

    if((sample->flags & IMU_SAMPLE_ACC_VALID) != 0U) {
        float ax = sample->acc.x;
        float ay = sample->acc.y;
        float az = sample->acc.z;

        attitude->angle.roll = atan2f(ay, az);
        attitude->angle.pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
        attitude->angle.yaw = 0.0f;
    }
    else {
        attitude->angle.roll = 0.0f;
        attitude->angle.pitch = 0.0f;
        attitude->angle.yaw = 0.0f;
    }

    imu_attitude_quat_from_euler(attitude->angle.roll, attitude->angle.pitch, attitude->angle.yaw, &attitude->quat
    );

    attitude->last_update_us = sample->gyro_timestamp_us;
    attitude->static_count = 0U;
    attitude->static_detected = 0U;
    attitude->calibrated = 1U;
    attitude->has_angle = 1U;

    return IMU_ATTITUDE_STATUS_OK;
}

static float imu_attitude_acc_norm(const ImuAcc* acc) {
    if(acc == 0) {
        return 0.0f;
    }

    return sqrtf(acc->x * acc->x + acc->y * acc->y + acc->z * acc->z);
}

static uint8_t imu_attitude_acc_reliable(const ImuAttitude* attitude, const ImuSample* sample) {
    float norm;
    uint32_t acc_age_us;

    if(attitude == 0 || sample == 0) {
        return 0U;
    }

    if((sample->flags & IMU_SAMPLE_ACC_VALID) == 0U) {
        return 0U;
    }

    acc_age_us = sample->gyro_timestamp_us - sample->acc_timestamp_us;
    if(acc_age_us > attitude->config.max_acc_age_us) {
        return 0U;
    }

    norm = imu_attitude_acc_norm(&sample->acc);

    if(fabsf(norm - attitude->config.acc_norm) > attitude->config.acc_norm_tolerance) {
        return 0U;
    }

    return 1U;
}

static float imu_attitude_wrap_pi(float angle) {
    if(!isfinite(angle)) {
        return 0.0f;
    }

    angle = fmodf(angle, IMU_ATTITUDE_2PI);

    if(angle >= IMU_ATTITUDE_PI) {
        angle -= IMU_ATTITUDE_2PI;
    }
    else if(angle < -IMU_ATTITUDE_PI) {
        angle += IMU_ATTITUDE_2PI;
    }

    return angle;
}

static float imu_attitude_clampf(float value, float min_value, float max_value) {
    if(value < min_value) {
        return min_value;
    }

    if(value > max_value) {
        return max_value;
    }

    return value;
}

static void imu_attitude_quat_normalize(ImuQuat* quat) {
    float norm;

    if(quat == 0) {
        return;
    }

    norm = sqrtf(
        quat->w * quat->w +
        quat->x * quat->x +
        quat->y * quat->y +
        quat->z * quat->z
    );

    if(norm <= IMU_ATTITUDE_EPS) {
        quat->w = 1.0f;
        quat->x = 0.0f;
        quat->y = 0.0f;
        quat->z = 0.0f;
        return;
    }

    quat->w /= norm;
    quat->x /= norm;
    quat->y /= norm;
    quat->z /= norm;
}

static void imu_attitude_quat_from_euler(float roll, float pitch, float yaw, ImuQuat* quat) {
    float cr;
    float sr;
    float cp;
    float sp;
    float cy;
    float sy;

    if(quat == 0) {
        return;
    }

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);
    cy = cosf(yaw * 0.5f);
    sy = sinf(yaw * 0.5f);

    quat->w = cr * cp * cy + sr * sp * sy;
    quat->x = sr * cp * cy - cr * sp * sy;
    quat->y = cr * sp * cy + sr * cp * sy;
    quat->z = cr * cp * sy - sr * sp * cy;

    imu_attitude_quat_normalize(quat);
}

static void imu_attitude_quat_to_euler(const ImuQuat* quat, ImuAngle* angle) {
    float sinp;

    if(quat == 0 || angle == 0) {
        return;
    }

    angle->roll = atan2f(
        2.0f * (quat->w * quat->x + quat->y * quat->z),
        1.0f - 2.0f * (quat->x * quat->x + quat->y * quat->y)
    );

    sinp = 2.0f * (quat->w * quat->y - quat->z * quat->x);
    sinp = imu_attitude_clampf(sinp, -1.0f, 1.0f);

    angle->pitch = asinf(sinp);

    angle->yaw = atan2f(
        2.0f * (quat->w * quat->z + quat->x * quat->y),
        1.0f - 2.0f * (quat->y * quat->y + quat->z * quat->z)
    );

    angle->yaw = imu_attitude_wrap_pi(angle->yaw);
}

static void imu_attitude_quat_integrate_gyro(ImuQuat* quat, const ImuGyro* gyro, float dt) {
    float qw;
    float qx;
    float qy;
    float qz;
    float gx;
    float gy;
    float gz;

    if(quat == 0 || gyro == 0 || dt <= 0.0f) {
        return;
    }

    qw = quat->w;
    qx = quat->x;
    qy = quat->y;
    qz = quat->z;

    gx = gyro->x;
    gy = gyro->y;
    gz = gyro->z;

    quat->w += 0.5f * (-qx * gx - qy * gy - qz * gz) * dt;
    quat->x += 0.5f * (qw * gx + qy * gz - qz * gy) * dt;
    quat->y += 0.5f * (qw * gy - qx * gz + qz * gx) * dt;
    quat->z += 0.5f * (qw * gz + qx * gy - qy * gx) * dt;

    imu_attitude_quat_normalize(quat);
}

static void imu_attitude_update_complementary(ImuAttitude* attitude, const ImuSample* sample, const ImuGyro* gyro, float dt) {
    float roll_gyro;
    float pitch_gyro;
    float yaw_gyro;

    if(attitude == 0 || sample == 0 || gyro == 0 || dt <= 0.0f) {
        return;
    }

    roll_gyro = attitude->angle.roll + gyro->x * dt;
    pitch_gyro = attitude->angle.pitch + gyro->y * dt;
    yaw_gyro = attitude->angle.yaw + gyro->z * dt;

    attitude->angle.roll = roll_gyro;
    attitude->angle.pitch = pitch_gyro;
    attitude->angle.yaw = imu_attitude_wrap_pi(yaw_gyro);

    if(imu_attitude_acc_reliable(attitude, sample) != 0U) {
        float ax = sample->acc.x;
        float ay = sample->acc.y;
        float az = sample->acc.z;

        float roll_acc = atan2f(ay, az);
        float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

        float alpha = attitude->config.complementary_tau /
            (attitude->config.complementary_tau + dt);

        attitude->angle.roll =
            alpha * roll_gyro + (1.0f - alpha) * roll_acc;

        attitude->angle.pitch =
            alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;
    }

    attitude->angle.roll = imu_attitude_wrap_pi(attitude->angle.roll);
    attitude->angle.pitch = imu_attitude_wrap_pi(attitude->angle.pitch);
    attitude->angle.yaw = imu_attitude_wrap_pi(attitude->angle.yaw);

    imu_attitude_quat_from_euler(
        attitude->angle.roll,
        attitude->angle.pitch,
        attitude->angle.yaw,
        &attitude->quat
    );
}

static void imu_attitude_update_mahony(ImuAttitude* attitude, const ImuSample* sample, const ImuGyro* gyro, float dt) {
    ImuGyro corrected;
    float ax;
    float ay;
    float az;
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;

    if(attitude == 0 || sample == 0 || gyro == 0 || dt <= 0.0f) {
        return;
    }

    corrected = *gyro;

    if(imu_attitude_acc_reliable(attitude, sample) != 0U) {
        ax = sample->acc.x;
        ay = sample->acc.y;
        az = sample->acc.z;

        norm = sqrtf(ax * ax + ay * ay + az * az);

        if(norm > IMU_ATTITUDE_EPS) {
            ax /= norm;
            ay /= norm;
            az /= norm;

            vx = 2.0f * (attitude->quat.x * attitude->quat.z -
                attitude->quat.w * attitude->quat.y);

            vy = 2.0f * (attitude->quat.w * attitude->quat.x +
                attitude->quat.y * attitude->quat.z);

            vz = attitude->quat.w * attitude->quat.w -
                attitude->quat.x * attitude->quat.x -
                attitude->quat.y * attitude->quat.y +
                attitude->quat.z * attitude->quat.z;

            ex = ay * vz - az * vy;
            ey = az * vx - ax * vz;

            if(attitude->config.mahony_ki > 0.0f) {
                attitude->gyro_bias.x += attitude->config.mahony_ki * ex * dt;
                attitude->gyro_bias.y += attitude->config.mahony_ki * ey * dt;
            }

            corrected.x += attitude->config.mahony_kp * ex;
            corrected.y += attitude->config.mahony_kp * ey;
        }
    }

    imu_attitude_quat_integrate_gyro(&attitude->quat, &corrected, dt);
    imu_attitude_quat_to_euler(&attitude->quat, &attitude->angle);
}

static uint8_t imu_attitude_is_static(const ImuAttitude* attitude, const ImuSample* sample, const ImuGyro* gyro_corrected) {
    float acc_norm;
    float gyro_threshold;
    float acc_tolerance;

    if(attitude == 0 || sample == 0 || gyro_corrected == 0) {
        return 0U;
    }

    if(attitude->config.static_bias_enable == 0U) {
        return 0U;
    }

    if((sample->flags & IMU_SAMPLE_ACC_VALID) == 0U ||
        (sample->flags & IMU_SAMPLE_GYRO_VALID) == 0U) {
        return 0U;
    }

    gyro_threshold = attitude->config.static_gyro_threshold;

    if(fabsf(gyro_corrected->x) > gyro_threshold ||
        fabsf(gyro_corrected->y) > gyro_threshold ||
        fabsf(gyro_corrected->z) > gyro_threshold) {
        return 0U;
    }

    acc_norm = imu_attitude_acc_norm(&sample->acc);
    acc_tolerance = attitude->config.static_acc_tolerance;

    if(fabsf(acc_norm - attitude->config.acc_norm) > acc_tolerance) {
        return 0U;
    }

    return 1U;
}

static void imu_attitude_update_static_bias(ImuAttitude* attitude, const ImuSample* sample, ImuGyro* gyro_corrected) {
    float alpha;
    uint16_t static_target;

    if(attitude == 0 || sample == 0 || gyro_corrected == 0) {
        return;
    }

    if(attitude->config.static_bias_enable == 0U) {
        attitude->static_count = 0U;
        attitude->static_detected = 0U;
        return;
    }

    if(imu_attitude_is_static(attitude, sample, gyro_corrected) != 0U) {
        if(attitude->static_count < 0xFFFFU) {
            attitude->static_count++;
        }
    }
    else {
        attitude->static_count = 0U;
        attitude->static_detected = 0U;
        return;
    }

    static_target = attitude->config.static_bias_samples;

    if(attitude->static_count < static_target) {
        attitude->static_detected = 0U;
        return;
    }

    attitude->static_detected = 1U;

    alpha = attitude->config.static_bias_alpha;

    attitude->gyro_bias.x =
        (1.0f - alpha) * attitude->gyro_bias.x +
        alpha * sample->gyro.x;

    attitude->gyro_bias.y =
        (1.0f - alpha) * attitude->gyro_bias.y +
        alpha * sample->gyro.y;

    attitude->gyro_bias.z =
        (1.0f - alpha) * attitude->gyro_bias.z +
        alpha * sample->gyro.z;

    gyro_corrected->x = sample->gyro.x - attitude->gyro_bias.x;
    gyro_corrected->y = sample->gyro.y - attitude->gyro_bias.y;
    gyro_corrected->z = sample->gyro.z - attitude->gyro_bias.z;
}

static void imu_attitude_force_zero_if_static(const ImuAttitude* attitude, ImuGyro* gyro_corrected) {
    float threshold;

    if(attitude == 0 || gyro_corrected == 0) {
        return;
    }

    if(attitude->static_detected == 0U) {
        return;
    }

    threshold = attitude->config.static_force_zero_threshold;

    if(fabsf(gyro_corrected->x) < threshold) {
        gyro_corrected->x = 0.0f;
    }

    if(fabsf(gyro_corrected->y) < threshold) {
        gyro_corrected->y = 0.0f;
    }

    if(fabsf(gyro_corrected->z) < threshold) {
        gyro_corrected->z = 0.0f;
    }
}