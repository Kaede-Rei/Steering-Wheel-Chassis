#ifndef _imu_attitude_h_
#define _imu_attitude_h_

#include "imu.h"

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief IMU 姿态融合状态码
 */
typedef enum {
    IMU_ATTITUDE_STATUS_OK = 0,
    IMU_ATTITUDE_STATUS_ERROR,
    IMU_ATTITUDE_STATUS_INVALID_PARAM,
    IMU_ATTITUDE_STATUS_CALIBRATING,
    IMU_ATTITUDE_STATUS_NOT_READY,
} ImuAttitudeStatus;

/**
 * @brief 四元数
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} ImuQuat;

/**
 * @brief IMU 姿态融合对象
 */
typedef struct {
    ImuAttitudeConfig config;

    ImuQuat quat;
    ImuAngle angle;

    /**
     * @brief 当前估计的陀螺零偏
     *
     * 单位 rad/s
     */
    ImuGyro gyro_bias;

    /**
     * @brief 姿态融合内部使用的去零偏角速度
     *
     * 对上层控制来说，应该优先使用这个 gyro
     */
    ImuGyro gyro_filtered;

    ImuAcc acc_filtered;

    uint32_t last_update_us;

    ImuGyro calib_sum;
    ImuGyro calib_sq_sum;
    uint16_t calib_count;

    uint16_t static_count;
    uint8_t static_detected;

    uint8_t calibrated;
    uint8_t has_angle;
} ImuAttitude;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化姿态融合对象
 *
 * @param attitude 姿态融合对象
 * @param config 配置
 * @return ImuAttitudeStatus 状态码
 */
ImuAttitudeStatus imu_attitude_init(
    ImuAttitude* attitude,
    const ImuAttitudeConfig* config
);

/**
 * @brief 更新姿态融合
 *
 * @param attitude 姿态融合对象
 * @param sample IMU 原始采样
 * @return ImuAttitudeStatus 状态码
 */
ImuAttitudeStatus imu_attitude_update(
    ImuAttitude* attitude,
    const ImuSample* sample
);

/**
 * @brief 获取欧拉角
 *
 * @param attitude 姿态融合对象
 * @param angle 输出角度
 * @return ImuAttitudeStatus 状态码
 */
ImuAttitudeStatus imu_attitude_get_angle(
    const ImuAttitude* attitude,
    ImuAngle* angle
);

/**
 * @brief 获取四元数
 *
 * @param attitude 姿态融合对象
 * @param quat 输出四元数
 * @return ImuAttitudeStatus 状态码
 */
ImuAttitudeStatus imu_attitude_get_quat(
    const ImuAttitude* attitude,
    ImuQuat* quat
);

/**
 * @brief 获取姿态融合内部使用的去零偏角速度
 *
 * @param attitude 姿态融合对象
 * @param gyro 输出角速度
 * @return ImuAttitudeStatus 状态码
 */
ImuAttitudeStatus imu_attitude_get_gyro(
    const ImuAttitude* attitude,
    ImuGyro* gyro
);

/**
 * @brief 重置 yaw
 *
 * 保持当前 roll / pitch，只重置 yaw
 *
 * @param attitude 姿态融合对象
 * @param yaw 新 yaw，单位 rad
 * @return ImuAttitudeStatus 状态码
 */
ImuAttitudeStatus imu_attitude_reset_yaw(
    ImuAttitude* attitude,
    float yaw
);

#endif