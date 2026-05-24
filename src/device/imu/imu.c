#include "imu.h"
#include "imu_attitude.h"

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 当前绑定的具体 IMU 实例
 */
const ImuInterface* imu_instance = 0;
static ImuAttitude s_imu_attitude;
static uint8_t s_imu_attitude_enabled = 0U;

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 绑定具体 IMU 实例
 */
ImuStatus imu_set_instance(const ImuInterface* instance) {
    if(instance == 0) {
        return IMU_STATUS_INVALID_PARAM;
    }

    imu_instance = instance;
    return IMU_STATUS_OK;
}

/**
 * @brief 初始化当前绑定的 IMU 实例
 */
ImuStatus imu_init(const void* config) {
    if(imu_instance == 0 || imu_instance->init == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    return imu_instance->init(config);
}

/**
 * @brief 更新当前绑定 IMU 的数据缓存
 */
ImuStatus imu_update(void) {
    ImuStatus status = IMU_STATUS_OK;

    if(imu_instance == 0 || imu_instance->update == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    status = imu_instance->update();
    if(status != IMU_STATUS_OK && status != IMU_STATUS_NOT_READY) {
        return status;
    }

    if(s_imu_attitude_enabled != 0U && status == IMU_STATUS_OK) {
        ImuSample sample = { 0 };

        if(imu_get_sample(&sample) == IMU_STATUS_OK) {
            (void)imu_attitude_update(&s_imu_attitude, &sample);
        }
    }

    return status;
}

/**
 * @brief 获取最近一次缓存的加速度
 */
ImuAcc imu_get_acc(void) {
    ImuAcc acc = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_acc == 0) {
        return acc;
    }

    return imu_instance->get_acc();
}

/**
 * @brief 获取最近一次缓存的角速度
 */
ImuGyro imu_get_gyro(void) {
    ImuGyro gyro = { 0.0f, 0.0f, 0.0f };

    if(imu_instance == 0 || imu_instance->get_gyro == 0) {
        return gyro;
    }

    return imu_instance->get_gyro();
}

/**
 * @brief 获取最近一次缓存的姿态角
 */
ImuAngle imu_get_angle(void) {
    ImuAngle angle = { 0.0f, 0.0f, 0.0f };

    if(s_imu_attitude_enabled != 0U &&
        imu_attitude_get_angle(&s_imu_attitude, &angle) == IMU_ATTITUDE_STATUS_OK) {
        return angle;
    }

    if(imu_instance == 0 || imu_instance->get_angle == 0) {
        return angle;
    }

    return imu_instance->get_angle();
}

/**
 * @brief 鑾峰彇鏈€杩戜竴娆＄紦瀛樼殑 sample
 */
ImuStatus imu_get_sample(ImuSample* sample) {
    if(sample == 0) {
        return IMU_STATUS_INVALID_PARAM;
    }

    if(imu_instance == 0) {
        return IMU_STATUS_NO_INSTANCE;
    }

    if(imu_instance->get_sample != 0) {
        return imu_instance->get_sample(sample);
    }

    if(imu_instance->get_acc == 0 || imu_instance->get_gyro == 0) {
        return IMU_STATUS_UNSUPPORTED;
    }

    sample->acc = imu_instance->get_acc();
    sample->gyro = imu_instance->get_gyro();
    sample->temperature = 0.0f;
    sample->timestamp_us = 0U;
    sample->flags = IMU_SAMPLE_ACC_NEW | IMU_SAMPLE_GYRO_NEW;

    if(s_imu_attitude.config.now_us != 0) {
        sample->timestamp_us = s_imu_attitude.config.now_us();
    }

    return IMU_STATUS_OK;
}

/**
 * @brief 鍚敤 IMU 閫氱敤濮挎€佽瀺鍚?
 */
ImuStatus imu_attitude_enable(const ImuAttitudeConfig* config) {
    if(config == 0) {
        return IMU_STATUS_INVALID_PARAM;
    }

    if(imu_attitude_init(&s_imu_attitude, config) != IMU_ATTITUDE_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    s_imu_attitude_enabled = 1U;
    return IMU_STATUS_OK;
}

/**
 * @brief 鍏抽棴 IMU 閫氱敤濮挎€佽瀺鍚?
 */
ImuStatus imu_attitude_disable(void) {
    s_imu_attitude_enabled = 0U;
    return IMU_STATUS_OK;
}

/**
 * @brief 閲嶇疆 yaw 瑙?
 */
ImuStatus imu_reset_yaw(float yaw) {
    if(s_imu_attitude_enabled == 0U) {
        return IMU_STATUS_NOT_READY;
    }

    if(imu_attitude_reset_yaw(&s_imu_attitude, yaw) != IMU_ATTITUDE_STATUS_OK) {
        return IMU_STATUS_ERROR;
    }

    return IMU_STATUS_OK;
}

/**
 * @brief 将状态码转换为常量字符串
 */
const char* imu_status_str(ImuStatus status) {
    if(imu_instance != 0 && imu_instance->status_str != 0) {
        return imu_instance->status_str(status);
    }

    switch(status) {
#define X(name, value) case IMU_STATUS_##name: return #name;
        IMU_STATUS_TABLE
#undef X
        default: return "UNKNOWN";
    }
}
