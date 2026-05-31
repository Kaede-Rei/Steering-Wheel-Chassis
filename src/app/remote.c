/**
 * @file remote.c
 * @brief 遥控应用层实现
 */
#include "remote.h"

#include "chassis.h"
#include "chassis_yaw_hold.h"
#include "fs_ia10b.h"
#include "imu/imu.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 右摇杆横向通道索引
 */
#define REMOTE_CH_RIGHT_X 0u

/**
 * @brief 右摇杆纵向通道索引
 */
#define REMOTE_CH_RIGHT_Y 1u

/**
 * @brief 左摇杆横向通道索引
 */
#define REMOTE_CH_LEFT_X  3u

/**
 * @brief SWB 三挡开关通道索引
 */
#define REMOTE_CH_SWB     5u

/**
 * @brief SWC 三挡开关通道索引
 */
#define REMOTE_CH_SWC     6u

/**
 * @brief VRA 旋钮通道索引
 */
#define REMOTE_CH_VRA     8u

/**
 * @brief VRB 旋钮通道索引
 */
#define REMOTE_CH_VRB     9u

/**
 * @brief 遥控器中位原始值
 */
#define REMOTE_CENTER                1500

/**
 * @brief 遥控通道归一化半量程
 */
#define REMOTE_SPAN                  500.0f

/**
 * @brief 摇杆输入死区
 */
#define REMOTE_DEADBAND              10u

/**
 * @brief 遥控在线判定超时时间，单位 ms
 */
#define REMOTE_TIMEOUT_MS            100u

/**
 * @brief 遥控处理周期，单位 s
 */
#define REMOTE_CONTROL_PERIOD_S      0.010f

/**
 * @brief 高速档最大 x 向速度，单位 m/s
 */
#define REMOTE_FAST_MAX_VX_MPS       2.0f

/**
 * @brief 高速档最大 y 向速度，单位 m/s
 */
#define REMOTE_FAST_MAX_VY_MPS       2.0f

/**
 * @brief 高速档最大角速度，单位 rad/s
 */
#define REMOTE_FAST_MAX_WZ_RAD_S     8.0f

/**
 * @brief 中速档最大 x 向速度，单位 m/s
 */
#define REMOTE_MID_MAX_VX_MPS        1.0f

/**
 * @brief 中速档最大 y 向速度，单位 m/s
 */
#define REMOTE_MID_MAX_VY_MPS        1.0f

/**
 * @brief 中速档最大角速度，单位 rad/s
 */
#define REMOTE_MID_MAX_WZ_RAD_S      4.0f

/**
 * @brief 低速档最大 x 向速度，单位 m/s
 */
#define REMOTE_SLOW_MAX_VX_MPS       0.5f

/**
 * @brief 低速档最大 y 向速度，单位 m/s
 */
#define REMOTE_SLOW_MAX_VY_MPS       0.5f

/**
 * @brief 低速档最大角速度，单位 rad/s
 */
#define REMOTE_SLOW_MAX_WZ_RAD_S     2.0f

/**
 * @brief 旋钮低位阈值
 */
#define REMOTE_VR_LOW_THRESHOLD      1200u

/**
 * @brief 三挡开关低位原始值
 */
#define REMOTE_SW_LOW                2000u

/**
 * @brief 三挡开关高位原始值
 */
#define REMOTE_SW_HIGH               1000u

/**
 * @brief 三挡开关挡位判定容差
 */
#define REMOTE_SW_SELECT_TOLERANCE   250u

/**
 * @brief 遥控三挡速度上限配置
 */
typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

/**
 * @brief 最近一次对外输出的遥控命令快照
 */
static RemoteCommand s_command = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将原始通道值归一化到 [-1, 1]
 * @param value 遥控通道原始值
 * @param deadband 死区阈值
 * @return float 归一化后的通道值
 */
static float remote_channel_to_norm(uint16_t value, uint16_t deadband);

/**
 * @brief 根据 SWB 挡位选择当前速度上限
 * @param swb SWB 通道原始值
 * @return RemoteSpeedLimit 当前挡位的速度上限
 */
static RemoteSpeedLimit remote_get_speed_limit(uint16_t swb);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化遥控应用层状态
 */
void remote_init(void) {
    ChassisYawHoldConfig yaw_hold_config = chassis_yaw_hold_default_config();

    memset(&s_command, 0, sizeof(s_command));

    yaw_hold_config.kp = 48.0f;
    yaw_hold_config.kd = 2.0f;
    yaw_hold_config.v_deadband = 0.01f;
    yaw_hold_config.wz_deadband = 0.08f;
    yaw_hold_config.wz_limit = 1.0f;
    chassis_yaw_hold_init(&yaw_hold_config);
}

/**
 * @brief 执行一次遥控应用层轮询
 */
void remote_process(void) {
    FsIa10bData rc_data;
    RemoteSpeedLimit speed_limit;
    ibus_maintain();

    if(!ibus_get_data(&rc_data) || !ibus_is_online(REMOTE_TIMEOUT_MS)) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        s_command.online = false;
        chassis_yaw_hold_reset();
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }

    if(rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_LOW) {
        (void)chassis.set_steer_then_drive_enabled(false);
    }
    else if(rc_data.channel[REMOTE_CH_SWC] == REMOTE_CENTER) {
        (void)chassis.set_steer_then_drive_enabled(true);
    }

    if(rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH
        || rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        s_command.online = true;
        chassis_yaw_hold_reset();
        (void)chassis.brake();
        return;
    }

    if(rc_data.channel[REMOTE_CH_VRB] <= REMOTE_VR_LOW_THRESHOLD) {
        speed_limit = remote_get_speed_limit(rc_data.channel[REMOTE_CH_SWB]);
        s_command.vx = remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND) * speed_limit.max_vx;
        s_command.vy = -remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND) * speed_limit.max_vy;
        s_command.wz = -remote_channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND) * speed_limit.max_wz;
        s_command.online = true;

        if(chassis_yaw_hold_is_active()) {
            s_command.wz = chassis_yaw_hold_apply(
                s_command.vx,
                s_command.vy,
                s_command.wz,
                imu_get_angle().yaw,
                imu_get_gyro_corrected().z,
                REMOTE_CONTROL_PERIOD_S);
        }

        (void)chassis.set_velocity(s_command.vx, s_command.vy, s_command.wz);
    }
    else {
        s_command.online = true;
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        chassis_yaw_hold_reset();
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
    }
}

/**
 * @brief 获取最近一次遥控应用层输出的命令
 * @param out 输出命令缓冲区
 * @return bool `true` 表示遥控链路在线
 */
bool remote_get_command(RemoteCommand* out) {
    if(out == NULL) {
        return false;
    }

    *out = s_command;
    return s_command.online;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将原始通道值归一化到 [-1, 1]
 * @param value 遥控通道原始值
 * @param deadband 死区阈值
 * @return float 归一化后的通道值
 */
static float remote_channel_to_norm(uint16_t value, uint16_t deadband) {
    int32_t diff = (int32_t)value - REMOTE_CENTER;
    float normalized;

    if(diff < 0) {
        if((uint32_t)(-diff) <= deadband) {
            return 0.0f;
        }
    }
    else if((uint32_t)diff <= deadband) {
        return 0.0f;
    }

    normalized = (float)diff / REMOTE_SPAN;
    if(normalized > 1.0f) {
        return 1.0f;
    }
    if(normalized < -1.0f) {
        return -1.0f;
    }

    return normalized;
}

/**
 * @brief 根据 SWB 挡位选择当前速度上限
 * @param swb SWB 通道原始值
 * @return RemoteSpeedLimit 当前挡位的速度上限
 */
static RemoteSpeedLimit remote_get_speed_limit(uint16_t swb) {
    RemoteSpeedLimit limit;

    if(swb >= (REMOTE_SW_LOW - REMOTE_SW_SELECT_TOLERANCE)) {
        limit.max_vx = REMOTE_FAST_MAX_VX_MPS;
        limit.max_vy = REMOTE_FAST_MAX_VY_MPS;
        limit.max_wz = REMOTE_FAST_MAX_WZ_RAD_S;
    }
    else if(swb <= (REMOTE_SW_HIGH + REMOTE_SW_SELECT_TOLERANCE)) {
        limit.max_vx = REMOTE_SLOW_MAX_VX_MPS;
        limit.max_vy = REMOTE_SLOW_MAX_VY_MPS;
        limit.max_wz = REMOTE_SLOW_MAX_WZ_RAD_S;
    }
    else {
        limit.max_vx = REMOTE_MID_MAX_VX_MPS;
        limit.max_vy = REMOTE_MID_MAX_VY_MPS;
        limit.max_wz = REMOTE_MID_MAX_WZ_RAD_S;
    }

    return limit;
}
