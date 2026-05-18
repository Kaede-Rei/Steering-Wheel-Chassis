#include "remote.h"

#include "chassis.h"
#include "fs_ia10b.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/**
 * @brief 右摇杆左右对应的 i.BUS 通道索引
 */
#define REMOTE_CH_RIGHT_X 0u
/**
 * @brief 右摇杆前后对应的 i.BUS 通道索引
 */
#define REMOTE_CH_RIGHT_Y 1u
/**
 * @brief 左摇杆前后对应的 i.BUS 通道索引
 */
#define REMOTE_CH_LEFT_Y 2u
/**
 * @brief 左摇杆左右对应的 i.BUS 通道索引
 */
#define REMOTE_CH_LEFT_X  3u

/**
 * @brief 顶部开关的通道索引
 */
#define REMOTE_CH_SWA 4u
#define REMOTE_CH_SWB 5u
#define REMOTE_CH_SWC 6u
#define REMOTE_CH_SWD 7u

/**
 * @brief 两个拨轮的通道索引
 */
#define REMOTE_CH_VRA 8u
#define REMOTE_CH_VRB 9u

/**
 * @brief 遥控器通道中位值
 */
#define REMOTE_CENTER      1500
/**
 * @brief 遥控器通道满量程半幅
 */
#define REMOTE_SPAN        500.0f
/**
 * @brief 摇杆死区，单位为原始通道值
 */
#define REMOTE_DEADBAND    10u
/**
 * @brief 遥控链路在线超时时间，单位 ms
 */
#define REMOTE_TIMEOUT_MS  100u

/**
 * @brief 遥控映射到的最大平移/旋转速度
 */
#define REMOTE_FAST_MAX_VX_MPS   2.0f
#define REMOTE_FAST_MAX_VY_MPS   2.0f
#define REMOTE_FAST_MAX_WZ_RAD_S 8.0f
#define REMOTE_MID_MAX_VX_MPS    1.0f
#define REMOTE_MID_MAX_VY_MPS    1.0f
#define REMOTE_MID_MAX_WZ_RAD_S  4.0f
#define REMOTE_SLOW_MAX_VX_MPS   0.5f
#define REMOTE_SLOW_MAX_VY_MPS   0.5f
#define REMOTE_SLOW_MAX_WZ_RAD_S 2.0f

/**
 * @brief VR 通道下拨与上拨阈值，单位为原始通道值
 */
#define REMOTE_VR_LOW_THRESHOLD 1200u
#define REMOTE_VR_HIGH_THRESHOLD 1800u

/**
 * @brief SW 通道下拨与上拨数值，单位为原始通道值
 */
#define REMOTE_SW_LOW 2000u
#define REMOTE_SW_HIGH 1000u
#define REMOTE_SW_SELECT_TOLERANCE 250u

typedef struct {
    float max_vx;
    float max_vy;
    float max_wz;
} RemoteSpeedLimit;

/**
 * @brief 遥控服务最近一次输出的控制指令
 */
static RemoteCommand s_command = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

/**
 * @brief 将遥控器原始通道值归一化到 [-1, 1]
 * @param value 原始通道值
 * @param deadband 死区阈值
 * @return float 归一化结果
 */
static float remote_channel_to_norm(uint16_t value, uint16_t deadband);
static RemoteSpeedLimit remote_get_speed_limit(uint16_t swb);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化遥控服务内部状态
 */
void remote_init(void) {
    memset(&s_command, 0, sizeof(s_command));
}

/**
 * @brief 执行一次遥控服务轮询
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
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
        return;
    }

    if(rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_LOW || rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD) {
        s_command.vx = 0.0f;
        s_command.vy = 0.0f;
        s_command.wz = 0.0f;
        s_command.online = true;
        (void)chassis.brake();
        return;
    }

    if(rc_data.channel[REMOTE_CH_VRB] <= REMOTE_VR_LOW_THRESHOLD) {
        speed_limit = remote_get_speed_limit(rc_data.channel[REMOTE_CH_SWB]);
        s_command.vx = remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_Y], REMOTE_DEADBAND) * speed_limit.max_vx;
        s_command.vy = -remote_channel_to_norm(rc_data.channel[REMOTE_CH_RIGHT_X], REMOTE_DEADBAND) * speed_limit.max_vy;
        s_command.wz = -remote_channel_to_norm(rc_data.channel[REMOTE_CH_LEFT_X], REMOTE_DEADBAND) * speed_limit.max_wz;
        s_command.online = true;
        (void)chassis.set_velocity(s_command.vx, s_command.vy, s_command.wz);
    }
    else {
        s_command.online = true;
        (void)chassis.set_velocity(0.0f, 0.0f, 0.0f);
    }
}

/**
 * @brief 获取最近一次遥控服务输出的控制指令
 * @param out 输出控制指令缓冲区
 * @return bool `true` 表示当前遥控链路在线，`false` 表示离线
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
 * @brief 将遥控器原始通道值归一化到 [-1, 1]
 * @param value 原始通道值
 * @param deadband 死区阈值
 * @return float 归一化结果
 */
static float remote_channel_to_norm(uint16_t value, uint16_t deadband) {
    int32_t diff = (int32_t)value - REMOTE_CENTER;
    float normalized;

    if(diff < 0) {
        if((uint32_t)(-diff) <= deadband) {
            return 0.0f;
        }
    }
    else {
        if((uint32_t)diff <= deadband) {
            return 0.0f;
        }
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
