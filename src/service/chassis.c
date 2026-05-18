#include "chassis.h"

#include "bus_motor/agv_motor.h"
#include "bus_motor/dji_motor.h"
#include "bus_motor/dm_motor.h"
#include "delay.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

/** @brief 底盘接口单例的文件内短别名 */
#define ch chassis_interface

/** @brief 默认车轮角速度到驱动电机角速度的传动比 */
#define CHASSIS_DEFAULT_WHEEL_DRIVE_RATIO 1.0f
/** @brief 转向电机 S 曲线规划后的最大跟踪速度，单位 rad/s */
#define CHASSIS_STEER_TRACK_SPEED_RAD_S   11.5f
/** @brief 转向电机 S 曲线规划的最低跟踪速度，单位 rad/s */
#define CHASSIS_STEER_MIN_SPEED_RAD_S     5.5f
/** @brief 转向电机从最低速度平滑加速到最高速度的时间，单位 s */
#define CHASSIS_STEER_SPEED_RAMP_TIME_S   0.20f
/** @brief 转向接近目标角时开始平滑降速的角度窗口，单位 rad */
#define CHASSIS_STEER_SLOWDOWN_ANGLE_RAD  0.628f
/** @brief 底盘控制任务周期，单位 s */
#define CHASSIS_CONTROL_PERIOD_S          0.002f
/** @brief 认为转向目标发生明显变化并重启 S 曲线的角度阈值，单位 rad */
#define CHASSIS_STEER_TARGET_CHANGE_RAD   0.03f
/** @brief 普通行驶时允许驱动电机出力的转向角误差阈值，单位 rad */
#define CHASSIS_DRIVE_ANGLE_TOL_RAD       0.157f
/** @brief 驻车刹车流程判定转向角到位的误差阈值，单位 rad */
#define CHASSIS_BRAKE_ANGLE_TOL_RAD       0.157f
/** @brief 圆周率常量，用于角度归一化 */
#define CHASSIS_PI                        3.14159265358979323846f
/** @brief 转向角完整周期，单位 rad */
#define CHASSIS_2PI                       (2.0f * CHASSIS_PI)
/** @brief 转向电机位置命令安全边界，单位 rad */
#define CHASSIS_STEER_POS_LIMIT_RAD       12.4f
/** @brief 等效舵角切换滞回，单位 rad */
#define CHASSIS_EQUIV_ANGLE_HYST_RAD      0.12f
/** @brief 普通行驶等效舵角最终优化滞回，单位 rad */
#define CHASSIS_DRIVE_EQUIV_HYST_RAD      0.03f

/** @brief 单个舵轮模块与实际 CAN 电机 ID 的映射关系 */
typedef struct {
    ChassisModule module; /**< 舵轮模块逻辑编号 */
    uint8_t dm_id;        /**< 转向达妙电机 CAN ID */
    uint8_t dji_id;       /**< 驱动大疆电机 CAN ID */
    int8_t drive_sign;    /**< 驱动电机安装方向修正符号，取值为 1 或 -1 */
} ChassisModuleMap;

/** @brief 底盘服务内部运行实例 */
static Chassis s_chassis = { 0 };
/** @brief 底盘服务对外只读快照，坐标系已转换为外部约定 */
static Chassis s_chassis_view = { 0 };
/** @brief 每个舵轮当前 S 曲线加速阶段累计时间，单位 s */
static float s_steer_speed_ramp_time[CHASSIS_MODULE_COUNT] = { 0.0f };
/** @brief 每个舵轮上一次用于速度规划的目标转向角，单位 rad */
static float s_steer_speed_last_target[CHASSIS_MODULE_COUNT] = { 0.0f };
/** @brief 每个舵轮的 S 曲线速度规划状态是否已初始化 */
static uint8_t s_steer_speed_initialized[CHASSIS_MODULE_COUNT] = { 0u };
/** @brief 转向电机所在 FDCAN 句柄 */
static FDCAN_HandleTypeDef* s_dm_can = NULL;
/** @brief 驱动电机所在 FDCAN 句柄 */
static FDCAN_HandleTypeDef* s_dji_can = NULL;

/** @brief 展开舵轮模块表，生成逻辑模块到电机 ID 的映射项 */
#define X(name, index, dm_id, dji_index) [CHASSIS_MODULE_##name] = { CHASSIS_MODULE_##name, (dm_id), (uint8_t)((dji_index) + 1u), (((dji_index) == 1 || (dji_index) == 2) ? -1 : 1) },
/** @brief 四个舵轮模块的固定电机映射表 */
static const ChassisModuleMap s_module_map[CHASSIS_MODULE_COUNT] = {
    CHASSIS_MODULE_TABLE
};
#undef X

/** @brief 展开底盘状态表，生成接口结构体内的状态码常量 */
#define X(name, str) .name = CHASSIS_##name,
/** @brief 底盘服务对外接口单例 */
const struct ChassisInterface chassis_interface = {
    {
        CHASSIS_STATUS_TABLE
    },
    .init = chassis_init,
    .init_with_config = chassis_init_with_config,
    .set_velocity = chassis_set_velocity,
    .process = chassis_process,
    .stop = chassis_stop,
    .brake = chassis_brake,
    .get_chassis = chassis_get_chassis,
    .get_state = chassis_get_state,
    .get_control = chassis_get_control,
    .error_code_to_str = chassis_error_code_to_str
};
#undef X

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static float chassis_wheel_omega_to_drive_omega(float wheel_omega);
static float chassis_drive_omega_to_wheel_omega(float drive_omega);
static ChassisConfig chassis_default_config(void);
static ChassisErrorCode chassis_check_config(const ChassisConfig* config);
static bool chassis_dm_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static bool chassis_dji_can_send(uint32_t id, const uint8_t* data, uint8_t len);
static ChassisErrorCode chassis_enable_steer_motors(void);
static void chassis_dm_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
static void chassis_dji_can_rx_callback(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header, const uint8_t data[8], void* user);
static void chassis_external_to_internal_twist(float vx_ext, float vy_ext, float wz_ext, float* vx_int, float* vy_int, float* wz_int);
static void chassis_internal_to_external_twist(float vx_int, float vy_int, float wz_int, float* vx_ext, float* vy_ext, float* wz_ext);
static float chassis_clampf(float value, float min, float max);
static float chassis_wrap_pi(float angle);
static float chassis_smoothstep(float x);
static float chassis_select_nearest_cyclic_angle(float current_angle, float target_angle);
static float chassis_select_nearest_equivalent_angle(float current_angle, float target_angle, float previous_target_angle);
static void chassis_optimize_drive_module_target(ChassisModule module);
static void chassis_reset_steer_speed_profiles(void);
static float chassis_calc_steer_track_speed(ChassisModule module, float target_angle);
static bool chassis_drive_targets_reached(void);
static void chassis_set_brake_targets(void);
static bool chassis_brake_targets_reached(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 使用默认配置初始化底盘
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_init(void) {
    /** @brief 默认底盘初始化配置 */
    ChassisConfig config = chassis_default_config();
    return chassis_init_with_config(&config);
}

/**
 * @brief 使用指定配置初始化底盘
 * @param config 底盘配置
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_init_with_config(const ChassisConfig* config) {
    /** @brief 转向电机 CAN 发送端口操作 */
    static const BusMotorPortOps steer_ops = {
        .send = chassis_dm_can_send,
    };
    /** @brief 驱动电机 CAN 发送端口操作 */
    static const BusMotorPortOps drive_ops = {
        .send = chassis_dji_can_send,
    };
    /** @brief 转向电机设备层初始化配置 */
    BusMotorConfig steer_config = {
        .ops = &steer_ops,
        .timeout_ms = 0u,
        .retry_count = 0u,
    };
    /** @brief 驱动电机设备层初始化配置 */
    BusMotorConfig drive_config = {
        .ops = &drive_ops,
        .timeout_ms = 0u,
        .retry_count = 0u,
    };
    /** @brief 底盘配置参数检查结果 */
    ChassisErrorCode config_status = chassis_check_config(config);

    if(config_status != ch.OK) {
        return config_status;
    }

    steer_motor_set_instance(&dm_motor_instance);
    drive_motor_set_instance(&dji_motor_instance);

    s_chassis.config = *config;
    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;
    s_chassis.initialized = 0u;
    chassis_reset_steer_speed_profiles();
    s_dm_can = config->dm_hcan;
    s_dji_can = config->dji_hcan;

    if(steer_motor.init(&steer_config) != MOTOR_STATUS_OK) {
        return ch.INVALID_PARAM;
    }
    if(drive_motor.init(&drive_config) != MOTOR_STATUS_OK) {
        return ch.INVALID_PARAM;
    }

    if(can_register_rx_callback(config->dm_hcan, chassis_dm_can_rx_callback, NULL) != STM32_HAL_CAN_OK) {
        return ch.CAN_REGISTER_FAILED;
    }
    if(can_register_rx_callback(config->dji_hcan, chassis_dji_can_rx_callback, NULL) != STM32_HAL_CAN_OK) {
        return ch.CAN_REGISTER_FAILED;
    }

    if(swheel.init(&s_chassis.kine, config->model) != swheel.OK) {
        return ch.KINEMATICS_FAILED;
    }

    delay_ms(500);
    if(chassis_enable_steer_motors() != ch.OK) {
        return ch.INVALID_PARAM;
    }

    s_chassis.initialized = 1u;
    return ch.OK;
}

/**
 * @brief 设置底盘目标速度
 * @param vx 底盘 x 方向目标线速度，单位 m/s
 * @param vy 底盘 y 方向目标线速度，单位 m/s
 * @param wz 底盘 z 轴目标角速度，单位 rad/s
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_set_velocity(float vx, float vy, float wz) {
    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;
    chassis_external_to_internal_twist(vx, vy, wz,
        &s_chassis.kine.control.vx, &s_chassis.kine.control.vy, &s_chassis.kine.control.wz);

    return ch.OK;
}

/**
 * @brief 执行一次底盘控制流程
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_process(void) {
    /** @brief 通用循环索引 */
    uint8_t i;

    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        /** @brief 当前循环处理的舵轮模块映射 */
        const ChassisModuleMap* map = &s_module_map[i];

        steer_motor.update_feedback(map->dm_id, NULL);
        drive_motor.update_feedback(map->dji_id, NULL);
    }

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        /** @brief 当前循环处理的舵轮模块映射 */
        const ChassisModuleMap* map = &s_module_map[i];

        s_chassis.kine.state.cur_wheels[map->module].wheel_omega =
            chassis_drive_omega_to_wheel_omega((float)map->drive_sign * drive_motor.get_spd(map->dji_id));
        s_chassis.kine.state.cur_wheels[map->module].steer_angle =
            steer_motor.get_pos(map->dm_id);
    }

    if(s_chassis.brake_requested != 0u) {
        chassis_set_brake_targets();

        if(s_chassis.brake_latched == 0u) {
            for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
                /** @brief 当前循环处理的舵轮模块映射 */
                const ChassisModuleMap* map = &s_module_map[i];
                /** @brief 驻车流程中的目标转向角，单位 rad */
                float target_angle = s_chassis.kine.control.wheels[map->module].steer_angle;
                /** @brief S 曲线规划后的转向跟踪速度，单位 rad/s */
                float steer_speed = chassis_calc_steer_track_speed(map->module, target_angle);

                (void)drive_motor.stop(map->dji_id);
                (void)steer_motor.set_pos_vel(map->dm_id, target_angle, steer_speed);
            }

            if(chassis_brake_targets_reached()) {
                s_chassis.brake_latched = 1u;
            }
        }

        if(s_chassis.brake_latched != 0u) {
            for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
                /** @brief 当前循环处理的舵轮模块映射 */
                const ChassisModuleMap* map = &s_module_map[i];

                (void)steer_motor.brake(map->dm_id);
                (void)drive_motor.brake(map->dji_id);
            }
        }

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            s_chassis.kine.state.cur_wheels[i].wheel_omega = 0.0f;
        }
        s_chassis.kine.state.cur_vx = 0.0f;
        s_chassis.kine.state.cur_vy = 0.0f;
        s_chassis.kine.state.cur_wz = 0.0f;
    }
    else {
        /** @brief 所有舵轮角度是否已到位并允许驱动输出 */
        bool drive_ready;

        if(swheel.ik(&s_chassis.kine) != swheel.OK) {
            return ch.KINEMATICS_FAILED;
        }

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            /** @brief 当前循环处理的舵轮模块映射 */
            const ChassisModuleMap* map = &s_module_map[i];

            chassis_optimize_drive_module_target(map->module);
        }

        drive_ready = chassis_drive_targets_reached();

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            /** @brief 当前循环处理的舵轮模块映射 */
            const ChassisModuleMap* map = &s_module_map[i];
            /** @brief 当前舵轮目标转向角，单位 rad */
            float target_angle = s_chassis.kine.control.wheels[map->module].steer_angle;
            /** @brief 当前舵轮目标驱动电机角速度，单位 rad/s */
            float target_speed =
                chassis_wheel_omega_to_drive_omega(s_chassis.kine.control.wheels[map->module].wheel_omega);
            /** @brief S 曲线规划后的转向跟踪速度，单位 rad/s */
            float steer_speed = chassis_calc_steer_track_speed(map->module, target_angle);

            (void)steer_motor.set_pos_vel(map->dm_id, target_angle, steer_speed);
            (void)drive_motor.set_spd(map->dji_id, drive_ready ? ((float)map->drive_sign * target_speed) : 0.0f);
        }

        if(swheel.fk(&s_chassis.kine) != swheel.OK) {
            return ch.KINEMATICS_FAILED;
        }
    }

    s_chassis_view = s_chassis;
    chassis_internal_to_external_twist(s_chassis.kine.state.cur_vx, s_chassis.kine.state.cur_vy, s_chassis.kine.state.cur_wz,
        &s_chassis_view.kine.state.cur_vx, &s_chassis_view.kine.state.cur_vy, &s_chassis_view.kine.state.cur_wz);

    return ch.OK;
}

/**
 * @brief 停止底盘运动
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_stop(void) {
    /** @brief 通用循环索引 */
    uint8_t i;

    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;
    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;
    chassis_reset_steer_speed_profiles();

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        (void)steer_motor.brake(s_module_map[i].dm_id);
        (void)drive_motor.stop(s_module_map[i].dji_id);
    }

    return ch.OK;
}

/**
 * @brief 请求底盘进入驻车刹车状态
 * @return ChassisErrorCode 状态码
 */
ChassisErrorCode chassis_brake(void) {
    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;
    if(s_chassis.brake_requested == 0u) {
        s_chassis.brake_latched = 0u;
        chassis_reset_steer_speed_profiles();
    }
    s_chassis.brake_requested = 1u;

    return ch.OK;
}

/**
 * @brief 获取底盘实例只读视图
 * @return const Chassis* 底盘实例指针
 */
const Chassis* chassis_get_chassis(void) {
    return &s_chassis_view;
}

/**
 * @brief 获取底盘当前状态只读视图
 * @return const SteerWheelState* 当前状态指针
 */
const SteerWheelState* chassis_get_state(void) {
    return &s_chassis_view.kine.state;
}

/**
 * @brief 获取底盘当前控制量只读视图
 * @return const SteerWheelControl* 当前控制量指针
 */
const SteerWheelControl* chassis_get_control(void) {
    return &s_chassis_view.kine.control;
}

/**
 * @brief 将底盘状态码转换为静态字符串
 * @param status 底盘状态码
 * @return const char* 状态码名称
 */
/** @brief 展开底盘状态表，生成状态码到字符串的 switch 分支 */
#define X(name, str) case CHASSIS_##name: return str;
const char* chassis_error_code_to_str(ChassisErrorCode status) {
    switch(status) {
        CHASSIS_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static ChassisConfig chassis_default_config(void) {
    /** @brief 默认底盘配置参数 */
    ChassisConfig config = {
        .dm_hcan = &hfdcan1,
        .dji_hcan = &hfdcan2,
        .model = {
            .length = 0.26572986916f,
            .width = 0.26572986916f,
            .wheel_radius = 0.057965f,
            .max_wheel_linear_speed = 2.0f
        },
        .wheel_drive_ratio = CHASSIS_DEFAULT_WHEEL_DRIVE_RATIO
    };

    return config;
}

static ChassisErrorCode chassis_check_config(const ChassisConfig* config) {
    if(config == NULL || config->dm_hcan == NULL || config->dji_hcan == NULL) {
        return ch.INVALID_PARAM;
    }
    if(config->model.length <= 0.0f || config->model.width <= 0.0f
        || config->model.wheel_radius <= 0.0f || config->model.max_wheel_linear_speed < 0.0f
        || config->wheel_drive_ratio <= 0.0f) {
        return ch.INVALID_MODEL;
    }

    return ch.OK;
}

static bool chassis_dm_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    if(s_dm_can == NULL) {
        return false;
    }

    return can_send(s_dm_can, id, data, len) == STM32_HAL_CAN_OK;
}

static bool chassis_dji_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
    if(s_dji_can == NULL) {
        return false;
    }

    return can_send(s_dji_can, id, data, len) == STM32_HAL_CAN_OK;
}

static ChassisErrorCode chassis_enable_steer_motors(void) {
    /** @brief 通用循环索引 */
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        /** @brief 当前待初始化的转向电机 CAN ID */
        uint8_t dm_id = s_module_map[i].dm_id;

        (void)dm_motor_clear_error(dm_id);
        delay_ms(100);
        (void)steer_motor.enable(dm_id);
        delay_ms(100);
        (void)steer_motor.switch_mode(dm_id, DM_MOTOR_MODE_POS_VEL);
        (void)steer_motor.set_spd(dm_id, CHASSIS_STEER_TRACK_SPEED_RAD_S);
        (void)steer_motor.set_tor(dm_id, 0.0f);
        (void)steer_motor.brake(dm_id);
    }

    return ch.OK;
}

static float chassis_wheel_omega_to_drive_omega(float wheel_omega) {
    return wheel_omega * s_chassis.config.wheel_drive_ratio;
}

static float chassis_drive_omega_to_wheel_omega(float drive_omega) {
    return drive_omega / s_chassis.config.wheel_drive_ratio;
}

static void chassis_dm_can_rx_callback(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[8],
    void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    (void)dm_motor_parse_feedback_frame(header->Identifier, data, NULL);
}

static void chassis_dji_can_rx_callback(FDCAN_HandleTypeDef* hcan,
    const FDCAN_RxHeaderTypeDef* header,
    const uint8_t data[8],
    void* user) {
    (void)hcan;
    (void)user;

    if(header == NULL) {
        return;
    }

    (void)dji_motor_parse_feedback_frame(header->Identifier, data, NULL);
}

static void chassis_external_to_internal_twist(float vx_ext, float vy_ext, float wz_ext, float* vx_int, float* vy_int, float* wz_int) {
    if(vx_int != NULL) *vx_int = vx_ext;
    if(vy_int != NULL) *vy_int = -vy_ext;
    if(wz_int != NULL) *wz_int = -wz_ext;
}

static void chassis_internal_to_external_twist(float vx_int, float vy_int, float wz_int, float* vx_ext, float* vy_ext, float* wz_ext) {
    if(vx_ext != NULL) *vx_ext = vx_int;
    if(vy_ext != NULL) *vy_ext = -vy_int;
    if(wz_ext != NULL) *wz_ext = -wz_int;
}

static float chassis_clampf(float value, float min, float max) {
    if(value < min) {
        return min;
    }
    if(value > max) {
        return max;
    }

    return value;
}

static float chassis_wrap_pi(float angle) {
    while(angle > CHASSIS_PI) {
        angle -= CHASSIS_2PI;
    }
    while(angle <= -CHASSIS_PI) {
        angle += CHASSIS_2PI;
    }

    return angle;
}

static float chassis_select_nearest_cyclic_angle(float current_angle, float target_angle) {
    /** @brief 从当前角到目标角的最短周期角误差，单位 rad */
    float best_target = target_angle;
    /** @brief 当前最优目标角与反馈角的距离，单位 rad */
    float best_error = fabsf(target_angle - current_angle);
    /** @brief 周期候选值索引 */
    int8_t k;

    for(k = -2; k <= 2; ++k) {
        /** @brief 当前周期候选目标角，单位 rad */
        float candidate = target_angle + (float)k * CHASSIS_2PI;
        /** @brief 当前周期候选目标角与反馈角的距离，单位 rad */
        float error;

        if(candidate < -CHASSIS_STEER_POS_LIMIT_RAD || candidate > CHASSIS_STEER_POS_LIMIT_RAD) {
            continue;
        }

        error = fabsf(candidate - current_angle);
        if(error < best_error) {
            best_error = error;
            best_target = candidate;
        }
    }

    return chassis_clampf(best_target, -CHASSIS_STEER_POS_LIMIT_RAD, CHASSIS_STEER_POS_LIMIT_RAD);
}

static float chassis_smoothstep(float x) {
    if(x <= 0.0f) {
        return 0.0f;
    }
    if(x >= 1.0f) {
        return 1.0f;
    }

    return x * x * (3.0f - 2.0f * x);
}

static float chassis_select_nearest_equivalent_angle(float current_angle, float target_angle, float previous_target_angle) {
    /** @brief 归一化后的原始目标角候选值，单位 rad */
    float option_a = chassis_select_nearest_cyclic_angle(current_angle, target_angle);
    /** @brief 翻转 180 度后的等效目标角候选值，单位 rad */
    float option_b = chassis_select_nearest_cyclic_angle(current_angle, target_angle + CHASSIS_PI);
    /** @brief 上一周期目标角的最近周期候选值，单位 rad */
    float option_prev = chassis_select_nearest_cyclic_angle(current_angle, previous_target_angle);
    /** @brief 当前角到原始目标角候选值的最短角度误差，单位 rad */
    float error_a = option_a - current_angle;
    /** @brief 当前角到等效目标角候选值的最短角度误差，单位 rad */
    float error_b = option_b - current_angle;
    /** @brief 当前角到上一周期目标角候选值的最短角度误差，单位 rad */
    float error_prev = option_prev - current_angle;
    /** @brief 上一周期目标角相对原始目标角的等效性误差，单位 rad */
    float prev_equiv_error_a = fabsf(chassis_wrap_pi(option_prev - option_a));
    /** @brief 上一周期目标角相对翻转目标角的等效性误差，单位 rad */
    float prev_equiv_error_b = fabsf(chassis_wrap_pi(option_prev - option_b));

    if((prev_equiv_error_a < CHASSIS_DRIVE_ANGLE_TOL_RAD || prev_equiv_error_b < CHASSIS_DRIVE_ANGLE_TOL_RAD)
        && fabsf(error_prev) <= (fminf(fabsf(error_a), fabsf(error_b)) + CHASSIS_EQUIV_ANGLE_HYST_RAD)) {
        return option_prev;
    }

    if(fabsf(error_b) < fabsf(error_a)) {
        return option_b;
    }

    return option_a;
}

static void chassis_optimize_drive_module_target(ChassisModule module) {
    /** @brief 当前舵轮反馈转向角，单位 rad */
    float current_angle;
    /** @brief IK 输出的目标转向角，单位 rad */
    float target_angle;
    /** @brief IK 输出的目标车轮角速度，单位 rad/s */
    float target_omega;
    /** @brief 不翻转轮速时的最近周期目标角，单位 rad */
    float option_a;
    /** @brief 翻转轮速时的最近周期目标角，单位 rad */
    float option_b;
    /** @brief 不翻转轮速方案的转向误差，单位 rad */
    float error_a;
    /** @brief 翻转轮速方案的转向误差，单位 rad */
    float error_b;

    if(module >= CHASSIS_MODULE_COUNT) {
        return;
    }

    current_angle = s_chassis.kine.state.cur_wheels[module].steer_angle;
    target_angle = s_chassis.kine.control.wheels[module].steer_angle;
    target_omega = s_chassis.kine.control.wheels[module].wheel_omega;

    option_a = chassis_select_nearest_cyclic_angle(current_angle, target_angle);
    option_b = chassis_select_nearest_cyclic_angle(current_angle, target_angle + CHASSIS_PI);
    error_a = option_a - current_angle;
    error_b = option_b - current_angle;

    if(fabsf(error_b) + CHASSIS_DRIVE_EQUIV_HYST_RAD < fabsf(error_a)) {
        s_chassis.kine.control.wheels[module].steer_angle = option_b;
        s_chassis.kine.control.wheels[module].wheel_omega = -target_omega;
    }
    else {
        s_chassis.kine.control.wheels[module].steer_angle = option_a;
    }
}

static void chassis_reset_steer_speed_profiles(void) {
    /** @brief 通用循环索引 */
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        s_steer_speed_ramp_time[i] = 0.0f;
        s_steer_speed_last_target[i] = 0.0f;
        s_steer_speed_initialized[i] = 0u;
    }
}

static float chassis_calc_steer_track_speed(ChassisModule module, float target_angle) {
    /** @brief 当前舵轮反馈转向角，单位 rad */
    float current_angle;
    /** @brief 当前舵轮目标角与反馈角的绝对误差，单位 rad */
    float angle_error;
    /** @brief 新目标角相对上一次规划目标角的变化量，单位 rad */
    float target_delta;
    /** @brief S 曲线加速阶段归一化时间，范围通常为 0 到 1 */
    float ramp_ratio;
    /** @brief 接近目标角时的 S 曲线降速比例，范围为 0 到 1 */
    float slowdown_ratio;
    /** @brief 最终用于插值最低/最高转向速度的归一化比例 */
    float speed_ratio;

    if(module >= CHASSIS_MODULE_COUNT) {
        return CHASSIS_STEER_TRACK_SPEED_RAD_S;
    }

    current_angle = s_chassis.kine.state.cur_wheels[module].steer_angle;
    angle_error = fabsf(chassis_wrap_pi(target_angle - current_angle));

    if(angle_error <= CHASSIS_DRIVE_ANGLE_TOL_RAD) {
        s_steer_speed_ramp_time[module] = 0.0f;
        s_steer_speed_last_target[module] = target_angle;
        s_steer_speed_initialized[module] = 1u;
        return CHASSIS_STEER_MIN_SPEED_RAD_S;
    }

    target_delta = chassis_wrap_pi(target_angle - s_steer_speed_last_target[module]);
    if(s_steer_speed_initialized[module] == 0u || fabsf(target_delta) > CHASSIS_STEER_TARGET_CHANGE_RAD) {
        s_steer_speed_ramp_time[module] = 0.0f;
        s_steer_speed_last_target[module] = target_angle;
        s_steer_speed_initialized[module] = 1u;
    }

    s_steer_speed_ramp_time[module] += CHASSIS_CONTROL_PERIOD_S;
    ramp_ratio = s_steer_speed_ramp_time[module] / CHASSIS_STEER_SPEED_RAMP_TIME_S;
    slowdown_ratio = chassis_smoothstep(angle_error / CHASSIS_STEER_SLOWDOWN_ANGLE_RAD);
    speed_ratio = chassis_smoothstep(ramp_ratio);
    if(slowdown_ratio < speed_ratio) {
        speed_ratio = slowdown_ratio;
    }

    return CHASSIS_STEER_MIN_SPEED_RAD_S
        + (CHASSIS_STEER_TRACK_SPEED_RAD_S - CHASSIS_STEER_MIN_SPEED_RAD_S) * speed_ratio;
}

static bool chassis_drive_targets_reached(void) {
    /** @brief 通用循环索引 */
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        /** @brief 当前舵轮反馈转向角，单位 rad */
        float current_angle = s_chassis.kine.state.cur_wheels[i].steer_angle;
        /** @brief 当前舵轮目标转向角，单位 rad */
        float target_angle = s_chassis.kine.control.wheels[i].steer_angle;
        /** @brief 当前舵轮目标角与反馈角的最短角度误差，单位 rad */
        float angle_error = chassis_wrap_pi(target_angle - current_angle);

        if(fabsf(angle_error) > CHASSIS_DRIVE_ANGLE_TOL_RAD) {
            return false;
        }
    }

    return true;
}

static void chassis_set_brake_targets(void) {
    /** @brief 底盘半轴距，单位 m */
    const float hx = s_chassis.config.model.length * 0.5f;
    /** @brief 底盘半轮距，单位 m */
    const float hy = s_chassis.config.model.width * 0.5f;
    /** @brief 左前轮驻车目标角，单位 rad */
    float target_fl = atan2f(hy, -hx);
    /** @brief 右前轮驻车目标角，单位 rad */
    float target_fr = atan2f(-hy, -hx);
    /** @brief 右后轮驻车目标角，单位 rad */
    float target_rr = atan2f(-hy, hx);
    /** @brief 左后轮驻车目标角，单位 rad */
    float target_rl = atan2f(hy, hx);

    s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].wheel_omega = 0.0f;

    s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_FL].steer_angle,
            target_fl,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].steer_angle);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_FR].steer_angle,
            target_fr,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].steer_angle);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_RR].steer_angle,
            target_rr,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].steer_angle);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_RL].steer_angle,
            target_rl,
            s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].steer_angle);
}

static bool chassis_brake_targets_reached(void) {
    /** @brief 通用循环索引 */
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        /** @brief 当前舵轮反馈转向角，单位 rad */
        float current_angle = s_chassis.kine.state.cur_wheels[i].steer_angle;
        /** @brief 当前舵轮驻车目标转向角，单位 rad */
        float target_angle = s_chassis.kine.control.wheels[i].steer_angle;
        /** @brief 当前舵轮驻车目标角与反馈角的最短角度误差，单位 rad */
        float angle_error = chassis_wrap_pi(target_angle - current_angle);

        if(fabsf(angle_error) > CHASSIS_BRAKE_ANGLE_TOL_RAD) {
            return false;
        }
    }

    return true;
}
