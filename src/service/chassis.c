#include "chassis.h"

#include "bus_motor/agv_motor.h"
#include "bus_motor/dji_motor.h"
#include "bus_motor/dm_motor.h"
#include "delay.h"
#include "fdcan.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //

#define ch chassis_interface

#define CHASSIS_DEFAULT_WHEEL_DRIVE_RATIO 1.0f
#define CHASSIS_STEER_TRACK_SPEED_RAD_S   6.28f
#define CHASSIS_BRAKE_ANGLE_TOL_RAD       0.12f
#define CHASSIS_PI                        3.14159265358979323846f

typedef struct {
    ChassisModule module;
    uint8_t dm_id;
    uint8_t dji_id;
    int8_t drive_sign;
} ChassisModuleMap;

static Chassis s_chassis = { 0 };
static Chassis s_chassis_view = { 0 };
static FDCAN_HandleTypeDef* s_dm_can = NULL;
static FDCAN_HandleTypeDef* s_dji_can = NULL;

#define X(name, index, dm_id, dji_index) [CHASSIS_MODULE_##name] = { CHASSIS_MODULE_##name, (dm_id), (uint8_t)((dji_index) + 1u), (((dji_index) == 1 || (dji_index) == 2) ? -1 : 1) },
static const ChassisModuleMap s_module_map[CHASSIS_MODULE_COUNT] = {
    CHASSIS_MODULE_TABLE
};
#undef X

#define X(name, str) .name = CHASSIS_##name,
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
static float chassis_wrap_pi(float angle);
static float chassis_select_nearest_equivalent_angle(float current_angle, float target_angle);
static void chassis_set_brake_targets(void);
static bool chassis_brake_targets_reached(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

ChassisErrorCode chassis_init(void) {
    ChassisConfig config = chassis_default_config();
    return chassis_init_with_config(&config);
}

ChassisErrorCode chassis_init_with_config(const ChassisConfig* config) {
    static const BusMotorPortOps steer_ops = {
        .send = chassis_dm_can_send,
    };
    static const BusMotorPortOps drive_ops = {
        .send = chassis_dji_can_send,
    };
    BusMotorConfig steer_config = {
        .ops = &steer_ops,
        .timeout_ms = 0u,
        .retry_count = 0u,
    };
    BusMotorConfig drive_config = {
        .ops = &drive_ops,
        .timeout_ms = 0u,
        .retry_count = 0u,
    };
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

ChassisErrorCode chassis_process(void) {
    uint8_t i;

    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        const ChassisModuleMap* map = &s_module_map[i];

        steer_motor.update_feedback(map->dm_id, NULL);
        drive_motor.update_feedback(map->dji_id, NULL);
    }

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
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
                const ChassisModuleMap* map = &s_module_map[i];

                (void)drive_motor.stop(map->dji_id);
                (void)steer_motor.set_spd(map->dm_id, CHASSIS_STEER_TRACK_SPEED_RAD_S);
                (void)steer_motor.set_pos(map->dm_id, s_chassis.kine.control.wheels[map->module].steer_angle);
            }

            if(chassis_brake_targets_reached()) {
                s_chassis.brake_latched = 1u;
            }
        }

        if(s_chassis.brake_latched != 0u) {
            for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
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
        if(swheel.ik(&s_chassis.kine) != swheel.OK) {
            return ch.KINEMATICS_FAILED;
        }

        for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
            const ChassisModuleMap* map = &s_module_map[i];
            float target_speed =
                chassis_wheel_omega_to_drive_omega(s_chassis.kine.control.wheels[map->module].wheel_omega);

            (void)drive_motor.set_spd(map->dji_id, (float)map->drive_sign * target_speed);
            (void)steer_motor.set_spd(map->dm_id, CHASSIS_STEER_TRACK_SPEED_RAD_S);
            (void)steer_motor.set_pos(map->dm_id, s_chassis.kine.control.wheels[map->module].steer_angle);
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

ChassisErrorCode chassis_stop(void) {
    uint8_t i;

    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;
    s_chassis.brake_requested = 0u;
    s_chassis.brake_latched = 0u;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        (void)steer_motor.brake(s_module_map[i].dm_id);
        (void)drive_motor.stop(s_module_map[i].dji_id);
    }

    return ch.OK;
}

ChassisErrorCode chassis_brake(void) {
    if(s_chassis.initialized == 0u) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;
    s_chassis.brake_requested = 1u;

    return ch.OK;
}

const Chassis* chassis_get_chassis(void) {
    return &s_chassis_view;
}

const SteerWheelState* chassis_get_state(void) {
    return &s_chassis_view.kine.state;
}

const SteerWheelControl* chassis_get_control(void) {
    return &s_chassis_view.kine.control;
}

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
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
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

static float chassis_wrap_pi(float angle) {
    while(angle > CHASSIS_PI) {
        angle -= 2.0f * CHASSIS_PI;
    }
    while(angle <= -CHASSIS_PI) {
        angle += 2.0f * CHASSIS_PI;
    }

    return angle;
}

static float chassis_select_nearest_equivalent_angle(float current_angle, float target_angle) {
    float option_a = chassis_wrap_pi(target_angle);
    float option_b = chassis_wrap_pi(target_angle + CHASSIS_PI);
    float error_a = chassis_wrap_pi(option_a - current_angle);
    float error_b = chassis_wrap_pi(option_b - current_angle);

    if(fabsf(error_b) < fabsf(error_a)) {
        return option_b;
    }

    return option_a;
}

static void chassis_set_brake_targets(void) {
    const float hx = s_chassis.config.model.length * 0.5f;
    const float hy = s_chassis.config.model.width * 0.5f;
    float target_fl = atan2f(hy, -hx);
    float target_fr = atan2f(-hy, -hx);
    float target_rr = atan2f(-hy, hx);
    float target_rl = atan2f(hy, hx);

    s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].wheel_omega = 0.0f;
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].wheel_omega = 0.0f;

    s_chassis.kine.control.wheels[CHASSIS_MODULE_FL].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_FL].steer_angle, target_fl);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_FR].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_FR].steer_angle, target_fr);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RR].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_RR].steer_angle, target_rr);
    s_chassis.kine.control.wheels[CHASSIS_MODULE_RL].steer_angle =
        chassis_select_nearest_equivalent_angle(s_chassis.kine.state.cur_wheels[CHASSIS_MODULE_RL].steer_angle, target_rl);
}

static bool chassis_brake_targets_reached(void) {
    uint8_t i;

    for(i = 0u; i < CHASSIS_MODULE_COUNT; ++i) {
        float current_angle = s_chassis.kine.state.cur_wheels[i].steer_angle;
        float target_angle = s_chassis.kine.control.wheels[i].steer_angle;
        float angle_error = chassis_wrap_pi(target_angle - current_angle);

        if(fabsf(angle_error) > CHASSIS_BRAKE_ANGLE_TOL_RAD) {
            return false;
        }
    }

    return true;
}
