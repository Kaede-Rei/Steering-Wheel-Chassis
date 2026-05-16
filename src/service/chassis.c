#include "chassis.h"
#include "delay.h"
#include "fdcan.h"
#include <stddef.h>

#include "motor/DJI_Motor.h"
#include "motor/DM_Motor.h"

#define ch chassis_interface

#define CHASSIS_DEFAULT_WHEEL_DRIVE_RATIO 1.0f
#define CHASSIS_PI 3.14159265358979323846f

typedef struct {
    ChassisModule module;
    uint8_t dm_id;
    uint8_t dji_index;
} ChassisModuleMap;

static Chassis s_chassis;

#define X(name, index, dm_id, dji_index) [CHASSIS_MODULE_##name] = { CHASSIS_MODULE_##name, (dm_id), (dji_index) },
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
    .task_500hz = chassis_task_500hz,
    .stop = chassis_stop,
    .get = chassis_get,
    .state = chassis_state,
    .control = chassis_control,
    .error_code_to_str = chassis_error_code_to_str
};
#undef X

static float chassis_wheel_omega_to_dji_rpm(float wheel_omega);
static float chassis_dji_rpm_to_wheel_omega(float dji_rpm);
static ChassisConfig chassis_default_config(void);
static ChassisErrorCode chassis_check_config(const ChassisConfig* config);
static void chassis_dm_can_rx_callback(BspCanHandle* hcan, const BspCanRxHeader* header, const uint8_t data[8], void* user);
static void chassis_dji_can_rx_callback(BspCanHandle* hcan, const BspCanRxHeader* header, const uint8_t data[8], void* user);

ChassisErrorCode chassis_init(void) {
    ChassisConfig config = chassis_default_config();
    return chassis_init_with_config(&config);
}

ChassisErrorCode chassis_init_with_config(const ChassisConfig* config) {
    ChassisErrorCode config_status = chassis_check_config(config);
    if(config_status != ch.OK) {
        return config_status;
    }

    s_chassis.config = *config;
    s_chassis.initialized = 0U;

    DM_Motor_Init(config->dm_hcan);
    DJI_Motor_Init(config->dji_hcan);

    if(can.register_rx_callback(config->dm_hcan, chassis_dm_can_rx_callback, NULL) != can.OK) {
        return ch.CAN_REGISTER_FAILED;
    }
    if(can.register_rx_callback(config->dji_hcan, chassis_dji_can_rx_callback, NULL) != can.OK) {
        return ch.CAN_REGISTER_FAILED;
    }

    if(swheel.init(&s_chassis.kine, config->model) != swheel.OK) {
        return ch.KINEMATICS_FAILED;
    }

    delay_ms(500);
    for(uint8_t i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        DM_Motor_Clear_Error(s_module_map[i].dm_id);
        delay_ms(100);
        DM_Motor_Enable(s_module_map[i].dm_id);
        delay_ms(100);
    }

    s_chassis.initialized = 1U;
    return ch.OK;
}

ChassisErrorCode chassis_set_velocity(float vx, float vy, float wz) {
    if(s_chassis.initialized == 0U) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = vx;
    s_chassis.kine.control.vy = vy;
    s_chassis.kine.control.wz = wz;

    return ch.OK;
}

ChassisErrorCode chassis_task_500hz(void) {
    if(s_chassis.initialized == 0U) {
        return ch.NOT_INITIALIZED;
    }

    for(uint8_t i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        const ChassisModuleMap* map = &s_module_map[i];
        const uint8_t dm_index = map->dm_id - 1U;
        const uint8_t dji_index = map->dji_index;

        s_chassis.kine.state.cur_wheels[map->module].wheel_omega =
            chassis_dji_rpm_to_wheel_omega((float)dji_motors[dji_index].real_rpm);
        s_chassis.kine.state.cur_wheels[map->module].steer_angle = dm_motors[dm_index].pos;
    }

    if(swheel.ik(&s_chassis.kine) != swheel.OK) {
        return ch.KINEMATICS_FAILED;
    }

    for(uint8_t i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        const ChassisModuleMap* map = &s_module_map[i];
        const uint8_t dji_index = map->dji_index;

        dji_motors[dji_index].target_rpm =
            chassis_wheel_omega_to_dji_rpm(s_chassis.kine.control.wheels[map->module].wheel_omega);
        if(dji_index == 1U || dji_index == 2U) {
            dji_motors[dji_index].target_rpm = -dji_motors[dji_index].target_rpm;
        }

        DM_Motor_Set_Angle_Rad(map->dm_id, s_chassis.kine.control.wheels[map->module].steer_angle);
        // DM_Motor_Ctrl_PosVel(map->dm_id, s_chassis.kine.control.wheels[map->module].steer_angle, 1.0f);
        DJI_Motor_Calc_PID(&dji_motors[dji_index]);
    }

    DJI_Motor_Send_Currents();

    if(swheel.fk(&s_chassis.kine) != swheel.OK) {
        return ch.KINEMATICS_FAILED;
    }

    return ch.OK;
}

ChassisErrorCode chassis_stop(void) {
    if(s_chassis.initialized == 0U) {
        return ch.NOT_INITIALIZED;
    }

    s_chassis.kine.control.vx = 0.0f;
    s_chassis.kine.control.vy = 0.0f;
    s_chassis.kine.control.wz = 0.0f;

    for(uint8_t i = 0U; i < CHASSIS_MODULE_COUNT; ++i) {
        dji_motors[s_module_map[i].dji_index].target_rpm = 0.0f;
        DJI_Motor_Calc_PID(&dji_motors[s_module_map[i].dji_index]);
    }
    DJI_Motor_Send_Currents();

    return ch.OK;
}

const Chassis* chassis_get(void) {
    return &s_chassis;
}

const SteerWheelState* chassis_state(void) {
    return &s_chassis.kine.state;
}

const SteerWheelControl* chassis_control(void) {
    return &s_chassis.kine.control;
}

#define X(name, str) case CHASSIS_##name: return str;
const char* chassis_error_code_to_str(ChassisErrorCode status) {
    switch(status) {
        CHASSIS_STATUS_TABLE
        default: return "UNKNOWN";
    }
}
#undef X

void Chassis_Init(void) {
    (void)chassis_init();
}

void Chassis_Set_Velocity(float vx, float vy, float wz) {
    (void)chassis_set_velocity(vx, vy, wz);
}

void Chassis_Task_500Hz(void) {
    (void)chassis_task_500hz();
}

static ChassisConfig chassis_default_config(void) {
    ChassisConfig config = {
        .dm_hcan = &hfdcan1,
        .dji_hcan = &hfdcan2,
        .model = {
            .length = 0.2657f,
            .width = 0.2657f,
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

static float chassis_wheel_omega_to_dji_rpm(float wheel_omega) {
    const float wheel_rpm = wheel_omega * 60.0f / (2.0f * CHASSIS_PI);
    return wheel_rpm * s_chassis.config.wheel_drive_ratio * M3508_REDUCTION_RATIO;
}

static float chassis_dji_rpm_to_wheel_omega(float dji_rpm) {
    const float wheel_rpm = dji_rpm / (s_chassis.config.wheel_drive_ratio * M3508_REDUCTION_RATIO);
    return wheel_rpm * (2.0f * CHASSIS_PI) / 60.0f;
}

static void chassis_dm_can_rx_callback(BspCanHandle* hcan,
    const BspCanRxHeader* header,
    const uint8_t data[8],
    void* user) {
    (void)hcan;
    (void)user;
    DM_Motor_Parse(header, data);
}

static void chassis_dji_can_rx_callback(BspCanHandle* hcan,
    const BspCanRxHeader* header,
    const uint8_t data[8],
    void* user) {
    (void)hcan;
    (void)user;
    DJI_Motor_Parse(header->Identifier, data);
}
