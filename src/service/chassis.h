#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "bsp_can.h"
#include "steer_wheel_kine.h"
#include <stdint.h>

#define chassis chassis_interface

#define CHASSIS_STATUS_TABLE \
    X(OK, "OK") \
    X(INVALID_PARAM, "Invalid Parameter") \
    X(INVALID_MODEL, "Invalid Model") \
    X(CAN_REGISTER_FAILED, "CAN RX Callback Register Failed") \
    X(KINEMATICS_FAILED, "Steer Wheel Kinematics Failed") \
    X(NOT_INITIALIZED, "Chassis Not Initialized")

#define CHASSIS_MODULE_TABLE \
    X(FL, 0, 1, 0) \
    X(FR, 1, 2, 1) \
    X(RR, 2, 3, 2) \
    X(RL, 3, 4, 3)

#define X(name, str) CHASSIS_##name,
typedef enum {
    CHASSIS_STATUS_TABLE
} ChassisErrorCode;
#undef X

#define X(name, index, dm_id, dji_index) CHASSIS_MODULE_##name = (index),
typedef enum {
    CHASSIS_MODULE_TABLE
    CHASSIS_MODULE_COUNT = 4
} ChassisModule;
#undef X

typedef struct {
    BspCanHandle* dm_hcan;
    BspCanHandle* dji_hcan;
    SteerWheelModel model;
    float wheel_drive_ratio;
} ChassisConfig;

typedef struct {
    SteerWheel kine;
    ChassisConfig config;
    uint8_t initialized;
} Chassis;

#define X(name, str) ChassisErrorCode name;
extern const struct ChassisInterface {
    struct {
        CHASSIS_STATUS_TABLE
    };
    ChassisErrorCode (*init)(void);
    ChassisErrorCode (*init_with_config)(const ChassisConfig* config);
    ChassisErrorCode (*set_velocity)(float vx, float vy, float wz);
    ChassisErrorCode (*task_500hz)(void);
    ChassisErrorCode (*stop)(void);
    const Chassis* (*get)(void);
    const SteerWheelState* (*state)(void);
    const SteerWheelControl* (*control)(void);
    const char* (*error_code_to_str)(ChassisErrorCode status);
} chassis_interface;
#undef X

ChassisErrorCode chassis_init(void);
ChassisErrorCode chassis_init_with_config(const ChassisConfig* config);
ChassisErrorCode chassis_set_velocity(float vx, float vy, float wz);
ChassisErrorCode chassis_task_500hz(void);
ChassisErrorCode chassis_stop(void);
const Chassis* chassis_get(void);
const SteerWheelState* chassis_state(void);
const SteerWheelControl* chassis_control(void);
const char* chassis_error_code_to_str(ChassisErrorCode status);

void Chassis_Init(void);
void Chassis_Set_Velocity(float vx, float vy, float wz);
void Chassis_Task_500Hz(void);

#endif
