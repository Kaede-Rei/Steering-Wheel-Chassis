#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "bsp_can.h"
#include <stdint.h>

#define M3508_REDUCTION_RATIO (3591.0f / 187.0f)

typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float max_integral;
    float output;
    float max_output;
} PID_Controller_t;

typedef struct {
    uint8_t id;
    uint16_t real_angle;
    uint16_t last_angle;
    int16_t real_rpm;
    int16_t real_current;
    uint8_t temperature;
    float output_rpm;
    int32_t total_rotor_tick;
    float output_angle_deg;
    uint8_t init_flag;
    float target_rpm;
    PID_Controller_t speed_pid;
} DJI_Motor_t;

extern DJI_Motor_t dji_motors[4];

void DJI_Motor_Init(BspCanHandle* hcan);
void DJI_Motor_Parse(uint32_t std_id, const uint8_t* data);
void DJI_Motor_Calc_PID(DJI_Motor_t* motor);
void DJI_Motor_Send_Currents(void);

#endif
