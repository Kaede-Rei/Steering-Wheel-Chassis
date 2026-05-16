#include "DJI_Motor.h"
#include <stddef.h>

DJI_Motor_t dji_motors[4];
static BspCanHandle* dji_motor_hcan = NULL;

void DJI_Motor_Init(BspCanHandle* hcan) {
    dji_motor_hcan = hcan;

    for(uint8_t i = 0U; i < 4U; ++i) {
        dji_motors[i].id = i + 1U;
        dji_motors[i].real_angle = 0U;
        dji_motors[i].last_angle = 0U;
        dji_motors[i].real_rpm = 0;
        dji_motors[i].real_current = 0;
        dji_motors[i].temperature = 0U;
        dji_motors[i].output_rpm = 0.0f;
        dji_motors[i].total_rotor_tick = 0;
        dji_motors[i].output_angle_deg = 0.0f;
        dji_motors[i].init_flag = 0U;
        dji_motors[i].target_rpm = 0.0f;

        dji_motors[i].speed_pid.kp = 8.0f;
        dji_motors[i].speed_pid.ki = 0.1f;
        dji_motors[i].speed_pid.kd = 0.0f;
        dji_motors[i].speed_pid.error = 0.0f;
        dji_motors[i].speed_pid.last_error = 0.0f;
        dji_motors[i].speed_pid.integral = 0.0f;
        dji_motors[i].speed_pid.max_integral = 5000.0f;
        dji_motors[i].speed_pid.output = 0.0f;
        dji_motors[i].speed_pid.max_output = 16384.0f;
    }
}

void DJI_Motor_Parse(uint32_t std_id, const uint8_t* data) {
    if(data == NULL || std_id < 0x201U || std_id > 0x204U) {
        return;
    }

    DJI_Motor_t* motor = &dji_motors[std_id - 0x201U];

    motor->real_angle = (uint16_t)((data[0] << 8) | data[1]);
    motor->real_rpm = (int16_t)((data[2] << 8) | data[3]);
    motor->real_current = (int16_t)((data[4] << 8) | data[5]);
    motor->temperature = data[6];

    if(motor->init_flag == 0U) {
        motor->last_angle = motor->real_angle;
        motor->init_flag = 1U;
    }

    int16_t delta = (int16_t)(motor->real_angle - motor->last_angle);
    if(delta > 4096) {
        delta -= 8192;
    }
    else if(delta < -4096) {
        delta += 8192;
    }

    motor->total_rotor_tick += delta;
    motor->last_angle = motor->real_angle;
    motor->output_rpm = (float)motor->real_rpm / M3508_REDUCTION_RATIO;
    motor->output_angle_deg = ((float)motor->total_rotor_tick / 8192.0f) * 360.0f / M3508_REDUCTION_RATIO;
}

void DJI_Motor_Calc_PID(DJI_Motor_t* motor) {
    if(motor == NULL) {
        return;
    }

    PID_Controller_t* pid = &motor->speed_pid;
    pid->error = motor->target_rpm - (float)motor->real_rpm;

    pid->integral += pid->error;
    if(pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    }
    if(pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    pid->output = (pid->kp * pid->error)
        + (pid->ki * pid->integral)
        + (pid->kd * (pid->error - pid->last_error));
    pid->last_error = pid->error;

    if(pid->output > pid->max_output) {
        pid->output = pid->max_output;
    }
    if(pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }
}

void DJI_Motor_Send_Currents(void) {
    if(dji_motor_hcan == NULL) {
        Error_Handler();
    }

    uint8_t tx_data[8];
    const int16_t c1 = (int16_t)dji_motors[0].speed_pid.output;
    const int16_t c2 = (int16_t)dji_motors[1].speed_pid.output;
    const int16_t c3 = (int16_t)dji_motors[2].speed_pid.output;
    const int16_t c4 = (int16_t)dji_motors[3].speed_pid.output;

    tx_data[0] = (uint8_t)((c1 >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(c1 & 0xFF);
    tx_data[2] = (uint8_t)((c2 >> 8) & 0xFF);
    tx_data[3] = (uint8_t)(c2 & 0xFF);
    tx_data[4] = (uint8_t)((c3 >> 8) & 0xFF);
    tx_data[5] = (uint8_t)(c3 & 0xFF);
    tx_data[6] = (uint8_t)((c4 >> 8) & 0xFF);
    tx_data[7] = (uint8_t)(c4 & 0xFF);

    (void)can_send(dji_motor_hcan, 0x200U, tx_data, 8U);
}
