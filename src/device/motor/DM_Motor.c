#include "DM_Motor.h"
#include <stddef.h>
#include <string.h>

DM_Motor_State_t dm_motors[4];
static BspCanHandle* dm_motor_hcan = NULL;

static void DM_Motor_Send_Cmd(BspCanHandle* hcan, uint8_t id, uint8_t cmd_byte) {
    const uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_byte };
    (void)can_send(hcan, id + 0x100U, data, 8U);
}

static BspCanHandle* DM_Motor_Get_CAN(void) {
    if(dm_motor_hcan == NULL) {
        Error_Handler();
    }
    return dm_motor_hcan;
}

void DM_Motor_Init(BspCanHandle* hcan) {
    dm_motor_hcan = hcan;
    memset(dm_motors, 0, sizeof(dm_motors));
}

void DM_Motor_Enable(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFC); }
void DM_Motor_Disable(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFD); }
void DM_Motor_Save_Zero(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFE); }
void DM_Motor_Clear_Error(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFB); }

void DM_Motor_Ctrl_MIT(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff) {
    uint8_t data[8];

    const uint16_t p = float_to_uint(p_des, DM_P_MIN, DM_P_MAX, 16);
    const uint16_t v = float_to_uint(v_des, DM_V_MIN, DM_V_MAX, 12) & 0x0FFFU;
    const uint16_t kp_i = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12) & 0x0FFFU;
    const uint16_t kd_i = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12) & 0x0FFFU;
    const uint16_t t = float_to_uint(t_ff, DM_T_MIN, DM_T_MAX, 12) & 0x0FFFU;

    data[0] = (uint8_t)((p >> 8) & 0xFFU);
    data[1] = (uint8_t)(p & 0xFFU);
    data[2] = (uint8_t)((v >> 4) & 0xFFU);
    data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp_i >> 8));
    data[4] = (uint8_t)(kp_i & 0xFFU);
    data[5] = (uint8_t)((kd_i >> 4) & 0xFFU);
    data[6] = (uint8_t)(((kd_i & 0x0FU) << 4) | (t >> 8));
    data[7] = (uint8_t)(t & 0xFFU);

    (void)can_send(DM_Motor_Get_CAN(), id, data, 8U);
}

void DM_Motor_Set_Angle_Rad(uint8_t id, float angle_rad) {
    DM_Motor_Ctrl_MIT(id, angle_rad, 0.0f, 3.0f, 0.032f, 0.0f);
}

void DM_Motor_Parse(const BspCanRxHeader* header, const uint8_t* data) {
    if(data == NULL) {
        return;
    }

    uint8_t id = data[0] & 0x0FU;
    if((id < 1U || id > 4U) && header != NULL) {
        id = (uint8_t)(header->Identifier & 0x0FU);
    }
    if(id < 1U || id > 4U) {
        return;
    }

    DM_Motor_State_t* motor = &dm_motors[id - 1U];
    const int p_int = (data[1] << 8) | data[2];
    const int v_int = (data[3] << 4) | (data[4] >> 4);
    const int t_int = ((data[4] & 0x0F) << 8) | data[5];

    motor->id = id;
    motor->state = (uint8_t)(data[0] >> 4);
    motor->pos = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);
    motor->vel = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
    motor->torque = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
    motor->t_mos = (int8_t)data[6];
    motor->t_rotor = (int8_t)data[7];
}

void DM_Motor_Ctrl_PosVel(uint8_t id, float p_des, float v_des) {
    uint8_t data[8];
    memcpy(&data[0], &p_des, sizeof(p_des));
    memcpy(&data[4], &v_des, sizeof(v_des));
    (void)can_send(DM_Motor_Get_CAN(), id + 0x100U, data, 8U);
}

void DM_Motor_Ctrl_Vel(uint8_t id, float v_des) {
    uint8_t data[4];
    memcpy(&data[0], &v_des, sizeof(v_des));
    (void)can_send(DM_Motor_Get_CAN(), id + 0x200U, data, 4U);
}
