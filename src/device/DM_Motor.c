#include "DM_Motor.h"
#include <string.h>

DM_Motor_State_t dm_motors[4];
static CAN_HandleTypeDef* dm_motor_hcan = NULL;

void DM_Motor_Init(CAN_HandleTypeDef* hcan) {
    dm_motor_hcan = hcan;
}

/* 内部基础指令发送函数 */
static void DM_Motor_Send_Cmd(CAN_HandleTypeDef* hcan, uint8_t id, uint8_t cmd_byte) {
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd_byte };
    can_send(hcan, id, data, 8);
}

static CAN_HandleTypeDef* DM_Motor_Get_CAN(void) {
    if(dm_motor_hcan == NULL) {
        Error_Handler();
    }
    return dm_motor_hcan;
}

void DM_Motor_Enable(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFC); }
void DM_Motor_Disable(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFD); }
void DM_Motor_Save_Zero(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFE); }
void DM_Motor_Clear_Error(uint8_t id) { DM_Motor_Send_Cmd(DM_Motor_Get_CAN(), id, 0xFB); }

/* MIT 控制模式报文打包与发送 */
void DM_Motor_Ctrl_MIT(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff) {
    uint8_t data[8];

    uint16_t p = float_to_uint(p_des, DM_P_MIN, DM_P_MAX, 16);
    uint16_t v = float_to_uint(v_des, DM_V_MIN, DM_V_MAX, 12) & 0x0FFF;
    uint16_t kp_i = float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12) & 0x0FFF;
    uint16_t kd_i = float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12) & 0x0FFF;
    uint16_t t = float_to_uint(t_ff, DM_T_MIN, DM_T_MAX, 12) & 0x0FFF;

    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;
    data[2] = (v >> 4) & 0xFF;
    data[3] = ((v & 0xF) << 4) | (kp_i >> 8);
    data[4] = kp_i & 0xFF;
    data[5] = (kd_i >> 4) & 0xFF;
    data[6] = ((kd_i & 0xF) << 4) | (t >> 8);
    data[7] = t & 0xFF;

    can_send(DM_Motor_Get_CAN(), id, data, 8);
}

/* 专门用于舵向电机(6220)的弧度控制极简 API */
void DM_Motor_Set_Angle_Rad(uint8_t id, float angle_rad) {
    // Kp=3.0, Kd=0.03 为测试参考值，实车可调
    DM_Motor_Ctrl_MIT(id, angle_rad, 0.0f, 3.0f, 0.032f, 0.0f);
}

/* 达妙电机反馈报文解析 (MIT 模式) */
void DM_Motor_Parse(const CAN_RxHeaderTypeDef* header, const uint8_t* data) {
    if(data == NULL) return;

    /*
     * 达妙 MIT 反馈常见格式：data[0] 高 4 位为状态，低 4 位为电机 ID。
     * 旧代码使用 header->StdId & 0x0F，若反馈仲裁 ID 是 master id / 0x000 / 0x100 类，
     * 会导致 id 不在 1~4 范围内，表现为 CAN 有反馈但 dm_motors[] 永远不更新。
     */
    uint8_t id = data[0] & 0x0F;
    if((id < 1U || id > 4U) && header != NULL) {
        id = header->StdId & 0x0FU;
    }
    if(id < 1U || id > 4U) return;

    uint8_t index = id - 1U;
    dm_motors[index].id = id;
    dm_motors[index].state = (data[0] >> 4);

    int p_int = (data[1] << 8) | data[2];
    int v_int = (data[3] << 4) | (data[4] >> 4);
    int t_int = ((data[4] & 0xF) << 8) | data[5];

    dm_motors[index].pos = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);
    dm_motors[index].vel = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
    dm_motors[index].torque = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
    dm_motors[index].t_mos = data[6];
    dm_motors[index].t_rotor = data[7];
}

/* 位置速度模式 */
void DM_Motor_Ctrl_PosVel(uint8_t id, float p_des, float v_des) {
    uint8_t data[8];
    memcpy(&data[0], &p_des, 4);
    memcpy(&data[4], &v_des, 4);
    can_send(DM_Motor_Get_CAN(), id + 0x100, data, 8);
}

/* 纯速度模式 */
void DM_Motor_Ctrl_Vel(uint8_t id, float v_des) {
    uint8_t data[4];
    memcpy(&data[0], &v_des, 4);
    can_send(DM_Motor_Get_CAN(), id + 0x200, data, 4);
}