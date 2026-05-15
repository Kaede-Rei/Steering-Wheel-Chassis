#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "main.h"
#include "bsp_can.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 达妙电机物理极值参数 */
#define DM_P_MIN  -12.5f
#define DM_P_MAX   12.5f
#define DM_V_MIN  -45.0f
#define DM_V_MAX   45.0f
#define DM_KP_MIN  0.0f
#define DM_KP_MAX  500.0f
#define DM_KD_MIN  0.0f
#define DM_KD_MAX  5.0f
#define DM_T_MIN  -18.0f
#define DM_T_MAX   18.0f

/* 电机反馈状态数据结构 */
typedef struct {
    uint8_t  id;
    uint8_t  state;
    float    pos;
    float    vel;
    float    torque;
    int8_t   t_mos;
    int8_t   t_rotor;
} DM_Motor_State_t;

extern DM_Motor_State_t dm_motors[4];

/* 绑定电机控制所使用的 CAN 总线 */
void DM_Motor_Init(CAN_HandleTypeDef* hcan);

/* 浮点数映射到无符号整数 */
static inline uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if(x_float > x_max) x_float = x_max;
    else if(x_float < x_min) x_float = x_min;
    return (uint16_t)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/* 无符号整数映射回浮点数 */
static inline float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* 基础控制 API 声明 */
void DM_Motor_Enable(uint8_t id);
void DM_Motor_Disable(uint8_t id);
void DM_Motor_Save_Zero(uint8_t id);
void DM_Motor_Clear_Error(uint8_t id);

/* 核心 MIT 模式发送 API */
void DM_Motor_Ctrl_MIT(uint8_t id, float p_des, float v_des, float kp, float kd, float t_ff);

/* 其它控制模式 API */
void DM_Motor_Ctrl_PosVel(uint8_t id, float p_des, float v_des);
void DM_Motor_Ctrl_Vel(uint8_t id, float v_des);

/* 专门用于舵向电机(6220)的弧度控制极简 API */
void DM_Motor_Set_Angle_Rad(uint8_t id, float angle_rad);

/* 达妙电机反馈解析函数 */
void DM_Motor_Parse(const CAN_RxHeaderTypeDef* header, const uint8_t* data);

#endif /* __DM_MOTOR_H__ */