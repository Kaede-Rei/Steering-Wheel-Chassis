#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "main.h"

// 大疆 M3508 减速箱精确减速比
#define M3508_REDUCTION_RATIO (3591.0f / 187.0f) 

/* 速度环 PID 结构体 */
typedef struct {
    float kp, ki, kd;
    float error, last_error;
    float integral, max_integral;
    float output, max_output;
} PID_Controller_t;

/* 大疆电机结构体 */
typedef struct {
    uint8_t  id;             // 电机 ID (1~4)

    // --- 1. 原始反馈数据 (转子) ---
    uint16_t real_angle;     // 转子当前角度 (0~8191)
    uint16_t last_angle;     // 转子上一次角度
    int16_t  real_rpm;       // 转子真实转速
    int16_t  real_current;   // 转子真实电流
    uint8_t  temperature;    // 电机温度

    // --- 2. 解算后的数据 (输出轴 / 供监视用) ---
    float    output_rpm;       // 输出轴真实转速 (RPM)
    int32_t  total_rotor_tick; // 转子累计走过的总刻度 (多圈累加)
    float    output_angle_deg; // 输出轴的绝对累计角度 (度)
    uint8_t  init_flag;        // 上电初始化标志位

    // --- 3. 控制目标 ---
    float    target_rpm;     // 转子转速

    // --- 4. 控制器 ---
    PID_Controller_t speed_pid;
} DJI_Motor_t;

extern DJI_Motor_t dji_motors[4];

void DJI_Motor_Init(CAN_HandleTypeDef* hcan);
void DJI_Motor_Parse(uint32_t std_id, const uint8_t* data);
void DJI_Motor_Calc_PID(DJI_Motor_t* motor);
void DJI_Motor_Send_Currents(void);

#endif