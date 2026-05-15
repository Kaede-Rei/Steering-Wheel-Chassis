#include "DJI_Motor.h"
#include "bsp_can.h"

DJI_Motor_t dji_motors[4];
static CAN_HandleTypeDef* dji_motor_hcan = NULL;

void DJI_Motor_Init(CAN_HandleTypeDef* hcan) {
    dji_motor_hcan = hcan;

    for(int i = 0; i < 4; i++) {
        dji_motors[i].id = i + 1;
        dji_motors[i].target_rpm = 0.0f;

        // 初始化解算变量
        dji_motors[i].init_flag = 0;
        dji_motors[i].total_rotor_tick = 0;
        dji_motors[i].output_angle_deg = 0.0f;
        dji_motors[i].output_rpm = 0.0f;

        // --- 速度环 PID 参数 (空载起步参数) ---
        dji_motors[i].speed_pid.kp = 8.0f;
        dji_motors[i].speed_pid.ki = 0.1f;
        dji_motors[i].speed_pid.kd = 0.0f;
        dji_motors[i].speed_pid.max_integral = 5000.0f;
        dji_motors[i].speed_pid.max_output = 16384.0f;

        dji_motors[i].speed_pid.error = 0;
        dji_motors[i].speed_pid.last_error = 0;
        dji_motors[i].speed_pid.integral = 0;
    }
}

// 核心解析函数：处理多圈累加与减速比
void DJI_Motor_Parse(uint32_t std_id, const uint8_t* data) {
    if(std_id < 0x201 || std_id > 0x204) return;
    uint8_t index = std_id - 0x201;
    DJI_Motor_t* motor = &dji_motors[index];

    // 1. 解析底层原始数据
    motor->real_angle = (data[0] << 8) | data[1];
    motor->real_rpm = (data[2] << 8) | data[3];
    motor->real_current = (data[4] << 8) | data[5];
    motor->temperature = data[6];

    // 2. 第一次上电初始化 (防止一上电就产生巨大的差值跳变)
    if(motor->init_flag == 0) {
        motor->last_angle = motor->real_angle;
        motor->init_flag = 1;
    }

    // 3. 计算转子差值 (过零点处理)
    int16_t delta = motor->real_angle - motor->last_angle;
    if(delta > 4096) {
        delta -= 8192; // 实际是反转
    }
    else if(delta < -4096) {
        delta += 8192; // 实际是正转
    }

    // 4. 更新累计刻度与历史值
    motor->total_rotor_tick += delta;
    motor->last_angle = motor->real_angle;

    // 5. 换算输出轴数据 (供用户在 VSCode 调试观察用，不参与 PID)
    motor->output_rpm = (float)motor->real_rpm / M3508_REDUCTION_RATIO;
    motor->output_angle_deg = ((float)motor->total_rotor_tick / 8192.0f) * 360.0f / M3508_REDUCTION_RATIO;
}

// ⭐ 速度环 PID 计算 (完全使用转子原始数据，0 精度损失)
void DJI_Motor_Calc_PID(DJI_Motor_t* motor) {
    PID_Controller_t* pid = &motor->speed_pid;

    // 误差 = 目标转子转速 - 实际转子转速
    pid->error = motor->target_rpm - (float)motor->real_rpm;

    pid->integral += pid->error;
    if(pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    if(pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;

    pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * (pid->error - pid->last_error));
    pid->last_error = pid->error;

    if(pid->output > pid->max_output) pid->output = pid->max_output;
    if(pid->output < -pid->max_output) pid->output = -pid->max_output;
}

// 统一发送电流指令给 4 个电调
void DJI_Motor_Send_Currents(void) {
    if(dji_motor_hcan == NULL) {
        Error_Handler();
    }

    uint8_t tx_data[8];
    int16_t c1 = (int16_t)dji_motors[0].speed_pid.output;
    int16_t c2 = (int16_t)dji_motors[1].speed_pid.output;
    int16_t c3 = (int16_t)dji_motors[2].speed_pid.output;
    int16_t c4 = (int16_t)dji_motors[3].speed_pid.output;

    tx_data[0] = (c1 >> 8) & 0xFF;  tx_data[1] = c1 & 0xFF;
    tx_data[2] = (c2 >> 8) & 0xFF;  tx_data[3] = c2 & 0xFF;
    tx_data[4] = (c3 >> 8) & 0xFF;  tx_data[5] = c3 & 0xFF;
    tx_data[6] = (c4 >> 8) & 0xFF;  tx_data[7] = c4 & 0xFF;

    can_send(dji_motor_hcan, 0x200, tx_data, 8);
}