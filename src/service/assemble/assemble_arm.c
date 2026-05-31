#include "assemble.h"

#include "stm32_hal_uart.h"
#include "bus_servo/zhong_ling_servo.h"
#include "delay.h"

// ! ========================= 变 量 声 明 ========================= ! //

#define SERVO_SPEED 3.14f

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static inline bool servo_write(const uint8_t* data, uint16_t len) {
    return uart7_write_blocking((const char*)data, len);
}

static int servo_read(uint8_t* data, uint16_t len);

static void servo_flush_rx(void);

static const BusServoPortOps servo_port_ops = {
    .write = servo_write,
    .read = servo_read,
    .now_ms = HAL_GetTick,
    .delay_ms = delay_ms,
    .flush_rx = servo_flush_rx,
};

static const ZhongLingServoConfig servo_config = {
    .ops = &servo_port_ops,
    .timeout_ms = 100,
    .retry_count = 3,
    .pos_min_rad = -3.1415926f * 3.0f / 4.0f,
    .pos_center_rad = 0.0f,
    .pos_max_rad = 3.1415926f * 3.0f / 4.0f,
    .pwm_min = ZHONG_LING_SERVO_PWM_MIN,
    .pwm_center = ZHONG_LING_SERVO_PWM_CENTER,
    .pwm_max = ZHONG_LING_SERVO_PWM_MAX,
    .invert = false,
    .allow_torque_ignore = false,
};

static const ZhongLingServoPosSpdCmd arm_init_cmds[5] = {
    {.id = 0, .pos_rad = 0.0f, .spd_rad_s = SERVO_SPEED },
    {.id = 1, .pos_rad = 1.884f, .spd_rad_s = SERVO_SPEED },
    {.id = 2, .pos_rad = 2.355f, .spd_rad_s = SERVO_SPEED },
    {.id = 3, .pos_rad = -1.413f, .spd_rad_s = SERVO_SPEED },
    {.id = 4, .pos_rad = 0.0f, .spd_rad_s = SERVO_SPEED },
};

// static const ZhongLingServoPosSpdCmd arm_test_cmds[5] = {    当处于初始位姿时有:
//     {.id = 0, .pos_rad = 1.0f, .spd_rad_s = SERVO_SPEED },       朝天右手法则, 朝前为0°
//     {.id = 1, .pos_rad = 0.0f, .spd_rad_s = SERVO_SPEED },       朝右右手法则, 朝天为0°
//     {.id = 2, .pos_rad = 0.0f, .spd_rad_s = SERVO_SPEED },       朝左右手法则, 朝天为0°
//     {.id = 3, .pos_rad = 0.0f, .spd_rad_s = SERVO_SPEED },       朝右右手法则, 朝天为0°
//     {.id = 4, .pos_rad = 1.0f, .spd_rad_s = SERVO_SPEED },       朝左右手法则, 朝天为0°
// };

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_arm(void) {
    bus_servo_set_instance(&zhong_ling_servo_common_instance);
    bus_servo.init(&servo_config);

    zhong_ling_servo.set_multi_pos_spd(arm_init_cmds, 5);

    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static int servo_read(uint8_t* data, uint16_t len) {
    return 0;
}

static void servo_flush_rx(void) {
    return;
}
