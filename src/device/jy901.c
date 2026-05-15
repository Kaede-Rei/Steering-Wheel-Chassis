#include "jy901.h"

// 设置一个合理的超时时间 (毫秒)
#define I2C_TIMEOUT 100 

/**
 * @brief  发送唤醒JY-901或使其保持工作状态的指令
 */
int8_t JY901_WakeUp(void) {
    // 指令: 向寄存器 0x22 写入数据 {0xFF, 0xAA, 0x01, 0x00}
    uint8_t wakeup_payload[4] = {0xFF, 0xAA, 0x01, 0x00};
    
    // 使用 HAL_I2C_Mem_Write 直接向特定寄存器连续写入多字节
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, 
                                                 JY901_SLAVE_ADDRESS, 
                                                 JY901_REG_CONFIG_SLEEP, 
                                                 I2C_MEMADD_SIZE_8BIT, // 寄存器地址长度为8位
                                                 wakeup_payload, 
                                                 4, 
                                                 I2C_TIMEOUT);
    
    return (status == HAL_OK) ? 0 : -1;
}

/**
 * @brief  从JY-901读取Z轴角速度
 */
int8_t JY901_Read_Gyro_Z(float* gyro_z_dps) {
    uint8_t data_buffer[2]; 
    int16_t raw_value;

    // 使用 HAL_I2C_Mem_Read 直接从特定寄存器连续读取多字节
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 
                                                JY901_SLAVE_ADDRESS, 
                                                JY901_REG_GYRO_Z_L, 
                                                I2C_MEMADD_SIZE_8BIT, 
                                                data_buffer, 
                                                2, 
                                                I2C_TIMEOUT);

    if (status != HAL_OK) {
        return -1; // 读取失败
    }

    // 数据是低字节在前 (data_buffer[0]), 高字节在后 (data_buffer[1])
    raw_value = (int16_t)(((int16_t)data_buffer[1] << 8) | data_buffer[0]);

    // 根据资料公式转换: 角速度Z = ((WzH << 8) | WzL) / 32768.0 * 2000.0
    *gyro_z_dps = (float)raw_value / 32768.0f * 2000.0f;

    return 0; // 读取成功
}

/**
 * @brief  从JY-901读取Yaw轴角度 (偏航角)
 */
int8_t JY901_Read_Yaw(float* yaw_deg) {
    uint8_t data_buffer[2]; 
    int16_t raw_value;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 
                                                JY901_SLAVE_ADDRESS, 
                                                JY901_REG_ANGLE_YAW_L, 
                                                I2C_MEMADD_SIZE_8BIT, 
                                                data_buffer, 
                                                2, 
                                                I2C_TIMEOUT);

    if (status != HAL_OK) {
        return -1; // 读取失败
    }

    // 数据是低字节在前 (data_buffer[0]), 高字节在后 (data_buffer[1])
    raw_value = (int16_t)(((int16_t)data_buffer[1] << 8) | data_buffer[0]);

    // 根据资料公式转换: 偏航角Z = ((YawH << 8) | YawL) / 32768.0 * 180.0
    *yaw_deg = (float)raw_value / 32768.0f * 180.0f;

    return 0; // 读取成功
}