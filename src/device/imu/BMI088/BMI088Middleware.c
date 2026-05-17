#include "imu/BMI088/inc/BMI088Middleware.h"

#define BMI088_USING_SPI_UNIT   hspi2

extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

void BMI088_GPIO_init(void) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    BMI088_delay_ms(1);
}

void BMI088_com_init(void) {
}

void BMI088_delay_ms(uint16_t ms) {
    while(ms--) {
        BMI088_delay_us(1000);
    }
}

void BMI088_delay_us(uint16_t us) {
    uint32_t ticks = 0U;
    uint32_t told = 0U;
    uint32_t tnow = 0U;
    uint32_t tcnt = 0U;
    uint32_t reload = SysTick->LOAD;

    ticks = us * (SystemCoreClock / 1000000U);
    told = SysTick->VAL;

    while(1) {
        tnow = SysTick->VAL;
        if(tnow != told) {
            if(tnow < told) {
                tcnt += told - tnow;
            }
            else {
                tcnt += reload - tnow + told;
            }

            told = tnow;
            if(tcnt >= ticks) {
                break;
            }
        }
    }
}

void BMI088_ACCEL_NS_L(void) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void) {
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void) {
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata) {
    uint8_t rx_data = 0U;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}

SPI_HandleTypeDef* BMI088_get_spi_handle(void) {
    return &BMI088_USING_SPI_UNIT;
}

HAL_StatusTypeDef BMI088_SPI_TransmitReceive(
    uint8_t* tx_data,
    uint8_t* rx_data,
    uint16_t len,
    uint32_t timeout) {
    return HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, tx_data, rx_data, len, timeout);
}

HAL_StatusTypeDef BMI088_SPI_TransmitReceive_DMA(
    uint8_t* tx_data,
    uint8_t* rx_data,
    uint16_t len) {
    return HAL_SPI_TransmitReceive_DMA(&BMI088_USING_SPI_UNIT, tx_data, rx_data, len);
}
